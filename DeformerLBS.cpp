#include "DeformerLBS.h"
#include <maya/MFnMatrixData.h>
#include <maya/MDataHandle.h>
#include <maya/MOpenCLInfo.h>
#include <maya/MGlobal.h>
#include <maya/MPxSkinCluster.h>


MPoint DeformerLBS::Deform(
	int vertIdx,
	const MPoint& pt,
	const MMatrix& worldToLocal,
	MArrayDataHandle& transformsHandle,
	MArrayDataHandle& bindHandle,
	MArrayDataHandle& weightsHandle,
	MStatus* ptrStat) const
{
	MPoint skinned;

	// compute influences from each joint
	unsigned int numWeights = weightsHandle.elementCount(); // # of nonzero weights
	for (unsigned int wIdx = 0; wIdx < numWeights; wIdx++)
	{
		weightsHandle.jumpToArrayElement(wIdx); // jump to physical index
		double w = weightsHandle.inputValue().asDouble();

		// logical index corresponds to the joint index
		unsigned int jointIdx = weightsHandle.elementIndex(ptrStat);

		transformsHandle.jumpToElement(jointIdx); // jump to logical index
		MMatrix jointMat = MFnMatrixData(transformsHandle.inputValue().data()).matrix();

		bindHandle.jumpToElement(jointIdx); // jump to logical index
		MMatrix preBindMatrix = MFnMatrixData(bindHandle.inputValue().data()).matrix();
		jointMat = preBindMatrix * jointMat;

		skinned += (pt * jointMat) * w;
	}

	return skinned * worldToLocal;
}

void GPUDeformerLBS::Terminate()
{
	m_weightsBuffer.reset();
	m_influencesBuffer.reset();
	m_transformMatricesBuffer.reset();

	MOpenCLInfo::releaseOpenCLKernel(m_kernel);
	m_kernel.reset();
}

MPxGPUDeformer::DeformerStatus GPUDeformerLBS::Evaluate(
	MDataBlock& block,
	const MEvaluationNode& evaluationNode,
	const MString& pluginPath,
	const MGPUDeformerBuffer& inputPositions,
	MGPUDeformerBuffer& outputPositions)
{
	// # of vertices in the mesh
	const uint32_t numVertices = inputPositions.elementCount();

	// set up OpenCL kernel if not
	if (!m_kernel.get())
	{
		SetupKernel(pluginPath, numVertices);
	}
	else if (numVertices != m_numVertices)
	{
		SetWorkSize(numVertices);
	}

	// Load weights and transform matrices onto OpenCL buffer
	ExtractTransformMatrices(block, evaluationNode);
	ExtractWeights(block, evaluationNode);

	cl_int err = CL_SUCCESS;
	// set all of our kernel parameters
	uint32_t parameterId = 0;
	err = clSetKernelArg(m_kernel.get(), parameterId++, sizeof(cl_mem), (void*)outputPositions.buffer().getReadOnlyRef());
	MOpenCLInfo::checkCLErrorStatus(err);
	err = clSetKernelArg(m_kernel.get(), parameterId++, sizeof(cl_mem), (void*)inputPositions.buffer().getReadOnlyRef());
	MOpenCLInfo::checkCLErrorStatus(err);
	err = clSetKernelArg(m_kernel.get(), parameterId++, sizeof(cl_mem), (void*)m_weightsBuffer.getReadOnlyRef());
	MOpenCLInfo::checkCLErrorStatus(err);
	err = clSetKernelArg(m_kernel.get(), parameterId++, sizeof(cl_mem), (void*)m_influencesBuffer.getReadOnlyRef());
	MOpenCLInfo::checkCLErrorStatus(err);
	err = clSetKernelArg(m_kernel.get(), parameterId++, sizeof(cl_mem), (void*)m_transformMatricesBuffer.getReadOnlyRef());
	MOpenCLInfo::checkCLErrorStatus(err);
	err = clSetKernelArg(m_kernel.get(), parameterId++, sizeof(cl_uint), (void*)&numVertices);
	MOpenCLInfo::checkCLErrorStatus(err);

	// set up our input events.  The input event could be NULL, in that case we need to pass
	// slightly different parameters into clEnqueueNDRangeKernel.
	cl_event events[1] = { 0 };
	cl_uint eventCount = 0;
	if (inputPositions.bufferReadyEvent().get())
	{
		events[eventCount++] = inputPositions.bufferReadyEvent().get();
	}

	// run the kernel
	MAutoCLEvent kernelFinishedEvent;
	err = clEnqueueNDRangeKernel(
		MOpenCLInfo::getMayaDefaultOpenCLCommandQueue(),
		m_kernel.get(),
		1,
		nullptr,
		&m_globalWorkSize,
		&m_localWorkSize,
		eventCount,
		eventCount ? events : nullptr,
		kernelFinishedEvent.getReferenceForAssignment()
	);
	outputPositions.setBufferReadyEvent(kernelFinishedEvent);
	MOpenCLInfo::checkCLErrorStatus(err);
	if (err != CL_SUCCESS)
	{
		return MPxGPUDeformer::kDeformerFailure;
	}
}

MPxGPUDeformer::DeformerStatus GPUDeformerLBS::SetupKernel(const MString& pluginPath, uint32_t numVertices)
{
	m_numVertices = numVertices;

	const MString kernelFile = pluginPath + "/../skinLBS.cl";
	const MString kernelName = "skinLBS";
	m_kernel = MOpenCLInfo::getOpenCLKernel(kernelFile, kernelName);

	if (m_kernel.isNull())
	{
		MGlobal::displayError("Error: Failed to get kernel from file: " + kernelName + " in " + kernelFile);
	}

	return SetWorkSize(numVertices);
}

MPxGPUDeformer::DeformerStatus GPUDeformerLBS::SetWorkSize(uint32_t numVertices)
{
	cl_int err = CL_SUCCESS;

	// Set new # of vertices
	m_numVertices = numVertices;

	// Figure out a good work group size for our kernel
	m_localWorkSize = 0;
	m_globalWorkSize = 0;
	size_t retSize = 0;
	err = clGetKernelWorkGroupInfo(
		m_kernel.get(),
		MOpenCLInfo::getOpenCLDeviceId(),
		CL_KERNEL_WORK_GROUP_SIZE,
		sizeof(size_t),
		&m_localWorkSize,
		&retSize
	);
	MOpenCLInfo::checkCLErrorStatus(err);
	if (err != CL_SUCCESS || retSize == 0 || m_localWorkSize == 0)
	{
		return MPxGPUDeformer::kDeformerFailure;
	}

	// global work size must be a multiple of local work size
	const size_t remain = numVertices % m_localWorkSize;
	m_globalWorkSize = numVertices + (remain != 0 ? m_localWorkSize - remain : 0);

	return MPxGPUDeformer::kDeformerSuccess;
}

MStatus GPUDeformerLBS::ExtractWeights(MDataBlock& block, const MEvaluationNode& evaluationNode)
{
	MStatus status;
	const bool needUpdate = m_weightsBuffer.isNull() || m_influencesBuffer.isNull() || evaluationNode.dirtyPlugExists(MPxSkinCluster::weightList, &status);
	if (!needUpdate)
	{
		return status;
	}

	std::vector<float> weightsContainer;
	std::vector<uint32_t> influencesContainer;

	MArrayDataHandle weightListsHandle = block.inputArrayValue(MPxSkinCluster::weightList, &status);

	weightListsHandle.jumpToArrayElement(0);
	for (uint32_t vIdx = 0; vIdx < m_numVertices; vIdx++)
	{
		MArrayDataHandle weightsHandle = weightListsHandle.inputValue(&status).child(MPxSkinCluster::weights);

		const uint32_t numWeights = weightsHandle.elementCount(); // # of nonzero weights
		for (uint32_t wIdx = 0; wIdx < numWeights; wIdx++) {
			weightsHandle.jumpToArrayElement(wIdx); // jump to physical index

			const double w = weightsHandle.inputValue().asDouble();
			weightsContainer.push_back(static_cast<float>(w));

			const uint32_t jointIdx = weightsHandle.elementIndex(&status); // logical index corresponds to joint index
			influencesContainer.push_back(jointIdx);
		}

		// advance the weight list handle
		CHECK_MSTATUS(weightListsHandle.next());
	}

	cl_int err = CL_SUCCESS;
	if (!m_weightsBuffer.get())
	{
		m_weightsBuffer.attach(
			clCreateBuffer(MOpenCLInfo::getOpenCLContext(),
				CL_MEM_COPY_HOST_PTR | CL_MEM_READ_ONLY,
				weightsContainer.size() * sizeof(float),
				(void*)weightsContainer.data(),
				&err)
		);
	}
	else
	{
		err = clEnqueueWriteBuffer(
			MOpenCLInfo::getMayaDefaultOpenCLCommandQueue(),
			m_weightsBuffer.get(),
			CL_TRUE,
			0,
			weightsContainer.size() * sizeof(float),
			(void*)weightsContainer.data(),
			0,
			nullptr,
			nullptr);
	}

	if (!m_influencesBuffer.get())
	{
		m_influencesBuffer.attach(
			clCreateBuffer(MOpenCLInfo::getOpenCLContext(),
				CL_MEM_COPY_HOST_PTR | CL_MEM_READ_ONLY,
				influencesContainer.size() * sizeof(uint32_t),
				(void*)influencesContainer.data(),
				&err)
		);
	}
	else
	{
		err = clEnqueueWriteBuffer(
			MOpenCLInfo::getMayaDefaultOpenCLCommandQueue(),
			m_influencesBuffer.get(),
			CL_TRUE,
			0,
			influencesContainer.size() * sizeof(float),
			(void*)influencesContainer.data(),
			0,
			nullptr,
			nullptr);
	}

	return status;
}

MStatus GPUDeformerLBS::ExtractTransformMatrices(MDataBlock& block, const MEvaluationNode& evaluationNode)
{
	MStatus status;
	if (!m_transformMatricesBuffer.isNull() && !evaluationNode.dirtyPlugExists(MPxSkinCluster::matrix, &status))
	{
		return status;
	}

	std::vector<float> matricesContainer;

	MArrayDataHandle bindHandle = block.inputArrayValue(MPxSkinCluster::bindPreMatrix, &status);
	MArrayDataHandle transformsHandle = block.inputArrayValue(MPxSkinCluster::matrix, &status);
	uint32_t numJoints = transformsHandle.elementCount();

	for (uint32_t jIdx = 0; jIdx < numJoints; jIdx++)
	{
		MMatrix jointMat = MFnMatrixData(transformsHandle.inputValue().data()).matrix();
		MMatrix preBindMatrix = MFnMatrixData(bindHandle.inputValue().data()).matrix();
		jointMat = preBindMatrix * jointMat;

		// send as 4x3 matrix to GPU
		for (uint32_t c = 0; c < 3; c++)
		{
			for (uint32_t r = 0; r < 4; r++)
			{
				matricesContainer.push_back(static_cast<float>(jointMat(r, c)));
			}
		}

		transformsHandle.next();
		bindHandle.next();
	}

	cl_int err = CL_SUCCESS;
	if (!m_transformMatricesBuffer.get())
	{
		m_transformMatricesBuffer.attach(
			clCreateBuffer(MOpenCLInfo::getOpenCLContext(),
				CL_MEM_COPY_HOST_PTR | CL_MEM_READ_ONLY,
				matricesContainer.size() * sizeof(float),
				(void*)matricesContainer.data(),
				&err)
		);
	}
	else
	{
		err = clEnqueueWriteBuffer(
			MOpenCLInfo::getMayaDefaultOpenCLCommandQueue(),
			m_transformMatricesBuffer.get(),
			CL_TRUE,
			0,
			matricesContainer.size() * sizeof(float),
			(void*)matricesContainer.data(),
			0,
			nullptr,
			nullptr);
	}

	return status;
}
