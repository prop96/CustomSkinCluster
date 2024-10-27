#include "CustomSkinClusterGPU.h"
#include "CustomSkinCluster.h"
#include <maya/MOpenCLInfo.h>
#include <maya/MGlobal.h>
#include <maya/MFnMatrixData.h>


MGPUDeformerRegistrationInfo* CustomSkinClusterGPU::getGPUDeformerInfo()
{
	static CustomSkinClusterGPUInfo theOne;
	return &theOne;
}

bool CustomSkinClusterGPU::validateNodeInGraph(MDataBlock& block, const MEvaluationNode&, const MPlug& plug, MStringArray* messages)
{
	// support everything
	return true;
}

bool CustomSkinClusterGPU::validateNodeValues(MDataBlock& block, const MEvaluationNode&, const MPlug& plug, MStringArray* messages)
{
	// support everything
	return true;
}

CustomSkinClusterGPU::CustomSkinClusterGPU() = default;

CustomSkinClusterGPU::~CustomSkinClusterGPU()
{
	terminate();
}

void CustomSkinClusterGPU::terminate()
{
	weightsBuffer.reset();
	influencesBuffer.reset();
	transformMatricesBuffer.reset();

	MOpenCLInfo::releaseOpenCLKernel(fKernel);
	fKernel.reset();
}

MPxGPUDeformer::DeformerStatus CustomSkinClusterGPU::evaluate(
	MDataBlock& block,
	const MEvaluationNode& evaluationNode,
	const MPlug& outputPlug,
	const MPlugArray& inputPlugs,
	const MGPUDeformerData& inputData,
	MGPUDeformerData& outputData)
{
	// extract inputPositions Buffer from the inputPlug
	const MPlug& inputPlug = inputPlugs[0];
	const MGPUDeformerBuffer inputPositions = inputData.getBuffer(MPxGPUDeformer::sPositionsName(), inputPlug);

	// create outputPositions Buffer
	MGPUDeformerBuffer outputPositions = createOutputBuffer(inputPositions);

	if (!inputPositions.isValid() || !outputPositions.isValid())
	{
		return kDeformerFailure;
	}

	// # of vertices in the mesh
	const uint32_t numElements = inputPositions.elementCount();

	// Load weights and transform matrices onto OpenCL buffer
	ExtractTransformMatrices(block, evaluationNode, inputPlug);
	ExtractWeights(block, evaluationNode, inputPlug, numElements);

	// set up OpenCL kernel if not set up
	if (!fKernel.get())
	{
		MString kernelFile = CustomSkinCluster::pluginPath + "/../skinLBS.cl";
		MString kernelName = "skinLBS";

		fKernel = MOpenCLInfo::getOpenCLKernel(kernelFile, kernelName);

		SetupKernel(block, numElements);
	}

	cl_int err = CL_SUCCESS;
	// set all of our kernel parameters
	uint32_t parameterId = 0;
	err = clSetKernelArg(fKernel.get(), parameterId++, sizeof(cl_mem), (void*)outputPositions.buffer().getReadOnlyRef());
	MOpenCLInfo::checkCLErrorStatus(err);
	err = clSetKernelArg(fKernel.get(), parameterId++, sizeof(cl_mem), (void*)inputPositions.buffer().getReadOnlyRef());
	MOpenCLInfo::checkCLErrorStatus(err);
	err = clSetKernelArg(fKernel.get(), parameterId++, sizeof(cl_mem), (void*)weightsBuffer.getReadOnlyRef());
	MOpenCLInfo::checkCLErrorStatus(err);
	err = clSetKernelArg(fKernel.get(), parameterId++, sizeof(cl_mem), (void*)influencesBuffer.getReadOnlyRef());
	MOpenCLInfo::checkCLErrorStatus(err);
	err = clSetKernelArg(fKernel.get(), parameterId++, sizeof(cl_mem), (void*)transformMatricesBuffer.getReadOnlyRef());
	MOpenCLInfo::checkCLErrorStatus(err);
	err = clSetKernelArg(fKernel.get(), parameterId++, sizeof(cl_uint), (void*)&numElements);
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
		fKernel.get(),
		1,
		nullptr,
		&fGlobalWorkSize,
		&fLocalWorkSize,
		eventCount,
		eventCount ? events : nullptr,
		kernelFinishedEvent.getReferenceForAssignment()
	);
	outputPositions.setBufferReadyEvent(kernelFinishedEvent);
	MOpenCLInfo::checkCLErrorStatus(err);
	if (err != CL_SUCCESS)
	{
		return kDeformerFailure;
	}

	outputData.setBuffer(outputPositions);
	return kDeformerSuccess;
}

MPxGPUDeformer::DeformerStatus CustomSkinClusterGPU::SetupKernel(MDataBlock& block, int32_t numElements)
{
	cl_int err = CL_SUCCESS;

	if (fKernel.isNull())
	{
		MGlobal::displayError("Error: Failed to get kernel from file");
	}

	// Figure out a good work group size for our kernel
	fLocalWorkSize = 0;
	fGlobalWorkSize = 0;
	size_t retSize = 0;
	err = clGetKernelWorkGroupInfo(
		fKernel.get(),
		MOpenCLInfo::getOpenCLDeviceId(),
		CL_KERNEL_WORK_GROUP_SIZE,
		sizeof(size_t),
		&fLocalWorkSize,
		&retSize
	);
	MOpenCLInfo::checkCLErrorStatus(err);
	if (err != CL_SUCCESS || retSize == 0 || fLocalWorkSize == 0)
	{
		return kDeformerFailure;
	}

	// global work size must be a multiple of local work size
	const size_t remain = numElements % fLocalWorkSize;
	fGlobalWorkSize = numElements + (remain != 0 ? fLocalWorkSize - remain : 0);

	return kDeformerSuccess;
}

MStatus CustomSkinClusterGPU::ExtractWeights(MDataBlock& block, const MEvaluationNode& evaluationNode, const MPlug& plug, uint32_t numElements)
{
	MStatus status;
	const bool needUpdate = weightsBuffer.isNull() || influencesBuffer.isNull() || evaluationNode.dirtyPlugExists(CustomSkinCluster::weightList, &status);
	if (!needUpdate)
	{
		return status;
	}

	std::vector<float> weightsContainer;
	std::vector<uint32_t> influencesContainer;

	MArrayDataHandle weightListsHandle = block.inputArrayValue(CustomSkinCluster::weightList, &status);

	weightListsHandle.jumpToArrayElement(0);
	for (uint32_t vIdx = 0; vIdx < numElements; vIdx++)
	{
		MArrayDataHandle weightsHandle = weightListsHandle.inputValue(&status).child(CustomSkinCluster::weights);

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
	if (!weightsBuffer.get())
	{
		weightsBuffer.attach(
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
			weightsBuffer.get(),
			CL_TRUE,
			0,
			weightsContainer.size() * sizeof(float),
			(void*)weightsContainer.data(),
			0,
			nullptr,
			nullptr);
	}

	if (!influencesBuffer.get())
	{
		influencesBuffer.attach(
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
			influencesBuffer.get(),
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

MStatus CustomSkinClusterGPU::ExtractTransformMatrices(MDataBlock& block, const MEvaluationNode& evaluationNode, const MPlug& plug)
{
	MStatus status;
	if (!transformMatricesBuffer.isNull() && !evaluationNode.dirtyPlugExists(CustomSkinCluster::matrix, &status))
	{
		return status;
	}

	std::vector<float> matricesContainer;

	MArrayDataHandle bindHandle = block.inputArrayValue(CustomSkinCluster::bindPreMatrix, &status);
	MArrayDataHandle transformsHandle = block.inputArrayValue(CustomSkinCluster::matrix, &status);
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
				matricesContainer.push_back(static_cast<float>(jointMat(r,c)));
			}
		}

		transformsHandle.next();
		bindHandle.next();
	}

	cl_int err = CL_SUCCESS;
	if (!transformMatricesBuffer.get())
	{
		transformMatricesBuffer.attach(
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
			transformMatricesBuffer.get(),
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
