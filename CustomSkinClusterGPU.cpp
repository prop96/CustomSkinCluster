#include "CustomSkinClusterGPU.h"
#include "CustomSkinCluster.h"
#include <maya/MOpenCLInfo.h>
#include <maya/MGlobal.h>


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
	// inputPlugs は基本1つ（inputPositions）しかない？ 
	{
		const int numInputPlugs = inputPlugs.length();
		std::cout << numInputPlugs << std::endl;
	}

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

	// set up OpenCL kernel if not set up
	if (!fKernel.get())
	{
		SetupKernel(block, numElements);
	}

	cl_int err = CL_SUCCESS;
	// set all of our kernel parameters
	uint32_t parameterId = 0;
	err = clSetKernelArg(fKernel.get(), parameterId++, sizeof(cl_mem), (void*)outputPositions.buffer().getReadOnlyRef());
	MOpenCLInfo::checkCLErrorStatus(err);
	err = clSetKernelArg(fKernel.get(), parameterId++, sizeof(cl_mem), (void*)inputPositions.buffer().getReadOnlyRef());
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

	MString kernelFile = CustomSkinCluster::pluginPath + "/../identity.cl";
	MString kernelName = "identity";

	fKernel = MOpenCLInfo::getOpenCLKernel(kernelFile, kernelName);
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
