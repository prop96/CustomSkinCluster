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

	// check validity of input/outputPosition buffers
	if (!inputPositions.isValid() || !outputPositions.isValid())
	{
		return kDeformerFailure;
	}

	// main evaluate process
	m_lbsDeformer.Evaluate(block, evaluationNode, CustomSkinCluster::pluginPath, inputPositions, outputPositions);

	// set the results
	outputData.setBuffer(outputPositions);
	return kDeformerSuccess;
}
