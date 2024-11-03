#pragma once
#include <maya/MPoint.h>
#include <maya/MMatrix.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MStatus.h>
#include <maya/MPxGPUDeformer.h>

class DeformerLBS
{
public:
	DeformerLBS() = default;
	~DeformerLBS() = default;

	MPoint Deform(
		int vertIdx,
		const MPoint& pt,
		const MMatrix& worldToLocal,
		MArrayDataHandle& transformsHandle,
		MArrayDataHandle& bindHandle,
		MArrayDataHandle& weightsHandle,
		MStatus* ptrStat) const;
};

class GPUDeformerLBS
{
public:
	GPUDeformerLBS() = default;
	~GPUDeformerLBS() = default;

	void Terminate();

	MPxGPUDeformer::DeformerStatus Evaluate(
		MDataBlock& block,
		const MEvaluationNode& evaluationNode,
		const MString& pluginPath,
		const MGPUDeformerBuffer& inputPositions,
		MGPUDeformerBuffer& outputPositions);

private:
	MAutoCLKernel m_kernel;
	uint32_t m_numVertices;
	size_t m_localWorkSize;
	size_t m_globalWorkSize;

	MAutoCLMem m_weightsBuffer;
	MAutoCLMem m_influencesBuffer;
	MAutoCLMem m_transformMatricesBuffer;

	MPxGPUDeformer::DeformerStatus SetupKernel(const MString& pluginPath, uint32_t numVertices);
	MPxGPUDeformer::DeformerStatus SetWorkSize(uint32_t numVertices);

	MStatus ExtractWeights(MDataBlock& block, const MEvaluationNode& evaluationNode);
	MStatus ExtractTransformMatrices(MDataBlock& block, const MEvaluationNode& evaluationNode);
};
