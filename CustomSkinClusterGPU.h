#pragma once
#include "CustomSkinClusterGPU.h"
#include "DeformerLBS.h"
#include <maya/MPxGPUDeformer.h>
#include <maya/MGPUDeformerRegistry.h>


class CustomSkinClusterGPU : public MPxGPUDeformer {
public:
    static MGPUDeformerRegistrationInfo* getGPUDeformerInfo();
    static bool validateNodeInGraph(MDataBlock& block, const MEvaluationNode&, const MPlug& plug, MStringArray* messages);
    static bool validateNodeValues(MDataBlock& block, const MEvaluationNode&, const MPlug& plug, MStringArray* messages);

    CustomSkinClusterGPU() = default;
    ~CustomSkinClusterGPU() override = default;

    MPxGPUDeformer::DeformerStatus evaluate(
        MDataBlock& block,
        const MEvaluationNode& evaluationNode,
        const MPlug& outputPlug,
        const MPlugArray& inputPlugs,
        const MGPUDeformerData& inputData,
        MGPUDeformerData& outputData) override;

private:
    GPUDeformerLBS m_lbsDeformer;
};

/// <summary>
/// registration information for the CustomSkinClusterGPU
/// </summary>
class CustomSkinClusterGPUInfo : public MGPUDeformerRegistrationInfo {
public:
    CustomSkinClusterGPUInfo() {}
    ~CustomSkinClusterGPUInfo() override {}

    MPxGPUDeformer* createGPUDeformer() override {
        return new CustomSkinClusterGPU();
    }

    bool validateNodeInGraph(MDataBlock& block, const MEvaluationNode& evaluationNode, const MPlug& plug, MStringArray* messages) override
    {
        return CustomSkinClusterGPU::validateNodeInGraph(block, evaluationNode, plug, messages);
    }

    bool validateNodeValues(MDataBlock& block, const MEvaluationNode& evaluationNode, const MPlug& plug, MStringArray* messages) override
    {
        return CustomSkinClusterGPU::validateNodeValues(block, evaluationNode, plug, messages);
    }

};

