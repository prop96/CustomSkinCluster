#pragma once

#include "DeformerLBS.h"
#include "DeformerDDM.h"
#include "DeformerDeltaMush.h"
#include <maya/MPxSkinCluster.h>
#include <maya/MDataBlock.h>
#include <maya/MItGeometry.h>

class CustomSkinCluster : public MPxSkinCluster
{
public:
	MStatus deform(MDataBlock& block, MItGeometry& iter, const MMatrix& mat, unsigned int multiIdx) override;
	static MStatus initialize();
	static void* creator()
	{
		return new CustomSkinCluster();
	}

	static const MTypeId id;
	inline static const MString nodeTypeName = "customSkinCluster";
	static MString pluginPath;

	enum class SkinningType : int8_t
	{
		LBS = 0,
		DMLBS,
		DDM,
		DDM_v1,
		DDM_v2,
		DDM_v3,
		DDM_v4,
		DDM_v5,
	};

	static MObject customSkinningMethod;
	static MObject doRecompute;
	static MObject needRebindMesh;
	static MObject smoothAmount;
	static MObject smoothIteration;

private:
	DeformerDDM m_ddmDeformer;
	DeformerLBS m_lbsDeformer;
	DeformerDeltaMush m_dmDeformer;
};