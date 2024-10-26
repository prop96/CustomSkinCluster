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

	inline static const MString nodeTypeName = "customSkinCluster";
	static MString pluginPath;

public:
	static const MTypeId id;

	static MObject customSkinningMethod;
	static MObject doRecompute;
	static MObject needRebindMesh;
	static MObject smoothAmount;
	static MObject smoothIteration;

public:
	static void* creator()
	{
		return new CustomSkinCluster();
	}

private:

	DeformerDDM m_ddmDeformer;
	DeformerLBS m_lbsDeformer;
	DeformerDeltaMush m_dmDeformer;
};