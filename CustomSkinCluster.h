#pragma once

#include "DeformerDDM.h"
#include <maya/MPxSkinCluster.h>
#include <maya/MDataBlock.h>
#include <maya/MItGeometry.h>


class CustomSkinCluster : public MPxSkinCluster
{
public:
	MStatus deform(MDataBlock& block, MItGeometry& iter, const MMatrix& mat, unsigned int multiIdx) override;
	static void* creator();
	static MStatus initialize();

	inline static const MString nodeTypeName = "customSkinCluster";

public:
	static const MTypeId id;

	static MObject customSkinningMethod;
	static MObject doRecompute;

private:

	DeformerDDM m_ddmDeformer;

	MPoint deformLBS(
		int vertIdx,
		const MPoint& pt,
		const MMatrix& worldToLocal,
		MArrayDataHandle& transformsHandle,
		MArrayDataHandle& bindHandle,
		MArrayDataHandle& weightsHandle,
		MStatus* ptrStat);
};