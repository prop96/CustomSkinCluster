#pragma once

#include <maya/MPxSkinCluster.h>
#include <maya/MDataBlock.h>
#include <maya/MItGeometry.h>
#include <maya/MMatrix.h>

class CustomSkinCluster : public MPxSkinCluster
{
public:
	MStatus compute(const MPlug& plug, MDataBlock& data) override;
	MStatus deform(MDataBlock& block, MItGeometry& iter, const MMatrix& mat, unsigned int multiIdx) override;
	static void* creator();
	static MStatus initialize();

	inline static const MString nodeTypeName = "customSkinCluster";

public:
	static const MTypeId id;

	static MObject customSkinningMethod;

private:

	MPoint deformLBS(
		const MPoint& pt,
		const MMatrix& worldToLocal,
		MArrayDataHandle& transformsHandle,
		MArrayDataHandle& bindHandle,
		MArrayDataHandle& weightsHandle,
		MStatus* ptrStat);
};