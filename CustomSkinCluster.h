#pragma once

#include <maya/MPxSkinCluster.h>
#include <maya/MDataBlock.h>
#include <maya/MItGeometry.h>
#include <maya/MMatrix.h>
#include <unordered_map>

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
	static MObject pstarArray_CoR;

private:

	// skin weight as an unordered map, whose key is the bone idx
	using weight_map = std::unordered_map<unsigned int, float>;
	double ComputeSimilarlityForCoR(const weight_map& w0, const weight_map& w1, double sigma);

	MPoint deformLBS(
		const MPoint& pt,
		const MMatrix& worldToLocal,
		MArrayDataHandle& transformsHandle,
		MArrayDataHandle& bindHandle,
		MArrayDataHandle& weightsHandle,
		MStatus* ptrStat);
};