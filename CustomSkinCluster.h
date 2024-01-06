#pragma once

#include <maya/MPxSkinCluster.h>
#include <maya/MDataBlock.h>
#include <maya/MItGeometry.h>
#include <maya/MMatrix.h>

class CustomSkinCluster : public MPxSkinCluster
{
public:
	MStatus deform(MDataBlock& block, MItGeometry& iter, const MMatrix& mat, unsigned int multiIdx) override;
	static void* creator();
	static MStatus initialize();

public:
	static const MTypeId id;
};