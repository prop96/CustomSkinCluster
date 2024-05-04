#pragma once
#include <maya/MPoint.h>
#include <maya/MMatrix.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MStatus.h>

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
