#pragma once
#include <maya/MPoint.h>
#include <maya/MIntArray.h>
#include <maya/MVectorArray.h>
#include <maya/MPointArray.h>
#include <maya/MMatrix.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MStatus.h>
#include <vector>

class DeformerDeltaMush
{
public:
	DeformerDeltaMush();
	~DeformerDeltaMush() = default;

	MStatus InitializeData(MObject& mesh);
	MStatus InitializeData(MObject& mesh, uint32_t smoothingIter, double smoothingAmount);

	/// <summary>
	/// 
	/// </summary>
	/// <param name="deformed">[out]</param>
	/// <param name="skinned">[in]</param>
	void ApplyDeltaMush(
		const MPointArray& skinned,
		MPointArray& deformed) const;

	void SetSmoothingData(uint32_t iter, double amount);

	struct PointData {
		// できれば std::vector に変更した方がデバッグしやすい
		MIntArray NeighbourIndices;
		MVectorArray Delta;
		uint32_t NeighbourNum;
		double DeltaLength;
	};

	struct SmoothingData {
		uint32_t Iter = 0;
		double Amount = 1.0;
	};

private:
	MPointArray targetPos;
	std::vector<PointData> dataPoints;
	bool isInitialized;

	SmoothingData smoothingData;

	void ComputeSmoothedPoints(const MPointArray& src, MPointArray& smoothed) const;

	void ComputeDelta(const MPointArray& src, const MPointArray& smoothed);

	MMatrix ComputeTangentMatrix(const MPoint& pos, const MPoint& posNeighbor0, const MPoint& posNeighbor1) const;
};
