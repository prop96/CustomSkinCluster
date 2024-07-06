#include "DeformerDeltaMush.h"
#include <maya/MFnMatrixData.h>
#include <maya/MFnMesh.h>
#include <maya/MDataHandle.h>
#include <maya/MItMeshVertex.h>

DeformerDeltaMush::DeformerDeltaMush()
	: targetPos()
	, dataPoints()
	, isInitialized()
	, smoothingData()
{
}

MStatus DeformerDeltaMush::InitializeData(MObject& mesh)
{
	MStatus stat;

	// 頂点ごとのデータ配列を初期化
	dataPoints.clear();

	// 頂点の隣接情報を格納
	MItMeshVertex iter(mesh);
	for (iter.reset(); iter.isDone(); iter.next())
	{
		PointData pd;
		
		// 隣接頂点インデックスを取得
		iter.getConnectedVertices(pd.NeighbourIndices);

		// 隣接頂点数
		pd.NeighbourNum = pd.NeighbourIndices.length();

		// 隣接頂点ごとの delta を保持する配列を初期化
		pd.Delta.setLength(pd.NeighbourNum);

		dataPoints.push_back(std::move(pd));
	}

	MFnMesh meshFn(mesh);

	// メッシュの頂点座標を取得
	MPointArray posOriginal;
	meshFn.getPoints(posOriginal, MSpace::kObject);

	// Smoothing 処理 (posSmoothed を計算)
	MPointArray posSmoothed;
	ComputeSmoothedPoints(posOriginal, posSmoothed);

	// Delta を計算して dataPoints に格納
	ComputeDelta(posOriginal, posSmoothed);

	isInitialized = true;

	return stat;
}

MStatus DeformerDeltaMush::InitializeData(MObject& mesh, uint32_t smoothingIter, double smoothingAmount)
{
	SetSmoothingData(smoothingIter, smoothingAmount);
	return InitializeData(mesh);
}

void DeformerDeltaMush::ApplyDeltaMush(const MPointArray& skinned, MPointArray& deformed) const
{
	// NOTE: skinned はワールド座標系での頂点位置と想定

	double envelope = 1.0;
	double applyDelta = 1.0;

	if (!isInitialized)
	{
		//InitializeData()
	}

	uint32_t numVerts = skinned.length();
	deformed.setLength(numVerts);

	// compute mush
	MPointArray mushed;
	ComputeSmoothedPoints(skinned, mushed);

	// apply delta to mush
	for (uint32_t vertIdx = 0; vertIdx < numVerts; vertIdx++)
	{
		const PointData& pointData = dataPoints[vertIdx];

		// compute delta in animated pose
		MVector delta = MVector::zero;

		// looping the neighbours
		for (uint32_t neighborIdx = 0; neighborIdx < pointData.NeighbourNum - 1; neighborIdx++)
		{
			MMatrix mat = ComputeTangentMatrix(
				mushed[vertIdx],
				mushed[pointData.NeighbourIndices[neighborIdx]],
				mushed[pointData.NeighbourIndices[neighborIdx + 1]]);

			delta += (pointData.Delta[neighborIdx] * mat);
		}
		delta /= static_cast<double>(pointData.NeighbourNum);

		// delta の長さを合わせる
		delta = delta.normal() * pointData.DeltaLength;

		// add delta to mush
		MPoint deltaMushed = mushed[vertIdx] + delta * applyDelta;

		// envelope を考慮
		deformed[vertIdx] = skinned[vertIdx] + envelope * (deltaMushed - skinned[vertIdx]);
	}
}

void DeformerDeltaMush::SetSmoothingData(uint32_t iter, double amount)
{
	smoothingData.Iter = iter;
	smoothingData.Amount = amount;

	isInitialized = false;
}

void DeformerDeltaMush::ComputeSmoothedPoints(const MPointArray& src, MPointArray& smoothed) const
{
	const uint32_t numVerts = src.length();
	smoothed.setLength(numVerts);

	MPointArray srcCopy;
	srcCopy.copy(src);

	for (uint32_t itr = 0; itr < smoothingData.Iter; itr++)
	{
		for (uint32_t vertIdx = 0; vertIdx < numVerts; vertIdx++)
		{
			const PointData& pointData = dataPoints[vertIdx];

			// 隣接頂点の平均としてスムージング
			MVector smoothedPos = MVector::zero;
			for (const int neighbourIdx : pointData.NeighbourIndices)
			{
				smoothedPos += srcCopy[neighbourIdx];
			}
			smoothedPos *= 1.0 / double(pointData.NeighbourNum);

			smoothed[vertIdx] = srcCopy[vertIdx] + (smoothedPos - srcCopy[vertIdx]) * smoothingData.Amount;
		}

		srcCopy.copy(smoothed);
	}
}

void DeformerDeltaMush::ComputeDelta(const MPointArray& src, const MPointArray& smoothed)
{
	const uint32_t numVerts = src.length();

	// 各頂点ごとにデルタを計算
	for (uint32_t vertIdx = 0; vertIdx < numVerts; vertIdx++)
	{
		PointData& pointData = dataPoints[vertIdx];

		MVector delta = MVector(src[vertIdx] - smoothed[vertIdx]);
		pointData.DeltaLength = delta.length();

		// compute tangent matrix and delta in the tangent space
		for (uint32_t neighborIdx = 0; neighborIdx < pointData.NeighbourNum - 1; neighborIdx++)
		{
			MMatrix mat = ComputeTangentMatrix(
				smoothed[vertIdx],
				smoothed[pointData.NeighbourIndices[neighborIdx]],
				smoothed[pointData.NeighbourIndices[neighborIdx + 1]]);

			// 頂点の tangent space coordinate でデルタを保持する
			pointData.Delta[neighborIdx] = delta * mat.inverse();
		}
	}
}

MMatrix DeformerDeltaMush::ComputeTangentMatrix(const MPoint& pos, const MPoint& posNeighbor0, const MPoint& posNeighbor1) const
{
	// 注目している頂点と隣接頂点の作る三角形ポリゴンを考えて、tangent matrix を作る
	MVector v0 = posNeighbor0 - pos;
	MVector v1 = posNeighbor1 - pos;

	v0.normalize();
	v1.normalize();

	MVector t = v0;
	MVector n = t ^ v1;
	MVector b = n ^ t;

	MMatrix mat = MMatrix();
	{
		mat[0][0] = t.x;
		mat[0][1] = t.y;
		mat[0][2] = t.z;
		mat[0][3] = 0;
		mat[1][0] = b.x;
		mat[1][1] = b.y;
		mat[1][2] = b.z;
		mat[1][3] = 0;
		mat[2][0] = n.x;
		mat[2][1] = n.y;
		mat[2][2] = n.z;
		mat[2][3] = 0;
		mat[3][0] = 0;
		mat[3][1] = 0;
		mat[3][2] = 0;
		mat[3][3] = 1;
	}

	return mat;
}
