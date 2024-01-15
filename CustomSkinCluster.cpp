#include "CustomSkinCluster.h"
#include <maya/MItMeshVertex.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MMatrixArray.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MPointArray.h>
#include <maya/MPoint.h>
#include <vector>

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnPointArrayData.h>

const MTypeId CustomSkinCluster::id(0x00080031);
MObject CustomSkinCluster::customSkinningMethod;
MObject CustomSkinCluster::pstarArray_CoR;


MStatus CustomSkinCluster::compute(const MPlug& plug, MDataBlock& data)
{
	MStatus returnStatus;

	MString info = plug.info();
	cout << info << endl;

	if (plug == pstarArray_CoR)
	{
		// get the attribute instance of input[0].inputGeometry
		MObject mesh = data.inputArrayValue(input).inputValue().child(inputGeom).data();

		// weightLists
		MArrayDataHandle weightListsHandle = data.inputArrayValue(weightList);

		std::vector<float> areas;
		std::vector<MPoint> centers;
		std::vector<weight_map> averageWeights;

		// iterate through each triangles
		MItMeshPolygon itPolygon(mesh);
		for (; !itPolygon.isDone(); itPolygon.next())
		{
			int numTriangles;
			itPolygon.numTriangles(numTriangles);
			for (size_t triIdx = 0; triIdx < numTriangles; triIdx++)
			{
				// get the triangle info
				MPointArray pts;
				MIntArray vIdxs;
				itPolygon.getTriangle(triIdx, pts, vIdxs);

				// the vertices of the triangle
				MPoint pt0 = pts[0], pt1 = pts[1], pt2 = pts[2];
				int vIdx0 = vIdxs[0], vIdx1 = vIdxs[1], vIdx2 = vIdxs[2];

				// the area and the average position of the triangle
				double area = 0.5 * (MVector(pt1 - pt0) ^ MVector(pt2 - pt0)).length();
				MPoint center = (pt0 + pt1 + pt2) / 3.0;

				// compute the average skin weights
				weight_map weight_t;
				for (const auto& vIdx : vIdxs)
				{
					// get skin weights for the vertex
					weightListsHandle.jumpToElement(vIdx);
					MArrayDataHandle weightsHandle = weightListsHandle.inputValue().child(weights);
					unsigned int numWeights = weightsHandle.elementCount();
					for (unsigned int wIdx = 0; wIdx < numWeights; wIdx++)
					{
						weightsHandle.jumpToArrayElement(wIdx); // jump to physical index
						double w = weightsHandle.inputValue().asDouble() / 3.0;

						// logical index corresponds to the joint index
						unsigned int jointIdx = weightsHandle.elementIndex();

						if (weight_t.find(jointIdx) != weight_t.end())
						{
							weight_t[jointIdx] += w;
						}
						else
						{
							weight_t.emplace(jointIdx, w);
						}
					}
				}

				areas.push_back(area);
				centers.push_back(center);
				averageWeights.push_back(weight_t);
			}
		}

		MArrayDataHandle pstarArrayHandle = data.outputArrayValue(pstarArray_CoR, &returnStatus);
		CHECK_MSTATUS(returnStatus);


		// iterate through each vertices to compute pstar_i
		MItMeshVertex itVertex(mesh);
		MPointArray pointArray;
		pointArray.setLength(itVertex.count());
		for (; !itVertex.isDone(); itVertex.next())
		{
			MPoint vertexPosition = itVertex.position();
			cout << vertexPosition << endl;

			weight_map weight_i;

			// get skin weights
			weightListsHandle.jumpToElement(itVertex.index());
			MArrayDataHandle weightsHandle = weightListsHandle.inputValue().child(weights);
			unsigned int numWeights = weightsHandle.elementCount();
			for (unsigned int wIdx = 0; wIdx < numWeights; wIdx++)
			{
				weightsHandle.jumpToArrayElement(wIdx); // jump to physical index
				double w = weightsHandle.inputValue().asDouble() / 3.0;

				// logical index corresponds to the joint index
				unsigned int jointIdx = weightsHandle.elementIndex();

				weight_i.emplace(jointIdx, w);
			}

			double bottom = 0.0;
			MPoint top(0, 0, 0);

			for (unsigned int triIdx = 0; triIdx < areas.size(); triIdx++)
			{
				double similarity = ComputeSimilarlityForCoR(weight_i, averageWeights[triIdx], 0.1);
				bottom += similarity * areas[triIdx];
				top += similarity * centers[triIdx] * areas[triIdx];
			}

			MPoint pstar = std::abs(bottom) < 0.000001 ? MPoint(0, 0, 0) : top / bottom;
			pointArray.set(pstar, itVertex.index());

			pstarArrayHandle.jumpToArrayElement(itVertex.index());
			pstarArrayHandle.outputValue().setMVector(pstar);
		}

		//MObject tmp = pstarArrayHandle.inputArrayValue().
		//CHECK_MSTATUS(returnStatus);
		//MFnPointArrayData pointFn(tmp);
		//CHECK_MSTATUS(pointFn.set(pointArray));

		// clean the dirty flag of the rotation plug
		CHECK_MSTATUS(data.setClean(plug));
	}

	return MPxSkinCluster::compute(plug, data);
}

MStatus CustomSkinCluster::deform(MDataBlock& block, MItGeometry& iter, const MMatrix& localToWorld, unsigned int multiIdx)
{
	MStatus stat;

	// get the joint transforms
	MArrayDataHandle transformsHandle = block.inputArrayValue(matrix, &stat);
	CHECK_MSTATUS(stat);
	int numTransforms = transformsHandle.elementCount(&stat); // = # of joints
	CHECK_MSTATUS(stat);
	if (numTransforms == 0)
	{
		return MS::kSuccess;
	}

	MArrayDataHandle bindHandle = block.inputArrayValue(bindPreMatrix, &stat);
	CHECK_MSTATUS(stat);

	MArrayDataHandle weightListsHandle = block.inputArrayValue(weightList, &stat);
	CHECK_MSTATUS(stat);
	int numWeightLists = weightListsHandle.elementCount(); // = # of points
	if (numWeightLists == 0)
	{
		// if no weights, nothing to do
		return MS::kSuccess;
	}

	const MMatrix worldToLocal = localToWorld.inverse();

	// Iterate through each point in the geometry
	for (iter.reset(); !iter.isDone(); iter.next())
	{
		MPoint pt = iter.position();

		// get the weights for this point
		MArrayDataHandle weightsHandle = weightListsHandle.inputValue(&stat).child(weights);
		CHECK_MSTATUS(stat);

		// compute the skinned position
		MPoint skinned = deformLBS(pt, worldToLocal, transformsHandle, bindHandle, weightsHandle, &stat);
		CHECK_MSTATUS(stat);
		CHECK_MSTATUS(iter.setPosition(skinned));

		// advance the weight list handle
		CHECK_MSTATUS(weightListsHandle.next());
	}

	return stat;
}

void* CustomSkinCluster::creator()
{
	return new CustomSkinCluster();
}

MStatus CustomSkinCluster::initialize()
{
	MStatus stat;

	MFnEnumAttribute eAttr;
	customSkinningMethod = eAttr.create("customSkinningMethod", "cskMethod", 0, &stat);
	CHECK_MSTATUS(stat);
	CHECK_MSTATUS(eAttr.addField("LBS", 0));
	CHECK_MSTATUS(eAttr.addField("DQS", 1));
	CHECK_MSTATUS(addAttribute(customSkinningMethod));

	MFnNumericAttribute nAttr;
	pstarArray_CoR = nAttr.createPoint("pstarArray_CoR", "pstars", &stat);
	CHECK_MSTATUS(stat);
	nAttr.setArray(true);
	//nAttr.setWritable(false);
	addAttribute(pstarArray_CoR);

	attributeAffects(weightList, pstarArray_CoR);
	//attributeAffects(weights, pstarArray_CoR);
	attributeAffects(inputGeom, pstarArray_CoR);
	attributeAffects(customSkinningMethod, pstarArray_CoR);
	attributeAffects(pstarArray_CoR, outputGeom);

	return MStatus::kSuccess;
}

double CustomSkinCluster::ComputeSimilarlityForCoR(const weight_map& w0, const weight_map& w1, double sigma)
{
	double ret = 0.0f;
	for (auto pair0 : w0)
	{
		unsigned int j = pair0.first;

		for (auto pair1 : w1)
		{
			unsigned int k = pair1.first;
			if (w0.find(k) == w0.end() || w1.find(j) == w1.end() || j == k)
			{
				continue;
			}

			double tmp = (pair0.second * pair1.second - w0.at(k) * w1.at(j)) / sigma;
			ret += pair0.second * pair1.second * w0.at(k) * w1.at(j) * std::exp(-tmp * tmp);
		}
	}

	return ret;
}

MPoint CustomSkinCluster::deformLBS(
	const MPoint& pt,
	const MMatrix& worldToLocal,
	MArrayDataHandle& transformsHandle,
	MArrayDataHandle& bindHandle,
	MArrayDataHandle& weightsHandle,
	MStatus* ptrStat) 
{
	MPoint skinned;

	// compute influences from each joint
	unsigned int numWeights = weightsHandle.elementCount(); // # of nonzero weights
	for (unsigned int wIdx = 0; wIdx < numWeights; wIdx++)
	{
		weightsHandle.jumpToArrayElement(wIdx); // jump to physical index
		double w = weightsHandle.inputValue().asDouble();

		// logical index corresponds to the joint index
		unsigned int jointIdx = weightsHandle.elementIndex(ptrStat);

		transformsHandle.jumpToElement(jointIdx); // jump to logical index
		MMatrix jointMat = MFnMatrixData(transformsHandle.inputValue().data()).matrix();

		bindHandle.jumpToElement(jointIdx); // jump to logical index
		MMatrix preBindMatrix = MFnMatrixData(bindHandle.inputValue().data()).matrix();
		jointMat = preBindMatrix * jointMat;

		skinned += (pt * jointMat) * w;
	}

	return skinned * worldToLocal;
}
