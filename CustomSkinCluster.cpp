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


MStatus CustomSkinCluster::compute(const MPlug& plug, MDataBlock& data)
{
	MStatus returnStatus;

	MString info = plug.info();
	cout << info << endl;

	return MPxSkinCluster::compute(plug, data);
}

MStatus CustomSkinCluster::deform(MDataBlock& block, MItGeometry& iter, const MMatrix& localToWorld, unsigned int multiIdx)
{
	MStatus returnStat;

	// get the joint transforms
	MArrayDataHandle transformsHandle = block.inputArrayValue(matrix, &returnStat);
	CHECK_MSTATUS(returnStat);
	int numTransforms = transformsHandle.elementCount(&returnStat); // = # of joints
	CHECK_MSTATUS(returnStat);
	if (numTransforms == 0)
	{
		return MS::kSuccess;
	}

	MArrayDataHandle bindHandle = block.inputArrayValue(bindPreMatrix, &returnStat);
	CHECK_MSTATUS(returnStat);

	MArrayDataHandle weightListsHandle = block.inputArrayValue(weightList, &returnStat);
	CHECK_MSTATUS(returnStat);
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
		MArrayDataHandle weightsHandle = weightListsHandle.inputValue(&returnStat).child(weights);
		CHECK_MSTATUS(returnStat);

		// compute the skinned position
		MPoint skinned = deformLBS(pt, worldToLocal, transformsHandle, bindHandle, weightsHandle, &returnStat);
		CHECK_MSTATUS(returnStat);
		CHECK_MSTATUS(iter.setPosition(skinned));

		// advance the weight list handle
		CHECK_MSTATUS(weightListsHandle.next());
	}

	return returnStat;
}

void* CustomSkinCluster::creator()
{
	return new CustomSkinCluster();
}

MStatus CustomSkinCluster::initialize()
{
	MStatus returnStat;

	MFnEnumAttribute eAttr;
	customSkinningMethod = eAttr.create("customSkinningMethod", "cskMethod", 0, &returnStat);
	CHECK_MSTATUS(returnStat);
	CHECK_MSTATUS(eAttr.addField("LBS", 0));
	CHECK_MSTATUS(eAttr.addField("DQS", 1));
	CHECK_MSTATUS(addAttribute(customSkinningMethod));

	CHECK_MSTATUS(attributeAffects(customSkinningMethod, outputGeom));

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
