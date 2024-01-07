#include "CustomSkinCluster.h"
#include <maya/MItMeshVertex.h>
#include <maya/MMatrixArray.h>
#include <maya/MFnMatrixData.h>
#include <maya/MPoint.h>


const MTypeId CustomSkinCluster::id(0x00080031);

MStatus CustomSkinCluster::compute(const MPlug& plug, MDataBlock& data)
{
	if (plug == weightList || plug == weights)
	{
		MItMeshVertex itVertex(input);
		for (; !itVertex.isDone(); itVertex.next())
		{
			MPoint vertexPosition = itVertex.position();
			cout << vertexPosition << endl;
		}
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
		MPoint skinned;

		// get the weights for this point
		MArrayDataHandle weightsHandle = weightListsHandle.inputValue().child(weights);
		
		// compute influences from each joint
		unsigned int numWeights = weightsHandle.elementCount(); // # of nonzero weights
		for (unsigned int wIdx = 0; wIdx < numWeights; wIdx++)
		{
			weightsHandle.jumpToArrayElement(wIdx); // jump to physical index
			double w = weightsHandle.inputValue().asDouble();

			// logical index = joint index
			unsigned int jointIdx = weightsHandle.elementIndex(&stat);

			transformsHandle.jumpToElement(jointIdx); // jump to logical index
			MMatrix jointMat = MFnMatrixData(transformsHandle.inputValue().data()).matrix();

			bindHandle.jumpToElement(jointIdx); // jump to logical index
			MMatrix preBindMatrix = MFnMatrixData(bindHandle.inputValue().data()).matrix();
			jointMat = preBindMatrix * jointMat;

			skinned += (pt * jointMat) * w;
		}

		// set the final position
		iter.setPosition(skinned * worldToLocal);

		// advance the weight list handle
		weightListsHandle.next();
	}

	return stat;
}

void* CustomSkinCluster::creator()
{
	return new CustomSkinCluster();
}

MStatus CustomSkinCluster::initialize()
{
	return MStatus::kSuccess;
}
