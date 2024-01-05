#include "CustomSkinCluster.h"
#include <maya/MMatrixArray.h>
#include <maya/MFnMatrixData.h>
#include <maya/MPoint.h>


const MTypeId LBSCluster::id(0x00080031);

MStatus LBSCluster::deform(MDataBlock& block, MItGeometry& iter, const MMatrix& localToWorld, unsigned int multiIdx)
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
		int numWeights = weightsHandle.elementCount();
		int preJointIdx = -1;
		for (int idx = 0; idx < numWeights; idx++)
		{
			weightsHandle.jumpToElement(idx);
			double w = weightsHandle.inputValue().asDouble();

			// logical index represent the actual joint index
			int jointIdx = weightsHandle.elementIndex();
			if (jointIdx == preJointIdx)
			{
				numWeights++;
				preJointIdx = jointIdx;
				continue;
			}
			preJointIdx = jointIdx;
			
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

void* LBSCluster::creator()
{
	return new LBSCluster();
}

MStatus LBSCluster::initialize()
{
	return MStatus::kSuccess;
}
