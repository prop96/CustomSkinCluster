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
MObject CustomSkinCluster::doRecompute;


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

	const short& skinningMethod = block.inputValue(customSkinningMethod).asShort();

	// recompute if necessary
	const bool& doRecomputeVal = block.inputValue(doRecompute).asBool();
	if (doRecomputeVal && skinningMethod == 1)
	{
		MFnDependencyNode thisNode(thisMObject());
		MObject origGeom = thisNode.attribute("originalGeometry", &returnStat);

		MObject originalGeomV = block.inputArrayValue(origGeom, &returnStat).inputValue().asMesh();

		m_ddmDeformer.SetSmoothingProperty({1.0, 3, false});

		m_ddmDeformer.Precompute(originalGeomV, weightListsHandle);
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
		MPoint skinned;
		switch (skinningMethod)
		{
		case 0:
			skinned = deformLBS(iter.index(), pt, worldToLocal, transformsHandle, bindHandle, weightsHandle, &returnStat);
			break;
		case 1:
			skinned = m_ddmDeformer.Deform(iter.index(), pt, worldToLocal, transformsHandle, bindHandle, weightsHandle, &returnStat);
			break;
		default:
			break;
		}
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
	MFnNumericAttribute nAttr;

	customSkinningMethod = eAttr.create("customSkinningMethod", "cskMethod", 0, &returnStat);
	CHECK_MSTATUS(returnStat);
	CHECK_MSTATUS(eAttr.addField("LBS", 0));
	CHECK_MSTATUS(eAttr.addField("DDM", 1));
	CHECK_MSTATUS(addAttribute(customSkinningMethod));

	doRecompute = nAttr.create("doRecompute", "doRecompute", MFnNumericData::kBoolean, 1, &returnStat);
	CHECK_MSTATUS(returnStat);
	CHECK_MSTATUS(addAttribute(doRecompute));

	CHECK_MSTATUS(attributeAffects(customSkinningMethod, outputGeom));
	CHECK_MSTATUS(attributeAffects(doRecompute, outputGeom));

	return MStatus::kSuccess;
}


MPoint CustomSkinCluster::deformLBS(
	int vertIdx,
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
