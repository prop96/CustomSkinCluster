#include "CustomSkinCluster.h"
#include <maya/MItMeshVertex.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MPoint.h>
#include <vector>


namespace
{
	enum class SkinningType : short
	{
		LBS = 0,
		DMLBS,
		DDM,
	};
}


const MTypeId CustomSkinCluster::id(0x00080031);
MObject CustomSkinCluster::customSkinningMethod;
MObject CustomSkinCluster::doRecompute;
MObject CustomSkinCluster::needRebindMesh;
MObject CustomSkinCluster::smoothAmount;
MObject CustomSkinCluster::smoothIteration;

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

	const auto skinningMethod = static_cast<const SkinningType>(block.inputValue(customSkinningMethod).asShort());

	// recompute if necessary
	const bool& doRecomputeVal = block.inputValue(doRecompute).asBool();
	if (doRecomputeVal)
	{
		if (skinningMethod == SkinningType::DDM)
		{
			double smoothAmountVal = block.inputValue(smoothAmount).asDouble();
			int smoothItrVal = block.inputValue(smoothIteration).asInt();
			m_ddmDeformer.SetSmoothingProperty({ smoothAmountVal, smoothItrVal, false });

			MFnDependencyNode thisNode(thisMObject());
			MObject origGeom = thisNode.attribute("originalGeometry", &returnStat);
			MObject originalGeomVal = block.inputArrayValue(origGeom, &returnStat).inputValue().asMesh();
			bool& needRebindMeshVal = block.inputValue(needRebindMesh).asBool();

			m_ddmDeformer.Precompute(originalGeomVal, weightListsHandle, needRebindMeshVal);
			needRebindMeshVal = false;
		}
		else if (skinningMethod == SkinningType::DMLBS)
		{

		}
	}


	const MMatrix worldToLocal = localToWorld.inverse();

	// Iterate through each point in the geometry
	for (iter.reset(), weightListsHandle.jumpToArrayElement(0); !iter.isDone(); iter.next())
	{
		MPoint pt = iter.position();

		// get the weights for this point
		MArrayDataHandle weightsHandle = weightListsHandle.inputValue(&returnStat).child(weights);
		CHECK_MSTATUS(returnStat);

		// compute the skinned position
		MPoint skinned;
		switch (skinningMethod)
		{
		case SkinningType::LBS:
		case SkinningType::DMLBS:
			skinned = m_lbsDeformer.Deform(iter.index(), pt, worldToLocal, transformsHandle, bindHandle, weightsHandle, &returnStat);
			break;
		case SkinningType::DDM:
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

	if (skinningMethod == SkinningType::DMLBS)
	{
		// Apply Delta Mush here
		
	}

	return returnStat;
}

MStatus CustomSkinCluster::initialize()
{
	MStatus returnStat;

	MFnEnumAttribute eAttr;
	MFnNumericAttribute nAttr;

	customSkinningMethod = eAttr.create("customSkinningMethod", "cskMethod", 0, &returnStat);
	CHECK_MSTATUS(returnStat);
	CHECK_MSTATUS(eAttr.addField("LBS", static_cast<short>(SkinningType::LBS)));
	CHECK_MSTATUS(eAttr.addField("DM+LBS", static_cast<short>(SkinningType::DMLBS)));
	CHECK_MSTATUS(eAttr.addField("DDM", static_cast<short>(SkinningType::DDM)));
	CHECK_MSTATUS(addAttribute(customSkinningMethod));

	doRecompute = nAttr.create("doRecompute", "doRecompute", MFnNumericData::kBoolean, 1, &returnStat);
	CHECK_MSTATUS(returnStat);
	CHECK_MSTATUS(addAttribute(doRecompute));

	needRebindMesh = nAttr.create("needRebindMesh", "rebindMesh", MFnNumericData::kBoolean, 1, &returnStat);
	CHECK_MSTATUS(returnStat);
	CHECK_MSTATUS(addAttribute(needRebindMesh));

	smoothAmount = nAttr.create("smoothAmount", "smoothAmount", MFnNumericData::kDouble, 0.0, &returnStat);
	CHECK_MSTATUS(returnStat);
	CHECK_MSTATUS(nAttr.setMin(0.0));
	CHECK_MSTATUS(nAttr.setMax(1.0));
	CHECK_MSTATUS(addAttribute(smoothAmount));

	smoothIteration = nAttr.create("smoothItr", "smoothItr", MFnNumericData::kInt, 0, &returnStat);
	CHECK_MSTATUS(returnStat);
	CHECK_MSTATUS(nAttr.setMin(0));
	CHECK_MSTATUS(addAttribute(smoothIteration));

	CHECK_MSTATUS(attributeAffects(customSkinningMethod, outputGeom));
	CHECK_MSTATUS(attributeAffects(doRecompute, outputGeom));
	CHECK_MSTATUS(attributeAffects(needRebindMesh, outputGeom));
	CHECK_MSTATUS(attributeAffects(smoothAmount, outputGeom));
	CHECK_MSTATUS(attributeAffects(smoothIteration, outputGeom));

	return MStatus::kSuccess;
}
