#include "ReplaceSkinClusterCmd.h"
#include <maya/MSelectionList.h>
#include <maya/MGlobal.h>
#include <maya/MDagPath.h>
#include <maya/MFnSkinCluster.h>
#include <maya/MFnTransform.h>
#include <maya/MItSelectionList.h>
#include <maya/MPlug.h>
#include <maya/MPxSkinCluster.h>
#include <cassert>

#include <maya/MFnMesh.h>
#include <maya/MItDependencyGraph.h>
#include <maya/MFnArrayAttrsData.h>
#include <maya/MFnAttribute.h>
#include <maya/MMatrix.h>
#include <maya/MFnMatrixData.h>

ReplaceSkinClusterCmd::ReplaceSkinClusterCmd()
{
}

ReplaceSkinClusterCmd::~ReplaceSkinClusterCmd()
{
}

void* ReplaceSkinClusterCmd::creator()
{
	return new ReplaceSkinClusterCmd();
}

MStatus ReplaceSkinClusterCmd::doIt(const MArgList&)
{
	MStatus stat;

	// get selected objects
	MSelectionList selection;
	CHECK_MSTATUS(MGlobal::getActiveSelectionList(selection));
	unsigned int numSelection = selection.length(&stat);
	if (numSelection == 0)
	{
		return stat;
	}

	// find SkinCluster nodes of the selected objects
	MItSelectionList iter(selection);
	for (iter.reset(); !iter.isDone(); iter.next())
	{
		// get DAG path of the mesh node
		MDagPath dagPath;
		CHECK_MSTATUS(iter.getDagPath(dagPath));
		CHECK_MSTATUS(dagPath.extendToShape());

		// get the SkinCluster node
		MObject skinClusterObj = FindSkinClusterNode(dagPath, &stat);
		CHECK_MSTATUS(stat);
		if (skinClusterObj.isNull())
		{
			continue;
		}
		MFnSkinCluster skinClusterFn(skinClusterObj);

		// create the new SkinCluster node
		MObject customSkinClusterObj = dgMod.createNode("CustomSkinCluster", &stat);
		CHECK_MSTATUS(stat);
		MFnSkinCluster customSkinClusterFn(customSkinClusterObj);

		// connect input and output
		CHECK_MSTATUS(ConnectJointNodes(skinClusterFn, customSkinClusterFn));
		{
			MPlug bindPose = skinClusterFn.findPlug("bindPose", true, &stat);
			CHECK_MSTATUS(stat);
			MPlugArray plugs;
			bindPose.connectedTo(plugs, true, false, &stat);
			CHECK_MSTATUS(stat);

			MPlug message = plugs[0];
			MPlug bindPoseNew = customSkinClusterFn.findPlug("bindPose", true, &stat);
			CHECK_MSTATUS(dgMod.connect(message, bindPoseNew));
		}
		{
			MPlug inputPlug = skinClusterFn.findPlug("input", true, &stat).elementByLogicalIndex(0, &stat);
			CHECK_MSTATUS(stat);
			MPlug inputGeomPlug = inputPlug.child(MPxSkinCluster::inputGeom, &stat);
			CHECK_MSTATUS(stat);
			MPlugArray plugs;
			inputGeomPlug.connectedTo(plugs, true, false, &stat);
			CHECK_MSTATUS(stat);

			MPlug worldMesh = plugs[0];
			MPlug inputGeomPlugNew = customSkinClusterFn.findPlug("input", true, &stat).elementByLogicalIndex(0, &stat).child(MPxSkinCluster::inputGeom, &stat);
			CHECK_MSTATUS(dgMod.connect(worldMesh, inputGeomPlugNew));
		}
		{
			MPlug origGeomPlug = skinClusterFn.findPlug("originalGeometry", true, &stat).elementByLogicalIndex(0, &stat);
			CHECK_MSTATUS(stat);
			MPlugArray plugs;
			origGeomPlug.connectedTo(plugs, true, false, &stat);
			CHECK_MSTATUS(stat);

			MPlug outMesh = plugs[0];
			MPlug origGeomPlugNew = customSkinClusterFn.findPlug("originalGeometry", true, &stat).elementByLogicalIndex(0, &stat);
			CHECK_MSTATUS(dgMod.connect(outMesh, origGeomPlugNew));
		}
		{
			MPlug outputGeom = skinClusterFn.findPlug("outputGeometry", true, &stat).elementByLogicalIndex(0, &stat);
			CHECK_MSTATUS(stat);
			MPlugArray plugs;
			outputGeom.connectedTo(plugs, false, true, &stat);
			CHECK_MSTATUS(stat);

			MPlug outputGeomNew = customSkinClusterFn.findPlug("outputGeometry", true, &stat).elementByLogicalIndex(0, &stat);
			MPlug inMesh = plugs[0];
			CHECK_MSTATUS(dgMod.disconnect(outputGeom, inMesh));
			CHECK_MSTATUS(dgMod.connect(outputGeomNew, inMesh));
		}

		// copy attribute data
		{
			// weightList
			MPlug weightListSrc = skinClusterFn.findPlug("weightList", true, &stat);
			CHECK_MSTATUS(stat);
			MPlug weightListDst = customSkinClusterFn.findPlug("weightList", true, &stat);
			CHECK_MSTATUS(stat);
			CHECK_MSTATUS(dgMod.connect(weightListSrc, weightListDst));
		}
		{
			// bindPreMatrix
			MPlug bindPreMatrixSrc = skinClusterFn.findPlug("bindPreMatrix", true, &stat);
			unsigned int numMats = bindPreMatrixSrc.numElements(&stat);
			MPlug bindPreMatrixDst = customSkinClusterFn.findPlug("bindPreMatrix", true, &stat);
			CHECK_MSTATUS(bindPreMatrixDst.setNumElements(numMats));
			for (unsigned int mIdx = 0; mIdx < numMats; mIdx++)
			{
				MPlug bpmSrc = bindPreMatrixSrc.elementByLogicalIndex(mIdx, &stat);
				MObject mtxObj;
				bpmSrc.getValue(mtxObj);
				MFnMatrixData matFn(mtxObj);
				MMatrix mtx = matFn.matrix();

				MPlug bpmDst = bindPreMatrixDst.elementByLogicalIndex(mIdx, &stat);
				bpmDst.setMObject(mtxObj);
			}
		}

		// finally, delete the old SkinCluster node
		dgMod.doIt();
		MGlobal::deleteNode(skinClusterObj);
		//CHECK_MSTATUS(dgMod.deleteNode(skinClusterObj));
	}

	return dgMod.doIt();
}

MObject ReplaceSkinClusterCmd::FindSkinClusterNode(const MDagPath& meshPath, MStatus* ptrStat)
{
	MFnDagNode dagNode(meshPath);

	// mesh node must have inMesh attribute
	MPlug inMeshPlug = dagNode.findPlug("inMesh", ptrStat);
	if (*ptrStat != MS::kSuccess || !inMeshPlug.isConnected())
	{
		*ptrStat = MS::kNotFound;
		return MObject::kNullObj;
	}

	// search skin cluster node, upwards from the mesh node
	MItDependencyGraph dgIt(
		inMeshPlug,
		MFn::kInvalid,
		MItDependencyGraph::kUpstream,
		MItDependencyGraph::kDepthFirst,
		MItDependencyGraph::kPlugLevel,
		ptrStat);

	if (*ptrStat != MS::kSuccess)
	{
		return MObject::kNullObj;
	}

	for (dgIt.disablePruningOnFilter(); !dgIt.isDone(); dgIt.next())
	{
		MObject thisNode = dgIt.thisNode();
		if (thisNode.apiType() != MFn::kSkinClusterFilter)
		{
			continue;
		}

		return thisNode;
	}

	*ptrStat = MS::kNotFound;
	return MObject::kNullObj;
}

MStatus ReplaceSkinClusterCmd::ConnectJointNodes(const MFnSkinCluster& src, const MFnSkinCluster& dst)
{
	MStatus returnStatus;

	// get all the joints from the matrix attribute
	MPlug matrixSrc = src.findPlug("matrix", true, &returnStatus);
	CHECK_MSTATUS(returnStatus);

	// allocate the array buffers for the new SkinCluster
	unsigned int numJoints = matrixSrc.numElements();
	MPlug matrixDst = dst.findPlug("matrix", true, &returnStatus);
	CHECK_MSTATUS(returnStatus);
	CHECK_MSTATUS(matrixDst.setNumElements(numJoints));
	MPlug lockWeightsDst = dst.findPlug("lockWeights", true, &returnStatus);
	CHECK_MSTATUS(returnStatus);
	CHECK_MSTATUS(lockWeightsDst.setNumElements(numJoints));
	MPlug influenceColorDst = dst.findPlug("influenceColor", true, &returnStatus);
	CHECK_MSTATUS(returnStatus);
	CHECK_MSTATUS(influenceColorDst.setNumElements(numJoints));

	for (unsigned int matIdx = 0; matIdx < numJoints; matIdx++)
	{
		MPlug matrixElemPlug = matrixSrc.elementByLogicalIndex(matIdx, &returnStatus);
		CHECK_MSTATUS(returnStatus);

		// if matrix[idx] plug is connected as destination, the source plug should be a joint
		if (!matrixElemPlug.isDestination(&returnStatus))
		{
			continue;
		}
		CHECK_MSTATUS(returnStatus);

		// get the joint node
		MPlugArray jointPlugs;
		matrixElemPlug.connectedTo(jointPlugs, true, false, &returnStatus);
		CHECK_MSTATUS(returnStatus);
		assert(jointPlugs.length() == 1);
		MObject jointNode = jointPlugs[0].node(&returnStatus);
		CHECK_MSTATUS(returnStatus);
		MFnDependencyNode jointFn(jointNode);

		// connect
		// - joint.worldMatrix[0] -> newSkinCluster.matrix[matIdx]
		// - joint.liw -> newSkinCluster.lockWeights[matIdx]
		// - joint.objectColorRGB -> newSkinCluster.influenceColor[matIdx]
		{
			MPlug worldMatrix = jointPlugs[0];
			CHECK_MSTATUS(dgMod.connect(worldMatrix, matrixDst.elementByLogicalIndex(matIdx, &returnStatus)));
		}
		{
			MPlug liw = jointFn.findPlug("liw", true, &returnStatus);
			CHECK_MSTATUS(returnStatus);
			CHECK_MSTATUS(dgMod.connect(liw, lockWeightsDst.elementByLogicalIndex(matIdx, &returnStatus)));
		}
		{
			MPlug objColor = jointFn.findPlug("objectColorRGB", true, &returnStatus);
			CHECK_MSTATUS(returnStatus);
			CHECK_MSTATUS(dgMod.connect(objColor, influenceColorDst.elementByLogicalIndex(matIdx, &returnStatus)));
		}
	}

	return returnStatus;
}

MStatus ReplaceSkinClusterCmd::undoIt()
{
	return dgMod.doIt();
}

MStatus ReplaceSkinClusterCmd::redoIt()
{
	return dgMod.undoIt();
}
