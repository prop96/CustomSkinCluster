#include "ReplaceSkinClusterCmd.h"
#include "CustomSkinCluster.h"
#include <maya/MSelectionList.h>
#include <maya/MGlobal.h>
#include <maya/MDagPath.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnSkinCluster.h>
#include <maya/MPxSkinCluster.h>
#include <maya/MItSelectionList.h>
#include <maya/MItDependencyGraph.h>
#include <maya/MPlug.h>
#include <cassert>


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
	return redoIt();
}

MStatus ReplaceSkinClusterCmd::undoIt()
{
	return ReplaceSkinCluster();
}

MStatus ReplaceSkinClusterCmd::redoIt()
{
	return ReplaceSkinCluster();
}

MStatus ReplaceSkinClusterCmd::ReplaceSkinCluster()
{
	MStatus returnStat;

	// get selected objects
	MSelectionList selection;
	CHECK_MSTATUS(MGlobal::getActiveSelectionList(selection));
	unsigned int numSelection = selection.length(&returnStat);
	if (numSelection == 0)
	{
		return returnStat;
	}

	// find SkinCluster nodes of the selected objects
	MItSelectionList iter(selection);
	for (iter.reset(); !iter.isDone(); iter.next())
	{
		// get DAG path of the mesh node
		MDagPath dagPath;
		CHECK_MSTATUS(iter.getDagPath(dagPath));
		CHECK_MSTATUS(dagPath.extendToShape());

		// get the current SkinCluster node
		MObject srcSkclObj = FindSkinClusterNode(dagPath, &returnStat);
		CHECK_MSTATUS(returnStat);
		if (srcSkclObj.isNull())
		{
			continue;
		}
		MFnSkinCluster srcSkclFn(srcSkclObj);

		// create the new SkinCluster node
		MString newSkclType = srcSkclObj.apiType() == MFn::kPluginSkinCluster ? "skinCluster" : CustomSkinCluster::nodeTypeName;
		MObject dstSkclObj = dgMod.createNode(newSkclType, &returnStat);
		CHECK_MSTATUS(returnStat);
		MFnSkinCluster dstSkclFn(dstSkclObj);

		// connect input and output
		CHECK_MSTATUS(ReplaceConnectionToJoints(srcSkclFn, dstSkclFn));
		CHECK_MSTATUS(ReplaceConnection("bindPose", srcSkclFn, dstSkclFn, true));
		CHECK_MSTATUS(ReplaceConnection("input[0].inputGeometry", srcSkclFn, dstSkclFn, true));
		CHECK_MSTATUS(ReplaceConnection("originalGeometry[0]", srcSkclFn, dstSkclFn, true));
		CHECK_MSTATUS(ReplaceConnection("outputGeometry[0]", srcSkclFn, dstSkclFn, false));

		// copy attribute data by connecting them
		CHECK_MSTATUS(ConnectSameAttribute("weightList", srcSkclFn, dstSkclFn));
		CHECK_MSTATUS(ConnectSameAttribute("bindPreMatrix", srcSkclFn, dstSkclFn));
		CHECK_MSTATUS(ConnectSameAttribute("maxInfluences", srcSkclFn, dstSkclFn));
		CHECK_MSTATUS(ConnectSameAttribute("maintainMaxInfluences", srcSkclFn, dstSkclFn));

		// finally, delete the old SkinCluster node
		CHECK_MSTATUS(dgMod.doIt());
		CHECK_MSTATUS(MGlobal::deleteNode(srcSkclObj));
	}

	return dgMod.doIt();
}

MObject ReplaceSkinClusterCmd::FindSkinClusterNode(const MDagPath& meshPath, MStatus* ptrStat)
{
	MFnDagNode dagNode(meshPath);

	// mesh node must have inMesh attribute
	MPlug inMeshPlug = dagNode.findPlug("inMesh", ptrStat);
	CHECK_MSTATUS(*ptrStat);
	if (!inMeshPlug.isConnected(ptrStat))
	{
		return MObject::kNullObj;
	}
	CHECK_MSTATUS(*ptrStat);

	// search skin cluster node, upwards from the mesh node
	MItDependencyGraph dgIt(
		inMeshPlug,
		MFn::kInvalid,
		MItDependencyGraph::kUpstream,
		MItDependencyGraph::kDepthFirst,
		MItDependencyGraph::kPlugLevel,
		ptrStat);
	CHECK_MSTATUS(*ptrStat);

	for (dgIt.disablePruningOnFilter(); !dgIt.isDone(); dgIt.next())
	{
		// return if SkinCluster or CustomSkinCluster node
		MObject thisNode = dgIt.thisNode();
		if (thisNode.apiType() == MFn::kSkinClusterFilter || thisNode.apiType() == MFn::kPluginSkinCluster)
		{
			return thisNode;
		}
	}

	return MObject::kNullObj;
}

MStatus ReplaceSkinClusterCmd::ReplaceConnectionToJoints(const MFnSkinCluster& src, const MFnSkinCluster& dst)
{
	MStatus returnStatus;

	// get all the joints from the matrix attribute
	MPlug matrixSrc = src.findPlug("matrix", &returnStatus);
	CHECK_MSTATUS(returnStatus);
	unsigned int numJoints = matrixSrc.numElements(&returnStatus);
	CHECK_MSTATUS(returnStatus);

	for (unsigned int matIdx = 0; matIdx < numJoints; matIdx++)
	{
		MPlug matrixElemPlug = matrixSrc.elementByLogicalIndex(matIdx, &returnStatus);
		CHECK_MSTATUS(returnStatus);

		// if matrix[idx] plug is connected as destination, the source plug should be a joint
		if (!matrixElemPlug.isDestination(&returnStatus))
		{
			CHECK_MSTATUS(returnStatus);
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
			MPlug matrixDst = dst.findPlug("matrix", &returnStatus);
			CHECK_MSTATUS(returnStatus);
			CHECK_MSTATUS(dgMod.connect(worldMatrix, matrixDst.elementByLogicalIndex(matIdx, &returnStatus)));
		}
		{
			MPlug liw = jointFn.findPlug("liw", &returnStatus);
			CHECK_MSTATUS(returnStatus);
			MPlug lockWeightsDst = dst.findPlug("lockWeights", &returnStatus);
			CHECK_MSTATUS(returnStatus);
			CHECK_MSTATUS(dgMod.connect(liw, lockWeightsDst.elementByLogicalIndex(matIdx, &returnStatus)));
		}
		{
			MPlug objColor = jointFn.findPlug("objectColorRGB", &returnStatus);
			CHECK_MSTATUS(returnStatus);
			MPlug influenceColorDst = dst.findPlug("influenceColor", &returnStatus);
			CHECK_MSTATUS(returnStatus);
			CHECK_MSTATUS(dgMod.connect(objColor, influenceColorDst.elementByLogicalIndex(matIdx, &returnStatus)));
		}
	}

	return returnStatus;
}

MStatus ReplaceSkinClusterCmd::ReplaceConnection(const MString& attrName, const MFnSkinCluster& src, const MFnSkinCluster& dst, bool asDst)
{
	MStatus returnStatus;

	// split the attribute name by '.'
	MStringArray attrPath;
	CHECK_MSTATUS(attrName.split('.', attrPath));

	// parse the attribute path and obtain the instance of the plug
	MPlug srcPlug, dstPlug;
	for (auto it = attrPath.begin(); it != attrPath.end(); ++it)
	{
		// query if the string includes any parenthesis
		MStringArray strArray;
		CHECK_MSTATUS((*it).split('[', strArray));

		// don't assume the dual parenthesis such as "hoge[0][0]"
		assert(strArray.length() == 1 || strArray.length() == 2);

		// anyway, the first string represents the attribute name
		MObject obj = src.attribute(strArray[0], &returnStatus);
		CHECK_MSTATUS(returnStatus);

		// get the plug instance from MObject
		if (it == attrPath.begin())
		{
			srcPlug = src.findPlug(obj, &returnStatus);
			CHECK_MSTATUS(returnStatus);
			dstPlug = dst.findPlug(obj, &returnStatus);
			CHECK_MSTATUS(returnStatus);
		}
		else
		{
			srcPlug = srcPlug.child(obj, &returnStatus);
			CHECK_MSTATUS(returnStatus);
			dstPlug = dstPlug.child(obj, &returnStatus);
			CHECK_MSTATUS(returnStatus);
		}

		// if the string includes a parenthesis, get the corresponding element
		if (strArray.length() == 2)
		{
			CHECK_MSTATUS(strArray[1].split(']', strArray));
			unsigned int val = strArray[0].asUnsigned();
			srcPlug = srcPlug.elementByLogicalIndex(val, &returnStatus);
			CHECK_MSTATUS(returnStatus);
			dstPlug = dstPlug.elementByLogicalIndex(val, &returnStatus);
			CHECK_MSTATUS(returnStatus);
		}
	}

	// search plugs connecting to the given attribute in the src SkinCluster
	MPlugArray connectedPlugs;
	srcPlug.connectedTo(connectedPlugs, asDst, !asDst, &returnStatus);
	CHECK_MSTATUS(returnStatus);

	// connect the given attribute in the dst SkinCluster with the found plug
	for (const auto& plug : connectedPlugs)
	{
		if (asDst)
		{
			CHECK_MSTATUS(dgMod.disconnect(plug, srcPlug));
			CHECK_MSTATUS(dgMod.connect(plug, dstPlug));
		}
		else
		{
			CHECK_MSTATUS(dgMod.disconnect(srcPlug, plug));
			CHECK_MSTATUS(dgMod.connect(dstPlug, plug));
		}
	}

	return returnStatus;
}

MStatus ReplaceSkinClusterCmd::ConnectSameAttribute(const MString& attrName, const MFnSkinCluster& src, const MFnSkinCluster& dst)
{
	MStatus returnStatus;

	MPlug srcPlug = src.findPlug(attrName, &returnStatus);
	CHECK_MSTATUS(returnStatus);
	MPlug dstPlug = dst.findPlug(attrName, &returnStatus);
	CHECK_MSTATUS(returnStatus);
	CHECK_MSTATUS(dgMod.connect(srcPlug, dstPlug));

	return returnStatus;
}
