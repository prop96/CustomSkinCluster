#include "ReplaceSkinClusterCmd.h"
#include <maya/MSelectionList.h>
#include <maya/MGlobal.h>
#include <maya/MDagPath.h>
#include <maya/MFnSkinCluster.h>
#include <maya/MFnTransform.h>
#include <maya/MItSelectionList.h>
#include <maya/MPlug.h>

#include <maya/MFnMesh.h>
#include <maya/MItDependencyGraph.h>
#include <maya/MFnArrayAttrsData.h>

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

	// find all the Transform nodes of the selected objects
	MDagPath dagPath;
	MFnTransform transformFn;
	MFnSkinCluster skinClusterFn;
	MString name;
	//MItSelectionList iter(selection, MFn::kTransform);
	MItSelectionList iter(selection);
	for (iter.reset(); !iter.isDone(); iter.next())
	{
		iter.getDagPath(dagPath);
		//CHECK_MSTATUS(iter.getDagPath(dagPath));
		dagPath.extendToShape();
		auto apitype = dagPath.apiType();
		auto classname = dagPath.className();
		auto cc = dagPath.childCount();
		std::cout << apitype << classname << cc << endl;

		MObject skinClusterObj = FindSkinClusterNode(dagPath, &stat);
		if (skinClusterObj.isNull() || stat != MS::kSuccess)
		{
			continue;
		}

		MFnSkinCluster skinCluster(skinClusterObj);

		MPlug inputPlug = skinCluster.findPlug("weightList", true, &stat);
		if (stat == MS::kSuccess)
		{
			MObject obj;
			inputPlug.getValue(obj);

			MFnArrayAttrsData arrayData(obj);

			MFnArrayAttrsData::Type doubleType(MFnArrayAttrsData::kDoubleArray);
			bool flag = arrayData.checkArrayExist("weights", doubleType);
			std::cout << flag << std::endl;

			//MPlug childPlug = inputPlug.elementByLogicalIndex(0);
			//MPlug geomPlug = childPlug.child(0);

			//MPlug tmp = childPlug.elementByLogicalIndex(0);
			//double val;
			//tmp.getValue(val);
			//std::cout << val << std::endl;
		}


		//CHECK_MSTATUS(transformFn.setObject(dagPath));
		//skinClusterFn.setObject(dagPath);
		//CHECK_MSTATUS(skinClusterFn.setObject(dagPath));


		// generate RollingNode and connect to each Transform nodes
		//MObject rollNodeObj = dgMod.createNode("RollingNode", &stat);
		//CHECK_MSTATUS(stat);
		//MFnDependencyNode depNodeFn(rollNodeObj);

		//{
		//	MPlug translateX = transformFn.findPlug("translateX", true, &stat);
		//	CHECK_MSTATUS(stat);
		//	MPlug distance = depNodeFn.findPlug("distance", true, &stat);
		//	CHECK_MSTATUS(stat);
		//	CHECK_MSTATUS(dgMod.connect(translateX, distance));
		//}
		//{
		//	MPlug translateY = transformFn.findPlug("translateY", true, &stat);
		//	CHECK_MSTATUS(stat);
		//	MPlug radius = depNodeFn.findPlug("radius", true, &stat);
		//	CHECK_MSTATUS(stat);
		//	CHECK_MSTATUS(dgMod.connect(translateY, radius));
		//}
		//{
		//	MPlug rotation = depNodeFn.findPlug("rotation", true, &stat);
		//	CHECK_MSTATUS(stat);
		//	MPlug rotateZ = transformFn.findPlug("rotateZ", true, &stat);
		//	CHECK_MSTATUS(stat);
		//	CHECK_MSTATUS(dgMod.connect(rotation, rotateZ));
		//}
	}

	return redoIt();
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

MStatus ReplaceSkinClusterCmd::undoIt()
{
	return dgMod.doIt();
}

MStatus ReplaceSkinClusterCmd::redoIt()
{
	return dgMod.undoIt();
}
