#include "ReplaceSkinClusterCmd.h"
#include "CustomSkinCluster.h"
#include <maya/MFnPlugin.h>


// The initializePlugin method is called by Maya when the custom-node
// plugin is loaded.  It registers the custome node which provides
// Maya with the creator and initialize methods to be called when
// a custom node is created.
//
MStatus initializePlugin(MObject obj)
{
	MStatus stat;
	MFnPlugin plugin(obj, "prop96", "1.0", "Any");

	stat = plugin.registerCommand("ReplaceSkinCluster", ReplaceSkinClusterCmd::creator);
	if (!stat)
	{
		stat.perror("registerCommand failed");
		return stat;
	}

	stat = plugin.registerNode("LBSCluster", LBSCluster::id, LBSCluster::creator, LBSCluster::initialize, MPxNode::kSkinCluster);
	if (!stat)
	{
		stat.perror("registerSkinClusterNode failed");
		return stat;
	}

	return stat;
}

// The unitializePlugin is called when Maya needs to unload the plugin.
// It basically does the opposite of initialize by calling
// the deregisterCommand and deregisterNode to remove it.
//
MStatus uninitializePlugin(MObject obj)
{
	MStatus stat;
	MString errStr;
	MFnPlugin plugin(obj);

	stat = plugin.deregisterCommand("ReplaceSkinCluster");
	if (!stat)
	{
		stat.perror("deregisterCommand failed");
		return stat;
	}

	stat = plugin.deregisterNode(LBSCluster::id);
	if (!stat)
	{
		stat.perror("deregisterNode failed");
		return stat;
	}

	return stat;
}