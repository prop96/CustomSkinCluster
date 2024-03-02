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
	MStatus returnStat;
	MFnPlugin plugin(obj, "prop96", "1.0", "Any");

	returnStat = plugin.registerCommand(ReplaceSkinClusterCmd::commandName, ReplaceSkinClusterCmd::creator);
	if (!returnStat)
	{
		returnStat.perror("registerCommand failed");
		return returnStat;
	}

	returnStat = plugin.registerNode(CustomSkinCluster::nodeTypeName, CustomSkinCluster::id, CustomSkinCluster::creator, CustomSkinCluster::initialize, MPxNode::kSkinCluster);
	if (!returnStat)
	{
		returnStat.perror("register customSkinCluster Node failed");
		return returnStat;
	}

	return returnStat;
}

// The unitializePlugin is called when Maya needs to unload the plugin.
// It basically does the opposite of initialize by calling
// the deregisterCommand and deregisterNode to remove it.
//
MStatus uninitializePlugin(MObject obj)
{
	MStatus returnStat;
	MString errStr;
	MFnPlugin plugin(obj);

	returnStat = plugin.deregisterCommand(ReplaceSkinClusterCmd::commandName);
	if (!returnStat)
	{
		returnStat.perror("deregisterCommand failed");
		return returnStat;
	}

	returnStat = plugin.deregisterNode(CustomSkinCluster::id);
	if (!returnStat)
	{
		returnStat.perror("deregisterNode failed");
		return returnStat;
	}

	return returnStat;
}