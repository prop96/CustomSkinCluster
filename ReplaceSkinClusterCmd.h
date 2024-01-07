#pragma once

#include <maya/MIOStream.h>
#include <maya/MPxCommand.h>
#include <maya/MDGModifier.h>

// TODO: CustonSkinCluster を SkinCluster に戻すコマンドも用意するべき
class ReplaceSkinClusterCmd : public MPxCommand
{
public:
	ReplaceSkinClusterCmd();
	~ReplaceSkinClusterCmd() override;
	static void* creator();
	virtual bool isUndoable() const { return true; }
	MStatus doIt(const MArgList&) override;
	MStatus undoIt() override;
	MStatus redoIt() override;

private:
	MDGModifier dgMod;

	/// <summary>
	/// return SkinCluster or CustomSkinCluster node connecting to the given mesh
	/// </summary>
	/// <param name="meshPath"></param>
	/// <param name="ptrStat"></param>
	/// <returns></returns>
	MObject FindSkinClusterNode(const MDagPath& meshPath, MStatus* ptrStat);

	MStatus ConnectJointNodes(const MFnSkinCluster& src, const MFnSkinCluster& dst);

	/// <summary>
	/// Disconnect the given plug in the src SkinCluster, and reconnect to that in the dst SkinCluster
	/// </summary>
	/// <param name="attrName"></param>
	/// <param name="src"></param>
	/// <param name="dst"></param>
	/// <param name="asDst">if true, find the plug connecting from the other plug to the given attribute in src SkinCluster</param>
	/// <returns></returns>
	MStatus ReplaceConnection(const MString& attrName, const MFnSkinCluster& src, const MFnSkinCluster& dst, bool asDst);

	/// <summary>
	/// Connect the same attributes in src and dst SkinClusters to force copy the attribute data
	/// </summary>
	/// <param name="attrName"></param>
	/// <param name="src"></param>
	/// <param name="dst"></param>
	/// <returns></returns>
	MStatus ConnectSameAttribute(const MString& attrName, const MFnSkinCluster& src, const MFnSkinCluster& dst);

protected:
	const MString newSkinCluster = "CustomSkinCluster";
};
