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

	MObject FindSkinClusterNode(const MDagPath& meshPath, MStatus* ptrStat);
};
