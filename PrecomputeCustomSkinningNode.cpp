#include "PrecomputeCustomSkinningNode.h"
#include "DeltaMushUtil.h"
#include <maya/MItMeshVertex.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MMatrix.h>
#include <maya/MVector.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnMatrixArrayData.h>
#include <maya/MMatrixArray.h>
#include <maya/MIntArray.h>
#include <maya/MPointArray.h>
#include <maya/MItGeometry.h>
#include <maya/MArrayDataBuilder.h>

#include <maya/MFnMesh.h>

// instancing the static fields
MTypeId PrecomputeCustomSkinningNode::id(0x80089);
MObject PrecomputeCustomSkinningNode::originalGeometry;
MObject PrecomputeCustomSkinningNode::deltaMushMatrix;


// The compute method is called by Maya when the input values change and the output values need to be recomputed
MStatus PrecomputeCustomSkinningNode::compute(const MPlug& plug, MDataBlock& data)
{
	MStatus returnStat;

	MString info = plug.info(&returnStat);
	cout << info << endl;

	// if the deltaMushMatrix attribute is to be updated
	if (plug == deltaMushMatrix)
	{
		MObject mesh = data.inputValue(originalGeometry).data();

		MItMeshVertex itVertex(mesh);

		MFnMesh meshFn(mesh);
		MPointArray points;
		meshFn.getPoints(points);

		MObject matArrayObj = data.outputValue(deltaMushMatrix).data();//plug.asMObject(&returnStat);
		MFnMatrixArrayData matArrayFn;
		CHECK_MSTATUS(matArrayFn.setObject(matArrayObj));

		MMatrixArray matArray = matArrayFn.array();
		int num = points.length();
		matArray.setLength(points.length());

		// compute the "delta from mush" for each vertices
		for (itVertex.reset(); !itVertex.isDone(); itVertex.next())
		{
			DMUtil::CreateDeltaMushMatrix(matArray[itVertex.index()], itVertex, meshFn, points);
		}

		CHECK_MSTATUS(matArrayFn.set(matArray));
		int num2 = matArrayFn.length();
		cout << num2 << endl;

		// clean the dirty flag of the plug
		CHECK_MSTATUS(data.setClean(plug));
	}
	// for other attributes, return kUnknownParameter and the base class will deal with them
	else
	{
		returnStat = MS::kUnknownParameter;
	}

	return returnStat;
}

// The creator method creates an instance of the custom node class
// and is the first method called by Maya when a custom node needs to be created.
void* PrecomputeCustomSkinningNode::creator()
{
	return new PrecomputeCustomSkinningNode();
}

// The initialize routine is called after the node has been created.
// It sets up the input and output attributes and adds them to the node.
// Finally the dependencies are arranged so that when the inputs
// change Maya knowns to call compute to recalculate the output values.
MStatus PrecomputeCustomSkinningNode::initialize()
{
	MStatus returnStat;

	MFnTypedAttribute tAttr;

	originalGeometry = tAttr.create("originalGeometry", "origGeom", MFnData::kMesh, MObject::kNullObj, &returnStat);
	CHECK_MSTATUS(returnStat);
	deltaMushMatrix = tAttr.create("deltaMushMatrix", "dmMat", MFnData::kMatrixArray, MObject::kNullObj, &returnStat);
	CHECK_MSTATUS(returnStat);
	//CHECK_MSTATUS(tAttr.setUsesArrayDataBuilder(true));

	CHECK_MSTATUS(addAttribute(originalGeometry));
	CHECK_MSTATUS(addAttribute(deltaMushMatrix));

	// define the dependency of the attributes
	CHECK_MSTATUS(attributeAffects(originalGeometry, deltaMushMatrix));

	return MS::kSuccess;
}
