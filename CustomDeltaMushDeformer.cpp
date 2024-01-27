#include "CustomDeltaMushDeformer.h"
#include "DeltaMushUtil.h"
#include <maya/MFnUnitAttribute.h>
#include <maya/MDistance.h>
#include <maya/MPoint.h>
#include <maya/MItMeshVertex.h>
#include <maya/MPointArray.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnMatrixArrayData.h>
#include <cassert>

/*
* select object and type
*   deformer -type SwirlDeformer
*/

// instancing the static fields
MTypeId CustomDeltaMushDeformer::id(0x80095);
MObject CustomDeltaMushDeformer::deltaMushMatrix;

MStatus CustomDeltaMushDeformer::compute(const MPlug& plug, MDataBlock& data)
{
	MString info = plug.info();
	cout << info << endl;

	return MPxDeformerNode::compute(plug, data);
}

MStatus CustomDeltaMushDeformer::deform(MDataBlock& data, MItGeometry& iter, const MMatrix& localToWorld, unsigned int multiIdx)
{
	MStatus returnStat;

	// get the attribute instance of input[0].inputGeometry
	MObject skinnedMesh = data.inputArrayValue(input).inputValue().child(inputGeom).data();

	// get all the positions of the vertices
	MFnMesh meshFn(skinnedMesh);
	MPointArray skinnedPoints;
	meshFn.getPoints(skinnedPoints);

	// get the deltaMushMatrix plug
	MArrayDataHandle dmMatsObj = data.inputArrayValue(deltaMushMatrix);;
	//MFnMatrixArrayData matArrayFn(dmMatsObj);
	//MMatrixArray dmMats = matArrayFn.array();
	int num = dmMatsObj.elementCount();
	cout << num << endl;

	MItMeshVertex itVertex(skinnedMesh);
	for (itVertex.reset(); !itVertex.isDone(); itVertex.next())
	{
		auto pos = itVertex.position();

		//MMatrix inv = dmMats[itVertex.index()].inverse();
		//MMatrix inv = dmMatsObj.inputValue().asMatrix();
		dmMatsObj.next();
		MMatrix inv; inv.setToIdentity();

		MMatrix mat;
		DMUtil::CreateDeltaMushMatrix(mat, itVertex, meshFn, skinnedPoints);

		pos = pos * inv * mat;
		iter.setPosition(pos);
	}

	return returnStat;
}

void* CustomDeltaMushDeformer::creator()
{
	return new CustomDeltaMushDeformer();
}

MStatus CustomDeltaMushDeformer::initialize()
{
	MStatus returnStat;

	MFnTypedAttribute tAttr;

	deltaMushMatrix = tAttr.create("deltaMushMatrix", "dmMat", MFnData::kMatrixArray, MObject::kNullObj, &returnStat);
	CHECK_MSTATUS(returnStat);
	CHECK_MSTATUS(addAttribute(deltaMushMatrix));

	CHECK_MSTATUS(attributeAffects(deltaMushMatrix, outputGeom));

	return returnStat;
}
