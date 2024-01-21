#include "CustomDeltaMushDeformer.h"
#include <maya/MFnUnitAttribute.h>
#include <maya/MDistance.h>
#include <maya/MPoint.h>
#include <maya/MItMeshVertex.h>
#include <maya/MPointArray.h>
#include <maya/MFnTypedAttribute.h>
#include <cassert>

/*
* select object and type
*   deformer -type SwirlDeformer
*/

// instancing the static fields
MTypeId CustomDeltaMushDeformer::id(0x80095);
MObject CustomDeltaMushDeformer::startDist;
MObject CustomDeltaMushDeformer::matricesR;

MStatus CustomDeltaMushDeformer::compute(const MPlug& plug, MDataBlock& data)
{
	MString info = plug.info();
	cout << info << endl;

	return MPxDeformerNode::compute(plug, data);
}

MStatus SmoothMesh(MObject& mesh, const MPointArray& original, MPointArray& smoothed)
{
	const unsigned int numVerts = original.length();
	smoothed.setLength(numVerts);

	MItMeshVertex itVertex(mesh);
	for (itVertex.reset(); !itVertex.isDone(); itVertex.next())
	{
		MPoint pos(0, 0, 0);

		MIntArray connected;
		itVertex.getConnectedVertices(connected);
		const unsigned int numConnected = connected.length();
		assert(numConnected > 0);
		for (const int vidx : connected)
		{
			pos += original[vidx];
		}
		
		smoothed[itVertex.index()] = pos / numConnected;
	}
}

MStatus CustomDeltaMushDeformer::deform(MDataBlock& data, MItGeometry& iter, const MMatrix& localToWorld, unsigned int multiIdx)
{
	MStatus returnStat;

	//MObject originalMesh = data.inputArrayValue()

	// get all the positions of the vertices
	MPointArray skinnedPoints;
	iter.allPositions(skinnedPoints);

	// get the attribute instance of input[0].inputGeometry
	MObject skinnedMesh = data.inputArrayValue(input).inputValue().child(inputGeom).data();

	// store the new positions
	MPointArray skinnedSmoothed;
	SmoothMesh(skinnedMesh, skinnedPoints, skinnedSmoothed);

	iter.setAllPositions(skinnedSmoothed);

	return returnStat;
}

void* CustomDeltaMushDeformer::creator()
{
	return new CustomDeltaMushDeformer();
}

MStatus CustomDeltaMushDeformer::initialize()
{
	MStatus returnStat;

	MFnTypedAttribute tattrFn;

	matricesR = tattrFn.create("matricesR", "Rmat", MFnData::kMatrixArray, &returnStat);
	CHECK_MSTATUS(returnStat);
	CHECK_MSTATUS(addAttribute(matricesR));

	CHECK_MSTATUS(attributeAffects(inputGeom, matricesR));
	CHECK_MSTATUS(attributeAffects(matricesR, outputGeom));

	return returnStat;
}
