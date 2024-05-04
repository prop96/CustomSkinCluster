#pragma once
#include <Eigen/Core>
#include <maya/MMatrix.h>
#include <maya/MVector.h>
#include <maya/MPoint.h>


class MatrixUtil
{
public:
	static void SingularValueDecomposition(const MMatrix& mat, MMatrix& u, MMatrix& vt);

	static MMatrix BuildMatrixFromMPoint(const MPoint& a, const MPoint& b);

private:
	static void FromMMatrixToEigenMat3(const MMatrix& in, Eigen::Matrix3d& out);
	static void FromEigenMat3ToMMatrix(const Eigen::Matrix3d& in, MMatrix& out);
	static void FromEigenVec3ToMVector(const Eigen::Vector3d& in, MVector& out);
};
