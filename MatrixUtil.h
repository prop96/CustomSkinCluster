#pragma once
#include <Eigen/Core>
#include <maya/MMatrix.h>
#include <maya/MVector.h>
#include <maya/MPoint.h>
#include <maya/MQuaternion.h>


/// <summary>
/// Utility to use Eigen library with Maya API
/// </summary>
class MatrixUtil
{
public:
	/// <summary>
	/// do Singular Value Decomposition of 3x3 matrix: mat = U * S * V^t
	/// </summary>
	/// <param name="mat">3x3 matrix</param>
	/// <param name="u">3x3 matrix</param>
	/// <param name="vt">3x3 matrix</param>
	static void SingularValueDecomposition(const MMatrix& mat, MMatrix& u, MMatrix& vt);

	/// <summary>
	/// create 4x4 matrix from the given vectors
	/// </summary>
	/// <param name="a">4d vector</param>
	/// <param name="b">4d vector</param>
	/// <returns></returns>
	static MMatrix BuildMatrixFromMPoint(const MPoint& a, const MPoint& b);

	/// <summary>
	/// create a matrix whose elements are all zero
	/// </summary>
	/// <returns></returns>
	static MMatrix ZeroMatrix();

	/// <summary>
	/// Return 3x3 matrix by making unnecessary elements zero
	/// </summary>
	/// <param name="mat"></param>
	/// <returns></returns>
	static void To3x3Matrix(MMatrix& mat);

	/// <summary>
	/// compute the determinant of 3x3 matrix
	/// </summary>
	/// <param name="mat"></param>
	/// <returns></returns>
	static float Determinant3x3(const MMatrix& mat);

	/// <summary>
	/// Convert the rotation matrix to a quaternion
	/// </summary>
	/// <param name="mat"></param>
	/// <returns></returns>
	static MQuaternion MatrixToQuaternion(const MMatrix& mat);

	/// <summary>
	/// Convert the quaternion to a rotation matrix
	/// </summary>
	/// <param name="quat"></param>
	/// <returns></returns>
	static MMatrix QuaternionToMatrix(const MQuaternion& quat);

private:
	static void FromMMatrixToEigenMat3(const MMatrix& in, Eigen::Matrix3d& out);
	static void FromEigenMat3ToMMatrix(const Eigen::Matrix3d& in, MMatrix& out);
	static void FromEigenVec3ToMVector(const Eigen::Vector3d& in, MVector& out);
};
