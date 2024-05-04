#include "MatrixUtil.h"
#include <Eigen/Eigenvalues> 
#include <Eigen/SVD>

void MatrixUtil::SingularValueDecomposition(const MMatrix& mat, MMatrix& u, MMatrix& vt)
{
    // decompose: mat = U * S * V^t

    Eigen::Matrix3d target;
    FromMMatrixToEigenMat3(mat, target);

    // Jacobi SVD is fast for small matrices, while very slow for large ones.
    Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::ComputeFullU | Eigen::ComputeFullV> solver(
        target, Eigen::ComputeFullU | Eigen::ComputeFullV);

    FromEigenMat3ToMMatrix(solver.matrixU(), u);
    FromEigenMat3ToMMatrix(solver.matrixV().transpose(), vt);
}

MMatrix MatrixUtil::BuildMatrixFromMPoint(const MPoint& a, const MPoint& b)
{
    MMatrix mat;

    // [   ]             [ a0*b0  a0*b1  a0*b2 ]
    // [ a ] [  b  ]  =  [ a1*b0  ...    ...   ]
    // [   ]             [ a2*b0  ...    ...   ]
    for (int c = 0; c < 4; c++)
    {
        for (int r = 0; r < 4; r++)
        {
            mat[r][c] = a[r] * b[c];
        }
    }

    return mat;
}

void MatrixUtil::FromMMatrixToEigenMat3(const MMatrix& in, Eigen::Matrix3d& out)
{
    for (int c = 0; c < 3; c++)
    {
        for (int r = 0; r < 3; r++)
        {
            out(r, c) = in[r][c];
        }
    }
}

void MatrixUtil::FromEigenMat3ToMMatrix(const Eigen::Matrix3d& in, MMatrix& out)
{
    for (int c = 0; c < 4; c++)
    {
        for (int r = 0; r < 4; r++)
        {
            out[r][c] = r == 3 || c == 3 ? 0 : in.coeff(r, c);
        }
    }
}

void MatrixUtil::FromEigenVec3ToMVector(const Eigen::Vector3d& in, MVector& out)
{
    for (int idx = 0; idx < 3; idx++)
    {
        out[idx] = in[idx];
    }
}
