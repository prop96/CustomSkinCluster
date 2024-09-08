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

MMatrix MatrixUtil::ZeroMatrix()
{
    // default MMatrix should be diagonal
    MMatrix tmp;
    for (unsigned int idx = 0; idx < 4; idx++)
    {
        tmp[idx][idx] = 0;
    }

    return tmp;
}

void MatrixUtil::To3x3Matrix(MMatrix& mat)
{
    for (unsigned int idx = 0; idx < 4; idx++)
    {
        mat[3][idx] = 0;
        mat[idx][3] = 0;
    }

    mat[3][3] = 1;
}

float MatrixUtil::Determinant3x3(const MMatrix& mat)
{
    float det = mat[0][0] * mat[1][1] * mat[2][2]
              + mat[0][1] * mat[1][2] * mat[2][0]
              + mat[0][2] * mat[1][0] * mat[2][1]
              - mat[0][2] * mat[1][1] * mat[2][0]
              - mat[0][1] * mat[1][0] * mat[2][2]
              - mat[0][0] * mat[1][2] * mat[2][1];

    return det;
}

MQuaternion MatrixUtil::MatrixToQuaternion(const MMatrix& m)
{
    auto px = m[0][0] - m[1][1] - m[2][2] + 1;
    auto py = -m[0][0] + m[1][1] - m[2][2] + 1;
    auto pz = -m[0][0] - m[1][1] + m[2][2] + 1;
    auto pw = m[0][0] + m[1][1] + m[2][2] + 1;

    auto selected = 0;
    auto max = px;
    if (max < py) {
        selected = 1;
        max = py;
    }
    if (max < pz) {
        selected = 2;
        max = pz;
    }
    if (max < pw) {
        selected = 3;
        max = pw;
    }

    if (selected == 0) {
        auto x = std::sqrt(px) * 0.5f;
        auto d = 1 / (4 * x);
        return MQuaternion(
            x,
            (m[1][0] + m[0][1]) * d,
            (m[0][2] + m[2][0]) * d,
            (m[2][1] - m[1][2]) * d
        );
    }
    else if (selected == 1) {
        auto y = std::sqrt(py) * 0.5f;
        auto d = 1 / (4 * y);
        return MQuaternion(
            (m[1][0] + m[0][1]) * d,
            y,
            (m[2][1] + m[1][2]) * d,
            (m[0][2] - m[2][0]) * d
        );
    }
    else if (selected == 2) {
        auto z = std::sqrt(pz) * 0.5f;
        auto d = 1 / (4 * z);
        return MQuaternion(
            (m[0][2] + m[2][0]) * d,
            (m[2][1] + m[1][2]) * d,
            z,
            (m[1][0] - m[0][1]) * d
        );
    }
    else if (selected == 3) {
        auto w = std::sqrt(pw) * 0.5f;
        auto d = 1 / (4 * w);
        return MQuaternion(
            (m[2][1] - m[1][2]) * d,
            (m[0][2] - m[2][0]) * d,
            (m[1][0] - m[0][1]) * d,
            w
        );
    }

    return MQuaternion::identity;
}

MMatrix MatrixUtil::QuaternionToMatrix(const MQuaternion& q)
{
    auto xy2 = q.x * q.y * 2;
    auto xz2 = q.x * q.z * 2;
    auto xw2 = q.x * q.w * 2;
    auto yz2 = q.y * q.z * 2;
    auto yw2 = q.y * q.w * 2;
    auto zw2 = q.z * q.w * 2;
    auto ww2 = q.w * q.w * 2;

    MMatrix mat = MatrixUtil::ZeroMatrix();
    mat[0][0] = ww2 + 2 * q.x * q.x - 1;
    mat[1][0] = xy2 + zw2;
    mat[2][0] = xz2 - yw2;
    mat[0][1] = xy2 - zw2;
    mat[1][1] = ww2 + 2 * q.y * q.y - 1;
    mat[2][1] = yz2 + xw2;
    mat[0][2] = xz2 + yw2;
    mat[1][2] = yz2 - xw2;
    mat[2][2] = ww2 + 2 * q.z * q.z - 1;
    mat[3][3] = 1;

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
