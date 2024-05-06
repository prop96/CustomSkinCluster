#include "DeformerDDM.h"
#include "MeshLaplacian.h"
#include "MatrixUtil.h"
#include <maya/MPxSkinCluster.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMatrixData.h>
#include <maya/MPointArray.h>
#include <maya/MStatus.h>
#include "omp.h"


void DeformerDDM::SetSmoothingProperty(const SmoothingProperty& prop)
{
	m_smoothingProp = prop;
}

void DeformerDDM::Precompute(MObject& mesh, MArrayDataHandle& weightListsHandle)
{
	MFnMesh meshFn(mesh);

	MPointArray original;
	meshFn.getPoints(original);

	const unsigned int numVerts = original.length();

	// compute the smoothing matrix
	Eigen::SparseMatrix<double> B(numVerts, numVerts);
	MeshLaplacian::ComputeSmoothingMatrix(MItMeshEdge(mesh), numVerts,
		m_smoothingProp.Amount, m_smoothingProp.Iteration, m_smoothingProp.IsImplicit, B);

	m_psiMats.resize(numVerts);
	m_jointIdxs.resize(numVerts);

	for (unsigned int vIdx = 0; vIdx < numVerts; vIdx++)
	{
		weightListsHandle.jumpToArrayElement(vIdx);
		MArrayDataHandle weightsHandle = weightListsHandle.inputValue().child(MPxSkinCluster::weights);

		unsigned int numWeights = weightsHandle.elementCount(); // # of nonzero weights
		assert(numWeights <= MaxInfluence);

		for (unsigned int wIdx = 0; wIdx < MaxInfluence; wIdx++)
		{
			MMatrix tmp = MatrixUtil::ZeroMatrix();

			if (wIdx < numWeights)
			{
				// 不要かもだけど、安全を期しておく
				//weightListsHandle.jumpToArrayElement(vIdx);
				//weightsHandle = weightListsHandle.inputValue().child(MPxSkinCluster::weights);

				weightsHandle.jumpToArrayElement(wIdx); // jump to physical index
				unsigned int jointIdx = weightsHandle.elementIndex(); // logical index corresponds to the joint index

				m_jointIdxs[vIdx][wIdx] = jointIdx;

//#pragma omp parallel for
				for (int k = 0; k < numVerts; k++)
				{
					// first, compute w_kj
					weightListsHandle.jumpToArrayElement(k);
					MArrayDataHandle tmpWeightsHandle = weightListsHandle.inputValue().child(MPxSkinCluster::weights);

					double w_kj = 0.0f;
					for (unsigned int wIdx2 = 0; wIdx2 < tmpWeightsHandle.elementCount(); wIdx2++)
					{
						tmpWeightsHandle.jumpToArrayElement(wIdx2);

						unsigned int tmpJointIdx = tmpWeightsHandle.elementIndex();
						if (tmpJointIdx == jointIdx)
						{
							w_kj = tmpWeightsHandle.inputValue().asDouble();
							break;
						}
					}
					assert(w_kj >= 0.0 && w_kj <= 1.0);

					// compute ukuk
					MPoint pos = original[k];
					MMatrix ukuk = MatrixUtil::BuildMatrixFromMPoint(pos, pos);
					tmp += B.coeff(k, vIdx) * w_kj * ukuk;
				}
			}
			else
			{
				m_jointIdxs[vIdx][wIdx] = -1;
			}

			m_psiMats[vIdx][wIdx] = tmp;
		}
	}
}

MPoint DeformerDDM::Deform(
	int vertIdx,
	const MPoint& pt,
	const MMatrix& worldToLocal,
	MArrayDataHandle& transformsHandle,
	MArrayDataHandle& bindHandle,
	MArrayDataHandle& weightsHandle,
	MStatus* ptrStat) const
{
	MPoint skinned;

	MMatrix PsiM = MatrixUtil::ZeroMatrix();

	for (size_t idx = 0; idx < MaxInfluence; idx++)
	{
		// joint index
		const int j = m_jointIdxs[vertIdx][idx];
		if (j < 0)
		{
			continue;
		}

		transformsHandle.jumpToElement(j); // jump to logical index
		MMatrix jointMat = MFnMatrixData(transformsHandle.inputValue().data()).matrix();

		bindHandle.jumpToElement(j); // jump to logical index
		MMatrix preBindMatrix = MFnMatrixData(bindHandle.inputValue().data()).matrix();
		jointMat = preBindMatrix * jointMat;

		PsiM += m_psiMats[vertIdx][idx] * jointMat;
	}


	MMatrix Qi = PsiM;
	MPoint qi = PsiM[3];
	qi.w = 0.0;
	MPoint pi = (PsiM.transpose())[3];
	pi.w = 0.0;
	MMatrix Qpq = Qi - MatrixUtil::BuildMatrixFromMPoint(pi, qi); // 多分あってるけど逆かも?
	
	MMatrix u, vt;
	MatrixUtil::SingularValueDecomposition(Qpq.transpose(), u, vt);

	u[3][3] = 1.0;
	vt[3][3] = 1.0;
	qi.w = 1.0;
	pi.w = 1.0;

	MMatrix R = vt.transpose() * u.transpose(); // 逆だったり、転置必要かも
	MPoint t = qi - pi * R;
	t.w = 1.0;

	skinned = pt * R + t;

	return skinned * worldToLocal;
}

