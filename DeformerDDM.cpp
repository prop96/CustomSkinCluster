#include "DeformerDDM.h"
#include "MeshLaplacian.h"
#include "MatrixUtil.h"
#include <maya/MPxSkinCluster.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMatrixData.h>
#include <maya/MQuaternion.h>
#include <maya/MPointArray.h>
#include <maya/MStatus.h>
#include "omp.h"

namespace {
	inline float QuatDot(const MQuaternion& q1, const MQuaternion& q2) {
		return q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;
	}
}


void DeformerDDM::SetSmoothingProperty(const SmoothingProperty& prop)
{
	if (m_smoothingProp != prop)
	{
		m_smoothingProp = prop;
		m_isSmoothingMatDirty = true;
	}
}

void DeformerDDM::Precompute(MObject& mesh, MArrayDataHandle& weightListsHandle, bool needRebindMesh)
{
	MFnMesh meshFn(mesh);

	MPointArray original;
	meshFn.getPoints(original);

	const unsigned int numVerts = original.length();

	// recompute laplacian if necessary
	if (needRebindMesh)
	{
		MeshLaplacian::ComputeLaplacian(MItMeshEdge(mesh), numVerts, m_laplacian);
		m_isSmoothingMatDirty = true;
	}

	// compute the smoothing matrix if necessary
	if (m_isSmoothingMatDirty)
	{
		MeshLaplacian::ComputeSmoothingMatrix(m_laplacian, numVerts,
			m_smoothingProp.Amount, m_smoothingProp.Iteration, m_smoothingProp.IsImplicit, m_smoothingMat);

		m_isSmoothingMatDirty = false;
	}

	m_psiMats.resize(numVerts);
	m_jointIdxs.resize(numVerts);


	for (int vIdx = 0; vIdx < numVerts; vIdx++)
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
				// •s—v‚©‚à‚¾‚¯‚ÇAˆÀ‘S‚ðŠú‚µ‚Ä‚¨‚­
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
					tmp += m_smoothingMat.coeff(k, vIdx) * w_kj * ukuk;
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
	MMatrix Qpq = Qi - MatrixUtil::BuildMatrixFromMPoint(pi, qi); // ‘½•ª‚ ‚Á‚Ä‚é‚¯‚Ç‹t‚©‚à?
	
	MMatrix u, vt;
	MatrixUtil::SingularValueDecomposition(Qpq.transpose(), u, vt);

	u[3][3] = 1.0;
	vt[3][3] = 1.0;
	qi.w = 1.0;
	pi.w = 1.0;

	MMatrix R = vt.transpose() * u.transpose(); // ‹t‚¾‚Á‚½‚èA“]’u•K—v‚©‚à
	MPoint t = qi - pi * R;
	t.w = 1.0;

	skinned = pt * R + t;

	return skinned * worldToLocal;
}

MPoint DeformerDDM::Deform_v1(int vertIdx, const MPoint& pt, const MMatrix& worldToLocal, MArrayDataHandle& transformsHandle, MArrayDataHandle& bindHandle, MArrayDataHandle& weightsHandle, MStatus* ptrStat) const
{
	MPoint skinned;

	MMatrix PsiM = MatrixUtil::ZeroMatrix();
	MMatrix Psi = MatrixUtil::ZeroMatrix();

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

		Psi += m_psiMats[vertIdx][idx];
	}


	MMatrix Qi = PsiM;
	MPoint qi = PsiM[3];
	qi.w = 0.0;
	MPoint pi = (PsiM.transpose())[3];
	pi.w = 0.0;
	MMatrix Qpq = Qi - MatrixUtil::BuildMatrixFromMPoint(pi, qi); // ‘½•ª‚ ‚Á‚Ä‚é‚¯‚Ç‹t‚©‚à?

	MMatrix Ppp = Psi - MatrixUtil::BuildMatrixFromMPoint(pi, pi);

	MatrixUtil::To3x3Matrix(Qpq);
	MatrixUtil::To3x3Matrix(Ppp);
	qi.w = 1.0;
	pi.w = 1.0;

	MMatrix R = MatrixUtil::Determinant3x3(Qpq) / MatrixUtil::Determinant3x3(Ppp) * Ppp * Qpq.transpose().inverse();
	MPoint t = qi - pi * R;
	t.w = 1.0;

	skinned = pt * R + t;

	return skinned * worldToLocal;
}

MPoint DeformerDDM::Deform_v2(int vertIdx, const MPoint& pt, const MMatrix& worldToLocal, MArrayDataHandle& transformsHandle, MArrayDataHandle& bindHandle, MArrayDataHandle& weightsHandle, MStatus* ptrStat) const
{
	MPoint skinned;

	MQuaternion psiQ = MQuaternion(0, 0, 0, 0);
	MQuaternion base = MQuaternion(0, 0, 0, 0);
	MPoint chi_omegaM;
	MPoint chi;

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

		const MMatrix& Psi_ij = m_psiMats[vertIdx][idx];
		const float psi_ij = Psi_ij[3][3];

		const auto Mq_ij = psi_ij * MatrixUtil::MatrixToQuaternion(psi_ij * jointMat);
		if (base.isEquivalent(MQuaternion(0,0,0,0)))
		{
			base = Mq_ij;
		}

		if (QuatDot(base, Mq_ij) < 0.0)
		{
			psiQ = psiQ - Mq_ij;
		}
		else
		{
			psiQ = psiQ + Mq_ij;
		}

		const MPoint chi_ij = Psi_ij[3];
		chi_omegaM += chi_ij * jointMat;
		chi += chi_ij;
	}

	MMatrix R = MatrixUtil::QuaternionToMatrix(psiQ);

	chi.w = chi_omegaM.w = 1;
	MPoint t = chi_omegaM - chi * R;
	t.w = 1.0;

	skinned = pt * R + t;

	return skinned * worldToLocal;
}

MPoint DeformerDDM::Deform_v3(int vertIdx, const MPoint& pt, const MMatrix& worldToLocal, MArrayDataHandle& transformsHandle, MArrayDataHandle& bindHandle, MArrayDataHandle& weightsHandle, MStatus* ptrStat) const
{
	MPoint skinned;

	MMatrix psiM = MatrixUtil::ZeroMatrix();
	MPoint chi_omegaM;
	MPoint chi;

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

		const MMatrix& Psi_ij = m_psiMats[vertIdx][idx];
		const float psi_ij = Psi_ij[3][3];
		psiM += psi_ij * jointMat;

		const MPoint chi_ij = Psi_ij[3];
		chi_omegaM += chi_ij * jointMat;
		chi += chi_ij;
	}

	MatrixUtil::To3x3Matrix(psiM);
	MMatrix R = 1 / MatrixUtil::Determinant3x3(psiM) * psiM;
	MatrixUtil::To3x3Matrix(R);

	chi.w = chi_omegaM.w = 1;
	MPoint t = chi_omegaM - chi * R;
	t.w = 1.0;

	skinned = pt * R + t;

	return skinned * worldToLocal;
}

MPoint DeformerDDM::Deform_v4(int vertIdx, const MPoint& pt, const MMatrix& worldToLocal, MArrayDataHandle& transformsHandle, MArrayDataHandle& bindHandle, MArrayDataHandle& weightsHandle, MStatus* ptrStat) const
{
	MPoint skinned;

	MQuaternion psiQ = MQuaternion(0, 0, 0, 0);
	MQuaternion base = MQuaternion(0, 0, 0, 0);
	MMatrix omegaM = MatrixUtil::ZeroMatrix();
	MPoint pi;

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

		const MMatrix& Psi_ij = m_psiMats[vertIdx][idx];
		const float psi_ij = Psi_ij[3][3];

		const auto Mq_ij = psi_ij * MatrixUtil::MatrixToQuaternion(psi_ij * jointMat);
		if (base.isEquivalent(MQuaternion(0, 0, 0, 0)))
		{
			base = Mq_ij;
		}

		if (QuatDot(base, Mq_ij) < 0.0)
		{
			psiQ = psiQ - Mq_ij;
		}
		else
		{
			psiQ = psiQ + Mq_ij;
		}

		omegaM += psi_ij * jointMat;
		const MPoint chi_ij = Psi_ij[3];
		pi += chi_ij;
	}

	MMatrix R = MatrixUtil::QuaternionToMatrix(psiQ);

	pi.w = 1;
	MPoint t = pi * omegaM - pi * R;
	t.w = 1.0;

	skinned = pt * R + t;

	return skinned * worldToLocal;
}

MPoint DeformerDDM::Deform_v5(int vertIdx, const MPoint& pt, const MMatrix& worldToLocal, MArrayDataHandle& transformsHandle, MArrayDataHandle& bindHandle, MArrayDataHandle& weightsHandle, MStatus* ptrStat) const
{
	MPoint skinned;

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

		const MMatrix& Psi_ij = m_psiMats[vertIdx][idx];
		const float psi_ij = Psi_ij[3][3];

		skinned += (pt * jointMat) * psi_ij;
	}

	return skinned * worldToLocal;
}

