#pragma once
#include <maya/MMatrix.h>
#include <maya/MPoint.h>
#include <maya/MArrayDataHandle.h>
#include <Eigen/Sparse>
#include <Eigen/Core>
#include <vector>
#include <array>


class DeformerDDM
{
public:
	DeformerDDM() = default;
	~DeformerDDM() = default;

	struct SmoothingProperty
	{
		double Amount {0.0f};
		int Iteration {0};
		bool IsImplicit {false};

		friend bool operator==(const SmoothingProperty& a, const SmoothingProperty& b)
		{
			return a.Amount == b.Amount && a.Iteration == b.Iteration && a.IsImplicit == b.IsImplicit;
		}

		friend bool operator!=(const SmoothingProperty& a, const SmoothingProperty& b)
		{
			return !(a==b);
		}
	};

	void SetSmoothingProperty(const SmoothingProperty& prop);
	SmoothingProperty GetSmoothingProperty() const;

	/// <summary>
	/// Compute Psi matrices array
	/// </summary>
	void Precompute(MObject& mesh, MArrayDataHandle& weightListsHandle, bool needRebindMesh);

	MPoint Deform(
		int vertIdx,
		const MPoint& pt,
		const MMatrix& worldToLocal,
		MArrayDataHandle& transformsHandle,
		MArrayDataHandle& bindHandle,
		MArrayDataHandle& weightsHandle,
		MStatus* ptrStat) const;

private:

	static constexpr size_t MaxInfluence = 8;

	std::vector<std::array<MMatrix, MaxInfluence>> m_psiMats;
	std::vector<std::array<int32_t, MaxInfluence>> m_jointIdxs;

	SmoothingProperty m_smoothingProp;

	/// <summary>
	/// Laplacian matrix, which is determined by the mesh topology
	/// </summary>
	Eigen::SparseMatrix<double> m_laplacian;

	/// <summary>
	/// Smoothing matrix, which is determined by Laplacian matrix and the smoothing property
	/// </summary>
	Eigen::SparseMatrix<double> m_smoothingMat;

	/// <summary>
	/// dirty flag for recoputation of the smoothing matrix
	/// </summary>
	bool m_isSmoothingMatDirty = true;
};