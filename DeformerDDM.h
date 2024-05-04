#pragma once
#include <maya/MMatrix.h>
#include <maya/MPoint.h>
#include <maya/MArrayDataHandle.h>
#include <vector>
#include <array>


class DeformerDDM
{
public:
	DeformerDDM();
	~DeformerDDM();

	struct SmoothingProperty
	{
		float Amount {0.0f};
		int Iteration {0};
		bool IsImplicit {false};
	};

	void SetSmoothingProperty(const SmoothingProperty& prop);
	SmoothingProperty GetSmoothingProperty() const;

	/// <summary>
	/// Compute Psi matrices array
	/// </summary>
	void Precompute(MObject& mesh, MArrayDataHandle& weightListsHandle);

	MPoint Deform(
		int vertIdx,
		const MPoint& pt,
		const MMatrix& worldToLocal,
		MArrayDataHandle& transformsHandle,
		MArrayDataHandle& bindHandle,
		MArrayDataHandle& weightsHandle,
		MStatus* ptrStat) const;

private:

	std::vector<std::array<MMatrix, 4>> m_psiMats;
	std::vector<std::array<int32_t, 4>> m_jointIdxs;

	SmoothingProperty m_smoothingProp;
};