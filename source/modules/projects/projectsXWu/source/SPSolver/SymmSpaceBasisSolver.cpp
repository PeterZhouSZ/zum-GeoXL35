#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "SPSolver/SymmSpaceBasisSolver.h"
#include "SPSolver/SymmSpaceNullSpace.h"
#include "SToolBox/DefieldSymm.h"
#include "SToolBox/SamplerSymmBlock.h"
#include "SPSolver/LaplacianRegularizer.h"
#include "SPSolver/SymmSpaceBasis.h"
//---------------------------------------------------------------------------
#include "IterativeSolvers.h"

#include "ProgressWindow.h"
#include "InCorePCTopologyGraph.h"
#include "basics\DeformationTools.h"
#include "GeometricTools.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

SymmSpaceBasisSolver::SymmSpaceBasisSolver() : Dim(SymmBasis::Dim)
{
    Clear();
}

void SymmSpaceBasisSolver::Clear()
{
    SymmSpaceSolver::Clear();
    basisVec.clear();
}

int SymmSpaceBasisSolver::Initialize(
    UICPC* samplePC, const UICPC* deformPC, const UICTM* inputTM,
    const bool& useNullSpace,
    const float& regularizerWeight, const bool& useInertia,
    const float& interpo_sigma, const bool& seperateSymmGroups, const float& maxDist
    )
{
    Clear();
    SymmSpaceSolver::Initialize(
        samplePC, deformPC, inputTM,
        useNullSpace,
        regularizerWeight, useInertia,
        interpo_sigma, seperateSymmGroups, maxDist
        );

    const unsigned& num_points = samplePC->getNumPoints();
    basisVec.resize(num_points);

    collectSymmAxis(false, false);

    bIntialized_ = true;

    return 0;
}

void SymmSpaceBasisSolver::beginConstraints()
{
    SymmSpaceSolver::beginConstraints();

    //collectSymmAxis(true, false);

    //const bool debug_output = true;
    //if (debug_output)
    //{
    //    DR_FRAME = 1;
    //    for (size_t gi = 0; gi < symmVec_.size(); ++gi)
    //    {
    //        const OrbitQueue& orbits = orbitVec_[gi];
    //        const unsigned& num_orbit = orbits.size();
    //        debugRenderer->beginRenderJob_OneFrame("symmetric_space_basis_", DR_FRAME++);
    //        for (unsigned oi = 0; oi < num_orbit; ++oi) {
    //            const std::deque<unsigned>& orbit = orbits[oi];
    //            for (unsigned vi = 0; vi < orbit.size(); ++vi) {
    //                const unsigned& index = orbit[vi];
    //                const Vector3f& pos_oringin = currentPositions[index];
    //                basisVec[index].DebugDraw(pos_oringin);
    //            }
    //        }
    //        debugRenderer->endRenderJob();
    //    }
    //}
}

SymmSpaceBasisSolver::basis_array SymmSpaceBasisSolver::constructBasis(const Vector3f& primary) const
{
    //basis_array canonical_basis;
    //for (unsigned d = 0; d < Dim; ++d) {
    //    canonical_basis[d] = NULL_VECTOR3F;
    //    canonical_basis[d][d] = 1.f;
    //}
    //return canonical_basis;

    if (1 > DIM || 3 < Dim) error("SymmSpaceBasisSolver::constructBasis outer space basis not supported");

    basis_array ret;
    for (unsigned d = 0; d < Dim; ++d) {
        ret[d] = NULL_VECTOR3F;
    }
    ret[0] = primary;
    if (1 == DIM) return ret;

    float value = 2;
    unsigned min_d = 0;
    for (unsigned d = 0; d < 3; ++d) {
        const float value_d = abs(primary[d]);
        if (value_d > value) continue;
        value = value_d;
        min_d = d;
    }
    Vector3f axis_min = NULL_VECTOR3F;
    axis_min[min_d] = 1.f;

    const Vector3f& third = primary.crossProduct(axis_min);
    ret[1] = normalize(third.crossProduct(primary));
    if (2 == DIM) return ret;

    ret[2] = third;
    return ret;
}

void SymmSpaceBasisSolver::collectSymmAxis(const bool& update, const bool& debug_draw)
{
    //if (symmVec_.empty()) return;
    //DR_FRAME = 1;

    //const unsigned num_points = basisVec.size();
    //std::vector<bool> mark(num_points, false); // only one group orbit is possible by ordering
    //for (size_t gi = 0; gi < symmVec_.size(); ++gi)
    //{
    //    const DefieldSymm* defSymm = symmVec_[gi];
    //    if (defSymm->empty()) continue;
    //    const OrbitQueue& orbits = orbitVec_[gi];
    //    const unsigned& num_orbit = orbits.size();

    //    for (unsigned oi = 0; oi < num_orbit; ++oi) {
    //        const std::deque<unsigned>& orbit = orbits[oi];
    //        const unsigned& elem = orbit[0];
    //        const Vector3f& pos_elem_init = initialPositions[elem];
    //        const Vector3f& pos_elem_curr = currentPositions[elem];
    //        const Vector3f& elem_normal = (update) ?
    //            defSymm->GetNormalDirection(pos_elem_curr, pos_elem_init) :
    //            defSymm->GetNormalDirection(pos_elem_init, pos_elem_init);
    //        const basis_array& elem_basis = constructBasis(elem_normal);
    //        for (unsigned vi = 0; vi < orbit.size(); ++vi) {
    //            const unsigned& index = orbit[vi];
    //            if (mark[index]) continue;

    //            const Vector3f& pos_oringin = initialPositions[index];
    //            const Matrix3f& R = shrink4To3(defSymm->GetTransformation(vi));
    //            SymmBasis::basis_type B;
    //            for (unsigned d = 0; d < Dim; ++d) {
    //                B[d].delta = 0;

    //                /* point-wise tranformation *
    //                const Vector3f& pos_elem_disp = pos_elem_init + elem_basis[d];
    //                const Vector3f& pos_elem_disp_t = transformVector3f(defSymm->GetTransformation(vi), pos_elem_disp);
    //                B[d].axis = normalize(pos_elem_disp_t - pos_oringin);
    //                /* transform by rotation part of transformation matrix */
    //                B[d].axis = normalize(R * elem_basis[d]);
    //                /**/
    //            }
    //            basisVec[index] = SymmBasis(B, pos_oringin);

    //            mark[index] = true;
    //        }
    //    }

    //    const bool debug_output = false;
    //    if (debug_output)
    //    {
    //        for (unsigned oi = 0; oi < num_orbit; ++oi) {
    //            debugRenderer->beginRenderJob_OneFrame("debug_", DR_FRAME++);
    //            const std::deque<unsigned>& orbit = orbits[oi];
    //            const unsigned& elem = orbit[0];
    //            const Vector3f& pos_elem_init = initialPositions[elem];
    //            const Vector3f& pos_elem_curr = currentPositions[elem];
    //            const Vector3f& elem_normal = (update) ?
    //                defSymm->GetNormalDirection(pos_elem_curr, pos_elem_init) :
    //                defSymm->GetNormalDirection(pos_elem_init, pos_elem_init);
    //            const basis_array& elem_basis = constructBasis(elem_normal);
    //            for (unsigned vi = 0; vi < orbit.size(); ++vi) {
    //                const unsigned& index = orbit[vi];
    //                basisVec[index].DebugDraw();

    //                const Vector3f& pos_oringin = initialPositions[index];
    //                const Matrix3f& R = shrink4To3(defSymm->GetTransformation(vi));
    //                for (unsigned d = 0; d < Dim; ++d) {
    //                    Vector3f color = NULL_VECTOR3F;
    //                    color[d % 3] = 1.f;
    //                    const Vector3f& vec_r = R * elem_basis[d];
    //                    debugRenderer->addLine(
    //                        pos_oringin, pos_oringin + vec_r,
    //                        color, makeVector3f(1, 0, 1),
    //                        2);
    //                    const Vector3f& vec_t = transformVector3f(defSymm->GetTransformation(vi), elem_basis[d]);
    //                    debugRenderer->addLine(
    //                        pos_oringin, pos_oringin + vec_t,
    //                        makeVector3f(1, 1, 0), makeVector3f(0, 1, 1),
    //                        2);
    //                    debugRenderer->addPoint(
    //                        pos_oringin, makeVector3f(1, 1, 0)
    //                        );
    //                }
    //            }
    //            debugRenderer->endRenderJob();
    //        }
    //    }

    //    if (debug_draw)
    //    {
    //        debugRenderer->beginRenderJob_OneFrame("symmetric_space_basis_", DR_FRAME++);
    //        for (unsigned oi = 0; oi < num_orbit; ++oi) {
    //            const std::deque<unsigned>& orbit = orbits[oi];
    //            for (unsigned vi = 0; vi < orbit.size(); ++vi) {
    //                const unsigned& index = orbit[vi];
    //                basisVec[index].DebugDraw();
    //            }
    //        }
    //        debugRenderer->endRenderJob();
    //    }
    //}
}

StaticVector<float, SymmSpaceBasisSolver::DIM> SymmSpaceBasisSolver::getLocalCoordinates(
    const Vector3f& pos, const unsigned& index) const
{
    const Vector3f& pos_base = latestPositions[index];
    const Vector3f& diff = pos - pos_base;
    return basisVec[index] * diff;
}

DVectorD SymmSpaceBasisSolver::CollectDeformPos()
{
    const unsigned& num_basis = basisVec.size();
    const unsigned& num_var = Dim * num_basis;
    DVectorD result;
    result.setDim(num_var);

    for (unsigned bi = 0; bi < num_basis; ++bi) {
        for (unsigned d = 0; d < Dim; ++d) {
            result[bi] = basisVec[bi][d].delta;
        }
    }

    return result;
}

void SymmSpaceBasisSolver::DistributeDeformPos(const DVectorD& result)
{
    const unsigned& num_basis = basisVec.size();
    const unsigned& num_var = Dim * num_basis;
    if (num_var != result.getDim()) {
        error("SymmSpaceBasisSolver::DistributeDeformPos() - Size mismatch.");
    }

    for (unsigned bi = 0; bi < num_basis; ++bi) {
        for (unsigned d = 0; d < Dim; ++d) {
            basisVec[bi][d].delta = result[bi * Dim + d];
        }
        displacementVec[bi] = basisVec[bi].evaluate();
        currentPositions[bi] = latestPositions[bi] + displacementVec[bi];
    }
}

// fixed, no need to update during iterations
void SymmSpaceBasisSolver::addSymmetryConstraint(
    const unsigned& source,
    const unsigned& target,
    const Matrix4f& transformation,
    const float& weight
    )
{
    SparseVectorD constrVec;
    for (unsigned d = 0; d < Dim; ++d) {
        constrVec.addEntryBinary(Dim * source + d, 2*weight);
        constrVec.addEntryBinary(Dim * target + d, -2*weight);

        symmConstraints_.addRow(constrVec);
        symmConstValues_.appendInPlace(0);
    }
}

// fixed, no need to update during iterations
void SymmSpaceBasisSolver::addFixedConstraint(
    const unsigned& index,
    const float& weight
    )
{
    SparseVectorD constrVec;
    for (unsigned d = 0; d < Dim; ++d) {
        constrVec.setEntryBinary(Dim * index + d, 2*weight);

        fixedConstraints_.addRow(constrVec);
        fixedConstValues_.appendInPlace(0);
    }
}

// have to be updated during iterations
void SymmSpaceBasisSolver::addSoftConstraint(
    const unsigned& index,
    const Vector3f& toPos,
    const float& weight
    )
{
    const StaticVector<float, SymmSpaceBasisSolver::DIM>& toDelta = getLocalCoordinates(toPos, index);
    SparseVectorD constrVec;
    for (unsigned d = 0; d < Dim; ++d) {
        constrVec.setEntryBinary(Dim * index + d, 2*weight);

        softConstraints_.addRow(constrVec);
        softConstValues_.appendInPlace(2*weight*toDelta[d]);
    }

    const bool debug_show = false;
    if (debug_show) {
        debugRenderer->beginRenderJob_OneFrame("handle_constraints_", 0);
        const Vector3f& pos_oringin = initialPositions[index];
        const Vector3f& pos_target = pos_oringin + basisVec[index].evaluate(toDelta);
        debugRenderer->addLine(
            pos_oringin, pos_target,
            makeVector3f(1, 1, 0),
            makeVector3f(0, 1, 1),
            2);
        debugRenderer->addPoint(
            pos_oringin, makeVector3f(1, 1, 0)
            );
        basisVec[index].DebugDraw();
        debugRenderer->endRenderJob();
    }
}

// fixed, no need to update during iterations
void SymmSpaceBasisSolver::addColinearity(
    const unsigned& A,
    const unsigned& M,
    const unsigned& B,
    const float& weight
    )
{
    // B - M = M - A <==>
    // 2 * M - B - A = 0
    SparseVectorD constrVec;
    for (unsigned d = 0; d < Dim; ++d) {
        constrVec.addEntryBinary(Dim * M + d, 2*2*weight);
        constrVec.addEntryBinary(Dim * A + d, -2*weight);
        constrVec.addEntryBinary(Dim * B + d, -2*weight);

        fixedConstraints_.addRow(constrVec);
        fixedConstValues_.appendInPlace(0);
    }
}

// have to be updated during iterations
void SymmSpaceBasisSolver::addCoplanarity(
    const unsigned& S,
    const unsigned& T,
    const Vector3f& N,
    const float& weight
    )
{
    //// (S - S') * N = (T - T') * N <==>
    //// (S - T) * N = (S' - T') * N
    //SparseVectorD constrVec;
    //constrVec.addEntryBinary(S, -2*weight*N[d]);
    //constrVec.addEntryBinary(T, 2*weight*N[d]);

    //hardConstraints_.addRow(constrVec);
    //hardConstValues_.appendInPlace(0);
}
