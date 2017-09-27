#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "SPSolver/SymmSpaceSolver.h"
#include "SToolBox/DefieldSymm.h"
#include "SToolBox/DefieldSymmRotation.h"
#include "SToolBox/DefieldSymmDihedral.h"
#include "SToolBox/SamplerSymmBlock.h"
#include "SToolBox/SymmSampler.h"
//---------------------------------------------------------------------------
#include "IterativeSolvers.h"
#include "ProgressWindow.h"
#include "InCorePCTopologyGraph.h"
#include "basics\DeformationTools.h"
#include "GeometricTools.h"

#include "InCorePCTopologyGraph.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

template <typename FloatType>
inline FloatType GaussKernel(FloatType const& distSqr, FloatType const& epsSqr)
{
    return exp(-distSqr/2.f/epsSqr)/sqrt(2*3.14f*epsSqr);
}

template <typename FloatType>
inline FloatType WendlandKernel(FloatType const& distSqr, FloatType const& epsSqr)
{
    return (1 > distSqr) ? pow(1 - distSqr, 1 * distSqr / epsSqr) : 0.f;
}

SymmSpaceSolver::SymmSpaceSolver()
{
    Clear();
}

void SymmSpaceSolver::Clear()
{
    bIntialized_ = false;
    solverType_ = DIRECT_NULL;

    //symmVec_.clear();
    //orbitVec_.clear();

    initialPositions.clear();
    latestPositions.clear();
    currentPositions.clear();
    displacementVec.clear();

    bRefactor_ = true;
    softConstraints_.setRows();
    softConstValues_.setDim();
    hardConstraints_.setRows();
    hardConstValues_.setDim();
    SparseMatrixD symmConstraints_;
    DVectorD symmConstValues_;
    fixedConstraints_.setRows();
    fixedConstValues_.setDim();
    bUseNullSpace_ = true;
    nullSpace_.reset( new SymmSpaceNullSpace() );
    rangeSpace_.reset( new SymmSpaceDirectSolver() );
    directNullSpace_.reset( new SymmSpaceDirectNullSpaceSolver() );
}

int SymmSpaceSolver::Initialize(
    UICPC* samplePC, const UICPC* deformPC, const UICTM* inputTM,
    const bool& useNullSpace,
    const float& regularizerWeight, const bool& useInertia, const float& tpy_sigma,
    const float& interpo_sigma, const bool& seperateSymmGroups, const float& maxDist
    )
{
    Clear();

    if (!samplePC || !deformPC) {
        error("SymmSpaceBasisSolver::Initialize() - Type mismatch.");
        return -1;
    }

    SymmSampleGraph* symSamGraph = dynamic_cast<SymmSampleGraph*>(
        samplePC->getAttachments()->getData(SymmSampleGraph::getDefaultName()));
    if (!symSamGraph) {
        error("SymmSpaceSolver::Initialize() - SymmSampleGraph missing.");
        return -1;
    }

    // initialize points
    const unsigned& num_points = samplePC->getNumPoints();
    initialPositions.resize(num_points);
    latestPositions.resize(num_points);
    currentPositions.resize(num_points);
    displacementVec.resize(num_points, NULL_VECTOR3F);

    const PointSet& samplePS = *samplePC->getPointSet();
    const AAT& samPosAAT = samplePS.getAAT("position", 3, VAD::DATA_FORMAT_FLOAT32);
    const PointSet& defPS = *deformPC->getPointSet();
    const AAT& defPosAAT = defPS.getAAT("position", 3, VAD::DATA_FORMAT_FLOAT32);
    for (unsigned pi = 0; pi < num_points; ++pi)
    {
        initialPositions[pi] = samplePS.get3f(pi, samPosAAT);
        latestPositions[pi] = defPS.get3f(pi, defPosAAT);
        currentPositions[pi] = defPS.get3f(pi, defPosAAT);
    }

    // default use null-space solver
    bUseNullSpace_ = useNullSpace;
    const AAT& symGroupAAT = samplePS.getAAT("group");
    std::deque<unsigned> fillInIndice;
    for (unsigned pi = 0; pi < num_points; ++pi) {
        const int32& gid = SymmSampler::GroupCode(samplePS.get2i(pi, symGroupAAT));
        if (SymmSampler::FILL_IN != gid) continue;
        fillInIndice.push_back(pi);
    }
    nullSpace_->BuildRotatedBasis(*symSamGraph, fillInIndice, num_points);

    // interpolator
    FastSphereQuerryPtr query( new FastSphereQuerry(samplePC) );
    const unsigned& num_verts = inputTM->getNumPoints();
    SparseMatrixD liftWeight; liftWeight.setRows(num_verts); /* num_verts x num_sample */
    buildInterMatrix(samplePS, *inputTM->getPointSet(), query, interpo_sigma, seperateSymmGroups, maxDist, liftWeight);
    {
        const float ratio = 100 * (float)LibraryAdaptor::NumNonZero(liftWeight) / (num_verts * num_points);
        debugOutput << "non-zero ratio of interpolation matrix before remove zeros: " << ratio << "%\n";
    }
    //liftWeight.removeNearZeros(std::numeric_limits<double>::epsilon());
    //liftWeight.removeNearZeros(1e-6);
    //{
    //    const float ratio = 100 * (float)LibraryAdaptor::NumNonZero(liftWeight) / (num_verts * num_points);
    //    debugOutput << "non-zero ratio of interpolation matrix after remove zeros: " << ratio << "%\n";
    //}
    {
        liftWeight3.setRows(num_verts * 3);
        for (unsigned vi = 0; vi < num_verts; ++vi) {
            const SparseVectorD& weightVector = liftWeight[vi];
            SparseVectorD::EIteratorConst eit = weightVector.begin();
            SparseVectorD::EIteratorConst eit_end = weightVector.end();
            while (eit != eit_end) {
                const mpcard& index = eit->index;
                const float& weight = eit->value;
                for (unsigned d = 0; d < 3; ++d) {
                    liftWeight3[3 * vi + d].setEntryBinary(3 * index + d, weight);
                }
                ++eit;
            }
        }
    }
    if (bUseNullSpace_) meshInterpolater_.reset(new MeshInterpolaterN2M(samplePC, inputTM, nullSpace_->getNS(), liftWeight3));
    else meshInterpolater_.reset(new MeshInterpolaterS2M(samplePC, inputTM, liftWeight3));
    //meshInterpolater_.reset(new MeshInterpolaterS2M(samplePC, inputTM, liftWeight3));
    //meshInterpolater_.reset(new MeshInterpolaterN2M(samplePC, inputTM, nullSpace_->getNS(), liftWeight3));

    // Laplacian regularizer
    SparseMatrixD tpyWeight; tpyWeight.setRows(num_verts); /* num_verts x num_sample */
    buildInterMatrix(samplePS, *inputTM->getPointSet(), query, tpy_sigma, seperateSymmGroups, maxDist, tpyWeight);
    {
        const float ratio = 100 * (float)LibraryAdaptor::NumNonZero(tpyWeight) / (num_verts * num_points);
        debugOutput << "non-zero ratio of topology matrix before remove zeros: " << ratio << "%\n";
    }
    regularizer_.reset( new LaplacianRegularizerSS(samplePC, inputTM, tpyWeight, regularizerWeight, useInertia) );
    //regularizer_.reset( new LaplacianRegularizerMM(samplePC, inputTM, liftWeight, liftWeight3, regularizerWeight, useInertia) );

    bIntialized_ = true;

    return 0;
}

void SymmSpaceSolver::calculateWeight(
    const PointSet& inputPS, const unsigned& vi,
    const FastSphereQuerryPtr& query, const std::deque<int32>& symmetryClique,
    const float& interpo_sigma, const bool& seperateSymmGroups, const float& maxDist,
    SparseMatrixD& liftWeight)
{
    const bool debug_output = false;
    if (debug_output) debugRenderer->beginRenderJob_OneFrame("interpolation_", DR_FRAME++);

    const AAT& inputPosAAT = inputPS.getAAT("position", 3, VAD::DATA_FORMAT_FLOAT32);
    const AAT& symGroupAAT = inputPS.getAAT("group");
    const Vector3f& posInput = inputPS.get3f(vi, inputPosAAT);
    const int32& groupID = SymmSampler::GroupCode(inputPS.get2i(vi, symGroupAAT));

    const float epsSqr = interpo_sigma * interpo_sigma;
    mpcard pcnt = 0;

    SparseVectorD& weightVector = liftWeight[vi];
    mpcard numAllPoints; mpcard *allIndices;
    query->querry(posInput, maxDist, &allIndices, numAllPoints);
    float sumw = 0.f;
    for (size_t qi = 0; qi < numAllPoints; ++qi)
    {
        const mpcard& index = allIndices[qi];
        if (seperateSymmGroups)
        {
            if (symmetryClique[index] != groupID
                && SymmSampler::FILL_IN != symmetryClique[index]
            && SymmSampler::UNTOUCHED != groupID
                ) { // do not affect other symmetry group while interpolating
                    continue;
            }
        }
        const Vector3f& posinit = initialPositions[index];
        const float& distSqr = normQuad(posinit - posInput);
        const float& weight = GaussKernel(distSqr, epsSqr);
        //const float& weight = WendlandKernel(distSqr, epsSqr);
        if (weight < 1e-6) continue; // note: different query method require different stoping

        weightVector.addEntryBinary(index, weight);
        sumw += weight;
        ++pcnt;

        if (debug_output)
        {
            debugRenderer->addLine(
                posInput, posinit,
                SymmSampler::MapGroupColor(symmetryClique[index]),
                SymmSampler::MapGroupColor(symmetryClique[index]),
                1);
        }
    }
    if (1 < pcnt) weightVector /= sumw;
    if (debug_output)
    {
        SparseVectorD::EIteratorConst eit = weightVector.begin();
        SparseVectorD::EIteratorConst eit_end = weightVector.end();
        while (eit != eit_end) {
            const mpcard& index = eit->index;
            const float& weight = eit->value;
            debugRenderer->addFastSphere(
                initialPositions[index], weight,
                makeVector3f(0, 1, 1)
                );
            ++eit;
            debugOutput << weight << " ";
        }
        debugOutput << "\n";
    }

    if (debug_output)
    {
        debugRenderer->endRenderJob();
    }
}

void SymmSpaceSolver::buildInterMatrix(
    const PointSet& samplePS, const PointSet& inputPS, const FastSphereQuerryPtr& query,
    const float& interpo_sigma, const bool& seperateSymmGroups, const float& maxDist,
    SparseMatrixD& liftWeight)
{
    const unsigned& num_points = samplePS.getNumEntries();
    std::deque<int32> symmetryClique(num_points);
    const AAT& symGroupAAT = samplePS.getAAT("group");
    for (unsigned pi = 0; pi < num_points; ++pi)
    {
        symmetryClique[pi] = SymmSampler::GroupCode(samplePS.get2i(pi, symGroupAAT));
    }

    const unsigned& num_verts = inputPS.getNumEntries();
    for (unsigned vi = 0; vi < num_verts; ++vi) {
        calculateWeight(inputPS, vi, query, symmetryClique,
            interpo_sigma, seperateSymmGroups, maxDist,
            liftWeight);
    }
}

Vector3f SymmSpaceSolver::getPosition(ARConstraintAnchor const& at)
{
    mpint index = at.topologicalAnchor;
    if (index == ARConstraintAnchor::NO_TOP_ANCHOR) {
        error("should not happen");
        return NULL_VECTOR3F;
    }
    return currentPositions[index];
}

Vector3f SymmSpaceSolver::GetPosition(const unsigned& index)
{
    return currentPositions[index];
}

Vector3f SymmSpaceSolver::GetDisplacement(const unsigned& index)
{
    return displacementVec[index];
}

Vector3f SymmSpaceSolver::Evaluate(const Vector3f& position)
{
    error("SymmSpaceSolver::Evaluate - do not use this function");
    return NULL_VECTOR3F;
}

void SymmSpaceSolver::beginConstraints()
{
    softConstraints_.setRows();
    softConstValues_.setDim();
    hardConstraints_.setRows();
    hardConstValues_.setDim();
}

void SymmSpaceSolver::Refactor(void)
{
    bRefactor_ = true;
}

void SymmSpaceSolver::SolveDirect(void)
{
    hardConstraints_.appendRows(symmConstraints_);
    hardConstValues_.appendInPlace(symmConstValues_);
    hardConstraints_.appendRows(fixedConstraints_);
    hardConstValues_.appendInPlace(fixedConstValues_);

    //{
    //    SparseMatrixD QT(3);
    //    QT[0].addEntryBinary(0, 6);
    //    QT[0].addEntryBinary(1, 2);
    //    QT[0].addEntryBinary(2, 1);
    //    QT[1].addEntryBinary(0, 2);
    //    QT[1].addEntryBinary(1, 5);
    //    QT[1].addEntryBinary(2, 2);
    //    QT[2].addEntryBinary(0, 1);
    //    QT[2].addEntryBinary(1, 2);
    //    QT[2].addEntryBinary(2, 4);
    //    DVectorD LT(3);
    //    LT.setZero();
    //    LT[0] = 8;
    //    LT[1] = 3;
    //    LT[2] = 3;
    //    SparseMatrixD HC(2);
    //    HC[0].addEntryBinary(0, 1);
    //    HC[0].addEntryBinary(2, 1);
    //    HC[1].addEntryBinary(1, 1);
    //    HC[1].addEntryBinary(2, 1);
    //    DVectorD HV(2);
    //    HV.setZero();
    //    HV[0] = 3;
    //    SparseMatrixD SC(1);
    //    SC[0].addEntryBinary(0, 0);
    //    SC[0].addEntryBinary(1, 100);
    //    SC[0].addEntryBinary(2, 0);
    //    DVectorD SV(1);
    //    SV[0] = 100;

    //    rangeSpace_->Solve(
    //        QT, LT,
    //        SC, SV,
    //        HC, HV,
    //        &result,
    //        bRefactor_,
    //        true
    //        );

    //    directNullSpace_->Solve(
    //        QT, LT,
    //        SC, SV,
    //        HC, HV,
    //        &result,
    //        bRefactor_,
    //        true
    //        );
    //}

    //rangeSpace_->Solve(
    //    regularizer_->getQT(), regularizer_->getLT(),
    //    softConstraints_, softConstValues_,
    //    hardConstraints_, hardConstValues_,
    //    &result,
    //    bRefactor_,
    //    false
    //    );

    DVectorD result;
    directNullSpace_->Solve(
        regularizer_->getQT(), regularizer_->getLT(),
        softConstraints_, softConstValues_,
        hardConstraints_, hardConstValues_,
        &result,
        bRefactor_,
        true
        );

    DistributeDeformPos(result);

    bRefactor_ = false;
}

void SymmSpaceSolver::SolveDirectNull(void)
{
    hardConstraints_.appendRows(fixedConstraints_);
    hardConstValues_.appendInPlace(fixedConstValues_);

    DVectorD result;
    nullSpace_->Solve(
        regularizer_->getQT(), regularizer_->getLT(),
        softConstraints_, softConstValues_,
        hardConstraints_, hardConstValues_,
        &result,
        bRefactor_,
        false
        );

    DistributeDeformPos(result);

    bRefactor_ = false;
}

void SymmSpaceSolver::SolveCG(void)
{
    //const Clapack::integer varDim = regularizer_->getQT().getNumRows();
    //const Clapack::integer lDim = regularizer_->getQT().getNumRows() + softConstraints_.getNumRows() +
    //    hardConstraints_.getNumRows() + fixedConstraints_.getNumRows();
    //const Clapack::integer rDim = regularizer_->getLT().getDim() + softConstValues_.getDim() +
    //    hardConstValues_.getDim() + fixedConstValues_.getDim();
    //if (lDim != rDim) {
    //    error("SymmSpaceSolver::SolveDirect - system dimension mis-match");
    //    return;
    //}

    //SparseMatrixD LHS(regularizer_->getQT());
    //LibraryAdaptor::SymmetricSparseMatrixBinder(&LHS, softConstraints_);
    //LibraryAdaptor::SymmetricSparseMatrixBinder(&LHS, hardConstraints_);
    //LibraryAdaptor::SymmetricSparseMatrixBinder(&LHS, fixedConstraints_);

    //DVectorD RHS(regularizer_->getLT());
    //RHS.appendInPlace(softConstValues_);
    //RHS.appendInPlace(hardConstValues_);
    //RHS.appendInPlace(fixedConstValues_);

    //DVectorD result = CollectDeformPos();
    //DVectorD var(rDim);
    //var.setZero();
    //for (unsigned ii = 0; ii < result.size(); ++ii) {
    //    var[ii] = result[ii];
    //}

    //card32 cgNumIterations = 10000;
    //conjugateGradientsSolveSymmetrized<float64>(
    //    LHS,
    //    var,
    //    RHS,
    //    cgNumIterations,
    //    0.00001f,
    //    false
    //    );
    //debugOutput << "CG: " << cgNumIterations << " iterations.\n";

    //result = var.subset(0, result.size());
    //DistributeDeformPos(result);
}

void SymmSpaceSolver::solve()
{
    switch (solverType_) {
    case DIRECT:
    case DIRECT_SYMMETRIC:
    case DIRECT_NULL:
        if (bUseNullSpace_) SolveDirectNull();
        else SolveDirect();
        break;
    case CG:
        SolveCG();
        break;
    }
}

void SymmSpaceSolver::Update(const bool& updateRegularizer)
{
    if (updateRegularizer) regularizer_->Update(latestPositions, currentPositions);

    if (bUseNullSpace_) meshInterpolater_->Update(nullSpace_->getCoord());
    else meshInterpolater_->Update(directNullSpace_->getSample());

    std::copy(currentPositions.begin(), currentPositions.end(), latestPositions.begin());
}

void SymmSpaceSolver::Interpolate(UICTM* resultTM)
{
    meshInterpolater_->Interpolate(resultTM);
}

LapSymmSpaceSolver::LapSymmSpaceSolver()
{
    Clear();
}

void LapSymmSpaceSolver::Clear()
{
    SymmSpaceSolver::Clear();
}

DVectorD LapSymmSpaceSolver::CollectDeformPos()
{
    const unsigned& num_points = displacementVec.size();
    DVectorD result;
    result.setDim(num_points * 3);

    for (unsigned bi = 0; bi < num_points; ++bi) {
        for (unsigned d = 0; d < 3; ++d) {
            result[3*bi+d] = displacementVec[bi][d];
        }
    }

    return result;
}

void LapSymmSpaceSolver::DistributeDeformPos(const DVectorD& result)
{
    const unsigned& num_points = displacementVec.size();
    if (num_points * 3 != result.getDim()) {
        error("SymmSpaceSolver::DistributeDeformPos() - Size mismatch.");
    }

    for (unsigned bi = 0; bi < num_points; ++bi) {
        for (unsigned d = 0; d < 3; ++d) {
            displacementVec[bi][d] = result[3*bi+d];
        }
        latestPositions[bi] = currentPositions[bi];
        currentPositions[bi] = latestPositions[bi] + displacementVec[bi];
    }

    {
        //DR_FRAME = 0;
        //debugRenderer->clearRenderJob_AllFrames("sample_solution_");
        //debugRenderer->beginRenderJob_OneFrame("sample_solution_", DR_FRAME++);
        //for (unsigned pi = 0; pi < num_points; ++pi) {
        //    debugRenderer->addLine(
        //        latestPositions[pi], currentPositions[pi],
        //        makeVector3f(1, 0, 0),
        //        makeVector3f(0, 0, 1),
        //        1);
        //}
        //debugRenderer->endRenderJob();
    }
}

// fixed, no need to update during iterations
void SymmSpaceSolver::addSymmetryConstraint(
    const unsigned& source,
    const unsigned& target,
    const Matrix4f& transformation,
    const float& weight
    )
{
    const unsigned& num_var = currentPositions.size();
    if (source >= num_var || target >= num_var) {
        error("SymmSpaceSolver::addSymmetryConstraint - out of bound");
    }

    const Matrix3f rotation = shrink4To3(transformation);
    //const Vector3f translation = shrink4To3(transformation[3]);
    const Matrix3f rotation_t = rotation.transpose();

    SparseVectorD constrVec;
    for (unsigned d = 0; d < 3; ++d) {
        for (unsigned r = 0; r < 3; ++r) {
            constrVec.addEntryBinary(3 * target + r, -2 * weight * rotation_t[r][d]);
        }
        constrVec.addEntryBinary(3 * source + d, 2 * weight);

        symmConstraints_.addRow(constrVec);
        symmConstValues_.appendInPlace(0);
    }

    const bool debug_show = true;
    if (debug_show) {
        const Vector3f& sourcePos = currentPositions[source];
        Vector3f sourcePos_t = transformVector3f(transformation, sourcePos);
        const Vector3f& targetPos = currentPositions[target];
        debugRenderer->addLine(
            sourcePos, sourcePos_t,
            makeVector3f(1, 0, 0),
            makeVector3f(0, 0, 1),
            2);
        debugRenderer->addLine(
            targetPos, sourcePos_t,
            makeVector3f(1, 1, 0),
            makeVector3f(1, 1, 0),
            2);
        debugRenderer->addPoint(
            targetPos, makeVector3f(1, 1, 0)
            );
    }
}

// fixed, no need to update during iterations
void SymmSpaceSolver::addFixedConstraint(
    const unsigned& index,
    const float& weight
    )
{
    const unsigned& num_var = currentPositions.size();
    if (index >= num_var) {
        error("SymmSpaceSolver::addFixedConstraint - out of bound");
    }

    SparseVectorD constrVec;
    for (unsigned d = 0; d < 3; ++d) {
        constrVec.addEntryBinary(3 * index + d, 2 * weight);
        /* hard constraints */
        fixedConstraints_.addRow(constrVec);
        fixedConstValues_.appendInPlace(0);
        /* soft constraints *
        softConstraints_.addRow(constrVec);
        softConstValues_.appendInPlace(0);
        /**/
    }

    const bool debug_show = true;
    if (debug_show) {
        const Vector3f& sourcePos = initialPositions[index];
        const Vector3f& targetPos = currentPositions[index];
        debugRenderer->addLine(
            sourcePos, targetPos,
            makeVector3f(1, 0, 0),
            makeVector3f(0, 0, 1),
            2);
        debugRenderer->addPoint(
            targetPos, makeVector3f(1, 1, 0)
            );
    }
}

// have to be updated during iterations
void SymmSpaceSolver::addSoftConstraint(
    const unsigned& index,
    const Vector3f& toPos,
    const float& weight
    )
{
    const unsigned& num_var = currentPositions.size();
    if (index >= num_var) {
        error("SymmSpaceSolver::addHandleConstraint - out of bound");
    }

    const Vector3f& toDelta = toPos - latestPositions[index];
    for (unsigned d = 0; d < 3; ++d) {
        SparseVectorD constrVec;
        constrVec.setEntryBinary(3 * index + d, 2 * weight);
        softConstraints_.addRow(constrVec);
        softConstValues_.appendInPlace(2 * weight * toDelta[d]);
    }
}

void SymmSpaceSolver::addSoftConstraint(
    const unsigned& index,
    const Matrix4f& transformation,
    const float& weight
    )
{
    const unsigned& num_var = currentPositions.size();
    if (index >= num_var) {
        error("SymmSpaceSolver::addHandleConstraint - out of bound");
    }

    const Vector3f pos = latestPositions[index];
    const Vector3f toPos = transformVector3f(transformation, pos);
    const Vector3f toDelta = toPos - pos;

    for (unsigned d = 0; d < 3; ++d) {
        SparseVectorD constrVec;
        constrVec.setEntryBinary(3 * index + d, 2 * weight);
        softConstraints_.addRow(constrVec);
        softConstValues_.appendInPlace(2 * weight * toDelta[d]);
    }

    const bool debug_show = true;
    if (debug_show) {
        debugRenderer->addLine(
            pos, toPos,
            makeVector3f(0, 1, 0),
            makeVector3f(0, 0, 1),
            2);
        debugRenderer->addPoint(
            pos, makeVector3f(1, 0, 0)
            );
    }
}

// have to be updated during iterations
void SymmSpaceSolver::addHardConstraint(
    const unsigned& index,
    const Matrix4f& transformation,
    const float& weight
    )
{
    const unsigned& num_var = currentPositions.size();
    if (index >= num_var) {
        error("SymmSpaceSolver::addHandleConstraint - out of bound");
    }

    const Vector3f pos = latestPositions[index];
    const Vector3f toPos = transformVector3f(transformation, pos);
    const Vector3f toDelta = toPos - pos;

    for (unsigned d = 0; d < 3; ++d) {
        SparseVectorD constrVec;
        constrVec.setEntryBinary(3 * index + d, 2 * weight);
        hardConstraints_.addRow(constrVec);
        hardConstValues_.appendInPlace(2 * weight * toDelta[d]);
    }

    const bool debug_show = true;
    if (debug_show) {
        debugRenderer->addLine(
            pos, toPos,
            makeVector3f(0, 1, 0),
            makeVector3f(0, 0, 1),
            2);
        debugRenderer->addPoint(
            pos, makeVector3f(1, 0, 0)
            );
    }
}

// fixed, no need to update during iterations
void SymmSpaceSolver::addColinearity(
    const unsigned& A0, const unsigned& A1,
    const unsigned& B0, const unsigned& B1,
    const float& weight
    )
{
    const unsigned& num_var = currentPositions.size();
    if (A0 >= num_var || A1 >= num_var || B0 >= num_var || B1 >= num_var) {
        error("SymmSpaceSolver::addColinearity - out of bound");
    }

    // B - M = M - A <==> 2 * M - B - A = 0
    // A1 - A0 = B1 - B0 <==> A1 + B0 - A0 - B1 = 0
    SparseVectorD constrVec;
    for (unsigned d = 0; d < 3; ++d) {
        constrVec.addEntryBinary(3 * A1 + d, 2 * weight);
        constrVec.addEntryBinary(3 * B0 + d, 2 * weight);
        constrVec.addEntryBinary(3 * A0 + d, -2 * weight);
        constrVec.addEntryBinary(3 * B1 + d, -2 * weight);
        /* hard constraints */
        fixedConstraints_.addRow(constrVec);
        fixedConstValues_.appendInPlace(0);
        /* soft constraints *
        softConstraints_.addRow(constrVec);
        softConstValues_.appendInPlace(0);
        /**/
    }

    const bool debug_show = true;
    if (debug_show) {
        const Vector3f& pos_a0 = currentPositions[A1];
        const Vector3f& pos_a1 = (pos_a0 + currentPositions[A0]) / 2;
        const Vector3f& pos_b0 = currentPositions[B0];
        const Vector3f& pos_b1 = (pos_b0 + currentPositions[B1]) / 2;
        debugRenderer->addLine(
            pos_a0, pos_a1,
            makeVector3f(1, 0, 0),
            makeVector3f(0, 0, 1),
            2);
        debugRenderer->addLine(
            pos_b0, pos_b1,
            makeVector3f(1, 0, 0),
            makeVector3f(0, 0, 1),
            2);
        debugRenderer->addPoint(
            pos_a0, makeVector3f(1, 1, 0)
            );
        debugRenderer->addPoint(
            pos_b0, makeVector3f(1, 1, 0)
            );
    }
}

// have to be updated during iterations
void SymmSpaceSolver::addCoplanarity(
    const unsigned& S,
    const unsigned& T,
    const Vector3f& N,
    const float& weight
    )
{
    const unsigned& num_var = currentPositions.size();
    if (S >= num_var || T >= num_var) {
        error("SymmSpaceSolver::addCoplanarity - out of bound");
    }

    // (S - S') * N = (T - T') * N <==> (S - T) * N = (S' - T') * N
    SparseVectorD constrVec;
    for (unsigned d = 0; d < 3; ++d) {
        constrVec.addEntryBinary(3 * S + d, -2 * weight * N[d]);
        constrVec.addEntryBinary(3 * T + d, 2 * weight * N[d]);
        /* hard constraints */
        hardConstraints_.addRow(constrVec);
        hardConstValues_.appendInPlace(0);
        /* soft constraints *
        softConstraints_.addRow(constrVec);
        softConstValues_.appendInPlace(0);
        /**/
    }

    const bool debug_show = true;
    if (debug_show) {
        const Vector3f& sourcePos = currentPositions[S];
        const Vector3f& targetPos = currentPositions[T];
        const Vector3f sourceVec = ((displacementVec[S]) * N) * N;
        const Vector3f targetVec = ((displacementVec[T]) * N) * N;
        debugRenderer->addLine(
            sourcePos, sourcePos + N,
            makeVector3f(1, 0, 0),
            makeVector3f(0, 0, 1),
            2);
        debugRenderer->addLine(
            targetPos, targetPos + N,
            makeVector3f(1, 0, 0),
            makeVector3f(0, 0, 1),
            2);
        debugRenderer->addLine(
            sourcePos, sourcePos + sourceVec,
            makeVector3f(1, 1, 0),
            makeVector3f(1, 1, 0),
            2);
        debugRenderer->addLine(
            targetPos, targetPos + targetVec,
            makeVector3f(1, 1, 0),
            makeVector3f(1, 1, 0),
            2);
        debugRenderer->addPoint(
            sourcePos, makeVector3f(1, 1, 0)
            );
    }
}
