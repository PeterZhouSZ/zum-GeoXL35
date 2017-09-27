//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "SPSolver/MeshInterpolater.h"
//----------------------------------------------------------------------
#include "Util\numerical\LibraryAdaptor.h"
//----------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

MeshInterpolaterN2M::MeshInterpolaterN2M(
    const UICPC* samplePC, const UICTM* inputTM,
    const SparseMatrixD& nullSpace, const SparseMatrixD& liftWeight
    )
{
    if (!samplePC || !inputTM) {
        return;
    }

    // initialize points
    const unsigned& num_verts = inputTM->getNumPoints();
    const unsigned& num_orbit_3 = nullSpace.getRows();
    nsDelta.setDim(num_orbit_3);
    nsDelta.setZero();
    meshDelta.setDim(num_verts * 3);
    meshDelta.setZero();

    liftWeightProj = LibraryAdaptor::SparseMatrixMultiplicationTransposed(liftWeight, nullSpace);
    //liftWeightProj.removeNearZeros(1e-6);

#ifdef USE_CUDA
    CudaAdaptor::SparseMatrix2CudaCSR(
        liftWeightProj,
        num_orbit_3,
        &csrInterWeight
        );
#endif
}

void MeshInterpolaterN2M::Update(const DVectorD& delta)
{
    if (nsDelta.getDim() != delta.getDim())
    {
        error("MeshInterpolaterN2M::Update - different size");
        return;
    }
    nsDelta += delta;
}

void MeshInterpolaterN2M::Interpolate(UICTM* resultTM)
{
    const bool debug_output = false;
    DR_FRAME = 0;
    const std::string debug_id = "interpolation_";
    debugRenderer->clearRenderJob_AllFrames(debug_id);

    PointSet& resultPS = *resultTM->getPointSet();
    AAT resultPosAAT = resultPS.getAAT("position", 3, VAD::DATA_FORMAT_FLOAT32);
    const unsigned& num_verts = resultPS.getNumEntries();
    if ((meshDelta.getDim() / 3) != num_verts)
    {
        std::ostringstream ss;
        ss << "MeshInterpolaterS2M::Interpolate - different size: "
            << (meshDelta.getDim() / 3) << ", " << num_verts;
        error(ss.str());
        return;
    }

#ifdef USE_CUDA
    if (CudaAdaptor::csrmv_01(csrInterWeight, nsDelta.data(), meshDelta.data())) return;
#else
    meshDelta = liftWeightProj * nsDelta;
#endif

    for (unsigned vi = 0; vi < num_verts; ++vi) {
        if (debug_output) debugRenderer->beginRenderJob_OneFrame(debug_id, DR_FRAME++);
        const Vector3f& posResult = resultPS.get3f(vi, resultPosAAT);
        const Vector3f delta = makeVector3f(meshDelta[3 * vi + 0], meshDelta[3 * vi + 1], meshDelta[3 * vi + 2]);
        resultPS.set3f(vi, resultPosAAT, posResult + delta);
        if (debug_output)
        {
            debugRenderer->addLine(
                posResult, posResult + delta,
                makeVector3f(1, 0, 0),
                makeVector3f(0, 0, 1),
                2);
        }
        if (debug_output) debugRenderer->endRenderJob();
    }

    //const unsigned& num_verts = resultPS.getNumEntries();
    //for (unsigned vi = 0; vi < num_verts; ++vi) {
    //    if (debug_output) debugRenderer->beginRenderJob_OneFrame(debug_id, DR_FRAME++);
    //    const Vector3f& posResult = resultPS.get3f(vi, resultPosAAT);
    //    const Vector3f& delta = evaluate(vi, debug_output);
    //    resultPS.set3f(vi, resultPosAAT, posResult + delta);
    //    if (debug_output)
    //    {
    //        debugRenderer->addLine(
    //            posResult, posResult + delta,
    //            makeVector3f(1, 0, 0),
    //            makeVector3f(0, 0, 1),
    //            2);
    //    }
    //    if (debug_output) debugRenderer->endRenderJob();
    //}

    resultTM->deleteDelOnWriteAttachments();
    nsDelta.setZero();
    meshDelta.setZero();
}

MeshInterpolaterS2M::MeshInterpolaterS2M(
    UICPC* samplePC, const UICTM* inputTM,
    const SparseMatrixD& liftWeight
    )
{
    if (!samplePC || !inputTM) {
        return;
    }

    // initialize points
    const unsigned& num_points = samplePC->getNumPoints();
    const unsigned& num_verts = inputTM->getNumPoints();
    sampleDelta.setDim(num_points * 3);
    sampleDelta.setZero();
    meshDelta.setDim(num_verts * 3);
    meshDelta.setZero();

    liftWeightProj = liftWeight;
    //liftWeightProj.removeNearZeros(1e-6);

#ifdef USE_CUDA
    CudaAdaptor::SparseMatrix2CudaCSR(
        liftWeightProj,
        num_points * 3,
        &csrInterWeight
        );
#endif
}

void MeshInterpolaterS2M::Update(const DVectorD& delta)
{
    if (sampleDelta.getDim() != delta.getDim())
    {
        std::ostringstream ss;
        ss << "MeshInterpolaterS2M::Update - different size: "
            << sampleDelta.getDim() << ", " << delta.getDim();
        error(ss.str());
        return;
    }
    sampleDelta += delta;

    //const unsigned& num_points = delta.size() * 3;
    //if (sampleDelta.getDim() != num_points)
    //{
    //    error("MeshInterpolater::Update - different size");
    //    return;
    //}
    //for (unsigned pi = 0; pi < num_points; ++pi) {
    //    for (unsigned d = 0; d < 3; ++d) {
    //        sampleDelta[pi * 3 + d] = delta[pi][d];
    //    }
    //}
}

void MeshInterpolaterS2M::Interpolate(UICTM* resultTM)
{
    const bool debug_output = false;
    DR_FRAME = 0;
    const std::string debug_id = "interpolation_";
    debugRenderer->clearRenderJob_AllFrames(debug_id);

    PointSet& resultPS = *resultTM->getPointSet();
    AAT resultPosAAT = resultPS.getAAT("position", 3, VAD::DATA_FORMAT_FLOAT32);
    const unsigned& num_verts = resultPS.getNumEntries();
    if ((meshDelta.getDim() / 3) != num_verts)
    {
        std::ostringstream ss;
        ss << "MeshInterpolaterS2M::Interpolate - different size: "
            << (meshDelta.getDim() / 3) << ", " << num_verts;
        error(ss.str());
        return;
    }

#ifdef USE_CUDA
    if (CudaAdaptor::csrmv_01(csrInterWeight, sampleDelta.data(), meshDelta.data())) return;
#else
    meshDelta = liftWeightProj * sampleDelta;
#endif

    for (unsigned vi = 0; vi < num_verts; ++vi) {
        if (debug_output) debugRenderer->beginRenderJob_OneFrame(debug_id, DR_FRAME++);
        const Vector3f& posResult = resultPS.get3f(vi, resultPosAAT);
        const Vector3f delta = makeVector3f(meshDelta[3 * vi + 0], meshDelta[3 * vi + 1], meshDelta[3 * vi + 2]);
        resultPS.set3f(vi, resultPosAAT, posResult + delta);
        if (debug_output)
        {
            debugRenderer->addLine(
                posResult, posResult + delta,
                makeVector3f(1, 0, 0),
                makeVector3f(0, 0, 1),
                2);
        }
        if (debug_output) debugRenderer->endRenderJob();
    }

    resultTM->deleteDelOnWriteAttachments();
    sampleDelta.setZero();
    meshDelta.setZero();
}

//const Vector3f MeshInterpolaterS2M::evaluate(const unsigned& vi, const bool& debug_output) const
//{
//    Vector3f weightedDelta = NULL_VECTOR3F;
//    const SparseVectorD& weightVector = interWeightMatrix[vi];
//
//    SparseVectorD::EIteratorConst eit = weightVector.begin();
//    SparseVectorD::EIteratorConst eit_end = weightVector.end();
//    while (eit != eit_end) {
//        const mpcard& index = eit->index;
//        const float& weight = eit->value;
//        const Vector3f& deformDelta = displacementVec[index];
//        weightedDelta += weight * deformDelta;
//        ++eit;
//
//        if (debug_output)
//        {
//            //const Vector3f& posInit = initialPositions[index];
//            const Vector3f& posCurr = currentPositions[index];
//            /**
//            debugRenderer->addLine(
//            posCurr, posCurr + weight * deformDelta,
//            makeVector3f(1, 1, 0),
//            makeVector3f(0, 1, 1),
//            1);
//            /**
//            debugRenderer->addLine( // deformation
//            posCurr, posCurr + deformDelta,
//            makeVector3f(1, 1, 0),
//            makeVector3f(0, 1, 1),
//            1);
//            /**/
//            debugRenderer->addLine( // weighting
//                posCurr, posCurr + weight * normalize(deformDelta),
//                makeVector3f(1, 1, 0),
//                makeVector3f(0, 1, 1),
//                1);
//            /**/
//        }
//    }
//
//    return weightedDelta;
//}
