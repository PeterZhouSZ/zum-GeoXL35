//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "SPSolver/LaplacianRegularizer.h"
#include "Util\numerical\EigenAdaptor.h"
#include "Util\numerical\CudaAdaptor.h"
//----------------------------------------------------------------------
#include "InCorePCTopologyGraph.h"
#include "basics\DeformationTools.h"
#include "GeometricTools.h"
//----------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

LaplacianRegularizer::LaplacianRegularizer(const float& weight, const bool& inertia, const unsigned& D) :
    regularizerWeight_(weight),
    bUseInertia_(inertia),
    Dim(D)
{

}

// if the result looks weird, swap source and target
Matrix3f EstimateRigidAlignmentRotation(
    const std::vector<Vector3f>& points0, const Vector3f& centroid0,
    const std::vector<Vector3f>& points1, const Vector3f& centroid1
    )
{
    const unsigned num_points = points1.size();

    ////// compute centroids
    ////Vector3f centroid0 = NULL_VECTOR3F, centroid1 = NULL_VECTOR3F;
    ////for (unsigned pi = 0; pi < num_points; ++pi) {
    ////    centroid0 += points0[pi];
    ////    centroid1 += points1[pi];
    ////}
    ////centroid0 /= num_points;
    ////centroid1 /= num_points;

    //// calculate cross-covariance matrix
    //Matrix3d cov = IDENTITY3D * 0.f;
    //for (unsigned pi = 0; pi < num_points; ++pi) {
    //    for (unsigned ii = 0; ii < 3; ++ii) {
    //        for (unsigned jj = 0; jj < 3; ++jj) {
    //            cov[ii][jj] +=
    //                (points0[pi][ii] - centroid0[ii]) *
    //                (points1[pi][jj] - centroid1[jj]);
    //        }
    //    }
    //}

    //// SVD
    //std::vector<double> cov_flat(9);
    //unsigned cnt = 0;
    //for (unsigned cc = 0; cc < 3; ++cc) {
    //    for (unsigned rr = 0; rr < 3; ++rr) {
    //        cov_flat[cnt++] = cov[cc][rr];
    //    }
    //}
    //Matrix3f Ut, V;
    //{
    //    std::vector<double> U, S, V_T;
    //    //Clapack::ClapackAdaptor::dgesvd(&cov_flat, &U, &S, &V_T, 3, 3);
    //    if (0 != Clapack::ClapackAdaptor::dgesvd(&cov_flat, &U, &S, &V_T, 3, 3)) return IDENTITY3F;
    //    //if (S[0]) // maybe check the geometric shape?
    //    for (unsigned cc = 0; cc < 3; ++cc) {
    //        for (unsigned rr = 0; rr < 3; ++rr) {
    //            Ut[cc][rr] = U[3 * rr + cc];
    //        }
    //    }
    //    for (unsigned cc = 0; cc < 3; ++cc) {
    //        for (unsigned rr = 0; rr < 3; ++rr) {
    //            V[cc][rr] = V_T[3 * rr + cc];
    //        }
    //    }
    //}

    //// compute rotation
    //Matrix3f I = V * Ut;
    //float det =
    //    I[0][0] * (I[1][1] * I[2][2] - I[1][2] * I[2][1]) -
    //    I[0][1] * (I[1][0] * I[2][2] - I[1][2] * I[2][0]) +
    //    I[0][2] * (I[1][0] * I[2][1] - I[1][1] * I[2][0]);
    //I = IDENTITY3F;
    //I[2][2] = (det > 0) - (det < 0);
    //return V * I * Ut;

    Eigen::Matrix3f COV(Eigen::Matrix3f::Zero());
    for (unsigned pi = 0; pi < num_points; ++pi) {
        for (unsigned ii = 0; ii < 3; ++ii) {
            for (unsigned jj = 0; jj < 3; ++jj) {
                COV(ii, jj) +=
                    (points0[pi][ii] - centroid0[ii]) *
                    (points1[pi][jj] - centroid1[jj]);
            }
        }
    }
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(COV, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();
    float det = (V * U.transpose()).determinant();
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    if (det < 0) I(2, 2) = -1;
    return EigenAdaptor::FromEigen(static_cast<Eigen::Matrix3f>(V * I * U.transpose()));
}

//void ComputeCotanWeights(
//    const InCorePCTopologyGraph* tpy_,
//    const UICPC* samplePC,
//    const unsigned& pi,
//    std::vector<float>* p_cotan_weight
//    )
//{
//    std::vector<float>& cotan_weight = *p_cotan_weight;
//
//    const unsigned& numNV = tpy_->getNumAdjacentVertices(pi);
//    cotan_weight.resize(numNV, 1.f);
//    if (3 > numNV) return;
//
//    PointSet const* samPS = samplePC->getPointSet();
//    AAT samPosAAT = samPS->getAAT("position", 3, VAD::DATA_FORMAT_FLOAT32);
//
//    const Vector3f& pos_pi = samPS->get3f(pi, samPosAAT);
//    for (unsigned ni = 0; ni < numNV; ++ni) {
//        const unsigned pi_ni = tpy_->getAdjacentVertex(pi, ni);
//        const unsigned ni_l = (numNV + ni - 1) % numNV;
//        const unsigned pi_ni_l = tpy_->getAdjacentVertex(pi, ni_l);
//        const unsigned ni_r = (numNV + ni + 1) % numNV;
//        const unsigned pi_ni_r = tpy_->getAdjacentVertex(pi, ni_r);
//        {
//            const Vector3f& pos_pi_ni = samPS->get3f(pi_ni, samPosAAT);
//            const Vector3f& pos_pi_ni_l = samPS->get3f(pi_ni_l, samPosAAT);
//            const Vector3f& pos_pi_ni_r = samPS->get3f(pi_ni_r, samPosAAT);
//
//            const Vector3f pi_vec_pi_ni = pos_pi - pos_pi_ni;
//            const Vector3f pi_vec_pi_ni_l = pos_pi - pos_pi_ni_l;
//            const Vector3f pi_ni_vec_pi_ni_l = pos_pi_ni - pos_pi_ni_l;
//            const Vector3f pi_vec_pi_ni_r = pos_pi - pos_pi_ni_r;
//            const Vector3f pi_ni_vec_pi_ni_r = pos_pi_ni - pos_pi_ni_r;
//
//            const float cross_l = norm(pi_vec_pi_ni_l.crossProduct(pi_ni_vec_pi_ni_l));
//            const float cotan_l = (abs(cross_l) < 0.01f) ?
//                0 : (pi_vec_pi_ni_l * pi_ni_vec_pi_ni_l) / cross_l;
//            const float cross_r = norm(pi_vec_pi_ni_r.crossProduct(pi_ni_vec_pi_ni_r));
//            const float cotan_r = (abs(cross_r) < 0.01f) ?
//                0 : (pi_vec_pi_ni_r * pi_ni_vec_pi_ni_r) / cross_r;
//            cotan_weight[ni] = (cotan_l + cotan_r) / 2;
//        }
//    }
//}

LaplacianRegularizerSS::LaplacianRegularizerSS(
    const UICPC* samplePC, const UICTM* inputTM,
    const SparseMatrixD& liftWeight,
    const float& weight,
    const bool& inertia,
    const unsigned& D) :
LaplacianRegularizer(weight, inertia, D)
{
    const unsigned& num_points = samplePC->getNumPoints();
    const unsigned& num_verts = inputTM->getNumPoints();
    TrimeshStatic tsMesh(inputTM, false);

    const unsigned& num_var = Dim * num_points;
    QT.setRows(num_var);
    QT.setZero();
    LT.setDim(num_var);
    LT.setZero();

    per_point_rotation_.resize(num_points, IDENTITY3F);
    //cotan_weight_.resize(num_points);
    //for (unsigned pi = 0; pi < num_points; ++pi) {
    //    ComputeCotanWeights(tpy_, samplePC,
    //        pi, &cotan_weight_[pi]);
    //}

    BuildRegularizerDelta(num_points, tsMesh, liftWeight);
    {
        //DR_FRAME = 0;
        //debugRenderer->clearRenderJob_AllFrames("interpolation_");
        //const PointSet& samplePS = *samplePC->getPointSet();
        //const AAT& samPosAAT = samplePS.getAAT("position");
        //for (unsigned pi = 0; pi < num_points; ++pi) {
        //    debugRenderer->beginRenderJob_OneFrame("interpolation_", DR_FRAME++);
        //    const unsigned& numNV = samTpy[pi].size();
        //    for (unsigned ni = 0; ni < numNV; ++ni) {
        //        const unsigned& pi_ni = samTpy[pi][ni];
        //        debugRenderer->addLine(
        //            samplePS.get3f(pi, samPosAAT), samplePS.get3f(pi_ni, samPosAAT),
        //            makeVector3f(1, 0, 0),
        //            makeVector3f(0, 0, 1),
        //            1);
        //    }
        //    debugRenderer->endRenderJob();
        //}
    }
    {
        //DR_FRAME = 0;
        //debugRenderer->clearRenderJob_AllFrames("interpolation_");
        //const unsigned& num_verts = tsMesh.GetNumVerts();
        //for (unsigned vi = 0; vi < num_verts; ++vi) {
        //    debugRenderer->beginRenderJob_OneFrame("interpolation_", DR_FRAME++);
        //    const std::vector<mpcard>& nverts = tsMesh.GetVert1RingVerts(vi);
        //    const unsigned& numNV = nverts.size();
        //    for (unsigned ni = 0; ni < numNV; ++ni) {
        //        const unsigned& vi_ni = nverts[ni];
        //        debugRenderer->addLine(
        //            tsMesh.GetVertPosition(vi), tsMesh.GetVertPosition(vi_ni),
        //            makeVector3f(1, 0, 0),
        //            makeVector3f(0, 0, 1),
        //            1);
        //    }
        //    debugRenderer->endRenderJob();
        //}
    }
}

void LaplacianRegularizerSS::BuildRegularizerDelta(
    const unsigned& num_points, const TrimeshStatic& tsMesh,
    const SparseMatrixD& liftWeight
    )
{
    const unsigned& dim_sample = Dim * num_points;
    const unsigned& num_verts = tsMesh.GetNumVerts();
    SparseMatrixD meshTpy; // record mesh connectivity
    meshTpy.setRows(num_verts);
    for (unsigned vi = 0; vi < num_verts; ++vi) {
        const std::vector<mpcard>& nverts = tsMesh.GetVert1RingVerts(vi);
        const unsigned& numNV = nverts.size();
        for (unsigned ni = 0; ni < numNV; ++ni) {
            const unsigned& vi_ni = nverts[ni];
            meshTpy[vi].addEntryBinary(vi_ni, 1);
        }
    }

    // add diagonal entries
    if (bUseInertia_) {
        for (unsigned bi = 0; bi < dim_sample; ++bi) {
            QT[bi].addEntryBinary(bi, 2 * regularizerWeight_);
        }
    }

#ifdef USE_CUDA
    CudaAdaptor::sparseAtQA(
        meshTpy, liftWeight,
        num_points, &meshTpy
        );
#else
    meshTpy = LibraryAdaptor::SparseMatrixQuadratic(meshTpy, liftWeight, num_points);
    meshTpy.removeNearZeros(std::numeric_limits<double>::epsilon());
#endif
    samTpy.resize(num_points);
    for (unsigned vi = 0; vi < num_points; ++vi) {
        const SparseVectorD& weightVector = meshTpy[vi];
        SparseVectorD::EIteratorConst eit = weightVector.begin();
        SparseVectorD::EIteratorConst eit_end = weightVector.end();
        while (eit != eit_end) {
            samTpy[vi].push_back(eit->index);
            ++eit;
        }
    }

    for (unsigned pi = 0; pi < num_points; ++pi) {
        const unsigned& numNV = samTpy[pi].size();
        for (unsigned ni = 0; ni < numNV; ++ni) {
            const unsigned& pi_ni = samTpy[pi][ni];
            for (unsigned d = 0; d < Dim; ++d) {
                QT[Dim * pi + d].addEntryBinary(Dim * pi + d,
                    2 * regularizerWeight_ /** cotan_weight[ni]*/);
                QT[Dim * pi + d].addEntryBinary(Dim * pi_ni + d,
                    -1 * regularizerWeight_ /** cotan_weight[ni]*/);
                QT[Dim * pi_ni + d].addEntryBinary(Dim * pi + d,
                    -1 * regularizerWeight_ /** cotan_weight[ni]*/);
            }
        }
    }
}

void LaplacianRegularizerSS::ComputePerPointRotations(
    const std::vector<Vector3f>& latestPositions,
    const std::vector<Vector3f>& currentPositions
    )
{
    const unsigned num_points = latestPositions.size();
    std::vector<Vector3f> corrForRotInit, corrForRotCurr;
    for (unsigned pi = 0; pi < num_points; ++pi) {
        const unsigned& numNV = samTpy[pi].size();
        corrForRotInit.clear();
        corrForRotCurr.clear();
        std::set<unsigned> indiceSet; // points may have same indice
        for (unsigned ni = 0; ni < numNV; ++ni) {
            const unsigned& pi_ni = samTpy[pi][ni];

            corrForRotInit.push_back(latestPositions[pi_ni]);
            corrForRotCurr.push_back(currentPositions[pi_ni]);
        }

        if (corrForRotInit.size() >= 4) {
            per_point_rotation_[pi] = EstimateRigidAlignmentRotation(
                corrForRotCurr, currentPositions[pi],
                corrForRotInit, latestPositions[pi]
            );
        } else {
            per_point_rotation_[pi] = IDENTITY3F;
            //debugOutput << "could not estimate rotation for vertex " << pi << "\n";
        }
    }
}

void LaplacianRegularizerSS::UpdateRotation(
    const std::vector<Vector3f>& currentPositions
    )
{
    LT.setZero();
    const unsigned& num_points = currentPositions.size();
    for (unsigned pi = 0; pi < num_points; ++pi) {
        const Vector3f& pos_pi = currentPositions[pi];
        const unsigned& numNV = samTpy[pi].size();

        //const std::vector<float>& cotan_weight = cotan_weight_[pi];
        for (unsigned ni = 0; ni < numNV; ++ni) {
            const unsigned& pi_ni = samTpy[pi][ni];
            const Vector3f& pos_pi_ni = currentPositions[pi_ni];

            const Vector3f pi_vec_pi_ni = pos_pi - pos_pi_ni;
            const Vector3f pi_vec_pi_ni_rot = (per_point_rotation_[pi] - IDENTITY3F) * pi_vec_pi_ni;

            for (unsigned d = 0; d < Dim; ++d) {
                LT[Dim * pi + d] +=
                    2 * regularizerWeight_ /** cotan_weight[ni]*/ * pi_vec_pi_ni_rot[d];
            }
        }
    }
}

void LaplacianRegularizerSS::Update(
    const std::vector<Vector3f>& latestPositions,
    const std::vector<Vector3f>& currentPositions
    )
{
    ComputePerPointRotations(latestPositions, currentPositions);
    UpdateRotation(currentPositions);
}

LaplacianRegularizerMM::LaplacianRegularizerMM(
    const UICPC* samplePC, const UICTM* inputTM,
    const SparseMatrixD& liftWeight, const SparseMatrixD& liftWeight3,
    const float& weight,
    const bool& inertia,
    const unsigned& D) :
LaplacianRegularizer(weight, inertia, D)
{
    const unsigned& num_points = samplePC->getNumPoints();
    const unsigned& num_verts = inputTM->getNumPoints();
    TrimeshStatic tsMesh(inputTM, false);

    const unsigned& num_var = Dim * num_points;
    QT.setRows(num_var);
    QT.setZero();
    LT.setDim(num_var);
    LT.setZero();

    per_point_rotation_.resize(num_points, IDENTITY3F);
    //cotan_weight_.resize(num_points);
    //for (unsigned pi = 0; pi < num_points; ++pi) {
    //    ComputeCotanWeights(tpy_, samplePC,
    //        pi, &cotan_weight_[pi]);
    //}

    BuildRegularizerDelta(num_points, tsMesh, liftWeight, liftWeight3);
    {
        //DR_FRAME = 0;
        //debugRenderer->clearRenderJob_AllFrames("interpolation_");
        //const PointSet& samplePS = *samplePC->getPointSet();
        //const AAT& samPosAAT = samplePS.getAAT("position");
        //for (unsigned pi = 0; pi < num_points; ++pi) {
        //    debugRenderer->beginRenderJob_OneFrame("interpolation_", DR_FRAME++);
        //    const unsigned& numNV = samTpy[pi].size();
        //    for (unsigned ni = 0; ni < numNV; ++ni) {
        //        const unsigned& pi_ni = samTpy[pi][ni];
        //        debugRenderer->addLine(
        //            samplePS.get3f(pi, samPosAAT), samplePS.get3f(pi_ni, samPosAAT),
        //            makeVector3f(1, 0, 0),
        //            makeVector3f(0, 0, 1),
        //            1);
        //    }
        //    debugRenderer->endRenderJob();
        //}
    }
    {
        //DR_FRAME = 0;
        //debugRenderer->clearRenderJob_AllFrames("interpolation_");
        //const unsigned& num_verts = tsMesh.GetNumVerts();
        //for (unsigned vi = 0; vi < num_verts; ++vi) {
        //    debugRenderer->beginRenderJob_OneFrame("interpolation_", DR_FRAME++);
        //    const std::vector<mpcard>& nverts = tsMesh.GetVert1RingVerts(vi);
        //    const unsigned& numNV = nverts.size();
        //    for (unsigned ni = 0; ni < numNV; ++ni) {
        //        const unsigned& vi_ni = nverts[ni];
        //        debugRenderer->addLine(
        //            tsMesh.GetVertPosition(vi), tsMesh.GetVertPosition(vi_ni),
        //            makeVector3f(1, 0, 0),
        //            makeVector3f(0, 0, 1),
        //            1);
        //    }
        //    debugRenderer->endRenderJob();
        //}
    }
}

void LaplacianRegularizerMM::BuildRegularizerDelta(
    const unsigned& num_points, const TrimeshStatic& tsMesh,
    const SparseMatrixD& liftWeight, const SparseMatrixD& liftWeight3
    )
{
    const unsigned& dim_sample = Dim * num_points;
    const unsigned& num_verts = tsMesh.GetNumVerts();
    SparseMatrixD lapMesh; // Laplacian on mesh
    lapMesh.setRows(3 * num_verts);
    SparseMatrixD meshTpy; // record mesh connectivity
    meshTpy.setRows(num_verts);
    for (unsigned vi = 0; vi < num_verts; ++vi) {
        const std::vector<mpcard>& nverts = tsMesh.GetVert1RingVerts(vi);
        const unsigned& numNV = nverts.size();
        for (unsigned ni = 0; ni < numNV; ++ni) {
            const unsigned& vi_ni = nverts[ni];
            for (unsigned d = 0; d < Dim; ++d) {
                lapMesh[Dim * vi + d].addEntryBinary(Dim * vi + d,
                    2 * regularizerWeight_ /** cotan_weight[ni]*/);
                lapMesh[Dim * vi + d].addEntryBinary(Dim * vi_ni + d,
                    -1 * regularizerWeight_ /** cotan_weight[ni]*/);
                lapMesh[Dim * vi_ni + d].addEntryBinary(Dim * vi + d,
                    -1 * regularizerWeight_ /** cotan_weight[ni]*/);
            }
            meshTpy[vi].addEntryBinary(vi_ni, 1);
        }
    }

    // project onto sample space - Laplacian
    //Timer timer_diff; timer_diff.getDeltaValue();
#ifdef USE_CUDA
    CudaAdaptor::sparseAtQA(
        lapMesh, liftWeight3,
        dim_sample, &QT
        );
    //debugOutput << convertTimeToString( timer_diff.getDeltaValue() ) << "\n";
#else
    QT = LibraryAdaptor::SparseMatrixQuadratic(lapMesh, liftWeight3, dim_sample);
    QT.removeNearZeros(std::numeric_limits<double>::epsilon());
    //debugOutput << convertTimeToString( timer_diff.getDeltaValue() ) << "\n";
#endif

    // add diagonal entries
    if (bUseInertia_) {
        for (unsigned bi = 0; bi < dim_sample; ++bi) {
            QT[bi].addEntryBinary(bi, 2 * regularizerWeight_);
        }
    }

    // project onto sample space - topology
    //timer_diff.getDeltaValue();
#ifdef USE_CUDA
    CudaAdaptor::sparseAtQA(
        meshTpy, liftWeight,
        num_points, &meshTpy
        );
    //debugOutput << convertTimeToString( timer_diff.getDeltaValue() ) << "\n";
#else
    meshTpy = LibraryAdaptor::SparseMatrixQuadratic(meshTpy, liftWeight, num_points);
    meshTpy.removeNearZeros(std::numeric_limits<double>::epsilon());
    //debugOutput << convertTimeToString( timer_diff.getDeltaValue() ) << "\n";
#endif
    samTpy.resize(num_points);
    for (unsigned vi = 0; vi < num_points; ++vi) {
        const SparseVectorD& weightVector = meshTpy[vi];
        SparseVectorD::EIteratorConst eit = weightVector.begin();
        SparseVectorD::EIteratorConst eit_end = weightVector.end();
        while (eit != eit_end) {
            samTpy[vi].push_back(eit->index);
            ++eit;
        }
    }
}

void LaplacianRegularizerMM::ComputePerPointRotations(
    const std::vector<Vector3f>& latestPositions,
    const std::vector<Vector3f>& currentPositions
    )
{
    const unsigned num_points = latestPositions.size();
    std::vector<Vector3f> corrForRotInit, corrForRotCurr;
    for (unsigned pi = 0; pi < num_points; ++pi) {
        const unsigned& numNV = samTpy[pi].size();
        corrForRotInit.clear();
        corrForRotCurr.clear();
        std::set<unsigned> indiceSet; // points may have same indice
        for (unsigned ni = 0; ni < numNV; ++ni) {
            const unsigned& pi_ni = samTpy[pi][ni];

            corrForRotInit.push_back(latestPositions[pi_ni]);
            corrForRotCurr.push_back(currentPositions[pi_ni]);
        }

        if (corrForRotInit.size() >= 4) {
            per_point_rotation_[pi] = EstimateRigidAlignmentRotation(
                corrForRotCurr, currentPositions[pi],
                corrForRotInit, latestPositions[pi]
            );
        } else {
            per_point_rotation_[pi] = IDENTITY3F;
            //debugOutput << "could not estimate rotation for vertex " << pi << "\n";
        }
    }
}

void LaplacianRegularizerMM::UpdateRotation(
    const std::vector<Vector3f>& currentPositions
    )
{
    LT.setZero();
    const unsigned& num_points = currentPositions.size();
    for (unsigned pi = 0; pi < num_points; ++pi) {
        const Vector3f& pos_pi = currentPositions[pi];
        const unsigned& numNV = samTpy[pi].size();

        //const std::vector<float>& cotan_weight = cotan_weight_[pi];
        for (unsigned ni = 0; ni < numNV; ++ni) {
            const unsigned& pi_ni = samTpy[pi][ni];
            const Vector3f& pos_pi_ni = currentPositions[pi_ni];

            const Vector3f pi_vec_pi_ni = pos_pi - pos_pi_ni;
            const Vector3f pi_vec_pi_ni_rot = (per_point_rotation_[pi] - IDENTITY3F) * pi_vec_pi_ni;

            for (unsigned d = 0; d < Dim; ++d) {
                LT[Dim * pi + d] +=
                    2 * regularizerWeight_ /** cotan_weight[ni]*/ * pi_vec_pi_ni_rot[d];
            }
        }
    }
}

void LaplacianRegularizerMM::Update(
    const std::vector<Vector3f>& latestPositions,
    const std::vector<Vector3f>& currentPositions
    )
{
    ComputePerPointRotations(latestPositions, currentPositions);
    UpdateRotation(currentPositions);
}
