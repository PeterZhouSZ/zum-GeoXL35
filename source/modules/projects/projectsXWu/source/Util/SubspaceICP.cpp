#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "SubspaceICP.h"
#include "Util/ColorSchemer.hpp"
//---------------------------------------------------------------------------
#include <set>
#include <functional>
#include <boost/heap/priority_queue.hpp>
#include <boost/format.hpp>
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

namespace {
    const float NotAlignedValue = 1e20f;
}

SubspaceICP::SubspaceICP(UICPC* pc, const Eigen::Matrix4f& worldFrame)
{
    // set up local/world transformations
    local2world_ = worldFrame;
    world2local_ = local2world_.inverse();
    const PointSet& ps = *pc->getPointSet();
    const float diagRadius = ps.getBoundingBox().getDiagonalLength() / 2;
    Eigen::Matrix4f scale2world = Eigen::Matrix4f::Identity();
    scale2world.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() * diagRadius;
    Eigen::Matrix4f scale2local = Eigen::Matrix4f::Identity();
    scale2local.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() / diagRadius;
    //const Matrix4f scale2world = expand3To4(IDENTITY3F);
    //const Matrix4f scale2local = expand3To4(IDENTITY3F);
    local2world_ = local2world_ * scale2world;
    world2local_ = scale2local * world2local_;

    // transform data to local
    PS_ = (PointSet*)ps.copy();
    const AAT posAAT = ps.getAAT("position");
    const AAT norALL = PS_->getAAT("normal");
    const Matrix4f world2local = EigenAdaptor::FromEigen(world2local_);
    const Matrix3f rotLocal = shrink4To3(world2local);
    const size_t numPoints = ps.getNumEntries();
    for (size_t ii = 0; ii < numPoints; ++ii) {
        const Vector3f pos = ps.get3f(ii, posAAT);
        const Vector3f posL = transformVector3f(world2local, pos);
        PS_->set3f(ii, posAAT, posL);
        const Vector3f nor = ps.get3f(ii, norALL);
        const Vector3f norL = rotLocal * nor;
        PS_->set3f(ii, norALL, norL);
    }

    // build NN search structure
    KNN_ = boost::shared_ptr<PointSetANNQuery>(new PointSetANNQuery(PS_, 1));

    alignStatus_ = NotAligned;
    axisFree_.fill(Free);

    median_point_dist_ = getMedianPointDistance(pc) / diagRadius;
    //median_point_dist_ = getMedianPointDistance(pc);
    updated_trans_ = Eigen::Matrix4f::Identity();
    last_trans_ = Eigen::Matrix4f::Identity();
    start_center_ = worldFrame.col(3).head<3>();
    start_center_ = (world2local_ * start_center_.homogeneous()).head<3>();
    setupParameters();
    gauss_truncate_ = .01f;
}

SubspaceICP::~SubspaceICP(void)
{
    delete PS_;
    PS_ = nullptr;
}

const Eigen::Matrix4f& SubspaceICP::GetUpdatedTransformation(void) const
{
    return updated_trans_;
}

void SubspaceICP::SetAxisLocked(const unsigned axis)
{
    if (5 < axis) return;
    axisFree_[axis] = Locked;
}

void SubspaceICP::setupParameters(
    float match_dist_factor /*= 10*/,
    float outlier_dist_factor /*= 4*/,
    float min_inlier_ratio /*= 0.8*/,
    //float min_match_ratio /*= 0.1*/,
    float max_dist_factor/*= 0.1f */,
    unsigned num_iter /*= 1*/,
    float residual_difference_factor/*= 0.01f*/,
    float transformation_difference /*= 0.001f*/,
    DebugLevel visualization/*= false*/
    )
{
    if (match_dist_factor < outlier_dist_factor)
        match_dist_factor = 2 * outlier_dist_factor;
    this->match_dist_ = median_point_dist_ * match_dist_factor;
    this->outlier_dist_ = median_point_dist_ * outlier_dist_factor;
    this->min_inlier_ratio_ = min_inlier_ratio;
    //this->min_match_ratio_ = min_match_ratio;
    this->max_dist_move_ = PS_->getBoundingBox().getDiagonalLength() * max_dist_factor;
    this->num_iter_ = num_iter;
    this->residual_difference_ = median_point_dist_ * residual_difference_factor;
    this->transformation_difference_ = transformation_difference;
    this->visualization_ = visualization;

    if (DebugLevel::Graphical > visualization) return;

    debugRenderer->beginRenderJob_OneFrame("data_local_", DR_FRAME++);
    const AAT posAAT = PS_->getAAT("position");
    Vector3f minP = makeVector3f(1e30, 1e30, 1e30);
    Vector3f maxP = makeVector3f(-1e30, -1e30, -1e30);
    const size_t numPoints = PS_->getNumEntries();
    for (size_t ii = 0; ii < numPoints; ++ii) {
        const Vector3f pos = PS_->get3f(ii, posAAT);
        if (pos[0] > maxP[0]) maxP = pos;
        if (pos[1] < minP[1]) minP = pos;
        debugRenderer->addPoint(pos, makeVector3f(0.8, 0.8, 0.8));
    }
    debugRenderer->addSphere(minP, match_dist_, makeVector3f(1, 0, 0));
    debugRenderer->addSphere(maxP, outlier_dist_, makeVector3f(1, 0, 0));
    debugRenderer->endRenderJob();
}

float SubspaceICP::gaussTruncate(const float& value, const float& sigma, const float& epsilon)
{
    const float ret = exp(-(value * value) / (sigma * sigma));
    return (ret < epsilon) ? 0 : ret;
}

void constructTransformationMatrix(
    const float& alpha, const float& beta, const float& gamma,
    const float& tx, const float& ty, const float& tz,
    Eigen::Matrix4f* transformation_matrix)
{
    // Construct the transformation matrix from rotation and translation 
    (*transformation_matrix) = Eigen::Matrix4f::Zero();
    (*transformation_matrix)(0, 0) = (cos(gamma) * cos(beta));
    (*transformation_matrix)(0, 1) = (-sin(gamma) * cos(alpha) + cos(gamma) * sin(beta) * sin(alpha));
    (*transformation_matrix)(0, 2) = (sin(gamma) * sin(alpha) + cos(gamma) * sin(beta) * cos(alpha));
    (*transformation_matrix)(1, 0) = (sin(gamma) * cos(beta));
    (*transformation_matrix)(1, 1) = (cos(gamma) * cos(alpha) + sin(gamma) * sin(beta) * sin(alpha));
    (*transformation_matrix)(1, 2) = (-cos(gamma) * sin(alpha) + sin(gamma) * sin(beta) * cos(alpha));
    (*transformation_matrix)(2, 0) = (-sin(beta));
    (*transformation_matrix)(2, 1) = (cos(beta) * sin(alpha));
    (*transformation_matrix)(2, 2) = (cos(beta) * cos(alpha));

    (*transformation_matrix)(0, 3) = (tx);
    (*transformation_matrix)(1, 3) = (ty);
    (*transformation_matrix)(2, 3) = (tz);
    (*transformation_matrix)(3, 3) = (1);
}

float SubspaceICP::estimateRigidTransformationLLS(
    PointSet* startPS
    )
{
    Eigen::Matrix6f ATA;
    Eigen::Vector6f ATb;
    ATA.setZero();
    ATb.setZero();

    // Approximate as a linear least squares problem
    const size_t numPoints = startPS->getNumEntries();
    const AAT norALL = PS_->getAAT("normal");
    const AAT posAAT = PS_->getAAT("position");

    float residual = 0;
    float sumWeight = 0;

    std::deque<float> weightVec;
    // calculate corresponding points
    for (size_t ii = 0; ii < numPoints; ++ii)
    {
        const Eigen::Vector3f posI = EigenAdaptor::ToEigen(startPS->get3f(ii, posAAT));
        const Eigen::Vector3f posI_trans = EigenAdaptor::Transform(updated_trans_, posI);

        // seek closest point
        const int32 indexNN = KNN_->getNearestPointIndex(EigenAdaptor::FromEigen(posI_trans));
        const Eigen::Vector3f posN = EigenAdaptor::ToEigen(PS_->get3f(indexNN, posAAT));
        const Eigen::Vector3f nmlN = EigenAdaptor::ToEigen(PS_->get3f(indexNN, norALL));

        const float distNN = (posN - posI_trans).norm();
        if (distNN > match_dist_) {
            match_indices_.push_back(ii);
            continue;
        }
        else if (distNN > outlier_dist_) {
            outlier_indices_.push_back(ii);
            continue;
        }
        inlier_indices_.push_back(ii);

        const float weight = gaussTruncate(distNN, outlier_dist_, gauss_truncate_);
        //const float weight = 1.f;
        weightVec.push_back(weight);
        sumWeight += weight;

        // project onto surfel
        Eigen::Vector3f projectedPoint = posI_trans - nmlN * (nmlN.dot(posI_trans - posN));
        float projectedDistance2 = (posI_trans - projectedPoint).squaredNorm();

        // current residual
        residual += projectedDistance2 * weight;

        if (DebugLevel::Graphical == visualization_)
        {
            //debugRenderer->addLine(
            //    EigenAdaptor::FromEigen(posI),
            //    EigenAdaptor::FromEigen(posI_trans),
            //    makeVector3f(1, 0, 0), makeVector3f(1, 0, 0),
            //    1.f);
            debugRenderer->addLine(
                EigenAdaptor::FromEigen(EigenAdaptor::Transform(local2world_, posI_trans)),
                EigenAdaptor::FromEigen(EigenAdaptor::Transform(local2world_, posN)),
                makeVector3f(0, 1, 0), makeVector3f(0, 0, 1),
                1.f);
            //debugRenderer->addLine(
            //    EigenAdaptor::FromEigen(posI_trans),
            //    EigenAdaptor::FromEigen(projectedPoint),
            //    makeVector3f(0, 0, 1), makeVector3f(0, 0, 1),
            //    1.f);
        }

        if (DebugLevel::Graphical == visualization_)
        {
            residualVec_.push_back(distNN);
        }

        const float& sx = posI_trans(0);
        const float& sy = posI_trans(1);
        const float& sz = posI_trans(2);
        const float& dx = posN(0);
        const float& dy = posN(1);
        const float& dz = posN(2);
        const float& nx = nmlN(0) * weight;
        const float& ny = nmlN(1) * weight;
        const float& nz = nmlN(2) * weight;

        float a = nz*sy - ny*sz;
        float b = nx*sz - nz*sx;
        float c = ny*sx - nx*sy;

        //    0  1  2  3  4  5
        //    6  7  8  9 10 11
        //   12 13 14 15 16 17
        //   18 19 20 21 22 23
        //   24 25 26 27 28 29
        //   30 31 32 33 34 35

        ATA.coeffRef(0) += a * a;
        ATA.coeffRef(1) += a * b;
        ATA.coeffRef(2) += a * c;
        ATA.coeffRef(3) += a * nx;
        ATA.coeffRef(4) += a * ny;
        ATA.coeffRef(5) += a * nz;
        ATA.coeffRef(7) += b * b;
        ATA.coeffRef(8) += b * c;
        ATA.coeffRef(9) += b * nx;
        ATA.coeffRef(10) += b * ny;
        ATA.coeffRef(11) += b * nz;
        ATA.coeffRef(14) += c * c;
        ATA.coeffRef(15) += c * nx;
        ATA.coeffRef(16) += c * ny;
        ATA.coeffRef(17) += c * nz;
        ATA.coeffRef(21) += nx * nx;
        ATA.coeffRef(22) += nx * ny;
        ATA.coeffRef(23) += nx * nz;
        ATA.coeffRef(28) += ny * ny;
        ATA.coeffRef(29) += ny * nz;
        ATA.coeffRef(35) += nz * nz;

        float d = nx*dx + ny*dy + nz*dz - nx*sx - ny*sy - nz*sz;
        ATb.coeffRef(0) += a * d;
        ATb.coeffRef(1) += b * d;
        ATb.coeffRef(2) += c * d;
        ATb.coeffRef(3) += nx * d;
        ATb.coeffRef(4) += ny * d;
        ATb.coeffRef(5) += nz * d;
    }

    // root mean square
    residual /= sumWeight;
    residual = sqrt(residual);

    // fill in symmetric entries
    ATA.coeffRef(6) = ATA.coeff(1);
    ATA.coeffRef(12) = ATA.coeff(2);
    ATA.coeffRef(13) = ATA.coeff(8);
    ATA.coeffRef(18) = ATA.coeff(3);
    ATA.coeffRef(19) = ATA.coeff(9);
    ATA.coeffRef(20) = ATA.coeff(15);
    ATA.coeffRef(24) = ATA.coeff(4);
    ATA.coeffRef(25) = ATA.coeff(10);
    ATA.coeffRef(26) = ATA.coeff(16);
    ATA.coeffRef(27) = ATA.coeff(22);
    ATA.coeffRef(30) = ATA.coeff(5);
    ATA.coeffRef(31) = ATA.coeff(11);
    ATA.coeffRef(32) = ATA.coeff(17);
    ATA.coeffRef(33) = ATA.coeff(23);
    ATA.coeffRef(34) = ATA.coeff(29);

    Eigen::Vector6f x;
    { // null-space method
        int dimC = std::accumulate(axisFree_.begin(), axisFree_.end(), 0);
        const int dimN = 6 - dimC;
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> N(6, dimN);
        for (unsigned d = 0, n = 0; d < 6; ++d) {
            if (Locked == axisFree_[d]) continue;
            N.col(n) = Eigen::Vector6f::Zero();
            N.col(n)(d) = 1;
            ++n;
        }
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> H = N.transpose() * ATA * N;
        Eigen::Matrix<float, Eigen::Dynamic, 1> g = N.transpose() * ATb;
        // Solve H*x = g
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> H_inv = H.ldlt().solve(
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Identity(dimN, dimN));
        x = N * (H_inv * g);
        {
            //debugOutput << "ATA =\n" << EigenAdaptor::ToString(ATA) << "\n";
            //debugOutput << "ATb =\n" << EigenAdaptor::ToString(ATb) << "\n";
            //debugOutput << "N =\n" << EigenAdaptor::ToString(N) << "\n";
            //debugOutput << "H =\n" << EigenAdaptor::ToString(H) << "\n";
            //debugOutput << "g =\n" << EigenAdaptor::ToString(g) << "\n";
            //debugOutput << "H_inv =\n" << EigenAdaptor::ToString(H_inv) << "\n";
            //debugOutput << "x =\n" << EigenAdaptor::ToString(x) << "\n";
        }
    }
    {
        //// Solve A*x = b
        ////Eigen::Vector6f x = static_cast<Eigen::Vector6f>(ATA.inverse() * ATb);
        //Eigen::Matrix6f ATA_inv = ATA.ldlt().solve(Eigen::Matrix6f::Identity());
        //x = static_cast<Eigen::Vector6f>(ATA_inv * ATb);
    }

    // Construct the transformation matrix from x
    Eigen::Matrix4f deltaTrans = Eigen::Matrix4f::Identity();
    constructTransformationMatrix(x(0), x(1), x(2), x(3), x(4), x(5), &deltaTrans);

    if (DebugLevel::Graphical == visualization_)
    {
        const Eigen::Vector3f cen1 = EigenAdaptor::Transform(updated_trans_, start_center_);
        const Eigen::Vector3f cen2 = EigenAdaptor::Transform(deltaTrans, cen1);
        debugRenderer->addLine(
            EigenAdaptor::FromEigen(EigenAdaptor::Transform(local2world_, start_center_)),
            EigenAdaptor::FromEigen(EigenAdaptor::Transform(local2world_, cen1)),
            makeVector3f(1, 0, 0), makeVector3f(0, 1, 0),
            5.f);
        debugRenderer->addLine(
            EigenAdaptor::FromEigen(EigenAdaptor::Transform(local2world_, cen1)),
            EigenAdaptor::FromEigen(EigenAdaptor::Transform(local2world_, cen2)),
            makeVector3f(0, 1, 0), makeVector3f(0, 0, 1),
            5.f);
        {
            //std::ostringstream ss;
            //ss << deltaTrans;
            //debugOutput << ss.str();
        }
    }

    updated_trans_ = deltaTrans * updated_trans_;

    return residual;
}

Eigen::Vector3f compute3DCentroid(const std::deque<Eigen::Vector3f>& points)
{
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    const size_t numP = points.size();
    for (size_t ii = 0; ii < numP; ++ii) {
        centroid += points[ii];
    }
    centroid /= (float)numP;
    return centroid;
}

float SubspaceICP::estimateRigidTransformationSVD(
    PointSet* startPS
    )
{
    const size_t numPoints = startPS->getNumEntries();
    const AAT norALL = PS_->getAAT("normal");
    const AAT posAAT = PS_->getAAT("position");

    float residual = 0;
    float sumWeight = 0;

    std::deque<float> weightVec;
    // calculate corresponding points
    for (size_t ii = 0; ii < numPoints; ++ii)
    {
        const Eigen::Vector3f posI = EigenAdaptor::ToEigen(startPS->get3f(ii, posAAT));
        const Eigen::Vector3f posI_trans = EigenAdaptor::Transform(updated_trans_, posI);

        // seek closest point
        const int32 indexNN = KNN_->getNearestPointIndex(EigenAdaptor::FromEigen(posI_trans));
        const Eigen::Vector3f posN = EigenAdaptor::ToEigen(PS_->get3f(indexNN, posAAT));
        const Eigen::Vector3f nmlN = EigenAdaptor::ToEigen(PS_->get3f(indexNN, norALL));

        const float distNN = (posN - posI_trans).norm();
        if (distNN > match_dist_) {
            match_indices_.push_back(ii);
            continue;
        } else if (distNN > outlier_dist_) {
            outlier_indices_.push_back(ii);
            continue;
        }
        inlier_indices_.push_back(ii);
        corrSorc_.push_back(posI_trans);
        corrDest_.push_back(posN);

        const float weight = gaussTruncate(distNN, outlier_dist_, gauss_truncate_);
        //const float weight = 1.f;
        weightVec.push_back(weight);
        sumWeight += weight;

        Eigen::Vector3f projectedPoint = posI_trans - nmlN * (nmlN.dot(posI_trans - posN));
        float projectedDistance2 = (posI_trans - projectedPoint).squaredNorm();

        // current residual
        residual += projectedDistance2 * weight;

        if (DebugLevel::Graphical == visualization_)
        {
            //debugRenderer->addLine(
            //    EigenAdaptor::FromEigen(posI),
            //    EigenAdaptor::FromEigen(posI_trans),
            //    makeVector3f(1, 0, 0), makeVector3f(1, 0, 0),
            //    1.f);
            debugRenderer->addLine(
                EigenAdaptor::FromEigen(EigenAdaptor::Transform(local2world_, posI_trans)),
                EigenAdaptor::FromEigen(EigenAdaptor::Transform(local2world_, posN)),
                makeVector3f(0, 1, 0), makeVector3f(0, 0, 1),
                1.f);
            //debugRenderer->addLine(
            //    EigenAdaptor::FromEigen(posI_trans),
            //    EigenAdaptor::FromEigen(projectedPoint),
            //    makeVector3f(0, 0, 1), makeVector3f(0, 0, 1),
            //    1.f);
        }

        if (DebugLevel::Graphical == visualization_)
        {
            residualVec_.push_back(distNN);
        }
    }

    // root mean square
    residual /= sumWeight;
    residual = sqrt(residual);

    // compute correlation matrix
    const Eigen::Vector3f cenSorc = compute3DCentroid(corrSorc_);
    const Eigen::Vector3f cenDest = compute3DCentroid(corrDest_);
    const size_t numCorr = corrSorc_.size();
    Eigen::Matrix3f COV = Eigen::Matrix3f::Zero();
    for (size_t pi = 0; pi < numCorr; ++pi) {
        COV += weightVec[pi] *
            (corrSorc_[pi] - cenSorc) *
            (corrDest_[pi] - cenDest).transpose();
    }

    // Compute the Singular Value Decomposition
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(COV, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();

    // Compute R = V * U'
    if (U.determinant() * V.determinant() < 0)
    {
        for (int d = 0; d < 3; ++d)
            V(d, 2) *= -1;
    }
    Eigen::Matrix3f R = V * U.transpose();

    // Return the correct transformation
    Eigen::Matrix4f deltaTrans = Eigen::Matrix4f::Identity();
    deltaTrans.topLeftCorner(3, 3) = R;
    const Eigen::Vector3f Rc(R * cenSorc);
    deltaTrans.block(0, 3, 3, 1) = cenDest - Rc;


    if (DebugLevel::Graphical == visualization_)
    {
        const Eigen::Vector3f cen1 = EigenAdaptor::Transform(updated_trans_, start_center_);
        const Eigen::Vector3f cen2 = EigenAdaptor::Transform(deltaTrans, cen1);
        debugRenderer->addLine(
            EigenAdaptor::FromEigen(EigenAdaptor::Transform(local2world_, start_center_)),
            EigenAdaptor::FromEigen(EigenAdaptor::Transform(local2world_, cen1)),
            makeVector3f(1, 0, 0), makeVector3f(0, 1, 0),
            5.f);
        debugRenderer->addLine(
            EigenAdaptor::FromEigen(EigenAdaptor::Transform(local2world_, cen1)),
            EigenAdaptor::FromEigen(EigenAdaptor::Transform(local2world_, cen2)),
            makeVector3f(0, 1, 0), makeVector3f(0, 0, 1),
            5.f);
        {
            //std::ostringstream ss;
            //ss << deltaTrans;
            //debugOutput << ss.str();
        }
    }

    updated_trans_ = deltaTrans * updated_trans_;

    return residual;
}

void SubspaceICP::debugDrawLast(PointSet* startPS)
{
    if (DebugLevel::Graphical != visualization_) return;
    if (inlier_indices_.empty()) return;

    const AAT posAAT = PS_->getAAT("position");

    {
        auto result = std::minmax_element(residualVec_.begin(), residualVec_.end());
        const float resMin = residualVec_[result.first - residualVec_.begin()];
        const float resMax = residualVec_[result.second - residualVec_.begin()];
        //const float distMM = resMax - resMin;
        const float distMM = outlier_dist_;
        const Matrix4f local2world = EigenAdaptor::FromEigen(local2world_);
        for (size_t ii = 0; ii < inlier_indices_.size(); ++ii) {
            const float value = (residualVec_[ii] - resMin) / (distMM);
            const Vector3f posI = startPS->get3f(inlier_indices_[ii], posAAT);
            const Vector3f posT = transformVector3f(
                EigenAdaptor::FromEigen(updated_trans_), posI);
            debugRenderer->addPoint(transformVector3f(local2world, posT),
                ColorSchemer::GetJetColor(value));
            debugRenderer->addPoint(transformVector3f(local2world, posI),
                ColorSchemer::GetJetColor(value));
        }
        for (size_t ii = 0; ii < outlier_indices_.size(); ++ii) {
            const Vector3f posI = startPS->get3f(outlier_indices_[ii], posAAT);
            const Vector3f posT = transformVector3f(
                EigenAdaptor::FromEigen(updated_trans_), posI);
            debugRenderer->addPoint(transformVector3f(local2world, posT),
                makeVector3f(0.2, 0.2, 0.2));
            debugRenderer->addPoint(transformVector3f(local2world, posI),
                makeVector3f(0.2, 0.2, 0.2));
        }
        for (size_t ii = 0; ii < match_indices_.size(); ++ii) {
            const Vector3f posI = startPS->get3f(match_indices_[ii], posAAT);
            const Vector3f posT = transformVector3f(
                EigenAdaptor::FromEigen(updated_trans_), posI);
            debugRenderer->addPoint(transformVector3f(local2world, posT),
                makeVector3f(0.5, 0.5, 0.5));
            debugRenderer->addPoint(transformVector3f(local2world, posI),
                makeVector3f(0.5, 0.5, 0.5));
        }
    }

    for (size_t ii = 0; ii < corrSorc_.size(); ++ii) {
        debugRenderer->addLine(
            EigenAdaptor::FromEigen(EigenAdaptor::Transform(local2world_, corrSorc_[ii])),
            EigenAdaptor::FromEigen(EigenAdaptor::Transform(local2world_, corrDest_[ii])),
            makeVector3f(0, 1, 0), makeVector3f(0, 0, 1),
            1.f);
    }
}

float SubspaceICP::CalculateResidualAfterICP(
    PointSet* startPS, const Eigen::Vector3f& startCenter,
    const Eigen::Matrix4f& originalTransformation
    )
{
    alignStatus_ = NotAligned;

    if (!startPS) return NotAlignedValue;
    const size_t numPoints = startPS->getNumEntries();
    if (numPoints < 4) return NotAlignedValue;

    if (abs(median_point_dist_) < std::numeric_limits<float>::epsilon()) {
        error("SubspaceICP::calculateResidualAfterICP() - Median Point Distance is not defined!");
        return NotAlignedValue;
    }

    float residual = 0;

    updated_trans_ = world2local_ * originalTransformation * local2world_;
    // transform to local frame
    const AAT posAAT = startPS->getAAT("position");
    const AAT norALL = PS_->getAAT("normal");
    const Matrix4f world2local = EigenAdaptor::FromEigen(world2local_);
    const Matrix3f rotLocal = shrink4To3(world2local);
    for (size_t ii = 0; ii < numPoints; ++ii) {
        const Vector3f pos = startPS->get3f(ii, posAAT);
        const Vector3f posL = transformVector3f(world2local, pos);
        startPS->set3f(ii, posAAT, posL);
        const Vector3f nor = startPS->get3f(ii, norALL);
        const Vector3f norL = rotLocal * nor;
        PS_->set3f(ii, norALL, norL);
    }
    start_center_ = (world2local_ * startCenter.homogeneous()).head<3>();
    Eigen::Vector3f startCenTrans = EigenAdaptor::Transform(updated_trans_, start_center_);

    unsigned pass = 0;
    for (; pass < num_iter_; ++pass) {
        float lastResidual = residual;
        last_trans_ = updated_trans_;

        if (DebugLevel::Graphical == visualization_)
        {
            const unsigned draw_frame = DR_FRAME++;
            const std::string draw_name_base = "icp_vis_";
            const std::string draw_name = str(boost::format("%1%%2%") % draw_name_base % draw_frame);
            debugRenderer->beginRenderJob_OneFrame(draw_name_base, draw_frame);
            //debugRenderer->clearDebugData(draw_name);
            residualVec_.clear();
        }

        inlier_indices_.clear();
        outlier_indices_.clear();
        match_indices_.clear();
        corrSorc_.clear();
        corrDest_.clear();

        residual = estimateRigidTransformationLLS(startPS); // converge faster
        //residual = estimateRigidTransformationSVD(startPS);

        // number of inliers
        const size_t numInliers = inlier_indices_.size();
        const size_t numMatches = numInliers + outlier_indices_.size();
        const size_t minInliers = min_inlier_ratio_ * (float)numMatches;
        if (numInliers < minInliers) {
            if (DebugLevel::None < visualization_)
            {
                prefixOutput.write();
                debugOutput << "#" << pass << ": "
                    << "diverge due to too few inliers: " << numInliers
                    << " [" << 100 * (float)numInliers / (float)numMatches << "%].\n";
            }
            break;
        }

        //// non-increamental residual
        //if (lastResidual < residual) { // may happen because of outlier checking
        //    if (DebugLevel::None < visualization_)
        //    {
        //        prefixOutput.write();
        //        debugOutput << "#" << pass << ": " << "diverge with increasing residual:"
        //            << lastResidual << " --> " << residual << "\n";
        //    }
        //    break;
        //}

        // relative residual (RMS) difference
        if (residual_difference_ > fabs(residual - lastResidual)) {
            alignStatus_ = Aligned;
            if (DebugLevel::None < visualization_)
            {
                prefixOutput.write();
                debugOutput << "#" << pass << ": "
                    << "converged with residual difference: "
                    << fabs(lastResidual - residual)
                    << " [" << residual_difference_ << "]."
                    << "\n";
                prefixOutput.write();
                const size_t numInliers = inlier_indices_.size();
                debugOutput << "#" << pass << ": "
                    << "number of inliers: " << numInliers
                    << " [" << 100 * (float)numInliers / (float)numMatches << "%].\n";
            }
            break;
        }

        // relative transformation difference
        if (0 < pass && (updated_trans_ - last_trans_).norm() < transformation_difference_) {
            alignStatus_ = Aligned;
            if (DebugLevel::None < visualization_)
            {
                prefixOutput.write();
                debugOutput << "#" << pass << ": " 
                    << "converged with residual difference: "
                    << fabs(lastResidual - residual)
                    << " [" << transformation_difference_ << "]."
                    << "\n";
                prefixOutput.write();
                const size_t numInliers = inlier_indices_.size();
                debugOutput << "#" << pass << ": "
                    << "number of inliers: " << numInliers
                    << " [" << 100 * (float)numInliers / (float)numMatches << "%].\n";
            }
            break;
        }

        // out-of-range
        if (!checkAllowedMovement(updated_trans_, startCenTrans)) {
            if (DebugLevel::None < visualization_)
            {
                prefixOutput.write();
                debugOutput << "#" << pass << ": " 
                    << "diverge outside of allowed movement\n";
            }
            break;
        }
    }

    // if not aligned after these iterations
    if (pass == num_iter_) {
        // considered aligned, parameter insufficient
        alignStatus_ = Aligned;
        if (DebugLevel::None < visualization_)
        {
            prefixOutput.write();
            debugOutput << "not converged in " << num_iter_ << " iterations.\n";
        }
    }

    // fall back to last transformation
    if (NotAligned == alignStatus_) {
        updated_trans_ = last_trans_;
    }

    if (DebugLevel::Graphical == visualization_)
    {
        debugRenderer->endRenderJob();
    }

    if (DebugLevel::Graphical == visualization_)
    {
        debugRenderer->beginRenderJob_OneFrame("draw_last", DR_FRAME++);
        debugDrawLast(startPS);
        debugRenderer->endRenderJob();
    }

    // transform back to world frame
    updated_trans_ = local2world_ * updated_trans_ * world2local_;

    return residual; // keep the possibility of out-of-iterations convergence
}

bool SubspaceICP::checkAllowedMovement(
    const Eigen::Matrix4f &trans,
    const Eigen::Vector3f& startCenTrans
    )
{
    Eigen::Vector3f transformedCenter = EigenAdaptor::Transform(trans, start_center_);

    Eigen::Vector3f moveVecA = transformedCenter - startCenTrans;
    float moveDist = moveVecA.norm();

    if (moveDist > max_dist_move_) return false;
    else return true;
}

unsigned SubspaceICP::FindInliers(
    PointSet* startPS,
    const Eigen::Matrix4f& originalTransformation
    )
{
    //if (!startPS) return 0;
    //const size_t numPoints = startPS->getNumEntries();
    //if (numPoints < 4) return 0;

    //if (abs(median_point_dist_) < std::numeric_limits<float>::epsilon()) {
    //    error("SubspaceICP::calculateResidualAfterICP() - Median Point Distance is not defined!");
    //    return 0;
    //}

    //float residual = 0;
    //float lastResidual = NotAlignedValue;

    //const AAT norALL = PS_->getAAT("normal");
    //const AAT posAAT = PS_->getAAT("position");

    //updated_trans_ = originalTransformation;
    //BoundingBox3f bb = startPS->getBoundingBox();
    //start_center_ = EigenAdaptor::ToEigen(bb.getCenter());
    //start_center_trans_ = EigenAdaptor::Transform(updated_trans_, start_center_);

    //unsigned pass;
    //for (pass = 0; pass < num_iter_; ++pass)
    //{
    //    if (DebugLevel::Graphical == visualization_)
    //    {
    //        const unsigned draw_frame = DR_FRAME++;
    //        const std::string draw_name_base = "icp_vis_";
    //        const std::string draw_name = str(boost::format("%1%%2%") % draw_name_base % draw_frame);
    //        debugRenderer->beginRenderJob_OneFrame(draw_name_base, draw_frame);
    //        //debugRenderer->clearDebugData(draw_name);
    //        residualVec_.clear();
    //    }

    //    inlier_indices_.clear();
    //    outlier_indices_.clear();
    //    match_indices_.clear();
    //    corrSorc_.clear();
    //    corrDest_.clear();

    //    residual = estimateRigidTransformationSVD(startPS);

    //    // relative residual difference
    //    if (0 < pass && fabs(lastResidual - residual) < transformation_difference_)
    //    {
    //        if (DebugLevel::None < visualization_)
    //        {
    //            prefixOutput.write();
    //            debugOutput << "#" << pass << ": "
    //                << "converged with residual difference: "
    //                << fabs(lastResidual - residual)
    //                << " [" << transformation_difference_ << "]."
    //                << "\n";
    //        }
    //        if (DebugLevel::Graphical == visualization_)
    //        {
    //            debugRenderer->endRenderJob();
    //        }
    //        break;
    //    }

    //    lastResidual = residual;
    //}

    //if (DebugLevel::None < visualization_)
    //{
    //    const size_t numInliers = inlier_indices_.size();
    //    prefixOutput.write();
    //    debugOutput << "after " << pass << " passes, "
    //        << "number of inliers: " << numInliers
    //        << " [" << 100 * (float)numInliers / (float)numPoints << "%].\n";
    //}
    //if (DebugLevel::Graphical == visualization_)
    //{
    //    debugDrawLast(startPS);
    //    debugRenderer->endRenderJob();
    //}

    return inlier_indices_.size();
}

