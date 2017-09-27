#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "CellPlane.h"
#include "Util\numerical\EigenAdaptor.h"
//---------------------------------------------------------------------------
#include "BoundingBox.h"
#include "PointCloudTools.h"
#include "PCCResampler.h"
#include "NormalEstimator.h"
#include "NormalUnifier.h"
#include "PointSetKNNQuery.h"
#include "AttachedIndexedOctree.h"
//---------------------------------------------------------------------------

//============================================================================

CellPlane::CellPlane(void) : KNN(nullptr)
{
    point = NULL_VECTOR3F;
    normal = ZAXIS_VECTOR3F;
    dir0 = dir1 = NULL_VECTOR3F;
    patch = nullptr;
    KNN = nullptr;
    sub = nullptr;
    isBon = false;
}

CellPlane::~CellPlane(void)
{
    if (nullptr != patch) delete patch;
}

PointSet* CellPlane::GetCellPS(UICPC* data)
{
    if (nullptr != patch) return patch;
    if (nullptr == data) error("CellPlane::GetPatchPS - no data");
    patch = data->getPointSet()->subset(points);
    return patch;
}

void CellPlane::EstimateNormal(PointSet* PS)
{
    PointSet* cellPS = PS->subset(points);
    const AAT NORMAL = cellPS->getAAT("normal");
    const AAT POSITION = cellPS->getAAT("position");

    const unsigned numPoints = cellPS->getNumEntries();
    if (4 > numPoints) {
        normal = cellPS->get3f(0, NORMAL);
        point = cellPS->get3f(0, POSITION);
    }
    else {
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for (unsigned ii = 0; ii < numPoints; ++ii) {
            const Eigen::Vector3f posI = EigenAdaptor::ToEigen(cellPS->get3f(ii, POSITION));
            centroid += posI;
        }
        centroid /= (float)numPoints;
        unsigned minI = 0;
        float minD = 1e10;
        for (unsigned ii = 1; ii < numPoints; ++ii) {
            const Eigen::Vector3f posI = EigenAdaptor::ToEigen(cellPS->get3f(ii, POSITION));
            float dist = (posI - centroid).squaredNorm();
            if (dist < minD) {
                minI = ii;
                minD = dist;
            }
        }
        normal = cellPS->get3f(minI, NORMAL);
        point = cellPS->get3f(minI, POSITION);
    }
}

void CellPlane::EstimatePlane(PointSet* PS)
{
    PointSet* cellPS = PS->subset(points);
    const AAT NORMAL = cellPS->getAAT("normal");
    const AAT POSITION = cellPS->getAAT("position");

    const unsigned numPoints = cellPS->getNumEntries();
    if (4 > numPoints) {
        normal = cellPS->get3f(0, NORMAL);
        point = cellPS->get3f(0, POSITION);
        // not setting plane directions, so won't show up plane in debug rendering
        return;
    }

    Eigen::MatrixXf COV(3, numPoints);
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (unsigned ii = 0; ii < numPoints; ++ii) {
        const Eigen::Vector3f posI = EigenAdaptor::ToEigen(cellPS->get3f(ii, POSITION));
        COV.col(ii) = posI;
        centroid += posI;
    }
    centroid /= (float)numPoints;

    for (unsigned ii = 0; ii < numPoints; ++ii) {
        COV.col(ii) -= centroid;
    }

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(COV, Eigen::ComputeThinU);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Vector3f S = svd.singularValues();
    //EigenAdaptor::WriteFile(COV);
    //{ stringstream ss; ss << S; debugOutput << ss.str() << "\n\n"; }
    normal = EigenAdaptor::FromEigen(static_cast<Eigen::Vector3f>(U.col(2)));
    dir0 = EigenAdaptor::FromEigen(static_cast<Eigen::Vector3f>(U.col(0))); // *S(0);
    dir1 = EigenAdaptor::FromEigen(static_cast<Eigen::Vector3f>(U.col(1))); // *S(1);
    point = EigenAdaptor::FromEigen(centroid);
}

void CellPlane::DrawPS(UICPC* data, const Vector3f &color)
{
    PointSet* ps = GetCellPS(data);
    const unsigned numPoints = patch->getNumEntries();
    const AAT NORMAL = patch->getAAT("normal");
    const AAT POSITION = patch->getAAT("position");

    for (unsigned ii = 0; ii < numPoints; ++ii) {
        const Vector3f posI = patch->get3f(ii, POSITION);
        debugRenderer->addPoint(posI, color);
    }
}

void CellPlane::DrawPlane(const Vector3f &color, const float& scale, const float& linewidth)
{
    debugRenderer->addLine(
        point, point + normal,
        color, makeVector3f(1, 0, 0),
        linewidth);
    debugRenderer->addPlanarEllipse(point, dir0 * scale, dir1 * scale, color, linewidth);
}
