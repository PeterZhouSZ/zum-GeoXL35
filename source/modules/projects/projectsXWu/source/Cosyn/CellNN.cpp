#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "CellNN.h"
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

CellNN::CellNN(void)
{
    patch = nullptr;
    KNN = nullptr;
    isBon = false;
}

CellNN::~CellNN(void)
{
    if (nullptr != patch) delete patch;
}

PointSet* CellNN::GetCellPS(UICPC* data)
{
    if (nullptr != patch) return patch;
    if (nullptr == data) error("CellNN::GetPatchPS - no data");
    patch = data->getPointSet()->subset(points);
    return patch;
}

void CellNN::DrawPS(UICPC* data, const Vector3f &color)
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
