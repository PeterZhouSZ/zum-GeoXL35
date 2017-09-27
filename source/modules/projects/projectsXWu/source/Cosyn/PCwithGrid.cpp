#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "PCwithGrid.h"
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

PatchTrans::PatchTrans(void)
{
    data = nullptr;
    patch = nullptr;
    center = NULL_VECTOR3F;
    radius = 0;
    trans = IDENTITY4F;
    score = std::numeric_limits<float>::max();
}

PatchTrans::~PatchTrans(void)
{
    if (nullptr != patch) delete patch;
}

PointSet* PatchTrans::GetPatchPS(void)
{
    if (nullptr != patch) delete patch;
    if (nullptr == data) error("PatchTrans::GetPatchPS - no data");
    patch = data->getPointSet()->subset(points);
    return patch;
}

UICPC* PatchTrans::GetPatchPCTransformed(void)
{
    if (nullptr == data) error("PatchTrans::GetPatchPCTransformed - no data");
    UICPC* transPC = new UICPC;
    PointSet* transPS = data->getPointSet()->subset(points);
    transPC->setPointSet(transPS);
    const AAT posAAT = transPS->getAAT("position");
    const unsigned numPoints = transPS->getNumEntries();
    for (unsigned ii = 0; ii < numPoints; ++ii) {
        const Vector3f pos = transPS->get3f(ii, posAAT);
        const Vector3f posT = transformVector3f(trans, pos);
        transPS->set3f(ii, posAAT, posT);
    }
    return transPC;
}

void PatchTrans::Draw(void)
{
    const float lineWidth = radius * 0.1f;
    // source
    const Vector3f f0 = center + frame[0] * radius;
    const Vector3f f1 = center + frame[1] * radius;
    const Vector3f f2 = center + frame[2] * radius;
    debugRenderer->addLine(center, f0, makeVector4f(1, 1, 1, 1), makeVector4f(1, 0, 0, 1), lineWidth);
    debugRenderer->addLine(center, f1, makeVector4f(1, 1, 1, 1), makeVector4f(0, 1, 0, 1), lineWidth);
    debugRenderer->addLine(center, f2, makeVector4f(1, 1, 1, 1), makeVector4f(0, 0, 1, 1), lineWidth);
    debugRenderer->addFastSphere(center, lineWidth * 2.f, makeVector3f(1, 1, 0), false);
    // target
    const Vector3f t0 = transformVector3f(trans, f0);
    const Vector3f t1 = transformVector3f(trans, f1);
    const Vector3f t2 = transformVector3f(trans, f2);
    const Vector3f target = transformVector3f(trans, center);
    debugRenderer->addLine(target, t0, makeVector4f(1, 1, 1, 1), makeVector4f(1, 0, 0, 1), lineWidth);
    debugRenderer->addLine(target, t1, makeVector4f(1, 1, 1, 1), makeVector4f(0, 1, 0, 1), lineWidth);
    debugRenderer->addLine(target, t2, makeVector4f(1, 1, 1, 1), makeVector4f(0, 0, 1, 1), lineWidth);
    debugRenderer->addFastSphere(target, lineWidth * 2.f, makeVector3f(0, 1, 1), false);
    // line
    debugRenderer->addLine(center, target, makeVector3f(1, 1, 0), makeVector3f(0, 1, 1), lineWidth);
    // points
    if (nullptr == patch) return;
    const AAT posAAT = patch->getAAT("position");
    const unsigned numPoints = patch->getNumEntries();
    for (unsigned ii = 0; ii < numPoints; ++ii) {
        const Vector3f pos = patch->get3f(ii, posAAT);
        debugRenderer->addPoint(pos,
            makeVector3f(1, 1, 0));
        const Vector3f posT = transformVector3f(trans, pos);
        debugRenderer->addPoint(posT,
            makeVector3f(0, 1, 1));
    }
}
