#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Test/PCITestCentr.h"
//---------------------------------------------------------------------------
#include "Timer.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

#include "Util/OrderedPointSet.h"
void PCITestCentr::TestOrderedPointSet(void)
{
    OrderedPointSet2f ops2;
    {
        OPoint2f p = { 0.f, 0.f };
        ops2.Insert(p);
        Vector3f p3;
        p3[0] = p[0]; p3[1] = p[1]; p3[2] = 0;
        debugRenderer->beginRenderJob_OneFrame("feature_edges_", DR_FRAME++);
        debugRenderer->addPoint(p3,
            makeVector3f(1, 1, 0));
        debugRenderer->addPoint(
            makeVector3f(ops2.mCenter[0], ops2.mCenter[1], 0),
            makeVector3f(1, 0, 0));
        for (int ii = 0; ii < ops2.mPoints.size(); ++ii) {
            debugRenderer->addLine(
                makeVector3f(ops2.mCenter[0], ops2.mCenter[1], 0),
                makeVector3f(ops2.mPoints[ii][0], ops2.mPoints[ii][1], 0),
                makeVector3f(1, 0, 0),
                makeVector3f(0, 0, 1),
                3);
        }
        debugRenderer->endRenderJob();
    }
    {
        OPoint2f p = { 1.f, 0.f };
        ops2.Insert(p);
        Vector3f p3;
        p3[0] = p[0]; p3[1] = p[1]; p3[2] = 0;
        debugRenderer->beginRenderJob_OneFrame("feature_edges_", DR_FRAME++);
        debugRenderer->addPoint(p3,
            makeVector3f(1, 1, 0));
        debugRenderer->addPoint(
            makeVector3f(ops2.mCenter[0], ops2.mCenter[1], 0),
            makeVector3f(1, 0, 0));
        for (int ii = 0; ii < ops2.mPoints.size(); ++ii) {
            debugRenderer->addLine(
                makeVector3f(ops2.mCenter[0], ops2.mCenter[1], 0),
                makeVector3f(ops2.mPoints[ii][0], ops2.mPoints[ii][1], 0),
                makeVector3f(1, 0, 0),
                makeVector3f(0, 0, 1),
                3);
        }
        debugRenderer->endRenderJob();
    }
    {
        OPoint2f p = { 1.f, 2.f };
        ops2.Insert(p);
        Vector3f p3;
        p3[0] = p[0]; p3[1] = p[1]; p3[2] = 0;
        debugRenderer->beginRenderJob_OneFrame("feature_edges_", DR_FRAME++);
        debugRenderer->addPoint(p3,
            makeVector3f(1, 1, 0));
        debugRenderer->addPoint(
            makeVector3f(ops2.mCenter[0], ops2.mCenter[1], 0),
            makeVector3f(1, 0, 0));
        for (int ii = 0; ii < ops2.mPoints.size(); ++ii) {
            debugRenderer->addLine(
                makeVector3f(ops2.mCenter[0], ops2.mCenter[1], 0),
                makeVector3f(ops2.mPoints[ii][0], ops2.mPoints[ii][1], 0),
                makeVector3f(1, 0, 0),
                makeVector3f(0, 0, 1),
                3);
        }
        debugRenderer->endRenderJob();
    }
    {
        OPoint2f p = { -3.f, 2.f };
        ops2.Insert(p);
        Vector3f p3;
        p3[0] = p[0]; p3[1] = p[1]; p3[2] = 0;
        debugRenderer->beginRenderJob_OneFrame("feature_edges_", DR_FRAME++);
        debugRenderer->addPoint(p3,
            makeVector3f(1, 1, 0));
        debugRenderer->addPoint(
            makeVector3f(ops2.mCenter[0], ops2.mCenter[1], 0),
            makeVector3f(1, 0, 0));
        for (int ii = 0; ii < ops2.mPoints.size(); ++ii) {
            debugRenderer->addLine(
                makeVector3f(ops2.mCenter[0], ops2.mCenter[1], 0),
                makeVector3f(ops2.mPoints[ii][0], ops2.mPoints[ii][1], 0),
                makeVector3f(1, 0, 0),
                makeVector3f(0, 0, 1),
                3);
        }
        debugRenderer->endRenderJob();
    }
    {
        OPoint2f p = { -3.f, -4.f };
        ops2.Insert(p);
        Vector3f p3;
        p3[0] = p[0]; p3[1] = p[1]; p3[2] = 0;
        debugRenderer->beginRenderJob_OneFrame("feature_edges_", DR_FRAME++);
        debugRenderer->addPoint(p3,
            makeVector3f(1, 1, 0));
        debugRenderer->addPoint(
            makeVector3f(ops2.mCenter[0], ops2.mCenter[1], 0),
            makeVector3f(1, 0, 0));
        for (int ii = 0; ii < ops2.mPoints.size(); ++ii) {
            debugRenderer->addLine(
                makeVector3f(ops2.mCenter[0], ops2.mCenter[1], 0),
                makeVector3f(ops2.mPoints[ii][0], ops2.mPoints[ii][1], 0),
                makeVector3f(1, 0, 0),
                makeVector3f(0, 0, 1),
                3);
        }
        debugRenderer->endRenderJob();
    }
    {
        OPoint2f p = { 5.f, -4.f };
        ops2.Insert(p);
        Vector3f p3;
        p3[0] = p[0]; p3[1] = p[1]; p3[2] = 0;
        debugRenderer->beginRenderJob_OneFrame("feature_edges_", DR_FRAME++);
        debugRenderer->addPoint(p3,
            makeVector3f(1, 1, 0));
        debugRenderer->addPoint(
            makeVector3f(ops2.mCenter[0], ops2.mCenter[1], 0),
            makeVector3f(1, 0, 0));
        for (int ii = 0; ii < ops2.mPoints.size(); ++ii) {
            debugRenderer->addLine(
                makeVector3f(ops2.mCenter[0], ops2.mCenter[1], 0),
                makeVector3f(ops2.mPoints[ii][0], ops2.mPoints[ii][1], 0),
                makeVector3f(1, 0, 0),
                makeVector3f(0, 0, 1),
                3);
        }
        debugRenderer->endRenderJob();
    }

    for (int ii = 0; ii < ops2.mPoints.size(); ++ii) {
        debugRenderer->beginRenderJob_OneFrame("feature_edges_", DR_FRAME++);
        debugRenderer->addLine(
            makeVector3f(ops2.mCenter[0], ops2.mCenter[1], 0),
            makeVector3f(ops2.mPoints[ii][0], ops2.mPoints[ii][1], 0),
            makeVector3f(1, 0, 0),
            makeVector3f(0, 0, 1),
            3);
        debugRenderer->endRenderJob();
    }

}
