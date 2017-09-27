#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Test/PCITestCentr.h"
//---------------------------------------------------------------------------
#include "Timer.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

#include "Symmetry/PatternRotation.h"
namespace {
    void test_construction(void)
    {
        array3vec3f pvec = {
            makeVector3f(1, 1, 3),
            makeVector3f(1, 1, -3),
            makeVector3f(2, 0.5, 0)
        };
        PatternRotation::Ptr rotpat (new PatternRotation);
        Vector3i ivec;
        if (!rotpat->SetupFrom3P(pvec, ivec)) {
            warning("PCITestCentr::TestPatternRotation: "
                "construction failed.");
            return;
        }

        Vector3f cen = rotpat->center_;
        debugRenderer->beginRenderJob_OneFrame("rotation_pattern_", DR_FRAME++);
        debugRenderer->addPoint(cen, makeVector3f(1, 1, 0));
        for (int ii = 0; ii < 3; ++ii) {
            debugRenderer->addLine(
                cen, pvec[ii],
                makeVector3f(1, 0, 0),
                makeVector3f(0, 0, 1),
                3);
        }
        rotpat->DrawWithDR();

        Vector3f cen_t = transformVector3f(rotpat->toOYZ_, cen);
        debugRenderer->addPoint(cen_t, makeVector3f(1, 1, 0));
        for (int ii = 0; ii < 3; ++ii) {
            debugRenderer->addLine(
                cen_t, transformVector3f(rotpat->toOYZ_, pvec[ii]),
                makeVector3f(1, 0, 0),
                makeVector3f(0, 0, 1),
                3);
        }
        debugRenderer->endRenderJob();
    }

    void test_orbit_check(void)
    {
        const int num = 6;
        array3vec3f pvec;
        boost::array<Vector3f, num> pvecfull;
        {
            float astep = 2 * M_PI / num;
            for (int ii = 0; ii < num; ++ii) {
                pvecfull[ii] = makeVector3f(
                    cos(astep*ii), sin(astep*ii), 0);
                pvecfull[ii] += makeVector3f(1, 2, 1);
            }
            pvec[0] = pvecfull[1];
            pvec[1] = pvecfull[2];
            pvec[2] = pvecfull[3];
        }

        PatternRotation::Ptr rotpat (new PatternRotation);
        Vector3i ivec;
        if (!rotpat->SetupFrom3P(pvec, ivec)) {
            warning("PCITestCentr::TestPatternRotation: "
                "construction failed.");
            return;
        }

        debugRenderer->beginRenderJob_OneFrame("rotation_pattern_", DR_FRAME++);
        rotpat->DrawWithDR();
        debugRenderer->endRenderJob();
        for (int ii = 4; ii < num; ++ii) {
            int ix;
            if (!rotpat->PointOnOrbit(pvecfull[ii], ix)) {
                warning("PCITestCentr::TestPatternRotation: "
                    "orbit check failed.");
            }
            rotpat->AddPoint(pvecfull[ii], ix);
            debugRenderer->beginRenderJob_OneFrame("rotation_pattern_", DR_FRAME++);
            rotpat->DrawWithDR();
            debugRenderer->endRenderJob();
        }

        Vector3f p4 = pvecfull[0];
        {
            Vector3f p = p4;
            int ix;
            debugOutput << rotpat->PointOnOrbit(p, ix) << ": "
                << norm(p-p4) << "\n";
        }
        {
            Vector3f p = p4;
            p[0] += 0.0001f;
            int ix;
            debugOutput << rotpat->PointOnOrbit(p, ix, 1e-2) << ": "
                << norm(p-p4) << "\n";
        }
        {
            Vector3f p = p4;
            p[0] += 0.001f;
            int ix;
            debugOutput << rotpat->PointOnOrbit(p, ix, 1e-2) << ": "
                << norm(p-p4) << "\n";
        }
        Vector3f cen = rotpat->center_;
        Vector3f cen_t = transformVector3f(rotpat->toOYZ_, cen);
        debugRenderer->beginRenderJob_OneFrame("rotation_pattern_", DR_FRAME++);
        for (int ii = 0; ii < num; ++ii) {
            debugRenderer->addLine(
                pvecfull[ii], pvecfull[(ii+1)%num],
                makeVector3f(1, 0, 0),
                makeVector3f(0, 0, 1),
                3);
            debugRenderer->addLine(
                transformVector3f(rotpat->toOYZ_, pvecfull[ii]),
                transformVector3f(rotpat->toOYZ_, pvecfull[(ii+1)%num]),
                makeVector3f(1, 0, 0),
                makeVector3f(0, 0, 1),
                3);
        }
        debugRenderer->addPoint(cen, makeVector3f(1, 1, 0));
        debugRenderer->addPoint(p4, makeVector3f(1, 1, 0));
        for (int ii = 0; ii < 3; ++ii) {
            debugRenderer->addLine(
                cen, pvec[ii],
                makeVector3f(1, 0, 0),
                makeVector3f(0, 0, 1),
                3);
        }
        debugRenderer->endRenderJob();
    }
}

void test_internal_transform(void)
{
    const int num = 6;
    array3vec3f pvec;
    boost::array<Vector3f, num> pvecfull, pvecfull_t;
    {
        float astep = 2 * M_PI / num;
        for (int ii = 0; ii < num; ++ii) {
            pvecfull[ii] = makeVector3f(
                cos(astep*ii), sin(astep*ii), 0);
            pvecfull[ii] += makeVector3f(1, 2, 1);
        }
        pvec[0] = pvecfull[1];
        pvec[1] = pvecfull[2];
        pvec[2] = pvecfull[3];
    }

    PatternRotation::Ptr rotpat (new PatternRotation);
    Vector3i ivec;
    if (!rotpat->SetupFrom3P(pvec, ivec)) {
        warning("PCITestCentr::TestPatternRotation: "
            "construction failed.");
        return;
    }
    for (int ii = 0; ii < num; ++ii) {
        pvecfull_t[ii] = transformVector3f(rotpat->toOYZ_, pvecfull[ii]);
    }

    const int num_2 = num + 1;
    boost::array<Vector3f, num_2> pvec_2, pvec_3;
    {
        float astep = 2 * M_PI / num_2;
        for (int ii = 0; ii < num_2; ++ii) {
            pvec_2[ii] = makeVector3f(
                0, cos(astep*ii), sin(astep*ii));
            pvec_3[ii] = transformVector3f(rotpat->toWorld_, pvec_2[ii]);
        }
    }

    Vector3f cen = rotpat->center_;
    Vector3f cen_t = transformVector3f(rotpat->toOYZ_, rotpat->center_);
    Vector3f cen_i = NULL_VECTOR3F;
    Vector3f cen_w = transformVector3f(rotpat->toWorld_, cen_i);
    debugRenderer->beginRenderJob_OneFrame("rotation_pattern_", DR_FRAME++);
    debugOutput << rotpat->toOYZ_ << "\n";
    debugOutput << rotpat->toWorld_ << "\n";

    debugRenderer->addPoint(cen, makeVector3f(1, 1, 0));
    debugRenderer->addLine(
        cen, cen+rotpat->normal_,
        makeVector3f(1, 0, 0),
        makeVector3f(0, 0, 1),
        3);
    for (int ii = 0; ii < num; ++ii) {
        debugRenderer->addLine(
            pvecfull_t[ii], pvecfull_t[(ii+1)%num],
            makeVector3f(1, 0, 0),
            makeVector3f(0, 0, 1),
            3);
        debugRenderer->addLine(
            cen_t, pvecfull_t[ii],
            makeVector3f(0, 1, 0),
            makeVector3f(0, 1, 0),
            3);
    }

    debugRenderer->addPoint(cen_i, makeVector3f(1, 1, 0));
    for (int ii = 0; ii < num_2; ++ii) {
        debugRenderer->addLine(
            pvec_2[ii], pvec_2[(ii+1)%num_2],
            makeVector3f(1, 0, 0),
            makeVector3f(0, 0, 1),
            3);
        debugRenderer->addLine(
            cen_i, pvec_2[ii],
            makeVector3f(0, 1, 0),
            makeVector3f(0, 1, 0),
            3);
    }

    debugRenderer->addPoint(cen_w, makeVector3f(1, 1, 0));
    debugRenderer->addLine(
        cen_w, cen_w+rotpat->normal_,
        makeVector3f(1, 0, 0),
        makeVector3f(0, 0, 1),
        3);
    for (int ii = 0; ii < num_2; ++ii) {
        debugRenderer->addLine(
            pvec_3[ii], pvec_3[(ii+1)%num_2],
            makeVector3f(1, 0, 0),
            makeVector3f(0, 0, 1),
            3);
        debugRenderer->addLine(
            cen_w, pvec_3[ii],
            makeVector3f(0, 1, 0),
            makeVector3f(0, 1, 0),
            3);
    }
    for (int ii = 0; ii < num_2; ++ii) {
        debugRenderer->addLine(
            pvec_2[ii], pvec_3[ii],
            makeVector3f(1, 0, 0),
            makeVector3f(0, 0, 1),
            3);
    }

    debugRenderer->endRenderJob();
}

void test_rotated_position(void)
{
    const int num = 7;
    array3vec3f pvec;
    boost::array<Vector3f, num> pvecfull, pvecfull_t;
    {
        float astep = 2 * M_PI / num;
        for (int ii = 0; ii < num; ++ii) {
            pvecfull[ii] = makeVector3f(
                cos(astep*ii), sin(astep*ii), 0);
            pvecfull[ii] += makeVector3f(1, 2, 1);
        }
        pvec[0] = pvecfull[1];
        pvec[1] = pvecfull[2];
        pvec[2] = pvecfull[3];
    }

    PatternRotation::Ptr rotpat (new PatternRotation);
    Vector3i ivec;
    if (!rotpat->SetupFrom3P(pvec, ivec)) {
        warning("PCITestCentr::TestPatternRotation: "
            "construction failed.");
        return;
    }
    for (int ii = 0; ii < num; ++ii) {
        pvecfull_t[ii] = transformVector3f(rotpat->toOYZ_, pvecfull[ii]);
    }

    boost::array<Vector3f, num> pvecnew;
    pvecnew[0] = makeVector3f(3, 4, 5);
    for (int ii = 1; ii < num; ++ii) {
        Matrix4f T4;
        rotpat->GetRotatedPos(
            pvecnew[0], ii, pvecnew[ii], T4);
    }

    Vector3f cen = rotpat->center_;
    Vector3f cen_t = transformVector3f(rotpat->toOYZ_, rotpat->center_);
    debugRenderer->beginRenderJob_OneFrame("rotation_pattern_", DR_FRAME++);

    debugRenderer->addPoint(cen, makeVector3f(1, 1, 0));
    debugRenderer->addLine(
        cen, cen+rotpat->normal_,
        makeVector3f(1, 0, 0),
        makeVector3f(0, 0, 1),
        3);
    for (int ii = 0; ii < num; ++ii) {
        debugRenderer->addLine(
            pvecfull_t[ii], pvecfull_t[(ii+1)%num],
            makeVector3f(1, 0, 0),
            makeVector3f(0, 0, 1),
            3);
        debugRenderer->addLine(
            cen_t, pvecfull_t[ii],
            makeVector3f(0, 1, 0),
            makeVector3f(0, 1, 0),
            3);
    }

    for (int ii = 0; ii < num; ++ii) {
        debugRenderer->addLine(
            pvecnew[ii], pvecnew[(ii+1)%num],
            makeVector3f(1, 0, 0),
            makeVector3f(0, 0, 1),
            3);
        debugRenderer->addLine(
            cen, pvecnew[ii],
            makeVector3f(0, 1, 0),
            makeVector3f(0, 1, 0),
            3);
    }
    debugRenderer->endRenderJob();
}

void PCITestCentr::TestPatternRotation(void)
{
    //test_construction();
    test_rotated_position();
    test_internal_transform();
    test_orbit_check();
}
