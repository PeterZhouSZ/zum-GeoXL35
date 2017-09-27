#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Test/PCITestCentr.h"
//---------------------------------------------------------------------------
#include "Timer.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

#include "Symmetry/PatternTranslation.h"
namespace {

}

void PCITestCentr::TestPatternTranslation(void)
{
    array2vec3f pvec = {
        makeVector3f(1, 1, 1),
        makeVector3f(2, 2, 2)
    };
    PatternTranslation::Ptr trapat (new PatternTranslation);
    Vector2i ivec;
    if (!trapat->SetupFrom2P(pvec, ivec)) {
        warning("PCITestCentr::TestPatternTranslation: "
            "construction failed.");
        return;
    }

    {
        Vector3f p = makeVector3f(4, 4, 4);
        int ix;
        if (!trapat->PointOnOrbit(p, ix)) {
            warning("PCITestCentr::TestPatternTranslation: "
                "orbit check failed.");
        }
        trapat->AddPoint(p, ix);
    }

    {
        Vector3f p = makeVector3f(-1, -1, -1);
        int ix;
        if (!trapat->PointOnOrbit(p, ix)) {
            warning("PCITestCentr::TestPatternTranslation: "
                "orbit check failed.");
        }
        trapat->AddPoint(p, ix);
    }

    debugRenderer->beginRenderJob_OneFrame("translation_pattern_", DR_FRAME++);
    trapat->DrawWithDR();
    debugRenderer->endRenderJob();

    OrbitPosMap::const_iterator jt, it = trapat->pmap_.begin();
    for (jt = it; ; ++it) {
        if (++jt == trapat->pmap_.end()) break;
        debugRenderer->beginRenderJob_OneFrame("translation_pattern_", DR_FRAME++);
        debugRenderer->addLine(
            it->second, jt->second,
            makeVector3f(1, 0, 0),
            makeVector3f(0, 0, 1),
            3);
        debugRenderer->addPoint(it->second, makeVector3f(1, 1, 0));
        debugRenderer->addPoint(jt->second, makeVector3f(1, 1, 0));
        debugRenderer->endRenderJob();
    }
}
