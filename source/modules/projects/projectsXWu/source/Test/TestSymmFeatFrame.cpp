#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Test/PCITestCentr.h"
//---------------------------------------------------------------------------
#include "Timer.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

#include "Symmetry/SymmFeatFrame.h"
#include "Symmetry/SymmDetCont.h"
#include "Symmetry/SymmFeatProxy.h"
#include "Symmetry/SymmDetHiPrec.h"
void PCITestCentr::TestSymmFeatFrame(void)
{
    UnstructuredInCoreTriangleMesh* mesh = nullptr;
    if (!get1stTrimesh(mesh)) return;

    TrimeshStatic::Ptr smesh(new TrimeshStatic(mesh));

    SymmDetHiPrec::Ptr symmDetHiPrec (new SymmDetHiPrec);
    SymmDetCont::Ptr symmDetCont (new SymmDetCont);
    symmDetCont->Clear();
    symmDetHiPrec->SetContext(symmDetCont);
    symmDetHiPrec->DetectSymmtry(smesh);

    // setup global properties	
    BoundingBox3f meshBoundingBox = getPCBBox(&*smesh->GetMesh());
    symmDetCont->diagonal_length = meshBoundingBox.getDiagonalLength();
    symmDetCont->spatial_tolerance = meshBoundingBox.getDiagonalLength()
        * symmDetCont->relative_spatial_tolerance;
    symmDetCont->grid_size = meshBoundingBox.getDiagonalLength()
        * symmDetCont->relative_grid_size;
    symmDetCont->sameline_th = 2 * symmDetCont->grid_size;
    symmDetCont->min_feat_length = 10 * symmDetCont->grid_size;

    // construct a proxy
    SymmFeatProxyLine::Ptr symmFeatProxyLine (new SymmFeatProxyLine);
    symmDetCont->smesh = smesh;
    symmFeatProxyLine->SetContext(symmDetCont);

    // compute lines features
    symmFeatProxyLine->ExtractFeature();

    SymmFeatFrame symmFeatFrame(symmDetCont->featArr);
    debugRenderer->beginRenderJob_OneFrame("frame_", DR_FRAME++);
    symmFeatFrame.DrawWithDR();
    debugRenderer->endRenderJob();

    VertMap verts = symmFeatFrame.verts_;
    EdgeVec edges = symmFeatFrame.edges_;
    VertMap::iterator vi;

    for (vi = verts.begin(); vi != verts.end(); ++vi) {
        debugRenderer->beginRenderJob_OneFrame("verts_", DR_FRAME++);
        std::vector<size_t> vv = symmFeatFrame.GetVertVerts(vi->first);
        for (size_t ii = 0; ii < vv.size(); ++ii) {
            debugRenderer->addLine(
                smesh->GetVertPosition(vi->first),
                smesh->GetVertPosition(vv[ii]),
                makeVector3f(1, 0, 0),
                makeVector3f(0, 0, 1),
                3);
        }
        debugRenderer->endRenderJob();
    }

    for (vi = verts.begin(); vi != verts.end(); ++vi) {
        debugRenderer->beginRenderJob_OneFrame("edges_", DR_FRAME++);
        std::vector<size_t> ve = symmFeatFrame.GetVertEdges(vi->first);
        for (size_t ii = 0; ii < ve.size(); ++ii) {
            debugRenderer->addLine(
                smesh->GetVertPosition(vi->first),
                smesh->GetVertPosition(symmFeatFrame.GetEdge2ndVert(ve[ii], vi->first)),
                makeVector3f(1, 0, 0),
                makeVector3f(0, 0, 1),
                3);
        }
        debugRenderer->endRenderJob();
    }
}
