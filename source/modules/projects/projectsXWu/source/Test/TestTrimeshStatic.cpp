#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Test/PCITestCentr.h"
//---------------------------------------------------------------------------
#include "Timer.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

#include "Util/TrimeshStatic.h"
#include "VertexArray.h"
void PCITestCentr::TestTrimeshStatic(void)
{
    UnstructuredInCoreTriangleMesh* mesh = nullptr;
    if (!get1stTrimesh(mesh)) return;

    TrimeshStatic smesh(mesh);
    //VertexArray& va_vert = *smesh.GetVertArray();
    //VertexArray& va_face = *smesh.GetFaceArray();
    const TSEdgeV& edges = smesh.GetEdges();
    const TSVertV& verts = smesh.GetVerts();
    const TSFaceV& faces = smesh.GetFaces();

    for (mpcard ei = 0; ei < edges.size(); ++ei) {
        std::stringstream ss;
        ss << "edge #" << ei << ": " << edges[ei]->halfs.size() << "\n";
        for (mpcard hi = 0; hi < edges[ei]->halfs.size(); ++hi) {
            TSHalf* edge_h = edges[ei]->halfs[hi];
            ss << "  " << edge_h->indx << ": "
                << edge_h->vsrc << " -> " << edge_h->vsrc << " " << edge_h->face
                << ", uplk: " << edge_h->uplk->indx << "\n";
            ss << "    next: " << edge_h->next->indx << " of "
                << edge_h->next->uplk->indx << "\n";
            ss << "    prev: " << edge_h->prev->indx << " of "
                << edge_h->prev->uplk->indx << "\n";
            //debugOutput << ss.str();
        }
    }

    //debugRenderer->clearDebugData();

    //debugRenderer->beginRenderJob_OneFrame("borders_", DR_FRAME++);
    //for (mpcard ei = 0; ei < edges.size(); ++ei) {
    //    if (!smesh.IsEdgeBorder(ei)) continue;
    //    debugRenderer->addLine(
    //        smesh.GetVertPosition(smesh.GetEdgeVert0(ei)),
    //        smesh.GetVertPosition(smesh.GetEdgeVert1(ei)),
    //        makeVector3f(1, 0, 0),
    //        makeVector3f(0, 0, 1),
    //        3);
    //}
    //debugRenderer->endRenderJob();

    //debugRenderer->beginRenderJob_OneFrame("creases_", DR_FRAME++);
    //for (mpcard ei = 0; ei < edges.size(); ++ei) {
    //    if (!smesh.IsEdgeCrease(ei)) continue;
    //    debugRenderer->addLine(
    //        smesh.GetVertPosition(smesh.GetEdgeVert0(ei)),
    //        smesh.GetVertPosition(smesh.GetEdgeVert1(ei)),
    //        makeVector3f(1, 0, 0),
    //        makeVector3f(0, 0, 1),
    //        3);
    //}
    //debugRenderer->endRenderJob();

    //debugRenderer->beginRenderJob_OneFrame("regular_edges_", DR_FRAME++);
    //for (mpcard ei = 0; ei < edges.size(); ++ei) {
    //    if (!smesh.IsEdgeRegular(ei)) continue;
    //    debugRenderer->addLine(
    //        smesh.GetVertPosition(smesh.GetEdgeVert0(ei)),
    //        smesh.GetVertPosition(smesh.GetEdgeVert1(ei)),
    //        makeVector3f(1, 0, 0),
    //        makeVector3f(0, 0, 1),
    //        3);
    //}
    //debugRenderer->endRenderJob();

    //debugRenderer->beginRenderJob_OneFrame("multiedges_", DR_FRAME++);
    //for (mpcard ei = 0; ei < edges.size(); ++ei) {
    //    if (!smesh.IsEdgeMultiple(ei)) continue;
    //    debugRenderer->addLine(
    //        smesh.GetVertPosition(smesh.GetEdgeVert0(ei)),
    //        smesh.GetVertPosition(smesh.GetEdgeVert1(ei)),
    //        makeVector3f(1, 0, 0),
    //        makeVector3f(0, 0, 1),
    //        3);
    //}
    //debugRenderer->endRenderJob();

    //debugRenderer->beginRenderJob_OneFrame("multiedges_", DR_FRAME++);
    //for (mpcard fi = 0; fi < faces.size(); ++fi) {
    //    Vector3f fn = smesh.GetFaceNormal(fi);
    //    Vector3f v0 = smesh.GetFaceCentroid(fi);
    //    debugRenderer->addLine(
    //        v0, v0+fn,
    //        makeVector3f(1, 0, 0),
    //        makeVector3f(0, 0, 1),
    //        3);
    //}
    //debugRenderer->endRenderJob();

    //for (mpcard fi = 0; fi < faces.size(); ++fi) {
    //    std::stringstream ss;
    //    std::vector<mpcard> nfs = smesh.GetFaceAdjacentFaces(fi);
    //    debugRenderer->beginRenderJob_OneFrame("face_neighbors_", DR_FRAME++);
    //    ss << "face #" << fi << "(" << nfs.size() << "): ";
    //    for (mpcard ni = 0; ni < nfs.size(); ++ni) {
    //        ss << nfs[ni] << " ";
    //        debugRenderer->addLine(
    //            smesh.GetFaceCentroid(fi),
    //            smesh.GetFaceCentroid(nfs[ni]),
    //            makeVector3f(1, 0, 0),
    //            makeVector3f(0, 0, 1),
    //            3);
    //    }
    //    ss << "\n";
    //    debugOutput << ss.str();
    //    debugRenderer->endRenderJob();
    //}

    for (mpcard vi = 0; vi < verts.size(); ++vi) {
        debugRenderer->beginRenderJob_OneFrame("vertex_neighbors_", DR_FRAME++);
        debugRenderer->addPoint(
            smesh.GetVertPosition(vi),
            makeVector3f(1, 1, 0));

        std::vector<mpcard> n_vert = smesh.GetVert1RingVerts(vi);
        for (mpcard ni = 0; ni < n_vert.size(); ++ni) {
            debugRenderer->addLine(
                smesh.GetVertPosition(n_vert[ni]),
                smesh.GetVertPosition(n_vert[(ni+1)%n_vert.size()]),
                makeVector3f(1, 0, 0),
                makeVector3f(0, 0, 1),
                3);
            debugOutput << n_vert[ni] << " ";
        }
        debugOutput << "\n";

        std::vector<mpcard> n_face = smesh.GetVert1RingFaces(vi);
        for (mpcard ni = 0; ni < n_face.size(); ++ni) {
            debugRenderer->addLine(
                smesh.GetFaceCentroid(n_face[ni]),
                smesh.GetFaceCentroid(n_face[(ni+1)%n_face.size()]),
                makeVector3f(1, 0, 0),
                makeVector3f(0, 0, 1),
                3);
            debugOutput << n_face[ni] << " ";
        }
        debugOutput << "\n";

        //std::vector<mpcard> n_edge = smesh.GetVert1RingEdges(vi);
        //for (mpcard ni = 0; ni < n_edge.size(); ++ni) {
        //    debugRenderer->addLine(
        //        smesh.GetVertPosition(smesh.GetEdgeVert0(n_edge[ni])),
        //        smesh.GetVertPosition(smesh.GetEdgeVert1(n_edge[ni])),
        //        makeVector3f(1, 0, 0),
        //        makeVector3f(0, 0, 1),
        //        3);
        //}

        debugRenderer->endRenderJob();
    }
}
