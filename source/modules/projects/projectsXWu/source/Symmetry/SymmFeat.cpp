#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Symmetry/SymmFeat.h"
#include "Util/NoUse.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

void SymmFeatLine::GetFrame(Matrix3f& frame, Vector3f& origin) const
{
    frame[0] = dir;
    frame[1] = normal;
    frame[2] = frame[0].crossProduct(frame[1]);
    origin = cen;
}

void SymmFeatLine::Setup(TrimeshStatic::Ptr smesh, const unsigned& indx)
{
    bool bb = smesh->IsEdgeBorder(indx);
    if (bb) {
        Setup(
            smesh->GetVertPosition(smesh->GetEdgeVert0(indx)),
            smesh->GetVertPosition(smesh->GetEdgeVert1(indx)),
            smesh->GetFaceNormal(smesh->GetEdgeFace(indx, 0)),
            NULL_VECTOR3F,
            bb);
    } else {
        Setup(
            smesh->GetVertPosition(smesh->GetEdgeVert0(indx)),
            smesh->GetVertPosition(smesh->GetEdgeVert1(indx)),
            smesh->GetFaceNormal(smesh->GetEdgeFace(indx, 0)),
            smesh->GetFaceNormal(smesh->GetEdgeFace(indx, 1)),
            bb);
    }
    edges.push_back(indx);
    v0 = smesh->GetEdgeVert0(indx);
    v1 = smesh->GetEdgeVert1(indx);
    vertice.push_back(v0);
    vertice.push_back(v1);
}

void SymmFeatLine::Setup(
           const Vector3f& p0, const Vector3f& p1,
           const Vector3f& n0, const Vector3f& n1,
           bool bb)
{
    edges.clear();
    v0 = 0; v1 = 0;
    vertice.clear();
    pos0 = p0; pos1 = p1;
    dir = pos1 - pos0;
    cen = (pos0 + pos1) / 2.f;
    leng = norm(dir);
    dir /= leng;
    isBder = bb;
    surfaceN0 = n0; surfaceN1 = n1;
    normal = normalize(surfaceN0 + surfaceN1);
    normalAngle = surfaceN0 * surfaceN1;
}

SymmFeatLine::Ptr SymmFeatLine::CopyTo(void)
{
    SymmFeatLine::Ptr line (new SymmFeatLine);
    line->edges = edges;
    line->vertice = vertice;
    line->v0 = v0; line->v1 = v1;
    line->pos0 = pos0; line->pos1 = pos1;
    line->cen = cen;
    line->dir = dir;
    line->leng = leng;
    line->isBder = isBder;
    line->surfaceN0 = surfaceN0; line->surfaceN1 = surfaceN1;
    line->normal = normal;
    line->normalAngle = normalAngle;
    return line;
}

bool SymmFeatLine::SimilarTo(SymmFeat::Ptr osf, const float& th) const
{
    SymmFeatLine::Ptr osfl = 
        boost::dynamic_pointer_cast<SymmFeatLine>(osf);
    if (nullptr == osfl)
        return false;

    return fabs(leng - osfl->leng) < th
        //&& fabs(normalAngle - osfl->normalAngle) < 5e-3
        && isBder == osfl->isBder
        && edges.size() == osfl->edges.size()
        && vertice.size() == osfl->vertice.size();
}

LineDir SymmFeatLine::SameAs(SymmFeatLine::Ptr osf, const float& th) const
{
    if (!SimilarTo(osf, th)) return E_DIR;
    if (!NU::IsParallel(dir, osf->dir)) return E_DIR;

    if (norm(pos0 - osf->pos0) < th && norm(pos1 - osf->pos1) < th)
        return SAME_DIR;
    else if (norm(pos0 - osf->pos1) < th && norm(pos1 - osf->pos0) < th)
        return DIFF_DIR;
    else return E_DIR;
}

void SymmFeatLine::UniformDirection(void)
{
    bool changed = false;
    if (numeric_limits<float>::epsilon() < abs(dir[0])) {
        if (0 > dir[0]) { dir *= -1.f; changed = true; }
    } else if (numeric_limits<float>::epsilon() < abs(dir[1])) {
        if (0 > dir[1]) { dir *= -1.f; changed = true; }
    } else if (numeric_limits<float>::epsilon() > dir[2]) { dir *= -1.f; changed = true; }
    if (!changed) return;
    Reverse();
}

void SymmFeatLine::Reverse(void) 
{
    reverse(edges.begin(), edges.end());
    reverse(vertice.begin(), vertice.end());
    { mpcard tmp = v0; v0 = v1; v1 = tmp; }
    { Vector3f tmp = pos0; pos0 = pos1; pos1 = tmp; }
    { Vector3f tmp = surfaceN0; surfaceN0 = surfaceN1; surfaceN1 = tmp; }
}

void SymmFeatLine::DrawWithDR(void)
{
    debugRenderer->addLine(
        pos0, pos1,
        makeVector3f(1, 0, 0),
        makeVector3f(0, 0, 1),
        3);
    debugRenderer->addPoint(
        pos0,
        makeVector3f(1, 1, 0)
        );
    debugRenderer->addPoint(
        pos1,
        makeVector3f(1, 1, 0)
        );
}

bool SymmFeatLine::Join(TrimeshStatic::Ptr smesh, SymmFeatLine::Ptr ol)
{
  //####################################################################
  //BEGIN TODO: For degenerate meshes, the algorithm tries to join lines
  //            with themselves. In addition, there seem to be
  //            configurations that let vertice grow to beyond 5 million
  //            elements... Hydrofoil.obj leads to such a case.
  //####################################################################
  if (this == ol.get())
  {
    warning("SymmFeatLine::Join() - trying to join with self.");

    return false;
  }
  //####################################################################
  // END TODO
  //####################################################################

    if (!NU::IsParallel(dir, ol->dir) ||
        fabs(fabs(normalAngle) - fabs(ol->normalAngle)) > 1e-5 ||
        isBder != ol->isBder
        ) return false;

    std::deque<mpcard>::iterator it, jt;
    std::deque<mpcard>::reverse_iterator rit, rjt;

    mpcard mp;
    if (v0 == ol->v0) {
        mp = v0;
        for (it = ol->edges.begin(); it != ol->edges.end(); ++it) {
            edges.push_front(*it);
        }
        vertice.pop_front();
        for (jt = ol->vertice.begin(); jt != ol->vertice.end(); ++jt) {
            vertice.push_front(*jt);
        }
        v0 = ol->v1;
        pos0 = smesh->GetVertPosition(v0);
        cen = (pos0 + pos1) / 2.f;
        leng = norm(pos1 - pos0);
    } else if (v0 == ol->v1) {
        mp = v0;
        for (rit = ol->edges.rbegin(); rit != ol->edges.rend(); ++rit) {
            edges.push_front(*rit);
        }
        vertice.pop_front();
        for (rjt = ol->vertice.rbegin(); rjt != ol->vertice.rend(); ++rjt) {
            vertice.push_front(*rjt);
        }
        v0 = ol->v0;
        pos0 = smesh->GetVertPosition(v0);
        cen = (pos0 + pos1) / 2.f;
        leng = norm(pos1 - pos0);
    } else if (v1 == ol->v0) {
        mp = v1;
        for (it = ol->edges.begin(); it != ol->edges.end(); ++it) {
            edges.push_back(*it);
        }
        vertice.pop_back();
        for (jt = ol->vertice.begin(); jt != ol->vertice.end(); ++jt) {
            vertice.push_back(*jt);
        }
        v1 = ol->v1;
        pos1 = smesh->GetVertPosition(v1);
        cen = (pos0 + pos1) / 2.f;
        leng = norm(pos1 - pos0);
    } else if (v1 == ol->v1) {
        mp = v1;
        for (rit = ol->edges.rbegin(); rit != ol->edges.rend(); ++rit) {
            edges.push_back(*rit);
        }
        vertice.pop_back();
        for (rjt = ol->vertice.rbegin(); rjt != ol->vertice.rend(); ++rjt) {
            vertice.push_back(*rjt);
        }
        v1 = ol->v0;
        pos1 = smesh->GetVertPosition(v1);
        cen = (pos0 + pos1) / 2.f;
        leng = norm(pos1 - pos0);
    } else return false;

    //debugRenderer->beginRenderJob_OneFrame("feature_edges_", DR_FRAME++);
    //debugRenderer->addLine(
    //    pos0, pos1,
    //    makeVector3f(1, 0, 0),
    //    makeVector3f(0, 0, 1),
    //    3);
    //debugRenderer->addPoint(
    //    smesh->GetVertexPosition(mp),
    //    makeVector3f(1, 1, 0));
    //debugRenderer->endRenderJob();

    return true;
}

SymmFeatSetLine::SymmFeatSetLine(const SymmFeatLine::Ptr& sfl)
{
    lset.clear();
    lset.push_back(sfl);
    elem = sfl;
    cen = NULL_VECTOR3F;
}

void SymmFeatSetLine::Insert(const SymmFeatLine::Ptr& sfl, const int& indx)
{
    size_t num = (float)lset.size();
    cen = normalize(
        cen * num / (num+1) +
        sfl->normal / (num+1));
    lset.push_back(sfl);

    // elements inserted are guaranteed to be similar
    std::nth_element(lset.begin(), lset.begin()+1, lset.end(),
        boost::bind(&SymmFeatSetLine::near_center,
        this, cen, _1, _2));
    elem = *lset.begin();
}
