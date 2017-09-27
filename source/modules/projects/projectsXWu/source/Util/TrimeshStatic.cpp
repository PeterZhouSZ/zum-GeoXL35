#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Util/TrimeshStatic.h"
#include "Util/NoUse.h"
//---------------------------------------------------------------------------
#include "VertexArray.h"
//---------------------------------------------------------------------------

TSEdge::~TEdge()
{ std::for_each( halfs.begin(), halfs.end(), NU::delete_ptr<TSHalf>() ); }

void TrimeshStatic::cleanAndSetupMesh(const UnstructuredInCoreTriangleMesh* mesh,
                                      bool const process)
{
	{
		UnstructuredInCoreTriangleMeshPtr pmesh
			(dynamic_cast<UnstructuredInCoreTriangleMesh*>(mesh->copy()));
		mMesh = pmesh;
	}

    if (process)
    {
        mMesh->snapMesh(1e-6);
        mMesh = NU::OpenMesh(&*mMesh);
        mMesh->snapMesh(1e-6);
    }

    mVertArray = VertexArrayPtr(new VertexArray(mMesh->getPointSet()));
    mFaceArray = VertexArrayPtr(new VertexArray(mMesh->getTriangles()));
    mFaces.resize(mFaceArray->getNumElements());
    mVerts.resize(mVertArray->getNumElements());
}

void TrimeshStatic::retrieveFaceNeighbors(void)
{
    for (mpcard fi = 0; fi < mFaces.size(); ++fi) {
        const TSHalfV& halfs = mFaces[fi].halfs;
        TSHalfV::const_iterator it = halfs.begin();
        for (; it != halfs.end(); ++it) {
            TSEdge* edge = (*it)->uplk;
            for (mpcard hi = 0; hi < edge->halfs.size(); ++hi) {
                mpcard nfi = edge->halfs[hi]->face;
                if (fi != nfi)
                    mFaces[fi].neighbors.push_back(nfi);
            }
        }
    }
}

namespace {
    int DR_FRAME = 0;
}

void TrimeshStatic::sortVertNeighbors(void)
{
    for (mpcard vi = 0; vi < mVerts.size(); ++vi) {
        TVert& verti = mVerts[vi];
        std::vector<mpcard> verts_r, edges_r, faces_r;
        std::vector<mpcard> n_edges = GetVert1RingEdges(vi);

        mpcard startedge = n_edges[0];
        std::map<mpcard, mpcard> tab;
        for (mpcard ni = 0; ni < n_edges.size(); ++ni) {
            if (IsEdgeBorder(n_edges[ni])) startedge = n_edges[ni];
            tab[GetEdge2ndVert(n_edges[ni], vi)] = n_edges[ni];
        }
        mpcard cnt = 0;
        edges_r.push_back(startedge);
        verts_r.push_back(GetEdge2ndVert(startedge, vi));
        faces_r.push_back(GetEdgeFace(startedge, 0));

        mpcard currvert, curredge;
        currvert = GetFace3rdVert(faces_r[0], vi, verts_r[0]);
        curredge = tab[currvert];
        while (!IsEdgeBorder(curredge) && cnt < n_edges.size()-1) {
            verts_r.push_back(currvert);
            edges_r.push_back(curredge);
            faces_r.push_back(GetEdgeNextFace(curredge, faces_r[cnt]));
            ++cnt;
            currvert = GetFace3rdVert(faces_r[cnt], vi, verts_r[cnt]);
            curredge = tab[currvert];
        }
        if (IsEdgeBorder(curredge)) {
            verts_r.push_back(currvert);
            edges_r.push_back(curredge);
        }

        verti.verts = verts_r;
        verti.edges = edges_r;
        verti.faces = faces_r;
    }
}

TrimeshStatic::TrimeshStatic(const UnstructuredInCoreTriangleMesh* mesh,
                             bool const process)
{
    cleanAndSetupMesh(mesh, process);

    for (mpcard fi = 0; fi < mFaces.size(); ++fi) {
        TFace& face = mFaces[fi];
        Vector3i tri = mFaceArray->getIndex3i(fi);

        std::vector<THalf*>& halfs = face.halfs;
        for (mpcard j = 0; j < 3; ++j) {
            halfs.push_back(new THalf);

            VKey_ key; key.insert(tri[j]); key.insert(tri[(j+1)%3]);
            EdgeTab::iterator it = mVP2E.find(key);
            if (mVP2E.end() == it) continue;
            TEdge* edge = it->second;
            if (1 > edge->halfs.size()) continue;
            if (edge->halfs[0]->vsrc != tri[j]) continue;
            std::swap(tri[0], tri[2]);
            mFaceArray->setIndex3i(fi, tri);
        }

        array3vec3f verts;
        verts[0] = mVertArray->getPosition3f(tri[0]);
        verts[1] = mVertArray->getPosition3f(tri[1]);
        verts[2] = mVertArray->getPosition3f(tri[2]);
        NU::CompTriNormal(verts, face.normal);

        for (mpcard j = 0; j < 3; ++j) {
            halfs[j]->vsrc = tri[j];
            halfs[j]->vtgt = tri[(j+1)%3];
            halfs[j]->face = fi;
            halfs[j]->next = halfs[(j+1)%3];
            halfs[j]->prev = halfs[(j-1)%3];

            VKey_ key; key.insert(tri[j]); key.insert(tri[(j+1)%3]);
            EdgeTab::iterator it = mVP2E.find(key);
            TEdge* edge;
            if (mVP2E.end() == it) {
                size_t ei = mEdges.size();
                edge = new TEdge(ei);
                mEdges.push_back(edge);
                mVP2E[key] = edge;
                halfs[j]->uplk = edge;
                mVerts[halfs[j]->vsrc].edges.push_back(ei);
                mVerts[halfs[j]->vtgt].edges.push_back(ei);
                mVerts[halfs[j]->vsrc].verts.push_back(halfs[j]->vtgt);
                mVerts[halfs[j]->vtgt].verts.push_back(halfs[j]->vsrc);
            } else {
                edge = it->second;
                halfs[j]->uplk = edge;
            }

            halfs[j]->indx = edge->halfs.size();
            edge->halfs.push_back(halfs[j]);

            mVerts[halfs[j]->vsrc].halfs.push_back(halfs[j]);
            mVerts[halfs[j]->vtgt].halfs_in.push_back(halfs[j]);
            mVerts[halfs[j]->vsrc].faces.push_back(fi);
            mVerts[halfs[j]->vtgt].faces.push_back(fi);
        }
    }

    retrieveFaceNeighbors();
    if (2000 < mVerts.size()) return;
    sortVertNeighbors();
}

TrimeshStatic::~TrimeshStatic(void)
{
    std::for_each( mEdges.begin(), mEdges.end(), NU::delete_ptr<TEdge>() );
}

mpcard TrimeshStatic::GetNumVerts(void) const
{
    return mVerts.size();
}

Vector3f TrimeshStatic::GetVertPosition(mpcard vi) const
{
    return mVertArray->getPosition3f(vi);
}

Vector3f TrimeshStatic::GetVertNormal(mpcard vi) const 
{
    return mVertArray->getNormal3f(vi);
}

mpcard TrimeshStatic::GetVertNumEdges(mpcard vi) const 
{
    return mVerts[vi].edges.size();
}

std::vector<mpcard> TrimeshStatic::GetVert1RingVerts(mpcard vi) const
{
    return mVerts[vi].verts;
}

std::vector<mpcard> TrimeshStatic::GetVert1RingEdges(mpcard vi) const
{
    return mVerts[vi].edges;
}

std::vector<mpcard> TrimeshStatic::GetVert1RingFaces(mpcard vi) const
{
    return mVerts[vi].faces;
}

TSHalfV TrimeshStatic::GetVertHalfs(mpcard vi) const 
{
    return mVerts[vi].halfs;
}

TSHalf* TrimeshStatic::GetVertHalf(mpcard vi, mpcard ei) const 
{
    return mVerts[vi].halfs[ei];
}

std::vector<mpcard> TrimeshStatic::GetVertFaces(mpcard vi) const 
{
    return mVerts[vi].faces;
}

mpcard TrimeshStatic::GetNumEdges(void) const 
{
    return mEdges.size();
}

bool TrimeshStatic::IsEdgeBorder(mpcard ei) const 
{
    return mEdges[ei]->halfs.size() == 1;
}

bool TrimeshStatic::IsEdgeRegular(mpcard ei) const
{
    return mEdges[ei]->halfs.size() == 2;
}

bool TrimeshStatic::IsEdgeMultiple(mpcard ei) const
{
    return mEdges[ei]->halfs.size() > 2;
}

bool TrimeshStatic::IsEdgeCrease(mpcard ei) const
{
    if (!IsEdgeRegular(ei)) return false;
    Vector3f n0 = GetFaceNormal(mEdges[ei]->halfs[0]->face);
    Vector3f n1 = GetFaceNormal(mEdges[ei]->halfs[1]->face);
    return fabs(n0*n1) < 1.0f - 1e-4;
}

mpcard TrimeshStatic::GetEdgeVert0(mpcard ei) const 
{
    return mEdges[ei]->halfs[0]->vsrc;
}

mpcard TrimeshStatic::GetEdgeVert1(mpcard ei) const 
{
    return mEdges[ei]->halfs[0]->vtgt;
}

mpcard TrimeshStatic::GetEdge2ndVert(mpcard ei, mpcard vi) const
{
    THalf* h = mEdges[ei]->halfs[0];
    return (h->vsrc == vi) ? h->vtgt : h->vsrc;
}

mpcard TrimeshStatic::GetEdgeNumFaces(mpcard ei) const 
{
    return mEdges[ei]->halfs.size();
}

mpcard TrimeshStatic::GetEdgeFace(mpcard ei, mpcard fi) const 
{
    return mEdges[ei]->halfs[fi]->face;
}

mpcard TrimeshStatic::GetEdgeNextFace(mpcard ei, mpcard fi) const
{
    THalf* h = mEdges[ei]->halfs[0];
    size_t s = mEdges[ei]->halfs.size();
    return (h->face == fi) ? mEdges[ei]->halfs[1%s]->face : h->face;
}

mpcard TrimeshStatic::GetNumFaces(void) const 
{
    return mFaces.size();
}

Vector3f TrimeshStatic::GetFaceNormal(mpcard fi) const 
{
    return mFaces[fi].normal;
}

Vector3f TrimeshStatic::GetFaceCentroid(mpcard fi) const 
{
    const Vector3i tri = GetFaceVertices(fi);
    return (
        GetVertPosition(tri[0]) +
        GetVertPosition(tri[1]) +
        GetVertPosition(tri[2]) ) / 3.0f;
}

mpcard TrimeshStatic::GetFaceVert(mpcard fi, mpcard vi) const 
{
    return mFaceArray->getIndex3i(fi)[vi];
}

mpcard TrimeshStatic::GetFace3rdVert(mpcard fi, mpcard v0, mpcard v1) const
{
    Vector3i tri = mFaceArray->getIndex3i(fi);
    mpcard j;
    for (j = 0; j < 3; ++j) {
        if (tri[j] != v0 && tri[j] != v1) break;
    }
    return tri[j];
}

TSHalf* TrimeshStatic::GetFaceHalf(mpcard fi, mpcard ei) const 
{
    return mFaces[fi].halfs[ei];
}

Vector3i TrimeshStatic::GetFaceVertices(mpcard fi) const 
{
    return mFaceArray->getIndex3i(fi);
}

std::vector<mpcard> TrimeshStatic::GetFaceAdjacentFaces(mpcard fi) const 
{
    return mFaces[fi].neighbors;
}
