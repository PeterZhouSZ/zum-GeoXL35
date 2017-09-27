#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Symmetry/SymmFeatFrame.h"
#include "Util/NoUse.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

SymmFeatFrame::SymmFeatFrame(const std::vector<SymmFeatLine::Ptr>& linevec)
: linevec_(linevec)
{
    boost::unordered_map< size_t, set<size_t> > vertShared;
    for (size_t li = 0; li < linevec.size(); ++li) {
        size_t v0 = linevec[li]->v0;
        size_t v1 = linevec[li]->v1;

        TEdge edge;
        edge.indx = li;
        edge.v0 = v0; edge.v1 = v1;
        edges_.push_back(edge);

        if (verts_.find(v0) == verts_.end()) {
            TVert vert;
            verts_.insert(make_pair(v0, vert));
        }
        if (verts_.find(v1) == verts_.end()) {
            TVert vert;
            verts_.insert(make_pair(v1, vert));
        }
        TVert& vert0 = verts_[v0];
        TVert& vert1 = verts_[v1];

        vert0.edges.push_back(li);
        vert1.edges.push_back(li);
        vert0.verts.push_back(v1);
        vert1.verts.push_back(v0);
    }

}

std::vector<size_t> SymmFeatFrame::GetVertEdges(size_t vi) const
{
    VertMap::const_iterator it = verts_.find(vi);
    if (it == verts_.end()) {
        return std::vector<size_t>();
    } else {
        return it->second.edges;
    }
}

std::vector<size_t> SymmFeatFrame::GetVertVerts(size_t vi) const
{
    VertMap::const_iterator it = verts_.find(vi);
    if (it == verts_.end()) {
        return std::vector<size_t>();
    } else {
        return it->second.verts;
    }
}

size_t SymmFeatFrame::GetEdge2ndVert(size_t ei, size_t vi) const
{
    const TEdge& edge = edges_[ei];
    return (edge.v0 == vi) ?
        edge.v1 : edge.v0;
}

void SymmFeatFrame::DrawWithDR(void)
{
    for (size_t ii = 0; ii < linevec_.size(); ++ii) {
        linevec_[ii]->DrawWithDR();
    }
}
