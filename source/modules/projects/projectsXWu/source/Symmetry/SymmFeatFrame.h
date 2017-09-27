#ifndef SymmFeatFrame_H
#define SymmFeatFrame_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "Symmetry/SymmFeat.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class PROJECTSXWU_API SymmFeatFrame
{
public:
    typedef boost::shared_ptr< SymmFeatFrame > Ptr;
    typedef boost::shared_ptr< const SymmFeatFrame > ConstPtr;

public:
    struct TEdge
    {
        size_t indx;
        size_t v0, v1;
    };

    struct TVert
    {
        std::vector<size_t> verts;
        std::vector<size_t> edges;
    };

public:
    SymmFeatFrame(const std::vector<SymmFeatLine::Ptr>& linevec);
    void DrawWithDR(void);

public:
    // ------------------------------------------
    // vertex interface
    // ------------------------------------------
    std::vector<size_t> GetVertEdges(size_t vi) const;
    std::vector<size_t> GetVertVerts(size_t vi) const;

    // ------------------------------------------
    // edge interface
    // ------------------------------------------
    size_t GetEdge2ndVert(size_t ei, size_t vi) const;

public:
    std::vector<SymmFeatLine::Ptr> linevec_;

    typedef std::vector<TEdge> EdgeVec;
    typedef boost::unordered_map< size_t, TVert > VertMap;

    EdgeVec edges_;
    VertMap verts_;
};
typedef SymmFeatFrame::EdgeVec EdgeVec;
typedef SymmFeatFrame::VertMap VertMap;

#endif
