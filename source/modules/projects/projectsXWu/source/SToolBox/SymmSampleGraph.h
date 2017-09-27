#ifndef SymmSampleGraph_H
#define SymmSampleGraph_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "projectsXWu.h"
#include <boost/graph/adjacency_list.hpp>
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class PROJECTSXWU_API SymmSampleGraph : public AttachedData
{
    DEFINE_CLASS( SymmSampleGraph )
public:
    typedef boost::shared_ptr< SymmSampleGraph > Ptr;
    typedef boost::shared_ptr< const SymmSampleGraph > ConstPtr;

public:
    struct vertex_property_t
    {
        Vector2i groupId;
        vertex_property_t(void) : groupId(makeVector2i(0, -1)) {}
    };

    struct edge_property_t
    {
        Vector2i groupId;
        Matrix4f transformation;
        Matrix4f rootTrans;
        edge_property_t(void) : groupId(makeVector2i(0, -1)), transformation(IDENTITY4F), rootTrans(IDENTITY4F) {}
        edge_property_t(const Vector2i& gid, const Matrix4f& trans, const Matrix4f& trans_r) :
            groupId(gid), transformation(trans), rootTrans(trans_r) {}
    };

    struct graph_property_t
    {
        unsigned rootId;
    };

    typedef boost::adjacency_list<
        boost::vecS, boost::vecS, boost::directedS,
        vertex_property_t, edge_property_t, graph_property_t
    > graph_t;

public:
    static std::string getDefaultName() { return "SymmSampleGraphAtt"; }

    //virtual void assign(const Object* obj, COPY_CONTEXT *context /* = nullptr */);
    //virtual void read(InputObjectStream *s);
    //virtual void write(OutputObjectStream *s) const;

public:
    void push_back(const graph_t& g) { graphVec.push_back(g); }

public:
    void DrawWithDR(const PointSet& PS, int& dr_frame, const bool& debug_output = false) const;
    static void DrawWithDR(const SymmSampleGraph::graph_t& orbitGraph, const PointSet& PS, int& dr_frame, const bool& debug_output = false);

public:
    std::deque< graph_t > graphVec;
};

#endif
