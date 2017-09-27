#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "SToolBox/SymmSampleGraph.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

IMPLEMENT_CLASS( SymmSampleGraph, 0 )
{
    BEGIN_CLASS_INIT( SymmSampleGraph );
}

//void SymmSampleGraph::assign( const Object* obj, COPY_CONTEXT *context ) 
//{
//    const SymmSampleGraph *other = dynamic_cast<const SymmSampleGraph*>(obj);
//    if (other) {
//        copyObjectProperties( obj, this );
//    } else {
//        pAssert(false);
//    }
//}
//
//
//void SymmSampleGraph::read( InputObjectStream *s )
//{
//    AttachedData::read(s);
//}
//
//void SymmSampleGraph::write( OutputObjectStream *s ) const
//{
//    AttachedData::write(s);
//
//}

void SymmSampleGraph::DrawWithDR(const PointSet& PS, int& dr_frame, const bool& debug_output) const
{
    debugRenderer->clearRenderJob_AllFrames("orbit_graph_");
    const AAT& posAAT = PS.getAAT("position");
    typedef boost::property_map<SymmSampleGraph::graph_t, boost::vertex_index_t>::type IndexMap;
    const unsigned& num_orbit = graphVec.size();
    for (unsigned oi = 0; oi < num_orbit; ++oi)
    {
        const SymmSampleGraph::graph_t& orbitGraph = graphVec[oi];
        SymmSampleGraph::DrawWithDR(orbitGraph, PS, dr_frame, debug_output);
    }
}

void SymmSampleGraph::DrawWithDR(const SymmSampleGraph::graph_t& orbitGraph, const PointSet& PS, int& dr_frame, const bool& debug_output)
{
    const AAT& posAAT = PS.getAAT("position");
    typedef boost::property_map<SymmSampleGraph::graph_t, boost::vertex_index_t>::type IndexMap;
    debugRenderer->beginRenderJob_OneFrame("orbit_graph_", dr_frame++);
    std::ostringstream ss;
    ss << "edges [" << num_edges(orbitGraph) << "]: ";
    IndexMap indMap = boost::get(boost::vertex_index, orbitGraph);
    const unsigned rootId = orbitGraph[boost::graph_bundle].rootId;
    debugRenderer->addPoint(
        PS.get3f(rootId, posAAT),
        makeVector3f(1, 0, 0)
        );
    boost::graph_traits<SymmSampleGraph::graph_t>::edge_iterator eit, eit_end;
    for (boost::tie(eit, eit_end) = edges(orbitGraph); eit != eit_end; ++eit)
    {
        const Vector3f& source = PS.get3f(indMap[boost::source(*eit, orbitGraph)], posAAT);
        const Vector3f& target = PS.get3f(indMap[boost::target(*eit, orbitGraph)], posAAT);
        debugRenderer->addLine(
            source, target,
            makeVector3f(1, 0, 1),
            makeVector3f(0, 1, 1),
            2);
        ss << "["
            << indMap[boost::source(*eit, orbitGraph)] << " -> " << indMap[boost::target(*eit, orbitGraph)]
        << "] ";
    }
    if (debug_output) debugOutput << ss.str() << "\n";
    debugRenderer->endRenderJob();
}
