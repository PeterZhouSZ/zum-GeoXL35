#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "PlaneCluster.h"
#include "Util/NoUse.h"
//#include <boost/function.hpp>
//---------------------------------------------------------------------------
#include "CopyObjectProperties.h"
#include "PropertyTableProperty.h"
#include "ColorTools.h"
#include "PCCComputeTopology.h"
#include "InCorePCTopologyGraph.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

IMPLEMENT_CLASS( PlaneCluster , 0 )
{
    BEGIN_CLASS_INIT( PlaneCluster );
    INIT_PROPERTY_TABLE();
    ADD_EMBEDDED_OBJECT_PROP(plane_, 0, Plane3f::getClass());
    //ADD_OBJECT_PROP(plane_, 0, Plane3f::getClass(), true);
    ADD_CARD32_VAR_ARRAY_PROP( indices_, 0 );
}

IMPLEMENT_CLASS( PlaneClusterExtractor , 0 )
{
    BEGIN_CLASS_INIT( PlaneClusterExtractor );
    INIT_PROPERTY_TABLE();
    ADD_FLOAT32_PROP(space_th_, 0);
    ADD_OBJECT_LIST_PROP( cluster_list_, 0, PlaneCluster::getClass() );
}

void PlaneCluster::assign( const Object* obj, COPY_CONTEXT *context ) 
{
    const PlaneCluster *other = dynamic_cast<const PlaneCluster*>(obj);
    if (other) {
        copyObjectProperties( obj, this );
    } else {
        pAssert(false);
    }
}


//void PlaneCluster::read( InputObjectStream *s )
//{
//    plane_.read(s);
//
//    mpcard listsize;
//    s->readMaxPlatformCard(listsize);
//    indices_.resize(listsize);
//    for (mpcard i = 0; i < listsize; ++i) {
//        s->read(indices_[i]);
//    }
//}
//
//void PlaneCluster::write( OutputObjectStream *s ) const
//{
//    plane_.write(s);
//
//    s->writeMaxPlatformCard(indices_.size());
//    for (card32 i = 0; i < indices_.size(); ++i) {
//        s->write(indices_[i]);
//    }
//}

void PlaneClusterExtractor::assign( const Object* obj, COPY_CONTEXT *context ) 
{
    const PlaneClusterExtractor *other = dynamic_cast<const PlaneClusterExtractor*>(obj);
    if (other) {
        copyObjectProperties( obj, this );
    } else {
        pAssert(false);
    }
}


//void PlaneClusterExtractor::read( InputObjectStream *s )
//{
//    s->read(space_th_);
//
//    mpcard listsize;
//    s->readMaxPlatformCard(listsize);
//    cluster_list_.resize(listsize);
//    for (mpcard i = 0; i < listsize; ++i) {
//        cluster_list_[i]->read(s);
//    }
//}
//
//void PlaneClusterExtractor::write( OutputObjectStream *s ) const
//{
//    s->write(space_th_);
//
//    s->writeMaxPlatformCard(cluster_list_.size());
//    for (card32 i = 0; i < cluster_list_.size(); ++i) {
//        cluster_list_[i]->write(s);
//    }
//}

bool PlaneCluster::push(const Vector3f& pos, const unsigned& id, const float& th)
{
    const float dist = plane_.calculateSignedDistance(pos);
    if (abs(dist) < th) {
        const float delta = dist / (indices_.size() + 1);
        plane_.movePoint(delta);
        indices_.push_back(id);
        return true;
    } else
        return false;
}

unsigned PlaneClusterExtractor::pushPoint(
    PointSet* queryps, const Vector3f& pos,
    const unsigned& ni, const NormalCluster& ncluster)
{
    const NormalCluster::IndexList& indices = ncluster.indices_;
    //unsigned cluster_id = 0;
    //for (; cluster_id < cluster_list_.size(); ++cluster_id) {
    //    if (cluster_list_[cluster_id]->push(pos, indices[ni], space_th_)) return cluster_id;
    //}
    unsigned cluster_id = cluster_list_.size();
    PlaneCluster* clu = new PlaneCluster(ncluster.normal_, pos);
    clu->indices_.push_back(indices[ni]);
    cluster_list_.push_back(clu);
    return cluster_id;
}

PlaneClusterExtractor::PlaneClusterExtractor(
    PointSet* ps, const NormalCluster& ncluster, const float& th, const size_t& num_th)
    : space_th_(th)
{
    const bool debug_draw = false;

    const NormalCluster::IndexList& indices = ncluster.indices_;
    PointSet* queryps = ps->subset(indices);

    UnstructuredInCorePointCloud *tmpPC = new UnstructuredInCorePointCloud();
    PointSet* tmpPS = (PointSet*)queryps->copy();
    tmpPC->setPointSet(tmpPS);
    PCCComputeTopology cmd;
    cmd.setup(PCCComputeTopology::TOPTYPE_EPS, th);
    InCorePCTopologyGraph* tpg = cmd.computeEpsTopology(tmpPS, tmpPC);

    unsigned max_top_size = 0;
    AAT positionAAT = queryps->getAAT("position");
    const unsigned numpoints = queryps->getNumEntries();
    std::vector<bool> check_vec(numpoints, false);
    std::deque<unsigned> exp_stack;
    for (unsigned ni = 0; ni < numpoints; ++ni) {
        if (check_vec[ni]) continue;
        check_vec[ni] = true;
        const Vector3f& pos = queryps->get3f(ni, positionAAT);
        unsigned cluster_id = pushPoint(queryps, pos, ni, ncluster);

        if (debug_draw) debugRenderer->beginRenderJob_OneFrame("plane_expansion_", DR_FRAME++);
        exp_stack.push_back(ni);
        while (!exp_stack.empty()) {
            const unsigned curr_index = exp_stack.front();
            exp_stack.pop_front();
            const unsigned num_adj = tpg->getNumAdjacentVertices(curr_index);
            if (max_top_size < num_adj) max_top_size = num_adj;
            for (unsigned ti = 0; ti < num_adj; ++ti) {
                const unsigned curr_index_ti = tpg->getAdjacentVertex(curr_index, ti);
                if (check_vec[curr_index_ti]) continue;
                const Vector3f& pos_ti = queryps->get3f(curr_index_ti, positionAAT);
                if (cluster_list_[cluster_id]->push(pos_ti, indices[curr_index_ti], th)) {
                    check_vec[curr_index_ti] = true;
                    exp_stack.push_back(curr_index_ti);
                    if (debug_draw)
                    {
                        debugRenderer->addPoint(pos_ti, makeVector3f(1, 0, 0));
                    }
                }
            }
        }
        if (debug_draw) debugRenderer->endRenderJob();
    }
    delete queryps;
    delete tpg;
    delete tmpPC;

    std::sort(cluster_list_.begin(), cluster_list_.end(), PlaneClusterCompSize);
    debugOutput << str( boost::format(
        "founded %1% plane clusters. the first cluster contains %2% [%3%%%] points.\n"
        )
        % cluster_list_.size() % cluster_list_.front()->size() % (100 * (float) cluster_list_.front()->size() / numpoints)
        );
    for (value_type* clu : cluster_list_) {
        debugOutput << clu->size() << ", ";
    }
    debugOutput << "\n";

    if (num_th != 0) RemoveSmallClusters(num_th);
    std::sort(cluster_list_.begin(), cluster_list_.end(), PlaneClusterComp);
    debugOutput << str( boost::format(
        "%1% plane clusters left after removing small ones (threshold: %2%, max top size: %3%).\n"
        )
        % cluster_list_.size() % num_th % max_top_size
        );
}

PlaneClusterExtractor::~PlaneClusterExtractor()
{
    for (value_type* clu : cluster_list_) {
        delete clu;
    }
    cluster_list_.clear();
}

void PlaneClusterExtractor::RemoveSmallClusters(const size_t& num_th)
{
    for (Iterator it = cluster_list_.begin(); it != cluster_list_.end();) {
        if ((*it)->size() < num_th) it = cluster_list_.erase(it);
        else ++it;
    }
}

void PlaneClusterExtractor::DrawWithDR(PointSet* ps)
{
    AAT positionAAT = ps->getAAT("position");
    size_t numcluster = cluster_list_.size();
    for (unsigned ci = 0; ci < numcluster; ++ci) {
        Vector3f color = rainbowGradient3f((double)ci / numcluster);
        const PlaneCluster& cluster = *cluster_list_[ci];
        const std::deque< unsigned >& indices = cluster.indices_;
        for (unsigned ii : indices) {
            debugRenderer->addPoint(
                ps->get3f(ii, positionAAT),
                color);
        }
    }
}
