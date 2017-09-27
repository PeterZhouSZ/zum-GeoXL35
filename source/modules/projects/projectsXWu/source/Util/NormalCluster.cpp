#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "NormalCluster.h"
#include "Util/NoUse.h"
//#include <boost/function.hpp>
//---------------------------------------------------------------------------
#include "ColorTools.h"
//---------------------------------------------------------------------------

bool NormalCluster::push(const Vector3f& nrm, const unsigned& id, const float& th)
{
    if (NU::IsParallelDeg(normal_, nrm, th)) {
        normal_ += (nrm - normal_) / (indices_.size() + 1); // update representing normal
        normal_ = normalize(normal_);
        indices_.push_back(id);
        return true;
    } else
        return false;
}

NormalClusterExtractor::NormalClusterExtractor(PointSet* ps, const float& th)
    : ps_(ps), space_th_(th)
{
    //typedef boost::function< bool (const Vector3f&, const unsigned&) > CompareFunc;
    //CompareFunc cfunc = boost::bind(NormalCluster::push, _1, _2, th);

    if (!ps->providesAttribute("normal")) {
        error("NormalClusterExtractor::NormalClusterExtractor() - no normal channel.");
        return;
    }
    AAT normalAAT = ps->getAAT("normal");
    const unsigned numpoints = ps->getNumEntries();
    for (unsigned ni = 0; ni < numpoints; ++ni) {
        const Vector3f& nrm = normalize(ps->get3f(ni, normalAAT));
        bool bfound = false;
        for (Iterator it = cluster_list_.begin(); it != cluster_list_.end(); ++it) {
            if (!it->push(nrm, ni, th)) continue;
            bfound = true;
            break;
        }
        if (bfound) continue;
        NormalCluster clu(nrm);
        clu.indices_.push_back(ni);
        cluster_list_.push_back(clu);
    }
    std::sort(cluster_list_.begin(), cluster_list_.end(), NormalClusterComp);

    debugOutput << str( boost::format(
        "founded %1% normal clusters. the largest cluster contains %2% [%3%%%] normals.\n"
        )
        % cluster_list_.size() % cluster_list_.front().size() % (100 * (float) cluster_list_.front().size() / numpoints)
        );
}

void NormalClusterExtractor::RemoveSmallClusters(const size_t& num_th)
{
    for (Iterator it = cluster_list_.begin(); it != cluster_list_.end();) {
        if (it->size() < num_th) it = cluster_list_.erase(it);
        else ++it;
    }
}

void NormalClusterExtractor::DrawWithDR()
{
    AAT positionAAT = ps_->getAAT("position");
    unsigned numcluster = cluster_list_.size();
    for (unsigned ci = 0; ci < numcluster; ++ci) {
        Vector3f color = rainbowGradient3f((double)ci / numcluster);
        const NormalCluster& ncluster = cluster_list_[ci];
        debugRenderer->addFineArrow(
            NULL_VECTOR3F, NULL_VECTOR3F + ncluster.normal_,
            color, 0.1f * (float) ncluster.size() / ps_->getNumEntries(), true);
        const NormalCluster::IndexList& indices = ncluster.indices_;
        for (unsigned ii : indices) {
            debugRenderer->addPoint(
                ps_->get3f(ii, positionAAT),
                color);
        }
        break;
    }
}
