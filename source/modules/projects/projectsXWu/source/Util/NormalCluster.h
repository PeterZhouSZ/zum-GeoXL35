#ifndef NormalCluster_H
#define NormalCluster_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class NormalCluster
{
public:
    typedef boost::shared_ptr< NormalCluster > Ptr;
    typedef boost::shared_ptr< const NormalCluster > ConstPtr;

    typedef std::deque< unsigned > IndexList;

public:
    NormalCluster(const Vector3f& n) : normal_(n) {}
    bool push(const Vector3f& nrm, const unsigned& id, const float& th = 5.f);
    const size_t size(void) const { return indices_.size(); }

public:
    Vector3f normal_;
    IndexList indices_;
};
inline bool NormalClusterComp(const NormalCluster& lhs, const NormalCluster& rhs) {
    return lhs.indices_.size() > rhs.indices_.size();
}

class NormalClusterExtractor
{
public:
    typedef boost::shared_ptr< NormalClusterExtractor > Ptr;
    typedef boost::shared_ptr< const NormalClusterExtractor > ConstPtr;

    typedef unsigned index_type;
    typedef NormalCluster value_type;
    typedef std::deque< value_type > ClusterList;
    typedef ClusterList::iterator Iterator;

public:
    NormalClusterExtractor(PointSet* ps, const float& th = 5.f);
    value_type& operator[](const index_type& idx) { return cluster_list_[idx]; }
    const value_type& operator[](const index_type& idx) const { return cluster_list_[idx]; }
    const size_t size(void) const { return cluster_list_.size(); }

    void RemoveSmallClusters(const size_t& num_th);
    void DrawWithDR();

private:
    ClusterList cluster_list_;
    PointSet* ps_;
    float space_th_; // angular discrimination threshold between planes
};

#endif
