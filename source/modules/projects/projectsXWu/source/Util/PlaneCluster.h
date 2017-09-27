#ifndef PlaneCluster_H
#define PlaneCluster_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "Util/NormalCluster.h"
//---------------------------------------------------------------------------
#include "Plane3f.h"
#include "ObjectListProperty.h"
//---------------------------------------------------------------------------

class PROJECTSXWU_API PlaneCluster : public Persistent
{
    DEFINE_CLASS( PlaneCluster )
public:
    typedef boost::shared_ptr< PlaneCluster > Ptr;
    typedef boost::shared_ptr< const PlaneCluster > ConstPtr;

    typedef std::deque< unsigned > IndexList;

public:
    PlaneCluster() {}
    PlaneCluster( const Vector3f &n, const Vector3f &p ) { plane_.setup(n, p); }
    virtual void assign(const Object* obj, COPY_CONTEXT *context /* = nullptr */);
    //virtual void read(InputObjectStream *s);
    //virtual void write(OutputObjectStream *s) const;

    bool push(const Vector3f& pos, const unsigned& id, const float& th = 5.f);
    const size_t size(void) const { return indices_.size(); }

public:
    Plane3f plane_;
    IndexList indices_;
    IMPLEMENT_VAR_ARRAY_METHODS_STL(indices_);
};
inline bool PlaneClusterComp(const PlaneCluster* lhs, const PlaneCluster* rhs) {
    return lhs->plane_.getDistance() < rhs->plane_.getDistance();
}
inline bool PlaneClusterCompSize(const PlaneCluster* lhs, const PlaneCluster* rhs) {
    return lhs->size() > rhs->size();
}

class PROJECTSXWU_API PlaneClusterExtractor : public AttachedData
{
    DEFINE_CLASS( PlaneClusterExtractor )
public:
    typedef boost::shared_ptr< PlaneClusterExtractor > Ptr;
    typedef boost::shared_ptr< const PlaneClusterExtractor > ConstPtr;

    typedef unsigned index_type;
    typedef PlaneCluster value_type;
    typedef std::deque< value_type* > ClusterList;
    typedef ClusterList::iterator Iterator;

public:
    PlaneClusterExtractor() : space_th_(0) {}
    PlaneClusterExtractor(PointSet* ps, const NormalCluster& ncluster,
        const float& th = 0.1f, const size_t& num_th = 0);
    ~PlaneClusterExtractor();
    static std::string getDefaultName() { return "PlaneClusterAtt"; }
    virtual void assign(const Object* obj, COPY_CONTEXT *context /* = nullptr */);
    //virtual void read(InputObjectStream *s);
    //virtual void write(OutputObjectStream *s) const;

    value_type& operator[](const index_type& idx) { return *cluster_list_[idx]; }
    const value_type& operator[](const index_type& idx) const { return *cluster_list_[idx]; }
    const size_t size(void) const { return cluster_list_.size(); }
    void addCluster(value_type* clu) { cluster_list_.push_back(clu); }

    void RemoveSmallClusters(const size_t& num_th);
    void DrawWithDR(PointSet* ps);

private:
    unsigned pushPoint(PointSet* queryps, const Vector3f& pos,
        const unsigned& ni, const NormalCluster& ncluster);

public:
    float space_th_; // distance discrimination threshold between planes
    ClusterList cluster_list_;
    IMPLEMENT_OBJECT_LIST_METHODS_STL_NO_PREFIX_M(cluster_list_, PlaneCluster);
};

#endif
