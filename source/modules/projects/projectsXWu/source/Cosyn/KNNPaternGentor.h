#ifndef KNNPaternGentor_H
#define KNNPaternGentor_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class KNNPaternGentor
{
public:
    typedef boost::shared_ptr< KNNPaternGentor > Ptr;
    typedef boost::shared_ptr< const KNNPaternGentor > ConstPtr;

    typedef std::array<int, 3> key_type;

public:
    KNNPaternGentor(const unsigned& num);

    void Generate(PointSet* PS);
    void Discretize(PointSet* PS, const float& discale);
    void ShowPattern(PointSet* PS);

private:
    BoundingBox3f makeBoundingBox(const key_type& key);

public:
    unsigned num_search;
    float discale_;
    std::deque< std::deque<unsigned> > knnVec_;
    std::deque< std::deque< key_type > > offVec_;
};

#endif
