#ifndef GroupGenerator_H
#define GroupGenerator_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "RepBoxLoader.h"
#include "SynthSymm/SynthLattice1D.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class GroupGenerator
{
public:
    typedef boost::shared_ptr< GroupGenerator > Ptr;
    typedef boost::shared_ptr< const GroupGenerator > ConstPtr;

public:
    GroupGenerator(void);

    void Generate(std::deque<RepBox::Ptr>* boxList);
    void Generate(std::deque< std::deque<RepBox::Ptr> >* boxList);

public:
    Eigen::Matrix4f ComputeTransformation(
        RepBox::Ptr source, RepBox::Ptr target
        );

public:
    std::deque< std::deque<SynthLattice1D::Ptr> > latVec_;
};

#endif
