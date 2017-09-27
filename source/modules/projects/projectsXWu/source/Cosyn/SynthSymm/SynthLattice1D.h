#ifndef SynthLattice1D_H
#define SynthLattice1D_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "Util\numerical\EigenAdaptor.h"
//---------------------------------------------------------------------------

class SynthLattice1D
{
public:
    typedef boost::shared_ptr< SynthLattice1D > Ptr;
    typedef boost::shared_ptr< const SynthLattice1D > ConstPtr;

public:
    SynthLattice1D(void);
    virtual inline std::string GetClassName(void) { return "SynthLattice1D"; }
    void DebugDraw(const Vector3f& color, const float32& radius);
    void SetTrans(const Eigen::Matrix4f& trans);

public:
    static Eigen::Matrix4f ComputeTransformation(
        const Eigen::Matrix4f& source,
        const Eigen::Matrix4f& target
        );

public:
    Eigen::Vector3f base_;
    Eigen::Matrix4f trans_;
    std::deque<int> elem_;

public:
    Eigen::Vector3f dir_;
};

#endif
