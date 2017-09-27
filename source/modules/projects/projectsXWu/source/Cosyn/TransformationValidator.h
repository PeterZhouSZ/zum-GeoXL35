#ifndef TransformationValidator_H
#define TransformationValidator_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "PCwithGrid.h"
#include "RepBoxLoader.h"
#include "SubspaceICP.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class TransformationValidator
{
public:
    typedef boost::shared_ptr< TransformationValidator > Ptr;
    typedef boost::shared_ptr< const TransformationValidator > ConstPtr;

public:
    TransformationValidator(void);

    void SetPC(UICPC* pc);
    void Validate(std::deque<RepBox::Ptr>* boxList);
    void Validate(std::deque< std::deque<RepBox::Ptr> >* boxList);

private:
    bool dampingSnap(
        const Eigen::Matrix4f& trans,
        const Vector3f& centroid, const float& searchRadius,
        FastSphereQuerryPtr rQuery, SubspaceICP::Ptr icp
        );

    Eigen::Matrix4f ComputeTransformation(
        RepBox::Ptr source, RepBox::Ptr target
        );

public:
    UICPC* examplePC_;
    float match_ratio_;
    float outlier_ratio_;
    float inlier_ratio_;
    unsigned num_icp_;
};

#endif
