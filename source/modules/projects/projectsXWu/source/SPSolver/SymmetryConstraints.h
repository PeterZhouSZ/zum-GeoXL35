//----------------------------------------------------------------------
#ifndef SymmetryConstraints_h_
#define SymmetryConstraints_h_
//----------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "SPSolver/SymmSpaceSolver.h"
//----------------------------------------------------------------------
//----------------------------------------------------------------------

class DefieldSymm;
struct PointSymmBlock;

class SymmetryConstraints
{
public:
    typedef boost::shared_ptr< SymmetryConstraints > Ptr;
    typedef boost::shared_ptr< const SymmetryConstraints > ConstPtr;

public:
    SymmetryConstraints();

    int BuildSymmetryConstraints(
        SceneObject* samobj,
        SymmSpaceSolver::Ptr receiver,
        const bool& bAddFixedRigion = true,
        const float& symmetryWeight = 1.0f
        );
};

#endif
