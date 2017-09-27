//----------------------------------------------------------------------
#ifndef HandleConstraints_h_
#define HandleConstraints_h_
//----------------------------------------------------------------------
#include "SPSolver/SymmSpaceSolver.h"
#include "Interface/Handle3DList.h"
//----------------------------------------------------------------------

class HandleConstraints
{
public:
    typedef boost::shared_ptr< HandleConstraints > Ptr;
    typedef boost::shared_ptr< const HandleConstraints > ConstPtr;

public:
    HandleConstraints(float32 const weight = 1);

    void BuildConstraints(
        SymmSpaceSolver::Ptr receiver,
        const Handle3DList::Ptr& handleList
        );

private:
    float32 weight_;
};

//----------------------------------------------------------------------
#endif //HandleConstraints_h_
//----------------------------------------------------------------------
