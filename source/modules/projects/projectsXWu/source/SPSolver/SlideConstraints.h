//----------------------------------------------------------------------
#ifndef SlideConstraints_h_
#define SlideConstraints_h_
//----------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "SPSolver/SymmSpaceSolver.h"
//----------------------------------------------------------------------
#include "ARConstraintsInterface.h"
#include "DeformationEvaluator.h"
//----------------------------------------------------------------------

class ColinearityConstraint
{
public:
    ColinearityConstraint(const std::deque<card32>& indice);

    void AddColinearity(
        SymmSpaceSolver::Ptr receiver,
        const float& colinearityWeight
        );

private:
    std::deque<card32> indice_;
};

class CoplanarityConstraint
{
public:
    CoplanarityConstraint(std::set<unsigned>& indice);

    void AddCoplanarity(
        const std::vector<Vector3f>& deformed,
        SymmSpaceSolver::Ptr receiver,
        const float& coplanarityWeight
        );

private:
    std::deque<unsigned> indice_;
};

class SlideConstraints
{
public:
    typedef boost::shared_ptr< SlideConstraints > Ptr;
    typedef boost::shared_ptr< const SlideConstraints > ConstPtr;

public:
    SlideConstraints();

    //< colinearity
    int BuildColinearity(
        SceneObject* samobj
        );
    void AddColinearity(
        SymmSpaceSolver::Ptr receiver,
        const float& colinearityWeight
        );

    //< coplanarity
    int BuildCoplanarity(
        SceneObject* samobj,
        const unsigned& min_size = 8,
        const float& tri_cos_th = 0.95f,
        const float& nor_cos_th = 0.01f
        );
    void AddCoplanarity(
        SymmSpaceSolver::Ptr receiver,
        const float& coplanarityWeight
        );
    void Update(const std::vector<Vector3f>& deformed);

private:
    std::deque<ColinearityConstraint> colinearities_;
    std::deque<CoplanarityConstraint> coplanarities_;
    std::vector<Vector3f> deformedPositions;
};

#endif
