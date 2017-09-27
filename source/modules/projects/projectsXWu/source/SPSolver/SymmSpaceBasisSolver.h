//----------------------------------------------------------------------
#ifndef SymmSpaceBasisSolver_h_
#define SymmSpaceBasisSolver_h_
//----------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "Util\numerical\ClapackAdaptor.h"
#include "SPSolver/SymmSpaceSolver.h"
#include "SPSolver/SymmSpaceBasis.h"
//----------------------------------------------------------------------
//----------------------------------------------------------------------

// this solver solve for all basis position, with local coordinate construction
class PROJECTSXWU_API SymmSpaceBasisSolver : public SymmSpaceSolver
{
public:
    typedef boost::shared_ptr< SymmSpaceBasisSolver > Ptr;
    typedef boost::shared_ptr< const SymmSpaceBasisSolver > ConstPtr;

    typedef SymmSpaceBasis<3> SymmBasis;
    enum { DIM = SymmBasis::Dim };
    typedef std::array<Vector3f, DIM> basis_array;

public:
    SymmSpaceBasisSolver();
    virtual int Initialize(
        UICPC* samplePC, const UICPC* deformPC, const UICTM* inputTM,
        const bool& useNullSpace,
        const float& regularizerWeight, const bool& useInertia,
        const float& interpo_sigma, const bool& seperateSymmGroups = true, const float& maxDist = 1e10
        );

public:
    virtual void addSymmetryConstraint(
        const unsigned& source,
        const unsigned& target,
        const Matrix4f& transformation,
        const float& weight);

    virtual void addFixedConstraint(
        const unsigned& index,
        const float& weight
        );

    virtual void addSoftConstraint(
        const unsigned& index,
        const Vector3f& toPos,
        const float& weight
        );

    virtual void addColinearity(
        const unsigned& A,
        const unsigned& M,
        const unsigned& B,
        const float& weight
        );

    virtual void addCoplanarity(
        const unsigned& S,
        const unsigned& T,
        const Vector3f& N,
        const float& weight
        );

public:
    virtual void beginConstraints();

protected:
    virtual void Clear();
    virtual DVectorD CollectDeformPos();
    virtual void DistributeDeformPos(DVectorD const& result);

protected:
    basis_array constructBasis(const Vector3f& primary) const;
    void collectSymmAxis(const bool& update = false, const bool& debug_draw = false);
    StaticVector<float, SymmBasis::Dim> getLocalCoordinates(const Vector3f& pos, const unsigned& index) const;

protected:
    std::deque< SymmBasis > basisVec;
    const unsigned Dim;
};

#endif
