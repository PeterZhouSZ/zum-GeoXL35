//----------------------------------------------------------------------
#ifndef SymmSpaceSolver_h_
#define SymmSpaceSolver_h_
//----------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "Util\numerical\LibraryAdaptor.h"
#include "SPSolver/SymmSpaceNullSpace.h"
#include "SPSolver/SymmSpaceDirectSolver.h"
#include "SPSolver/SymmSpaceDirectNullSpaceSolver.h"
#include "SPSolver/LaplacianRegularizer.h"
#include "SPSolver/MeshInterpolater.h"
//----------------------------------------------------------------------
#include "DeformationSolver.h"
#include "DeformationEvaluator.h"

#include "DynamicLinearAlgebra.h"
#include "SparseLinearAlgebra.h"
//----------------------------------------------------------------------

class DefieldSymm;
struct PointSymmBlock;

class PROJECTSXWU_API SymmSpaceSolver :
    public DeformationSolver,
    public DeformationEvaluator
{
public:
    typedef boost::shared_ptr< SymmSpaceSolver > Ptr;
    typedef boost::shared_ptr< const SymmSpaceSolver > ConstPtr;

    typedef std::deque< std::deque< unsigned > > OrbitQueue;

public:
    SymmSpaceSolver();
    virtual ~SymmSpaceSolver(void) {}
    virtual int Initialize(
        UICPC* samplePC, const UICPC* deformPC, const UICTM* inputTM,
        const bool& useNullSpace,
        const float& regularizerWeight, const bool& useInertia, const float& tpy_sigma,
        const float& interpo_sigma, const bool& seperateSymmGroups = true, const float& maxDist = 1e10
        );
    inline Vector3f GetPosition (const unsigned& index);
    inline Vector3f GetDisplacement(const unsigned& index);
    inline const std::vector<Vector3f> getLatestPositions(void) const { return latestPositions; }
    inline const std::vector<Vector3f> getCurrentPositions(void) const { return currentPositions; }
    inline const std::vector<Vector3f> getDisplacementVec(void) const { return displacementVec; }

public:
    enum SolverType { CG, DIRECT, DIRECT_NULL, DIRECT_SYMMETRIC, SPARSE };
    void Refactor(void);
    void SolveDirect(void);
    void SolveDirectNull(void);
    void SolveCG(void);
    virtual void Update(const bool& updateRegularizer = true);
    void Interpolate(UICTM* resultTM);

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

    virtual void addSoftConstraint(
        const unsigned& index,
        const Matrix4f& transformation,
        const float& weight
        );

    virtual void addHardConstraint(
        const unsigned& index,
        const Matrix4f& transformation,
        const float& weight
        );

    virtual void addColinearity(
        const unsigned& A0, const unsigned& A1,
        const unsigned& B0, const unsigned& B1,
        const float& weight
        );

    virtual void addCoplanarity(
        const unsigned& S,
        const unsigned& T,
        const Vector3f& N,
        const float& weight
        );

public:
    /* ARConstraintsInterface, ARTimeVariantConstraintsInterface */
    virtual void addGeneralConstraint(GeneralARConstraint const* c) {}
    virtual void addPositionConstraint(PositionARConstraint const* c) {}
    virtual void addHalfSpaceConstraint(ARHalfSpaceConstraint const* c) {}
    virtual void addSpatialDerivativeConstraint(SpatialDerivativeARConstraint const* c) {}
    virtual void addSimpleSpatialDerivativeConstraint(SimpleSpatialDerivativeARConstraint const* c) {}
    virtual void addVelocityConstraint(VelocityVectorARConstraint const* c) {}
    virtual void addInstantaneousVelocityConstraint(InstantaneousVelocityARConstraint const* c) {}

    /* DeformationSolver*/
    virtual void beginConstraints();
    virtual DeformationEvaluator* createEvaluator() { return this; }
    virtual void solve();

    /* AROptimizationContext */
    virtual AROptimizationContext* createOptContext() { return nullptr; }
    virtual void setup(AROptimizationContext* context, ARSettings const* settings) {}

    /* DeformationEvaluator */
    virtual Vector3f Evaluate (const Vector3f& position);
    virtual Vector3f getPosition (ARConstraintAnchor const& at);
    virtual void beginQuery(float32 time) {}
    virtual Vector3f getVelocity (ARConstraintAnchor const& at) { return NULL_VECTOR3F; }
    virtual Matrix3f getSpatialGradient(ARConstraintAnchor const& at) { return IDENTITY3F; }

protected:
    virtual void Clear();
    virtual DVectorD CollectDeformPos() = 0;
    virtual void DistributeDeformPos(const DVectorD& result) = 0;

private:
    void calculateWeight(const PointSet& inputPS, const unsigned& vi,
        const FastSphereQuerryPtr& query, const std::deque<int32>& symmetryClique,
        const float& interpo_sigma, const bool& seperateSymmGroups, const float& maxDist,
        SparseMatrixD& liftWeight
        );
    void buildInterMatrix(const PointSet& samplePS, const PointSet& inputPS, const FastSphereQuerryPtr& query,
        const float& interpo_sigma, const bool& seperateSymmGroups, const float& maxDist,
        SparseMatrixD& liftWeight
        );

protected:
    bool bIntialized_;
    SolverType solverType_;

    std::vector<Vector3f> initialPositions;
    std::vector<Vector3f> latestPositions;
    std::vector<Vector3f> currentPositions;
    std::vector<Vector3f> displacementVec;

    bool bRefactor_;
    SparseMatrixD softConstraints_;
    DVectorD softConstValues_;
    SparseMatrixD hardConstraints_;
    DVectorD hardConstValues_;
    SparseMatrixD symmConstraints_;
    DVectorD symmConstValues_;
    SparseMatrixD fixedConstraints_;
    DVectorD fixedConstValues_;

    SparseMatrixD liftWeight3;
    LaplacianRegularizer::Ptr regularizer_;
    MeshInterpolater::Ptr meshInterpolater_;

    bool bUseNullSpace_;
    SymmSpaceNullSpace::Ptr nullSpace_;
    SymmSpaceDirectSolver::Ptr rangeSpace_;
    SymmSpaceDirectNullSpaceSolver::Ptr directNullSpace_;
};

// this solver solve for all basis position, symmetry is added as additional constraint
class PROJECTSXWU_API LapSymmSpaceSolver : public SymmSpaceSolver
{
public:
    typedef boost::shared_ptr< LapSymmSpaceSolver > Ptr;
    typedef boost::shared_ptr< const LapSymmSpaceSolver > ConstPtr;

public:
    LapSymmSpaceSolver();

protected:
    virtual void Clear();
    virtual DVectorD CollectDeformPos();
    virtual void DistributeDeformPos(const DVectorD& result);
};

#endif
