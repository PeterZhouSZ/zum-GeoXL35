//----------------------------------------------------------------------
#ifndef TPSSolver_h_
#define TPSSolver_h_
//----------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//----------------------------------------------------------------------
#include "DeformationSolver.h"
#include "DeformationEvaluator.h"
//----------------------------------------------------------------------
#include "DynamicLinearAlgebra.h"
#include  "SparseLinearAlgebra.h"
//----------------------------------------------------------------------
#include "SparseGrid.h"
//----------------------------------------------------------------------
#include <vector>

namespace X4
{
//======================================================================
// Forward declarations
//----------------------------------------------------------------------
struct EStencil;
struct ESStencil;

class ProgressWindow;

//==============================================================================
// GridEntry struct
//------------------------------------------------------------------------------
struct GridEntry
{
  //============================================================================
  // Constructor
  //----------------------------------------------------------------------------
  GridEntry(Vector3i const& cell, Vector3f const& dPos)
    : gridCell (cell )
    , deformPos(dPos )
    , occupied (false)
  {}
    
  //============================================================================
  // Member variables
  //----------------------------------------------------------------------------
  Vector3i gridCell;
  Vector3f deformPos;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  bool occupied;
};

//======================================================================
// TPSSolver class
// Combines TSDeformationSolver, TSDeformationContext, and
// TSDeformationEvaluator.
//======================================================================
class PROJECTSXWU_API TPSSolver :
  public DeformationSolver,
  public DeformationEvaluator
{
private:
  //====================================================================
  // Nested
  //--------------------------------------------------------------------
  typedef UnstructuredInCorePointCloud UICPC;

public:
  //====================================================================
  // Constructors
  //--------------------------------------------------------------------
  TPSSolver(bool    const smoothed          = false,
            int32   const gridExtension     = 2,
            float32 const gridSpacing       = 0.2f,
            float32 const    identityWeight = 0.0f,
            float32 const regularizerWeight = 1.0f);
  
  //====================================================================
  // Member functions
  //--------------------------------------------------------------------
  //bool  Reweight() const;
  //bool& Reweight();
  
  //--------------------------------------------------------------------
  // Initialize - TPSSolver_Initialize.cpp
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  void Initialize(SceneObject const* object);

  void Update(TPSSolver& previous);

  void Scale();

  //--------------------------------------------------------------------
  // Regularizer - TPSSolver_Regularizer.cpp
  // Emulates beginConstraints(); clears the solver matrices and adds
  // the regularizer.
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  void AddRegularizer();
  
  //--------------------------------------------------------------------
  // Constraints - TPSSolver_Constraints.cpp
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  void AddPositionConstraint(Vector3f const& anchor,
                             Vector3f const& target,
                             Matrix3f const& quadric);
  
  //--------------------------------------------------------------------
  // Constraints smoothed - TPSSolver_ConstraintsSmoothed.cpp
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //void AddSmoothedPositionConstraint(Vector3f const& anchor,
  //                                   Vector3f const& target,
  //                                   Matrix3f const& quadric);
  
  //--------------------------------------------------------------------
  // B-spline constraints - TPSSolver_ConstraintsBSpline.cpp
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  void AddBSplinePositionConstraint(Vector3f const& anchor,
                                    Vector3f const& target,
                                    Matrix3f const& quadric);

  //--------------------------------------------------------------------
  // Symmetry constraints - TPSSolver_Symmetry.cpp
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  void AddSymmetryConstraint(Vector3f const& anchor,
                             Vector3f const& target,
                             Matrix3f const& rotation,
                             Vector3f const& translation,
                             float32  const  weight);
  
  void AddLinearSymmetryConstraint(Vector3f const& anchor,
                                   Vector3f const& target,
                                   Matrix3f const& rotation,
                                   Vector3f const& translation,
                                   float32  const  weight);

  float32 EvaluateLinearBasisFunction(Vector3i const& of,
                                      Vector3f const& at);

  StaticVector<float32, 8> CalculateLinearWeights(Vector3i const& base,
                                                  Vector3f const& point);

  //--------------------------------------------------------------------
  // Symmetry constraints - TPSSolver_SymmetrySmoothed.cpp
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //void AddSmoothedSymmetryConstraint(Vector3f const& anchor,
  //                                   Vector3f const& target,
  //                                   Matrix3f const& rotation,
  //                                   Vector3f const& translation,
  //                                   float32  const  weight);
  //
  //DVectorF CalculateSmoothedWeights(Vector3i const& base,
  //                                  Vector3f const& point);

  //--------------------------------------------------------------------
  // BSpline symmetry constraints - TPSSolver_SymmetryBSpline.cpp
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  void AddBSplineSymmetryConstraint(Vector3f const& anchor,
                                    Vector3f const& target,
                                    Matrix3f const& rotation,
                                    Vector3f const& translation,
                                    float32  const  weight);

  DVectorF CalculateRegularBSplineWeights(Vector3i const& base,
                                          Vector3f const& point);
  DVectorF CalculateShiftedBSplineWeights(Vector3i const& base,
                                          Vector3f const& point);

  //--------------------------------------------------------------------
  // Solve - TPSSolver_Solve.cpp
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  void Solve( card32               cgNumIterations        = 10000,
             float32        const& cgConvergenceThreshold = 0.00001f,
             bool           const  cgDebugOutput          = true,
             ProgressWindow*       progress               = 0);
  
  //--------------------------------------------------------------------
  // Evaluate - TPSSolver_Evaluate.cpp
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Vector3f Evaluate               (Vector3f const& position);
  Matrix3f EvaluateSpatialGradient(Vector3f const& position);

  //--------------------------------------------------------------------
  // Evaluate b-spline - TPSSolver_EvaluateBSpline.cpp
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Vector3f EvaluateBSpline               (Vector3f const& position);
  Matrix3f EvaluateBSplineSpatialGradient(Vector3f const& position);

  //--------------------------------------------------------------------
  // Evaluate smoothed - TPSSolver_EvaluateSmoothed.cpp
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //Vector3f EvaluateSmoothed               (Vector3f const& position);
  //Matrix3f EvaluateSmoothedSpatialGradient(Vector3f const& position);

  //====================================================================
  // DeformationSolver - TPSSolver_DeformationSolver.cpp
  //--------------------------------------------------------------------
  // inherited from ARConstraintsInterface
  virtual void addGeneralConstraint(GeneralARConstraint const* c);
  // inherited from ARConstraintsInterface
  virtual void addPositionConstraint(PositionARConstraint const* c);
  // inherited from ARConstraintsInterface
  virtual void addHalfSpaceConstraint(ARHalfSpaceConstraint const* c);
  // inherited from ARConstraintsInterface
  virtual void addSpatialDerivativeConstraint(
    SpatialDerivativeARConstraint const* c);
  // inherited from ARConstraintsInterface
  virtual void addSimpleSpatialDerivativeConstraint(
    SimpleSpatialDerivativeARConstraint const* c);
  // inherited from ARTimeVariantConstraintsInterface
  virtual void addVelocityConstraint(
    VelocityVectorARConstraint const* c);
  // inherited from ARTimeVariantConstraintsInterface
  virtual void addInstantaneousVelocityConstraint(
    InstantaneousVelocityARConstraint const* c);
  // inherited from DeformationSolver
  virtual DeformationEvaluator* createEvaluator();
  // inherited from DeformationSolver
  virtual void beginConstraints();
  // inherited from DeformationSolver
  virtual void solve();
	
  //====================================================================
  // DeformationEvaluator - TPSSolver_DeformationEvaluator.cpp
  //--------------------------------------------------------------------
	// inherited from AROptimizationTool
  virtual AROptimizationContext* createOptContext();
	// inherited from AROptimizationTool
  virtual void setup(AROptimizationContext* context,
                     ARSettings      const* settings);
  // inherited from DeformationEvaluator
  virtual void beginQuery(float32 time);
  // inherited from DeformationEvaluator
  virtual Vector3f getPosition       (ARConstraintAnchor const& at);
  // inherited from DeformationEvaluator
  virtual Vector3f getVelocity       (ARConstraintAnchor const& at);
  // inherited from DeformationEvaluator
  virtual Matrix3f getSpatialGradient(ARConstraintAnchor const& at);

private:
  //====================================================================
  // Member functions
  //--------------------------------------------------------------------
  void Clear();
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Vector3f Center(Vector3i const& cell) const;
  Vector3f Lower (Vector3i const& cell) const;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Vector3i Cell  (Vector3f const& point                      ) const;
  Vector3i Offset(Vector3f const& point, Vector3i const& cell) const;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Vector3i BSplineBaseCell(Vector3f const& point) const;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  bool GetDeformPos(Vector3i const& cell, Vector3f& deformPos);

  //--------------------------------------------------------------------
  // Initialize helper functions - TPSSolver_Initialize.cpp
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  void  AddPoint    (Vector3f const& point);
  int32 AddGridEntry(Vector3i const& cell );
  
  //--------------------------------------------------------------------
  // Solve helper functions - TPSSolver_Solve.cpp
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  DVectorF CollectDeformPos();
  void  DistributeDeformPos(DVectorF const& result);

  //--------------------------------------------------------------------
  // Evaluate helper functions - TPSSolver_Evaluate.cpp
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  bool GetEStencil(Vector3i const& cell, EStencil& stencil);

  //--------------------------------------------------------------------
  // Evaluate smoothed helper functions - TPSSolver_EvaluateSmoothed.cpp
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //bool GetESStencil(Vector3i const& cell, ESStencil& stencil);
  
  //====================================================================
  // Member variables
  //--------------------------------------------------------------------
  // Deformation grid
  SparseGrid<3, int32, Object> grid_;
  std::vector<GridEntry> gridEntries_;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  int32 gridExtension_;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  float32 gridSpacing_;
  
  //--------------------------------------------------------------------
  // Energy functional weights
  float32    identityWeight_;
  float32 regularizerWeight_;

  //--------------------------------------------------------------------
  // Smoothed deformation
  bool smoothed_;

  //--------------------------------------------------------------------
  // Solver matrices
  SparseMatrixF quadraticTerms_;
       DVectorF    linearTerms_;
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  DVectorF lastResult_;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  bool reweight_;
};

} //namespace X4

//----------------------------------------------------------------------
#endif //TPSSolver_h_
//----------------------------------------------------------------------
