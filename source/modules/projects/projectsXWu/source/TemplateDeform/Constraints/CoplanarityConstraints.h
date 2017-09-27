//----------------------------------------------------------------------
#ifndef CoplanarityConstraints_h_
#define CoplanarityConstraints_h_
//----------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//----------------------------------------------------------------------
#include "Util/TrimeshStatic.h"

#include <vector>

namespace X4
{
//======================================================================
// Forward declarations
//----------------------------------------------------------------------
class UnstructuredInCoreTriangleMesh;
class DeformationEvaluator;
class ARConstraintsInterface;
class PointSet;

//======================================================================
// CoplanarityConstraint
//----------------------------------------------------------------------
class CoplanarityConstraint
{
public:
  //====================================================================
  // Constructors
  //--------------------------------------------------------------------
  CoplanarityConstraint(std::vector<Vector3f> const& points);

  //====================================================================
  // Member functions
  //--------------------------------------------------------------------
  void Update(DeformationEvaluator& evaluator);
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  void Reset();
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  void Build(
    ARConstraintsInterface      & receiver,
    float32                const  gridSpacing,
    float32                const  weight);

private:
  //====================================================================
  // Member variables
  //--------------------------------------------------------------------
  std::vector<Vector3f> reference_;
  std::vector<Vector3f> deformed_;
};

//======================================================================
// CoplanarityConstraints
//----------------------------------------------------------------------
class CoplanarityConstraints
{
public:
  //====================================================================
  // Constructors
  //--------------------------------------------------------------------
  CoplanarityConstraints(float32 const angleThreshold = 5.0f,
                         float32 const  sizeThreshold = 0.01f);

  //====================================================================
  // Member functions
  //--------------------------------------------------------------------
  void Initialize(UnstructuredInCoreTriangleMesh& mesh);
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  void Update(DeformationEvaluator& evaluator);
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  void Build(
    ARConstraintsInterface      & receiver,
    float32                const  gridSpacing,
    float32                const  weight);

private:
  //====================================================================
  // Member variables
  //--------------------------------------------------------------------
  std::vector<CoplanarityConstraint> constraints_;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  float32 angleThreshold_;
  float32  sizeThreshold_;
};

} //namespace X4

//----------------------------------------------------------------------
#endif //CoplanarityConstraints_h_
//----------------------------------------------------------------------
