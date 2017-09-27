//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "TPSSolver.h"
//----------------------------------------------------------------------
#include "ARConstraints.h"
//----------------------------------------------------------------------

namespace X4
{
//======================================================================
// Member functions
//----------------------------------------------------------------------
AROptimizationContext* TPSSolver::createOptContext()
{
  notImplemented();
  
  return 0;
}
  
//----------------------------------------------------------------------
void TPSSolver::setup(AROptimizationContext* context,
                      ARSettings      const* settings)
{} //don't care
  
//----------------------------------------------------------------------
void TPSSolver::beginQuery(float32 time)
{
  if (time != 0.0f)
  {
    PException("TPSSolver::beginQuery() - Non-zero time value.");
  }
}
  
//----------------------------------------------------------------------
Vector3f TPSSolver::getPosition(ARConstraintAnchor const& at)
{
  if (smoothed_)
  {
    return EvaluateBSpline(at.pos);
    //return EvaluateSmoothed(at.pos);
  }

  return Evaluate(at.pos);
}
  
//----------------------------------------------------------------------
Vector3f TPSSolver::getVelocity(ARConstraintAnchor const& at)
{
  notImplemented();

  return NULL_VECTOR3F;
}
  
//----------------------------------------------------------------------
Matrix3f TPSSolver::getSpatialGradient(ARConstraintAnchor const& at)
{
  if (smoothed_)
  {
    return EvaluateBSplineSpatialGradient(at.pos);
    //return EvaluateSmoothedSpatialGradient(at.pos);
  }

  return EvaluateSpatialGradient(at.pos);
}
  
} //namespace X4
