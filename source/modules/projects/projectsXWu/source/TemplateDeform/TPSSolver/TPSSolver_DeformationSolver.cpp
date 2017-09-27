//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "TPSSolver.h"
//----------------------------------------------------------------------
#include "ARConstraints.h"
//----------------------------------------------------------------------
#include "DebugRenderer.h"
//----------------------------------------------------------------------

namespace X4
{
//======================================================================
// Member functions
//----------------------------------------------------------------------
void TPSSolver::addGeneralConstraint(GeneralARConstraint const* c)
{
  notImplemented();
}

//----------------------------------------------------------------------
void TPSSolver::addPositionConstraint(PositionARConstraint const* c)
{
  debugRenderer->beginRenderJob("Position_constraints", true, 3);

  Vector3f const lineColor0 = makeVector3f(1.0f, 0.50f, 0.0f);
  Vector3f const lineColor1 = makeVector3f(0.5f, 0.25f, 0.0f);

  debugRenderer->addLine(
    c->anchor.pos, c->toPoint, lineColor0, lineColor1, 4.0f);

  debugRenderer->endRenderJob();

//  debugOutput << c->covariance << "\n";


  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (smoothed_)
  {
    //AddSmoothedPositionConstraint(
    AddBSplinePositionConstraint(
      c->anchor.pos, c->toPoint, c->covariance);
    
    return;
  }

  AddPositionConstraint(c->anchor.pos, c->toPoint, c->covariance);
}

//----------------------------------------------------------------------
void TPSSolver::addHalfSpaceConstraint(ARHalfSpaceConstraint const* c)
{
  notImplemented();
}

//----------------------------------------------------------------------
void TPSSolver::addSpatialDerivativeConstraint(
  SpatialDerivativeARConstraint const* c)
{
  notImplemented();
}

//----------------------------------------------------------------------
void TPSSolver::addSimpleSpatialDerivativeConstraint(
  SimpleSpatialDerivativeARConstraint const* c)
{
  notImplemented();
}

//----------------------------------------------------------------------
void TPSSolver::addVelocityConstraint(
  VelocityVectorARConstraint const* c)
{
  notImplemented();
}

//----------------------------------------------------------------------
void TPSSolver::addInstantaneousVelocityConstraint(
    InstantaneousVelocityARConstraint const* c)
{
  notImplemented();
}

//----------------------------------------------------------------------
DeformationEvaluator* TPSSolver::createEvaluator()
{
  return this;
}

//----------------------------------------------------------------------
void TPSSolver::beginConstraints()
{
  AddRegularizer();
}

//----------------------------------------------------------------------
void TPSSolver::solve()
{
  Solve();
}

} //namespace X4
