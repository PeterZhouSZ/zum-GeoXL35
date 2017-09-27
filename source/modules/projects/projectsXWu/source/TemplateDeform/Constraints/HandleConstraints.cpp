//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "HandleConstraints.h"
//----------------------------------------------------------------------
#include "TemplateDeform/Tools/CICPHandle.h"
//----------------------------------------------------------------------
#include "ARConstraintsInterface.h"
#include "PointSet.h"
//----------------------------------------------------------------------
#include "ARConstraints.h"
//----------------------------------------------------------------------

namespace X4
{
//======================================================================
// Constructors
//----------------------------------------------------------------------
HandleConstraints::HandleConstraints(float32 const weight)
  : weight_(weight)
{}

//======================================================================
// Member functions
//----------------------------------------------------------------------
void HandleConstraints::BuildConstraints(
  ARConstraintsInterface      & receiver,
  PointSet               const& reference,
  std::vector<Handle3D*> const& handles)
{
  AAT const positionAAT = reference.getAAT("position");

  if (positionAAT == NULL_AAT)
  {
    warning("HandleCOnstraints::BuildHandleConstraints() - "
            "\"position\" attribute missing from reference.");

    return;
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (card32 i = 0; i < handles.size(); ++i)
  {
    CICPHandle* handle = dynamic_cast<CICPHandle*>(handles.at(i));

    if (!handle)
    {
      warning("HandleCOnstraints::BuildHandleConstraints() - "
              "dynamic_cast<CICPHandle*> failed");
      
      continue;
    }

    mpint const id = handle->id_;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    PositionARConstraint c;

    if (id == ARConstraintAnchor::NO_TOP_ANCHOR)
    {
      c.anchor =
        ARConstraintAnchor(handle->getPoint(1),
                           ARConstraintAnchor::NO_TOP_ANCHOR);
    }
    else
    {
      c.anchor =
        ARConstraintAnchor(reference.get3f(id, positionAAT), id);
    }

    c.covariance = IDENTITY3F * weight_;
    c.time = 0;
    c.toPoint = handle->getPoint(0);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    receiver.addPositionConstraint(&c);
  }
}

} //namespace X4
