//==============================================================================
// Includes
//------------------------------------------------------------------------------
#include "StdAfx.h"

//------------------------------------------------------------------------------
#include "TPSSolver.h"

//------------------------------------------------------------------------------
#include "BSplineBasisFunctions.h"

//==============================================================================
// Top-level namespace
//------------------------------------------------------------------------------
namespace X4
{

//==============================================================================
// Member functions
//------------------------------------------------------------------------------
Vector3f TPSSolver::EvaluateBSpline(Vector3f const& position)
{
  Vector3i const base = BSplineBaseCell(position);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  BSplinePositionStencil pStencil;

  {
    BSplineStencil stencil;

    if (!GetBSplineStencil(grid_, base, stencil))
    {
      throw ERangeCheck("TPSSolver::EvaluateBSpline() - "
                        "Point out of grid range.");
    }

    GetBSplinePositionStencil(stencil, pStencil, gridEntries_);
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  DVectorF const weights = CalculateRegularBSplineWeights(base, position);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Vector3f result = NULL_VECTOR3F;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (card32 i = 0; i < BSplineStencilSize; ++i)
  {
    result += pStencil.p[i] * weights[i];
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}

//------------------------------------------------------------------------------
Matrix3f TPSSolver::EvaluateBSplineSpatialGradient(Vector3f const& position)
{
  Vector3i const base = BSplineBaseCell(position);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  DVectorF const weights = CalculateRegularBSplineWeights(base, position);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Matrix3f result;

  result.setZero();
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  card32 i = 0;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (int32 z = 0; z < BSplineSupport; ++z)
  {
    for (int32 y = 0; y < BSplineSupport; ++y)
    {
      for (card32 x = 0; x < BSplineSupport; ++x)
      {
        Vector3i const offset = makeVector3i(x - 1, y - 1, z - 1);

        Vector3f const center = Center(base + offset);

        Matrix3f const gradient = EvaluateSpatialGradient(center);

        result += gradient * weights[i];

        ++i;
      }
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}

//==============================================================================
// Top-level namespace
//------------------------------------------------------------------------------
} // namespace X4
