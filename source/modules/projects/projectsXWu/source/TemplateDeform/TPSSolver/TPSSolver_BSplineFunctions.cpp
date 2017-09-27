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
Vector3i TPSSolver::BSplineBaseCell(Vector3f const& point) const
{
  Vector3i cell = Cell(point);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Vector3f const center = Center(cell);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // If you think about it in 1D, a point can either be left or right from the
  // center of a cell (apart from being located exactly at the center). For the
  // case of the point being on the left of the center, we decrement the
  // corresponding coordinate, so that the location of the point is consistent
  // in all cases.
  if (point[0] < center[0]) { cell[0] -= 1; }
  if (point[1] < center[1]) { cell[1] -= 1; }
  if (point[2] < center[2]) { cell[2] -= 1; }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return cell;
}

//------------------------------------------------------------------------------
DVectorF TPSSolver::CalculateRegularBSplineWeights(
  Vector3i const& base, Vector3f const& point)
{
  Vector3f const fraction = (point - Center(base)) / gridSpacing_;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Vector4f const xPolynomials = BSplineBasisPolynomials(fraction[0]);
  Vector4f const yPolynomials = BSplineBasisPolynomials(fraction[1]);
  Vector4f const zPolynomials = BSplineBasisPolynomials(fraction[2]);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  DVectorF weights;

  weights.setDim(BSplineStencilSize);

  weights.setZero();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  card32 index = 0;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (card32 z = 0; z < BSplineSupport; ++z)
  {
    for (card32 y = 0; y < BSplineSupport; ++y)
    {
      for (card32 x = 0; x < BSplineSupport; ++x)
      {
        weights[index++] = xPolynomials[x] * yPolynomials[y] * zPolynomials[z];
      }
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return weights;
}

//------------------------------------------------------------------------------
DVectorF TPSSolver::CalculateShiftedBSplineWeights(
  Vector3i const& base, Vector3f const& point)
{
  DVectorF weights;

  weights.setDim(BSplineStencilSize);

  weights.setZero();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Vector3f fraction = (point - Center(base)) / gridSpacing_;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Vector3f const offset = makeVector3f(floor(fraction[0]),
                                       floor(fraction[1]),
                                       floor(fraction[2]));

  Vector3i const iOffset = makeVector3i(offset[0], offset[1], offset[2]);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (offset[0] < -3.0f || offset[0] > 3.0f ||
      offset[1] < -3.0f || offset[1] > 3.0f ||
      offset[2] < -3.0f || offset[2] > 3.0f)
  {
    // point is not in the range of base.
    return weights;
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  fraction -= offset;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Vector4f const xPolynomials = BSplineBasisPolynomials(fraction[0]);
  Vector4f const yPolynomials = BSplineBasisPolynomials(fraction[1]);
  Vector4f const zPolynomials = BSplineBasisPolynomials(fraction[2]);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  card32 index = 0;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (int32 z = 0; z < BSplineSupport; ++z)
  {
    for (int32 y = 0; y < BSplineSupport; ++y)
    {
      for (int32 x = 0; x < BSplineSupport; ++x)
      {
        int32 const xIndex = x - iOffset[0];
        int32 const yIndex = y - iOffset[1];
        int32 const zIndex = z - iOffset[2];

        if (xIndex > -1 && xIndex < 4 &&
            yIndex > -1 && yIndex < 4 &&
            zIndex > -1 && zIndex < 4)
        {
          weights[index] =
            xPolynomials[xIndex] * yPolynomials[yIndex] * zPolynomials[zIndex];
        }

        ++index;
      }
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return weights;
}

//==============================================================================
// Top-level namespace
//------------------------------------------------------------------------------
} //namespace X4
