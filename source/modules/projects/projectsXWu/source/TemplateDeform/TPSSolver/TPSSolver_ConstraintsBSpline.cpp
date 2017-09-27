//==============================================================================
// Includes
//------------------------------------------------------------------------------
#include "StdAfx.h"

//------------------------------------------------------------------------------
#include "TPSSolver.h"

//------------------------------------------------------------------------------
#include "BSplineBasisFunctions.h"
#include "Scattering.h"
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#include "SparseLinearAlgebra.inline.h"
#include "SparseGrid.inline.h"
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//#include "TemplateDeform/Misc/TAssert.h"

//==============================================================================
// Top-level namespace
//------------------------------------------------------------------------------
namespace X4
{

//==============================================================================
// Unnamed namespace
//------------------------------------------------------------------------------
namespace
{

//==============================================================================
// Local functions
//------------------------------------------------------------------------------
DMatrixF ComputeQ(
  DVectorF const& b,
  Vector3f const& target,
  Matrix3f const& quadric)
{
  mpcard const size = b.size();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  DynamicMatrix<float32> result;

  result.setDimension(size * 3, size * 3, false);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (int32 i = 0; i < size; ++i)
  {
    for (int32 j = 0; j < size; ++j)
    {
      Matrix3f const mat3 = quadric * (2.0f * b[i] * b[j]);

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      for (int32 d0 = 0; d0 < 3; ++d0)
      {
        for (int32 d1 = 0; d1 < 3; ++d1)
        {
          result[3 * i + d0][3 * j + d1] = mat3[d0][d1];
        }
      }
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}

//------------------------------------------------------------------------------
DVectorF ComputeL(
  DVectorF const& b,
  Vector3f const& target,
  Matrix3f const& quadric)
{
  mpcard const size = b.getDim();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  DVectorF result;

  result.setDim(size * 3);

  result.setZero();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (int32 i = 0; i < size; ++i)
  {
    Vector3f const rhs3 = (quadric * target) * (2.0f * b[i]);
  
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    for (int32 d = 0; d < 3; ++d)
    {
      result[3 * i + d] = rhs3[d];
    }
  } 

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}

//==============================================================================
// Unnamed namespace
//------------------------------------------------------------------------------
} // namespace


//==============================================================================
// Member functions
//------------------------------------------------------------------------------
void TPSSolver::AddBSplinePositionConstraint(Vector3f const& anchor,
                                             Vector3f const& target,
                                             Matrix3f const& quadric)
{
  Vector3i const base = BSplineBaseCell(anchor);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  BSplineStencil stencil;

  if (!GetBSplineStencil(grid_, base, stencil))
  {
    throw ERangeCheck("TPSSolver::AddBSplinePositionConstraint() - "
                      "Anchor out of grid range.");
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  DVectorI const mapping = stencil.GetIndexMapping();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  DVectorF const weights = CalculateRegularBSplineWeights(base, anchor);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  DMatrixF const Q = ComputeQ(weights, target, quadric);
  DVectorF const l = ComputeL(weights, target, quadric);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  ScatterAddMat1(quadraticTerms_, Q, mapping);
  ScatterAddVec1(   linearTerms_, l, mapping);
}

//==============================================================================
// Top-level namespace
//------------------------------------------------------------------------------
} // namespace X4
