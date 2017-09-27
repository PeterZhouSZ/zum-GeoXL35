//==============================================================================
// #include guard
//------------------------------------------------------------------------------
#ifndef BSplineBasisFunctions_h_
#define BSplineBasisFunctions_h_

//==============================================================================
// Includes
//------------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#include "TPSSolver.h"
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//#include "SparseGrid.h"
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//#include "DynamicLinearAlgebra.h"
//#include  "SparseLinearAlgebra.h"
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//#include "SparseLinearAlgebra.inline.h"
//#include "SparseGrid.inline.h"

//==============================================================================
// Top-level namespace
//------------------------------------------------------------------------------
namespace X4
{

//==============================================================================
// Global variables
//------------------------------------------------------------------------------
card32 const BSplineSupport     = 4;
card32 const BSplineStencilSize = 4 * 4 * 4;

//==============================================================================
// BSplineStencil struct
//------------------------------------------------------------------------------
struct BSplineStencil
{
  //============================================================================
  // Member functions
  //----------------------------------------------------------------------------
  DVectorI GetIndexMapping() const
  {
    DVectorI mapping;

    mapping.setDim(BSplineStencilSize * 3);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    for (card32 j = 0; j < BSplineStencilSize; ++j)
    {
      card32 const current = i[j] * 3;
      
      mapping[j * 3 + 0] = current + 0;
      mapping[j * 3 + 1] = current + 1;
      mapping[j * 3 + 2] = current + 2;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    return mapping;
  }

  //============================================================================
  // Member variables
  //----------------------------------------------------------------------------
  int32 i[BSplineStencilSize];
};

//==============================================================================
// BSplineStencilPosition struct
//------------------------------------------------------------------------------
struct BSplinePositionStencil
{
  //============================================================================
  // Member variables
  //----------------------------------------------------------------------------
  Vector3f p[BSplineStencilSize];
};

//==============================================================================
// Global functions
//------------------------------------------------------------------------------
inline bool GetBSplineStencil(
  SparseGrid<3, int32, Object> const& grid,
  Vector3i                     const& cell,
  BSplineStencil                    & stencil)
{
  card32 i = 0;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (int32 z = 0; z < BSplineSupport; ++z)
  {
    for (int32 y = 0; y < BSplineSupport; ++y)
    {
      for (card32 x = 0; x < BSplineSupport; ++x)
      {
        Vector3i const offset = makeVector3i(x - 1, y - 1, z - 1);

        if (!grid.getObject(cell + offset, stencil.i[i++]))
        {
          return false;
        }
      }
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return true;
}

//------------------------------------------------------------------------------
inline void GetBSplinePositionStencil(
  BSplineStencil         const&  stencil,
  BSplinePositionStencil      & pStencil,
  std::vector<GridEntry> const& gridEntries)
{
  for (card32 i = 0; i < BSplineStencilSize; ++i)
  {
    pStencil.p[i] = gridEntries.at(stencil.i[i]).deformPos;
  }
}

//------------------------------------------------------------------------------
inline Vector4f BSplineBasisPolynomials(float32 u)
{
  if (u < 0.0f) { u = 0.0f; }
  if (u > 1.0f) { u = 1.0f; }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  float32 const u2 = u  * u;
  float32 const u3 = u2 * u;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // Calculate uniform cubic b-splines polynomial basis functions.
  // See "An Introduction to Splines for Use in Computer Graphics and Geometric
  // Modelling" by Bartels et al., page 30.
  Vector4f result = NULL_VECTOR4F;

  // Note that the order is reversed.

  result[0] = (1.0f - 3.0f * u + 3.0f * u2 -        u3) / 6.0f;
  result[1] = (4.0f            - 6.0f * u2 + 3.0f * u3) / 6.0f;
  result[2] = (1.0f + 3.0f * u + 3.0f * u2 - 3.0f * u3) / 6.0f;
  result[3] = (                                     u3) / 6.0f;
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}

//==============================================================================
// Top-level namespace
//------------------------------------------------------------------------------
} // namespace X4

//==============================================================================
// #include guard
//------------------------------------------------------------------------------
#endif // BSplineBasisFunctions_h_
