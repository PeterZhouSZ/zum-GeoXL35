//==============================================================================
// Includes
//------------------------------------------------------------------------------
#include "StdAfx.h"

//------------------------------------------------------------------------------
#include "TPSSolver.h"

//------------------------------------------------------------------------------
#include "Scattering.h"
#include "BSplineBasisFunctions.h"
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#include "SparseLinearAlgebra.inline.h"
#include "SparseGrid.inline.h"

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
inline card32 GetBSplineSMapping(
  BSplineStencil const& aStencil,
  BSplineStencil const& tStencil,
  DVectorI            & sMapping)
{
  sMapping.setDim(BSplineStencilSize * 2);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (card32 i = 0; i < BSplineStencilSize; ++i)
  {
    sMapping[i] = i;
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  card32 size = BSplineStencilSize;
  
  for (card32 i = 0; i < BSplineStencilSize; ++i)
  {
    bool found = false;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    for (card32 j = 0; j < BSplineStencilSize; ++j)
    {
      if (tStencil.i[i] == aStencil.i[j])
      {
        sMapping[BSplineStencilSize + i] = j;

        found = true;

        break;
      }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if (!found)
    {
      sMapping[BSplineStencilSize + i] = size++;
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return size;
}

//------------------------------------------------------------------------------
template <typename T>
inline DynamicVector<T> Combine(DynamicVector<T> const& a,
                                DynamicVector<T> const& b)
{
  DynamicVector<T> result;

  result.setDim(a.getDim() + b.getDim());

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  mpcard index = 0;

  for (card32 i = 0; i < a.getDim(); ++i)
  {
    result[index++] = a[i];
  }

  for (card32 i = 0; i < b.getDim(); ++i)
  {
    result[index++] = b[i];
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}

//------------------------------------------------------------------------------
inline DynamicMatrix<float32> ComputeQ1(DynamicVector<float32> const& b)
{
  mpcard const size = b.getDim();

  DynamicMatrix<float32> result;

  result.setDimension(3 * size, 3 * size, false);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (int32 col = 0; col < size; ++col)
  {
    for (int32 row = 0; row < size; ++row)
    {
      float32 const weight = b[col] * b[row];

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      for (int32 d = 0; d < 3; ++d)
      {
        result[3 * col + d][3 * row + d] = weight;
      }
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}

//------------------------------------------------------------------------------
inline DynamicMatrix<float32> ComputeQ2(DVectorF const& bx,
                                        DVectorF const& by,
                                        Matrix3f const& rotation,
                                        DVectorI const& mapping,
                                        card32   const  size)
{
  mpcard const vSize = mapping.size();

  if (vSize != bx.getDim() || vSize != by.getDim())
  {
    throw PException("ComputeQ2() - Size mismatch.");
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  DynamicMatrix<float32> result;

  result.setDimension(size * 3, size * 3, false);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Matrix3f const rotationT = rotation.transpose();

  for (int32 i = 0; i < vSize; ++i)
  {
    int32 const col = mapping[i];

    if (i >= BSplineStencilSize && col < BSplineStencilSize) { continue; }

    for (int32 j = 0; j < vSize; ++j)
    {
      int32 const row = mapping[j];

      if (j >= BSplineStencilSize && row < BSplineStencilSize) { continue; }

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      Matrix3f const entry = rotationT * bx[j] * by[i]
                           + rotation  * bx[i] * by[j];

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      for (int32 d0 = 0; d0 < 3; ++d0)
      {
        for (int32 d1 = 0; d1 < 3; ++d1)
        {
          result[3 * col + d0][3 * row + d1] -= entry[d0][d1];
        }
      }
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}

//------------------------------------------------------------------------------
static DVectorF ComputeL(DVectorF const& b, Vector3f const& t)
{
  mpcard const size = b.getDim();

  DVectorF result;

  result.setDim(size * 3);

  result.setZero();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (int32 i = 0; i < size; ++i)
  {
    Vector3f const rhs = t * b[i];

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    for (int32 d = 0; d < 3; ++d)
    {
      result[3 * i + d] = rhs[d];
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}



//----------------------------------------------------------------------
//template <typename T>
//void EnsureSymmetric(DynamicMatrix<T> const& matrix,
//                     std::string      const& name)
//{
//  mpcard const size = matrix.getColsDim();
//
//  if (matrix.getRowsDim() != size)
//  {
//    throw PException("Size mismatch");
//  }
//
//  for (card32 col = 0; col < size; ++col)
//  {
//    for (card32 row = 0; row < size; ++row)
//    {
//      if (matrix[col][row] != matrix[row][col])
//      {
//        std::stringstream error;
//
//        error << "EnsureSymmetric() - " << name << " ist not symmetric."
//              << " Detected at [" << col << "][" << row << "]; "
//              << matrix[col][row] << " != " << matrix[row][col] << ".";
//
//        throw PException(error.str());
//      }
//    }
//  }
//}

//==============================================================================
// Unnamed namespace
//------------------------------------------------------------------------------
}

//==============================================================================
// Member functions
//------------------------------------------------------------------------------
void TPSSolver::AddBSplineSymmetryConstraint(
  Vector3f const& anchor,
  Vector3f const& target,
  Matrix3f const& rotation,
  Vector3f const& translation,
  float32  const  weight)
{
  Vector3i const aBase = BSplineBaseCell(anchor);
  Vector3i const tBase = BSplineBaseCell(target);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  BSplineStencil* aStencil = new BSplineStencil;
  BSplineStencil* tStencil = new BSplineStencil;

  if (!GetBSplineStencil(grid_, aBase, *aStencil))
  {
    throw ERangeCheck("TPSSolver::AddBSplineSymmetryConstraint() - "
                      "Anchor out of grid range.");
  }

  if (!GetBSplineStencil(grid_, tBase, *tStencil))
  {
    throw ERangeCheck("TPSSolver::AddBSplineSymmetryConstraint() - "
                      "Target out of grid range.");
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  DVectorI const xMapping = aStencil->GetIndexMapping();
  DVectorI const yMapping = tStencil->GetIndexMapping();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  DVectorI sMapping;
  
  card32 const size = GetBSplineSMapping(*aStencil, *tStencil, sMapping);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  delete aStencil;
  delete tStencil;

  aStencil = 0;
  tStencil = 0;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  DVectorI additional;

  additional.setDim((size - BSplineStencilSize) * 3);

  for (card32 i = 0; i < BSplineStencilSize; ++i)
  {
    if (sMapping[BSplineStencilSize + i] < BSplineStencilSize) { continue; }

    int32 const index = sMapping[BSplineStencilSize + i] - BSplineStencilSize;

    additional[index * 3 + 0] = yMapping[i * 3 + 0];
    additional[index * 3 + 1] = yMapping[i * 3 + 1];
    additional[index * 3 + 2] = yMapping[i * 3 + 2];
  }
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  DVectorI const mapping = Combine(xMapping, additional);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  DVectorF const b1x = CalculateRegularBSplineWeights(aBase, anchor); 
  DVectorF const b2x = CalculateShiftedBSplineWeights(tBase, anchor);
  DVectorF const b1y = CalculateShiftedBSplineWeights(aBase, target);
  DVectorF const b2y = CalculateRegularBSplineWeights(tBase, target);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  DVectorF const bx  = Combine(b1x, b2x);
  DVectorF const by  = Combine(b1y, b2y);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  DMatrixF const Q1x  = ComputeQ1(b1x);
  DMatrixF const Q1y  = ComputeQ1(b2y);
  
  //EnsureSymmetric(Q1x,  "Q1x" );
  //EnsureSymmetric(Q1y,  "Q1y" );
  
  ScatterAddMat1(quadraticTerms_, Q1x  * weight, xMapping);
  ScatterAddMat1(quadraticTerms_, Q1y  * weight, yMapping);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  DMatrixF const Q2xy = ComputeQ2(bx, by, rotation, sMapping, size);
  
  //EnsureSymmetric(Q2xy, "Q2xy");

  ScatterAddMat1(quadraticTerms_, Q2xy * weight,  mapping);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  DVectorF const l1 = ComputeL(b1x, -rotation.transpose() * translation);
  DVectorF const l2 = ComputeL(b2y,                         translation); 
  
  ScatterAddVec1(linearTerms_, l1 * weight, xMapping);
  ScatterAddVec1(linearTerms_, l2 * weight, yMapping);
}

} //namespace X4
