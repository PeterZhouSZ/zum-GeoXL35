//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "TPSSolver.h"
//----------------------------------------------------------------------
#include "SparseLinearAlgebra.inline.h"
#include "SparseGrid.inline.h"
//----------------------------------------------------------------------

namespace X4
{
//======================================================================
// Local variables
//----------------------------------------------------------------------
static card32 const Support  = 7;
static card32 const Support3 = Support * Support * Support;

//======================================================================
// Local structs
//----------------------------------------------------------------------
struct SSStencil
{
  //====================================================================
  // Member functions
  //--------------------------------------------------------------------
  DVectorI GetIndexMapping()
  {
    DVectorI mapping;

    mapping.setDim(Support3 * 3);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    for (card32 j = 0; j < Support3; ++j)
    {
      card32 const current = i[j] * 3;
      
      mapping[j * 3 + 0] = current + 0;
      mapping[j * 3 + 1] = current + 1;
      mapping[j * 3 + 2] = current + 2;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    return mapping;
  }
  
  //====================================================================
  // Member variables
  //--------------------------------------------------------------------
  int32 i[Support3];
};

//======================================================================
// Local functions
//----------------------------------------------------------------------
template <typename FloatType>
inline FloatType WendlandKernel(FloatType distance)
{
  distance = fabs(distance   );
  distance = min (distance, 1); //from WinDef.h

  return pow(1 - distance * distance, 4);
}

//----------------------------------------------------------------------
inline bool GetSSStencil(SparseGrid<3, int32, Object> const& grid,
                         Vector3i                     const& cell,
                         SSStencil                         & stencil)
{
  card32 i = 0;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (card32 z = 0; z < Support; ++z)
  {
    for (card32 y = 0; y < Support; ++y)
    {
      for (card32 x = 0; x < Support; ++x)
      {
        Vector3i const offset = makeVector3i(x - 3, y - 3, z - 3);

        if (!grid.getObject(cell + offset, stencil.i[i++]))
        {
          return false;
        }
      }
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return true;
}

//----------------------------------------------------------------------
static card32 GetSSMapping(SSStencil const& aStencil,
                           SSStencil const& tStencil,
                           DVectorI       & sMapping)
{
  sMapping.setDim(Support3 * 2);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (card32 i = 0; i < Support3; ++i)
  {
    sMapping[i] = i;
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  card32 size = Support3;
  
  for (card32 i = 0; i < Support3; ++i)
  {
    bool found = false;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    for (card32 j = 0; j < Support3; ++j)
    {
      if (tStencil.i[i] == aStencil.i[j])
      {
        sMapping[Support3 + i] = j;

        found = true;

        break;
      }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if (!found)
    {
      sMapping[Support3 + i] = size++;
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return size;
}

//----------------------------------------------------------------------
static void ScatterAddMat1( SparseMatrix<float32>      & sparse,
                           DynamicMatrix<float32> const& dense,
                           DynamicVector<  int32> const& mapping)
{
  mpcard const size = mapping.getDim();

  if (size != dense.getColsDim() || size != dense.getRowsDim())
  {
    throw PException("ScatterAddMat1() - Size mismatch.");
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (int32 row = 0; row < size; ++row)
  {
    SparseVector<float32>& rowVec = sparse[mapping[row]];

    for (int32 col = 0; col < size; ++col)
    {
      if (dense[col][row])
      {
        rowVec.setEntryBinary(mapping[col], dense[col][row] + rowVec.getEntry(mapping[col]));
      }
    }

    //#pragma omp critical
    //sparse[mapping[row]] += rowVec;
  }
}

//----------------------------------------------------------------------
static void ScatterAddVec1(DynamicVector<float32>      & dest,
                           DynamicVector<float32> const& dense,
                           DynamicVector<  int32> const& mapping)
{
  mpcard const size = mapping.getDim();

  if (size != dense.getDim())
  {
    throw PException("ScatterAddVec1() - Size mismatch.");
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (int i = 0; i < size; ++i)
  {
    #pragma omp critical
    dest[mapping[i]] += dense[i];
  }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
template <typename T>
DynamicVector<T> Combine(DynamicVector<T> const& a,
                         DynamicVector<T> const& b)
{
  DynamicVector<T> result;

  result.setDim(a.getDim() + b.getDim());

  mpcard index = 0;

  for (card32 i = 0; i < a.getDim(); ++i)
  {
    result[index++] = a[i];
  }

  for (card32 i = 0; i < b.getDim(); ++i)
  {
    result[index++] = b[i];
  }

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

//----------------------------------------------------------------------
static DynamicMatrix<float32> ComputeQ1(DynamicVector<float32> const& b)
{
  mpcard const size = b.getDim();

  DynamicMatrix<float32> result;

  result.setDimension(3 * size, 3 * size, false);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (int32 col = 0; col < size; ++col)
  {
    for (int32 row = 0; row < size; ++row)
    {
      float32 const weight = b[col] * b[row];

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      for (int32 d = 0; d < 3; ++d)
      {
        result[3 * col + d][3 * row + d] = weight;
      }
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}

//----------------------------------------------------------------------
static DynamicMatrix<float32> ComputeQ2(DVectorF const& bx,
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

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  DynamicMatrix<float> result;

  result.setDimension(size * 3, size * 3, false);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Matrix3f const rotationT = rotation.transpose();

  for (int32 i = 0; i < vSize; ++i)
  {
    int32 const col = mapping[i];

    if (i >= Support3 && col < Support3) { continue; }

    for (int32 j = 0; j < vSize; ++j)
    {
      int32 const row = mapping[j];

      if (j >= Support3 && row < Support3) { continue; }

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      Matrix3f const entry = rotationT * bx[j] * by[i]
                           + rotation  * bx[i] * by[j];

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      for (int32 d0 = 0; d0 < 3; ++d0)
      {
        for (int32 d1 = 0; d1 < 3; ++d1)
        {
          result[3 * col + d0][3 * row + d1] -= entry[d0][d1];
        }
      }
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}

//----------------------------------------------------------------------
static DVectorF ComputeL(DVectorF const& b, Vector3f const& t)
{
  mpcard const size = b.getDim();

  DVectorF result;

  result.setDim(size * 3);

  result.setZero();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (int32 i = 0; i < size; ++i)
  {
    Vector3f const rhs = t * b[i];

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    for (int32 d = 0; d < 3; ++d)
    {
      result[3 * i + d] = rhs[d];
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}


//----------------------------------------------------------------------
//DVectorF TPSSolver::CalculateSmoothedWeights(Vector3i const& base,
//                                             Vector3f const& point)
//{
//  DVectorF weights;
//
//  weights.setDim(Support3);
//
//  weights.setZero();
//
//  card32 index = 0;
//
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  for (card32 z = 0; z < Support; ++z)
//  {
//    for (card32 y = 0; y < Support; ++y)
//    {
//      for (card32 x = 0; x < Support; ++x)
//      {
//        Vector3i const offset = makeVector3i(x - 3, y - 3, z - 3);
//
//        Vector3f const center = Center(base + offset);
//
//        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//        float32 const w =
//          WendlandKernel(norm(center - point) / 3.0f / gridSpacing_);
//
//        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//        weights[index++] = w;
//      }
//    }
//  }
//
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  return weights;
//}

//----------------------------------------------------------------------
//void TPSSolver::AddSmoothedSymmetryConstraint(
//  Vector3f const& anchor,
//  Vector3f const& target,
//  Matrix3f const& rotation,
//  Vector3f const& translation,
//  float32  const  weight)
//{
//  Vector3i const aCell = Cell(anchor);
//  Vector3i const tCell = Cell(target);
//  
//  Vector3i const aBase = aCell + Offset(anchor, aCell);
//  Vector3i const tBase = tCell + Offset(target, tCell);
//
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  SSStencil* aStencil = new SSStencil;
//  SSStencil* tStencil = new SSStencil;
//
//  if (!GetSSStencil(grid_, aBase, *aStencil))
//  {
//    throw ERangeCheck("TPSSolver::AddSymmetryConstraint() - "
//                      "Anchor out of grid range.");
//  }
//
//  if (!GetSSStencil(grid_, tBase, *tStencil))
//  {
//    throw ERangeCheck("TPSSolver::AddSymmetryConstraint() - "
//                      "Target out of grid range.");
//  }
//
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  DVectorI const xMapping = aStencil->GetIndexMapping();
//  DVectorI const yMapping = tStencil->GetIndexMapping();
//
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  DVectorI sMapping;
//  
//  card32 const size = GetSSMapping(*aStencil, *tStencil, sMapping);
//
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  delete aStencil;
//  delete tStencil;
//
//  aStencil = 0;
//  tStencil = 0;
//
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  DVectorI additional;
//
//  additional.setDim((size - Support3) * 3);
//
//  for (card32 i = 0; i < Support3; ++i)
//  {
//    if (sMapping[Support3 + i] < Support3) { continue; }
//
//    int32 const index = sMapping[Support3 + i] - Support3;
//
//    additional[index * 3 + 0] = yMapping[i * 3 + 0];
//    additional[index * 3 + 1] = yMapping[i * 3 + 1];
//    additional[index * 3 + 2] = yMapping[i * 3 + 2];
//  }
//  
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  DVectorI const mapping = Combine(xMapping, additional);
//  
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  DVectorF const b1x = CalculateSmoothedWeights(aBase, anchor);
//  DVectorF const b2x = CalculateSmoothedWeights(tBase, anchor);
//
//  DVectorF const b1y = CalculateSmoothedWeights(aBase, target);
//  DVectorF const b2y = CalculateSmoothedWeights(tBase, target);
//
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  DVectorF const bx  = Combine(b1x, b2x);
//  DVectorF const by  = Combine(b1y, b2y);
//  
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  DMatrixF const Q1x  = ComputeQ1(b1x);
//  DMatrixF const Q1y  = ComputeQ1(b2y);
//  
//  //EnsureSymmetric(Q1x,  "Q1x" );
//  //EnsureSymmetric(Q1y,  "Q1y" );
//  
//  ScatterAddMat1(quadraticTerms_, Q1x  * weight, xMapping);
//  ScatterAddMat1(quadraticTerms_, Q1y  * weight, yMapping);
//  
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  DMatrixF const Q2xy =
//      ComputeQ2(bx, by, rotation, sMapping, size);
//  
//  //EnsureSymmetric(Q2xy, "Q2xy");
//
//  ScatterAddMat1(quadraticTerms_, Q2xy * weight,  mapping);
//
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  DVectorF const l1 = ComputeL(b1x, -rotation.transpose() * translation);
//  DVectorF const l2 = ComputeL(b2y,                         translation); 
//  
//  ScatterAddVec1(linearTerms_, l1 * weight, xMapping);
//  ScatterAddVec1(linearTerms_, l2 * weight, yMapping);
//}

} //namespace X4
