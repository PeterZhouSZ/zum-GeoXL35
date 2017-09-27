//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "TPSSolver.h"
//----------------------------------------------------------------------
#include "SparseLinearAlgebra.inline.h"
#include "SparseGrid.inline.h"
//----------------------------------------------------------------------
//#include "TemplateDeform/Misc/TAssert.h"
//----------------------------------------------------------------------
#include "DebugRenderer.h"
//----------------------------------------------------------------------

namespace X4
{
//----------------------------------------------------------------------
struct SStencil
{
  //====================================================================
  // Member functions
  //--------------------------------------------------------------------
  StaticVector<int32, 24> GetIndexMapping()
  {
    StaticVector<int32, 24> mapping;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    mapping[ 0] = i[0] * 3;
    mapping[ 1] = i[0] * 3 + 1;
    mapping[ 2] = i[0] * 3 + 2;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    mapping[ 3] = i[1] * 3;
    mapping[ 4] = i[1] * 3 + 1;
    mapping[ 5] = i[1] * 3 + 2;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    mapping[ 6] = i[2] * 3;
    mapping[ 7] = i[2] * 3 + 1;
    mapping[ 8] = i[2] * 3 + 2;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    mapping[ 9] = i[3] * 3;
    mapping[10] = i[3] * 3 + 1;
    mapping[11] = i[3] * 3 + 2;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    mapping[12] = i[4] * 3;
    mapping[13] = i[4] * 3 + 1;
    mapping[14] = i[4] * 3 + 2;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    mapping[15] = i[5] * 3;
    mapping[16] = i[5] * 3 + 1;
    mapping[17] = i[5] * 3 + 2;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    mapping[18] = i[6] * 3;
    mapping[19] = i[6] * 3 + 1;
    mapping[20] = i[6] * 3 + 2;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    mapping[21] = i[7] * 3;
    mapping[22] = i[7] * 3 + 1;
    mapping[23] = i[7] * 3 + 2;
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    return mapping;
  }
  
  //====================================================================
  // Member variables
  //--------------------------------------------------------------------
  int32 i[8];
};

//======================================================================
// Local functions
//----------------------------------------------------------------------
inline bool GetSStencil(SparseGrid<3, int32, Object> const& grid,
                        Vector3i                     const& cell,
                        SStencil                          & stencil)
{
  return grid.getObject(cell + makeVector3i(0, 0, 0), stencil.i[0]) &&
         grid.getObject(cell + makeVector3i(0, 0, 1), stencil.i[1]) &&
         grid.getObject(cell + makeVector3i(0, 1, 0), stencil.i[2]) &&
         grid.getObject(cell + makeVector3i(0, 1, 1), stencil.i[3]) &&
         grid.getObject(cell + makeVector3i(1, 0, 0), stencil.i[4]) &&
         grid.getObject(cell + makeVector3i(1, 0, 1), stencil.i[5]) &&
         grid.getObject(cell + makeVector3i(1, 1, 0), stencil.i[6]) &&
         grid.getObject(cell + makeVector3i(1, 1, 1), stencil.i[7]);
}

//----------------------------------------------------------------------
template <typename FloatType, int SIZE>
static void ScatterAddMat1(
  SparseMatrix<FloatType>                  & sparse,
  StaticMatrix<FloatType, SIZE, SIZE> const& dense,
  StaticVector<int32,     SIZE>       const& mapping)
{
  TASSERT(SIZE);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (int32 row = 0; row < SIZE; ++row)
  {
    SparseVector<FloatType> rowVec;

    for (int32 col = 0; col < SIZE; ++col)
    {
      if (dense[col][row])
      {
        rowVec.setEntry(mapping[col], dense[col][row]);
      }
    }

    //#pragma omp critical
    sparse[mapping[row]] += rowVec;
  }
}

//----------------------------------------------------------------------
template <typename FloatType>
static void ScatterAddMat1(
   SparseMatrix<FloatType>      & sparse,
  DynamicMatrix<FloatType> const& dense,
  DynamicVector<int32    > const& mapping)
{
  int32 const size = dense.getColsDim();

  if (dense.getRowsDim() != size ||
        mapping.getDim() != size)
  {
    throw PException("Size mismatch.");
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (int32 row = 0; row < size; ++row)
  {
    SparseVector<FloatType> rowVec;

    for (int32 col = 0; col < size; ++col)
    {
      if (dense[col][row])
      {
        rowVec.setEntry(mapping[col], dense[col][row]);
      }
    }

    //#pragma omp critical
    sparse[mapping[row]] += rowVec;
  }
}

//----------------------------------------------------------------------
template <typename FloatType, int SIZE>
static void ScatterAddVec1(
  DVectorF                           & dest,
  StaticVector<FloatType, SIZE> const& dense,
  StaticVector<int32,     SIZE> const& mapping)
{
  TASSERT(SIZE);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (int i = 0; i < SIZE; ++i)
  {
    //#pragma omp critical
    dest[mapping[i]] += dense[i];
  }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
template <typename T, card32 SIZE>
StaticVector<T, 2 * SIZE> Combine(StaticVector<T, SIZE> a,
                                  StaticVector<T, SIZE> b)
{
  StaticVector<T, 2 * SIZE> result;

  for (card32 i = 0; i < SIZE; ++i)
  {
    result[i       ] = a[i];
    result[i + SIZE] = b[i];
  }

  return result;
}

//----------------------------------------------------------------------
//template <typename T, card32 SIZE>
//void EnsureSymmetric(StaticMatrix<T, SIZE, SIZE> const& matrix,
//                     std::string                 const& name)
//{
//  for (card32 col = 0; col < SIZE; ++col)
//  {
//    for (card32 row = 0; row < SIZE; ++row)
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
StaticMatrix<float32, 24, 24> ComputeQ1(
  StaticVector<float32, 8> const& b)
{
  StaticMatrix<float32, 24, 24> result;

  result.setZero();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (int32 col = 0; col < 8; ++col)
  {
    for (int32 row = 0; row < 8; ++row)
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
DynamicMatrix<float32> ComputeQ2(
  StaticVector<float32, 16> const& bx,
  StaticVector<float32, 16> const& by,
  Matrix3f                  const& rotation,
  StaticVector<int32,   16> const& mapping,
  card32                    const  size,
  StaticVector<bool,    16> const& execute)
{
  DynamicMatrix<float> result;

  result.setDimension(size * 3, size * 3, false);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Matrix3f const rotationT = rotation.transpose();

  for (int32 i = 0; i < 16; ++i)
  {
    int32 const col = mapping[i];

    for (int32 j = 0; j < 16; ++j)
    {
      int32 const row = mapping[j];

      if (!execute[i] || !execute[j]) { continue; }

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
StaticMatrix<float32, 48, 48> ComputeQ2(
  StaticVector<float32, 16> const& bx,
  StaticVector<float32, 16> const& by,
  Matrix3f                  const& rotation)
{
  StaticMatrix<float32, 48, 48> result;

  result.setZero();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Matrix3f const q2 = rotation.transpose() + rotation;

  for (int32 col = 0; col < 16; ++col)
  {
    for (int32 row = 0; row < 16; ++row)
    {
      float32 const weight = (bx[row] * by[col] +
                              bx[col] * by[row]) * 0.5f;

      Matrix3f const entry = q2 * weight;
      
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
StaticVector<float32, 24> ComputeL(StaticVector<float32, 8> const& b,
                                   Vector3f                 const& t)
{
  StaticVector<float32, 24> result;

  result.setZero();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (int32 i = 0; i < 8; ++i)
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
inline StaticVector<float32, 8> TPSSolver::CalculateLinearWeights(
  Vector3i const& base, Vector3f const& point)
{
  StaticVector<float32, 8> result;

  result.setZero();

  for (card32 i = 0; i < 8; ++i)
  {
    Vector3i const offset = makeVector3i(i >> 2, i >> 1 & 0x1, i & 0x1);

    result[i] = EvaluateLinearBasisFunction(base + offset, point);
  }

  return result;
}


//----------------------------------------------------------------------
inline float32 TPSSolver::EvaluateLinearBasisFunction(
  Vector3i const& of, Vector3f const& at)
{
  Vector3f const ofPosition = Center(of);

  Vector3f distance = (at - ofPosition) / gridSpacing_;

  distance[0] = fabs(distance[0]);
  distance[1] = fabs(distance[1]);
  distance[2] = fabs(distance[2]);

  if (distance[0] > 1.0f ||
      distance[1] > 1.0f ||
      distance[2] > 1.0f)
  {
    return 0.0f;
  }

  return (1.0f - distance[0]) *
         (1.0f - distance[1]) *
         (1.0f - distance[2]);
}

//----------------------------------------------------------------------
void TPSSolver::AddSymmetryConstraint(Vector3f const& anchor,
                                      Vector3f const& target,
                                      Matrix3f const& rotation,
                                      Vector3f const& translation,
                                      float32  const  weight)
{
  debugRenderer->beginRenderJob("Symmetry_constraints", true, 3);

  Vector3f const lineColor0 = makeVector3f(0.21f, 0.46f, 0.56f);
  Vector3f const lineColor1 = makeVector3f(1.00f, 1.00f, 1.00f);

  debugRenderer->addLine(anchor, target, lineColor0, lineColor1, 4.0f);

  debugRenderer->endRenderJob();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (smoothed_)
  {
    AddBSplineSymmetryConstraint(
//    AddSmoothedSymmetryConstraint(
      anchor, target, rotation, translation, weight);
    
    return;
  }

  AddLinearSymmetryConstraint(
    anchor, target, rotation, translation, weight);
}

//----------------------------------------------------------------------
void TPSSolver::AddLinearSymmetryConstraint(Vector3f const& anchor,
                                            Vector3f const& target,
                                            Matrix3f const& rotation,
                                            Vector3f const& translation,
                                            float32  const  weight)
{
  Vector3i const aCell = Cell(anchor);
  Vector3i const tCell = Cell(target);

  Vector3i const aOffset = Offset(anchor, aCell);
  Vector3i const tOffset = Offset(target, tCell);

  Vector3i const aBase = aCell + aOffset;
  Vector3i const tBase = tCell + tOffset;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  SStencil aStencil;
  SStencil tStencil;

  if (!GetSStencil(grid_, aBase, aStencil))
  {
    throw ERangeCheck("TPSSolver::AddSymmetryConstraint() - "
                      "Anchor out of grid range.");
  }

  if (!GetSStencil(grid_, tBase, tStencil))
  {
    throw ERangeCheck("TPSSolver::AddSymmetryConstraint() - "
                      "Target out of grid range.");
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  StaticVector<int32, 24> const xMapping = aStencil.GetIndexMapping();
  StaticVector<int32, 24> const yMapping = tStencil.GetIndexMapping();

  StaticVector<int32, 48> const  mapping = Combine(xMapping, yMapping);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  StaticVector<float32,  8> const b1x = CalculateLinearWeights(aBase, anchor);
  StaticVector<float32,  8> const b2x = CalculateLinearWeights(tBase, anchor);

  StaticVector<float32,  8> const b1y = CalculateLinearWeights(aBase, target);
  StaticVector<float32,  8> const b2y = CalculateLinearWeights(tBase, target);

  StaticVector<int32, 16> stencilMapping;
  StaticVector<bool,  16> execute;

  for (card32 i = 0; i < 8; ++i)
  {
    stencilMapping[i] = i;
    execute[i] = true;
  }

  card32 size = 8;

  for (card32 i = 0; i < 8; ++i)
  {
    int32 const current = tStencil.i[i];

    bool found = false;

    for (card32 j = 0; j < 8; ++j)
    {
      if (aStencil.i[j] != current) { continue; }

      found = true;

      stencilMapping[8 + i] = j;
      execute[8 + i] = false;

      break;
    }

    if (found) { continue; }

    stencilMapping[8 + i] = size;
    execute[8 + i] = true;

    ++size;
  }

  DynamicVector<int32> Dmapping;
  Dmapping.setDim(size * 3);

  for (card32 i = 0; i < 24; ++i)
  {
    Dmapping[i] = xMapping[i];
  }

  for (card32 i = 0; i < 8; ++i)
  {
    if (stencilMapping[8 + i] < 8) { continue; }

    int32 const index = stencilMapping[8 + i];

    Dmapping[index * 3 + 0] = yMapping[i * 3 + 0];
    Dmapping[index * 3 + 1] = yMapping[i * 3 + 1];
    Dmapping[index * 3 + 2] = yMapping[i * 3 + 2];
  }

  StaticVector<float32, 16> const bx  = Combine(b1x, b2x);
  StaticVector<float32, 16> const by  = Combine(b1y, b2y);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  StaticMatrix<float32, 24, 24> Q1x  = ComputeQ1(b1x);
  StaticMatrix<float32, 24, 24> Q1y  = ComputeQ1(b2y);

  DynamicMatrix<float32> Q2xy =
    ComputeQ2(bx, by, rotation, stencilMapping, size, execute);

  StaticVector<float32, 24> l1 = ComputeL(b1x, rotation.transpose() * translation);
  StaticVector<float32, 24> l2 = ComputeL(b2y,                        translation); 

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //EnsureSymmetric(Q1x,  "Q1x" );
  //EnsureSymmetric(Q1y,  "Q1y" );7
  //EnsureSymmetric(Q2xy, "Q2xy");
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  ScatterAddMat1(quadraticTerms_,  Q1x  * weight, xMapping);
  ScatterAddMat1(quadraticTerms_,  Q1y  * weight, yMapping);
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  ScatterAddMat1(quadraticTerms_,  Q2xy * weight, Dmapping);

  ScatterAddVec1(   linearTerms_, -l1   * weight, xMapping);
  ScatterAddVec1(   linearTerms_,  l2   * weight, yMapping);
}

} //namespace X4
