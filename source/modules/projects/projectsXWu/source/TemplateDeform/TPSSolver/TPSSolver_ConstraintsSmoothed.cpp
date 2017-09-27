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
struct CSStencil
{
  //====================================================================
  // Member functions
  //--------------------------------------------------------------------
  StaticVector<int32, Support3 * 3> GetIndexMapping()
  {
    StaticVector<int32, Support3 * 3> mapping;

    card32 index = 0;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    for (card32 z = 0; z < Support; ++z)
    {
      for (card32 y = 0; y < Support; ++y)
      {
        for (card32 x = 0; x < Support; ++x)
        {
          card32 const current = i[x][y][z] * 3;

          mapping[index++] = current + 0;
          mapping[index++] = current + 1;
          mapping[index++] = current + 2;
        }
      }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    return mapping;
  }
  
  //====================================================================
  // Member variables
  //--------------------------------------------------------------------
  int32 i[7][7][7];
};

//======================================================================
// Local functions
//----------------------------------------------------------------------
template <typename FloatType>
inline FloatType WendlandKernel(FloatType distance)
{
  distance = fabs(distance);
  distance = min (distance, 1); //from WinDef.h

  return pow(1 - distance * distance, 4);
}

//----------------------------------------------------------------------
inline bool GetCSStencil(SparseGrid<3, int32, Object> const& grid,
                         Vector3i                     const& cell,
                         CSStencil                         & stencil)
{
  for (card32 z = 0; z < Support; ++z)
  {
    for (card32 y = 0; y < Support; ++y)
    {
      for (card32 x = 0; x < Support; ++x)
      {
        Vector3i const offset = makeVector3i(x - 3, y - 3, z - 3);

        if (!grid.getObject(cell + offset, stencil.i[x][y][z]))
        {
          return false;
        }
      }
    }
  }

  return true;
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
    dest[mapping[i]] += dense[i];
  }
}

//----------------------------------------------------------------------
StaticVector<float32, Support3 * 3>* ComputeL(
  StaticVector<float32, Support3> const& b,
  Vector3f                        const& target,
  Matrix3f                        const& quadric)
{
  StaticVector<float32, Support3 * 3>* result =
    new StaticVector<float32, Support3 * 3>();

  result->setZero();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (int32 i = 0; i < Support3; ++i)
  {
    Vector3f const rhs3 = (quadric * target) * (2.0f * b[i]);
  
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    for (int32 d = 0; d < 3; ++d)
    {
      (*result)[3 * i + d] = rhs3[d];
    }
  } 

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}

//----------------------------------------------------------------------
StaticMatrix<float32, Support3 * 3, Support3 * 3>* ComputeQ(
  StaticVector<float32, Support3> const& b,
  Vector3f                        const& target,
  Matrix3f                        const& quadric)
{
  StaticMatrix<float32, Support3 * 3, Support3 * 3>* result =
    new StaticMatrix<float32, Support3 * 3, Support3 * 3>();

  result->setZero();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (int32 i = 0; i < Support3; ++i)
  {
    for (int32 j = 0; j < Support3; ++j)
    {
      Matrix3f const mat3 = quadric * (2.0f * b[i] * b[j]);

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      for (int32 d0 = 0; d0 < 3; ++d0)
      {
        for (int32 d1 = 0; d1 < 3; ++d1)
        {
          (*result)[3 * i + d0][3 * j + d1] = mat3[d0][d1];
        }
      }
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}


//----------------------------------------------------------------------
//void TPSSolver::AddSmoothedPositionConstraint(Vector3f const& anchor,
//                                              Vector3f const& target,
//                                              Matrix3f const& quadric)
//{
//  Vector3i const cell = Cell(anchor);
//  
//  Vector3i const base = cell + Offset(anchor, cell);
//
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  CSStencil stencil;
//
//  if (!GetCSStencil(grid_, base, stencil))
//  {
//    throw ERangeCheck("TPSSolver::AddSmoothedPositionConstraint() - "
//                      "Anchor out of grid range.");
//  }
//
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  StaticVector<float32, Support3> weights;
//
//  card32 index = 0;
//
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
//          WendlandKernel(norm(center - anchor) / 3.0f / gridSpacing_);
//
//        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//        weights[index++] = w;
//      }
//    }
//  }
//
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  StaticMatrix<float32, Support3 * 3, Support3 * 3>* Q =
//    ComputeQ(weights, target, quadric);
//
//  StaticVector<float32, Support3 * 3>* l =
//    ComputeL(weights, target, quadric);
//
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  StaticVector<int32, Support3 * 3> const mapping =
//    stencil.GetIndexMapping();
//
//  ScatterAddMat1(quadraticTerms_, *Q, mapping);
//  ScatterAddVec1(   linearTerms_, *l, mapping);
//  
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  delete Q;
//  delete l;
//}

} //namespace X4
