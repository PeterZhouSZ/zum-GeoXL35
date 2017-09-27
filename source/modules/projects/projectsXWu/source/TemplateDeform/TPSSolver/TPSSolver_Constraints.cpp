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
// Local structs
//----------------------------------------------------------------------
struct ConstraintTerms
{
  //====================================================================
  // Member variables
  //--------------------------------------------------------------------
  StaticMatrix<float32, 24, 24> quadratic;
  StaticVector<float32, 24>     linear;
};

//----------------------------------------------------------------------
struct CStencil
{
  //====================================================================
  // Member functions
  //--------------------------------------------------------------------
  StaticVector<int32, 24> GetIndexMapping()
  {
    StaticVector<int32, 24> mapping;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    mapping[ 0] = i000 * 3;
    mapping[ 1] = i000 * 3 + 1;
    mapping[ 2] = i000 * 3 + 2;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    mapping[ 3] = i001 * 3;
    mapping[ 4] = i001 * 3 + 1;
    mapping[ 5] = i001 * 3 + 2;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    mapping[ 6] = i010 * 3;
    mapping[ 7] = i010 * 3 + 1;
    mapping[ 8] = i010 * 3 + 2;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    mapping[ 9] = i011 * 3;
    mapping[10] = i011 * 3 + 1;
    mapping[11] = i011 * 3 + 2;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    mapping[12] = i100 * 3;
    mapping[13] = i100 * 3 + 1;
    mapping[14] = i100 * 3 + 2;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    mapping[15] = i101 * 3;
    mapping[16] = i101 * 3 + 1;
    mapping[17] = i101 * 3 + 2;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    mapping[18] = i110 * 3;
    mapping[19] = i110 * 3 + 1;
    mapping[20] = i110 * 3 + 2;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    mapping[21] = i111 * 3;
    mapping[22] = i111 * 3 + 1;
    mapping[23] = i111 * 3 + 2;
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    return mapping;
  }
  
  //====================================================================
  // Member variables
  //--------------------------------------------------------------------
  int32 i000;
  int32 i001;
  int32 i010;
  int32 i011;
  int32 i100;
  int32 i101;
  int32 i110;
  int32 i111;
};

//======================================================================
// Local functions
//----------------------------------------------------------------------
inline bool GetCStencil(SparseGrid<3, int32, Object> const& grid,
                        Vector3i                     const& cell,
                        CStencil                          & stencil)
{
  return grid.getObject(cell + makeVector3i(0, 0, 0), stencil.i000) &&
         grid.getObject(cell + makeVector3i(0, 0, 1), stencil.i001) &&
         grid.getObject(cell + makeVector3i(0, 1, 0), stencil.i010) &&
         grid.getObject(cell + makeVector3i(0, 1, 1), stencil.i011) &&
         grid.getObject(cell + makeVector3i(1, 0, 0), stencil.i100) &&
         grid.getObject(cell + makeVector3i(1, 0, 1), stencil.i101) &&
         grid.getObject(cell + makeVector3i(1, 1, 0), stencil.i110) &&
         grid.getObject(cell + makeVector3i(1, 1, 1), stencil.i111);
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
StaticVector<float32, 8> Weighting(Vector3f const& fraction)
{
  StaticVector<float32, 8> result;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (int32 i = 0; i < 8; ++i)
  {
    int32 index[3] = {0};

    index[2] =  i      % 2;
    index[1] = (i / 2) % 2;
    index[0] = (i / 4) % 2;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    float32& current = result[i];

    current = 1.0f;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    current *= ((index[0] == 1) ? (fraction[0]) : (1.0f - fraction[0]));
    current *= ((index[1] == 1) ? (fraction[1]) : (1.0f - fraction[1]));
    current *= ((index[2] == 1) ? (fraction[2]) : (1.0f - fraction[2]));
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}

//----------------------------------------------------------------------
ConstraintTerms ComputeConstraintTerms(Vector3f const& fraction,
                                       Vector3f const& target,
                                       Matrix3f const& quadric)
{
  StaticVector<float32, 8> const weights = Weighting(fraction);

  ConstraintTerms result;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (int32 i = 0; i < 8; ++i)
  {
    Vector3f const rhs3 = (quadric * target) * (2.0f * weights[i]);
  
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    for (int32 d = 0; d < 3; ++d)
    {
      result.linear[3 * i + d] = rhs3[d];
    }
  } 

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (int32 i = 0; i < 8; ++i)
  {
    for (int32 j = 0; j < 8; ++j)
    {
      Matrix3f const mat3 = quadric * (2.0f * weights[i] * weights[j]);

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      for (int32 d0 = 0; d0 < 3; ++d0)
      {
        for (int32 d1 = 0; d1 < 3; ++d1)
        {
          result.quadratic[3 * i + d0][3 * j + d1] = mat3[d0][d1];
        }
      }
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}

//----------------------------------------------------------------------
void TPSSolver::AddPositionConstraint(Vector3f const& anchor,
                                      Vector3f const& target,
                                      Matrix3f const& quadric)
{
  Vector3i const cell   = Cell  (anchor      );
  Vector3i const offset = Offset(anchor, cell);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Vector3f const center = Center(cell + offset);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Vector3f fraction = (anchor - center) / gridSpacing_;

  if (fraction[0] < 0.0f || fraction[0] > 1.0f ||
      fraction[1] < 0.0f || fraction[1] > 1.0f ||
      fraction[2] < 0.0f || fraction[2] > 1.0f)
  {
    debugOutput << "TPSSolver::AddPositionConstraint() - "
                << "Fraction is " << fraction << "\n";

    if (fraction[0] < 0) { fraction[0] = 0; }
    if (fraction[1] < 0) { fraction[1] = 0; }
    if (fraction[2] < 0) { fraction[2] = 0; }
    if (fraction[0] > 1) { fraction[0] = 1; }
    if (fraction[1] > 1) { fraction[1] = 1; }
    if (fraction[2] > 1) { fraction[2] = 1; }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  CStencil cStencil;

  if (!GetCStencil(grid_, cell + offset, cStencil))
  {
    throw ERangeCheck("TPSSolver::AddPositionConstraint() - "
                      "Anchor out of grid range.");
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  ConstraintTerms const constraintTerms =
    ComputeConstraintTerms(fraction, target, quadric);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  StaticVector<int32, 24> const mapping = cStencil.GetIndexMapping();

  ScatterAddMat1(quadraticTerms_, constraintTerms.quadratic, mapping);
  ScatterAddVec1(   linearTerms_, constraintTerms.linear,    mapping);
}


} //namespace X4
