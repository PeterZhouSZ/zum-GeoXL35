//==============================================================================
// #include guard
//------------------------------------------------------------------------------
#ifndef Scattering_h_
#define Scattering_h_

//==============================================================================
// Includes
//------------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#include "DynamicLinearAlgebra.h"
#include  "SparseLinearAlgebra.h"
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#include "DynamicLinearAlgebra.inline.h"
#include  "SparseLinearAlgebra.inline.h"

//==============================================================================
// Top-level namespace
//------------------------------------------------------------------------------
namespace X4
{

//==============================================================================
// Global functions
//------------------------------------------------------------------------------
inline void ScatterAddMat1( SparseMatrix<float32>      & sparse,
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
inline void ScatterAddVec1(DynamicVector<float32>      & dest,
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
    //#pragma omp critical
    dest[mapping[i]] += dense[i];
  }
}

//==============================================================================
// Top-level namespace
//------------------------------------------------------------------------------
} //namespace X4

//==============================================================================
// #include guard
//------------------------------------------------------------------------------
#endif // Scattering_h_
