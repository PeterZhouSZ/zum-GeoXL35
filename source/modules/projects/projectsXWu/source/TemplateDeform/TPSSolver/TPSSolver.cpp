//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "TPSSolver.h"
//----------------------------------------------------------------------
#include "SparseLinearAlgebra.inline.h"
//----------------------------------------------------------------------

namespace X4
{
//======================================================================
// Constructor
//----------------------------------------------------------------------
TPSSolver::TPSSolver(bool    const smoothed,
                     int32   const gridExtension,
                     float32 const gridSpacing,
                     float32 const    identityWeight,
                     float32 const regularizerWeight)
  : smoothed_         (smoothed         )
  , gridExtension_    (gridExtension    )
  , gridSpacing_      (gridSpacing      )
  ,    identityWeight_(   identityWeight)
  , regularizerWeight_(regularizerWeight)
//, reweight_         (false            )
{
  if (smoothed_ && gridExtension < 4)
  {
    throw PException("TPSSolver::TPSSolver() - "
                     "Grid extension to low for smoothed evaluation.");
  }
}

//======================================================================
// Member functions
//----------------------------------------------------------------------
//bool TPSSolver::Reweight() const
//{
//  return reweight_;
//}

//----------------------------------------------------------------------
//bool& TPSSolver::Reweight()
//{
//  return reweight_;
//}

//----------------------------------------------------------------------
void TPSSolver::Clear()
{
  quadraticTerms_ = SparseMatrixF();
     linearTerms_ =      DVectorF();
}

//----------------------------------------------------------------------
inline Vector3f TPSSolver::Center(Vector3i const& cell) const
{
  return makeVector3f((cell[0] + 0.5f) * gridSpacing_,
                      (cell[1] + 0.5f) * gridSpacing_,
                      (cell[2] + 0.5f) * gridSpacing_);
}

//----------------------------------------------------------------------
inline Vector3f TPSSolver::Lower(Vector3i const& cell) const
{
  return makeVector3f(cell[0] * gridSpacing_,
                      cell[1] * gridSpacing_,
                      cell[2] * gridSpacing_);
}

//----------------------------------------------------------------------
inline Vector3i TPSSolver::Cell(Vector3f const& point) const
{
  return makeVector3i(floor(point[0] / gridSpacing_),
                      floor(point[1] / gridSpacing_),
                      floor(point[2] / gridSpacing_));
}

//------------------------------------------------------------------------------
inline Vector3i TPSSolver::Offset(
  Vector3f const& point, Vector3i const& cell) const
{
  Vector3f const center = Center(cell);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Vector3i offset = NULL_VECTOR3I;

  if (point[0] < center[0]) { offset[0] = -1; }
  if (point[1] < center[1]) { offset[1] = -1; }
  if (point[2] < center[2]) { offset[2] = -1; }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return offset;
}

//------------------------------------------------------------------------------
inline bool TPSSolver::GetDeformPos(
  Vector3i const& cell, Vector3f& deformPos)
{
  int32 index = 0;

  bool result = grid_.getObject(cell, index);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (result)
  {
    deformPos = gridEntries_.at(index).deformPos;
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}

//----------------------------------------------------------------------
inline void TPSSolver::Update(TPSSolver& previous)
{
  for (std::vector<GridEntry>::iterator it = gridEntries_.begin();
    it != gridEntries_.end(); ++it)
  {
    try
    {
      it->deformPos = previous.Evaluate(it->deformPos);
    }
    catch (ERangeCheck const& exception)
    {
      //if (smoothed_)
      //{
      //  it->deformPos /= 12.5325f;
      //}
    }
  }
}

//----------------------------------------------------------------------
inline void TPSSolver::Scale()
{
  //if (!smoothed_) { return; }

  //for (mpcard i = 0; i < gridEntries_.size(); ++i)
  //{
  //  gridEntries_.at(i).deformPos /= 12.5325f; // Determined empirically.
  //}
}

} //namespace X4
