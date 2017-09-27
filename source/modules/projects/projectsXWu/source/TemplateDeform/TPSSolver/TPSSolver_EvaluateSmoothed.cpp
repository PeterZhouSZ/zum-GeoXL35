//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "TPSSolver.h"
//----------------------------------------------------------------------

namespace X4
{
//======================================================================
// Local structs
//----------------------------------------------------------------------
struct ESStencil
{
  //====================================================================
  // Member variables
  //--------------------------------------------------------------------
  Vector3f p[7][7][7];
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

//======================================================================
// Member functions
//----------------------------------------------------------------------
//Vector3f TPSSolver::EvaluateSmoothed(Vector3f const& position)
//{
//  Vector3i const cell = Cell(position);
//  
//  Vector3i const base = cell + Offset(position, cell);
//
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  ESStencil stencil;
//
//  if (!GetESStencil(base, stencil))
//  {
//    throw ERangeCheck("TPSSolver::EvaluateSmoothed() - "
//                      "Point out of grid range.");
//  }
//
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  //float32 weightSum = 0.0f;
//  
//  Vector3f result = NULL_VECTOR3F;
//
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  for (int32 z = 0; z < 7; ++z)
//  {
//    for (int32 y = 0; y < 7; ++y)
//    {
//      for (int32 x = 0; x < 7; ++x)
//      {
//        Vector3i const offset = makeVector3i(x - 3, y - 3, z - 3);
//
//        Vector3f const center = Center(base + offset);
//
//        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//        float32 const w =
//          WendlandKernel(norm(center - position) / 3.0f / gridSpacing_);
//
//        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//        result += stencil.p[x][y][z] * w;
//
//        //weightSum += w;
//      }
//    }
//  }
//
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  //if (weightSum > 1e-7)
//  //{
//  //  //result /= weightSum;
//  //}
//
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  return result;
//}
//
////----------------------------------------------------------------------
//Matrix3f TPSSolver::EvaluateSmoothedSpatialGradient(
//  Vector3f const& position)
//{
//  Vector3i const cell = Cell(position);
//  
//  Vector3i const base = cell + Offset(position, cell);
//
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  float32 weightSum = 0.0f;
//  
//  Matrix3f result;
//
//  result.setZero();
//  
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  for (int32 z = 0; z < 7; ++z)
//  {
//    for (int32 y = 0; y < 7; ++y)
//    {
//      for (int32 x = 0; x < 7; ++x)
//      {
//        Vector3i const offset = makeVector3i(x - 3, y - 3, z - 3);
//
//        Vector3f const center = Center(base + offset);
//
//        Matrix3f const gradient = EvaluateSpatialGradient(center);
//
//        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//        float32 const w =
//          WendlandKernel(norm(center - position) / 3.0f / gridSpacing_);
//
//        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//        result += gradient * w;
//
//        //weightSum += w;
//      }
//    }
//  }
//
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  //if (weightSum > 1e-7)
//  //{
//  //  //result /= weightSum;
//  //}
//
//  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  return result;
//}

//----------------------------------------------------------------------
//bool TPSSolver::GetESStencil(Vector3i const& cell, ESStencil& stencil)
//{
//  for (int32 z = 0; z < 7; ++z)
//  {
//    for (int32 y = 0; y < 7; ++y)
//    {
//      for (int32 x = 0; x < 7; ++x)
//      {
//        Vector3i const offset = makeVector3i(x - 3, y - 3, z - 3);
//
//        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//        if (!GetDeformPos(cell + offset, stencil.p[x][y][z]))
//        {
//          return false;
//        }
//      }
//    }
//  }
//
//  return true;
//}

} //namespace X4
