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
struct EStencil
{
  //====================================================================
  // Member functions
  //--------------------------------------------------------------------
  Vector3f Interpolate(Vector3f const& frac)
  {
    Vector3f const diff = makeVector3f(1.0f, 1.0f, 1.0f) - frac;

    return ((p000 * diff[0] + p100 * frac[0]) * diff[1] +
            (p010 * diff[0] + p110 * frac[0]) * frac[1]) * diff[2] +
           ((p001 * diff[0] + p101 * frac[0]) * diff[1] +
            (p011 * diff[0] + p111 * frac[0]) * frac[1]) * frac[2];
  }

  //====================================================================
  // Member variables
  //--------------------------------------------------------------------
  Vector3f p000;
  Vector3f p001;
  Vector3f p010;
  Vector3f p011;
  Vector3f p100;
  Vector3f p101;
  Vector3f p110;
  Vector3f p111;
};

//======================================================================
// Member functions
//----------------------------------------------------------------------
Vector3f TPSSolver::Evaluate(Vector3f const& position)
{
  Vector3i const cell   = Cell  (position      );
  Vector3i const offset = Offset(position, cell);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Vector3f const center = Center(cell + offset);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Vector3f fraction = (position - center) / gridSpacing_;

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
  EStencil stencil;

  if (!GetEStencil(cell + offset, stencil))
  {
    throw ERangeCheck("TPSSolver::Evaluate() - "
                      "Point out of grid range.");
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return stencil.Interpolate(fraction);
}

//----------------------------------------------------------------------
Matrix3f TPSSolver::EvaluateSpatialGradient(Vector3f const& position)
{
  Vector3i const cell   = Cell  (position      );
  Vector3i const offset = Offset(position, cell);
  
  Vector3i const lower  = cell + offset;
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Vector3f p000;
  Vector3f p001;
  Vector3f p010;
  Vector3f p100;

  if (!(GetDeformPos(lower + makeVector3i(0, 0, 0), p000) &&
        GetDeformPos(lower + makeVector3i(0, 0, 1), p001) &&
        GetDeformPos(lower + makeVector3i(0, 1, 0), p010) &&
        GetDeformPos(lower + makeVector3i(1, 0, 0), p100)))
  {
    throw ERangeCheck("TPSSolver::EvaluateSpatialGradient() - "
                      "Point out of grid range.");
  }
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Matrix3f result;

  result[0] = (p100 - p000) / gridSpacing_;
  result[1] = (p010 - p000) / gridSpacing_;
  result[2] = (p001 - p000) / gridSpacing_;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}

//----------------------------------------------------------------------
bool TPSSolver::GetEStencil(Vector3i const& cell, EStencil& stencil)
{
  return GetDeformPos(cell + makeVector3i(0, 0, 0), stencil.p000) &&
         GetDeformPos(cell + makeVector3i(0, 0, 1), stencil.p001) &&
         GetDeformPos(cell + makeVector3i(0, 1, 0), stencil.p010) &&
         GetDeformPos(cell + makeVector3i(0, 1, 1), stencil.p011) &&
         GetDeformPos(cell + makeVector3i(1, 0, 0), stencil.p100) &&
         GetDeformPos(cell + makeVector3i(1, 0, 1), stencil.p101) &&
         GetDeformPos(cell + makeVector3i(1, 1, 0), stencil.p110) &&
         GetDeformPos(cell + makeVector3i(1, 1, 1), stencil.p111);
}

} //namespace X4
