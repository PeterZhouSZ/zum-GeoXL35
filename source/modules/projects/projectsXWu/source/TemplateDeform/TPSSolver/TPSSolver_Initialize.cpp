//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "TPSSolver.h"
//----------------------------------------------------------------------
#include "SparseLinearAlgebra.inline.h"
#include "SparseGrid.inline.h"
//----------------------------------------------------------------------
#include "PartialDerivative.h"
#include "PartialDerivative.inline.h"
//----------------------------------------------------------------------
//#include "TemplateDeform/Misc/TAssert.h"
//----------------------------------------------------------------------
#include "DebugRenderer.h"
//----------------------------------------------------------------------

namespace X4
{
//======================================================================
// Member functions
//----------------------------------------------------------------------
void TPSSolver::Initialize(SceneObject const* object)
{
  UICPC const* pc = dynamic_cast<UICPC const*>(object);

  if (!pc)
  {
    throw PException("TPSSolver::AddRegularizer() - Type mismatch.");
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  PointSet const* ps = pc->getPointSet();

  AAT positionAAT = ps->getAAT("position", 3, VAD::DATA_FORMAT_FLOAT32);

  if (positionAAT == NULL_AAT)
  {
    throw PException("TPSSolver::AddRegularizer() - "
                     "Position field missing.");
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  mpcard numPoints = ps->getNumEntries();

  for (mpcard i = 0; i < numPoints; ++i)
  {
    AddPoint(ps->get3f(i, positionAAT));
  }
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Vector3f const lineColor = makeVector3f(0.2f, 0.2f, 1.0f);

  float32 const lineWidth = 1.0f;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  debugRenderer->clearDebugData("Grid");
  debugRenderer->beginRenderJob("Grid", true, 1);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (size_t i = 0; i < gridEntries_.size(); ++i)
  {
    GridEntry const& entry = gridEntries_.at(i);

    if (!entry.occupied) { continue; }
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    Vector3i const cell = entry.gridCell;

    Vector3f const v000 = Lower(cell + makeVector3i(0, 0, 0));
    Vector3f const v001 = Lower(cell + makeVector3i(0, 0, 1));
    Vector3f const v010 = Lower(cell + makeVector3i(0, 1, 0));
    Vector3f const v011 = Lower(cell + makeVector3i(0, 1, 1));
    Vector3f const v100 = Lower(cell + makeVector3i(1, 0, 0));
    Vector3f const v101 = Lower(cell + makeVector3i(1, 0, 1));
    Vector3f const v110 = Lower(cell + makeVector3i(1, 1, 0));
    Vector3f const v111 = Lower(cell + makeVector3i(1, 1, 1));

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // ground plane
    debugRenderer->addLine(v000, v001, lineColor, lineColor, lineWidth);
    debugRenderer->addLine(v001, v011, lineColor, lineColor, lineWidth);
    debugRenderer->addLine(v011, v010, lineColor, lineColor, lineWidth);
    debugRenderer->addLine(v010, v000, lineColor, lineColor, lineWidth);
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // top plane
    debugRenderer->addLine(v100, v101, lineColor, lineColor, lineWidth);
    debugRenderer->addLine(v101, v111, lineColor, lineColor, lineWidth);
    debugRenderer->addLine(v111, v110, lineColor, lineColor, lineWidth);
    debugRenderer->addLine(v110, v100, lineColor, lineColor, lineWidth);
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // sides
    debugRenderer->addLine(v000, v100, lineColor, lineColor, lineWidth);
    debugRenderer->addLine(v001, v101, lineColor, lineColor, lineWidth);
    debugRenderer->addLine(v011, v111, lineColor, lineColor, lineWidth);
    debugRenderer->addLine(v010, v110, lineColor, lineColor, lineWidth);
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  debugRenderer->endRenderJob();
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  debugRenderer->clearDebugData("Grid_full");
  debugRenderer->beginRenderJob("Grid_full", false, 1);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (size_t i = 0; i < gridEntries_.size(); ++i)
  {
    GridEntry const& entry = gridEntries_.at(i);

    if (entry.occupied) { continue; }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    Vector3i const cell = entry.gridCell;

    Vector3f const v000 = Lower(cell + makeVector3i(0, 0, 0));
    Vector3f const v001 = Lower(cell + makeVector3i(0, 0, 1));
    Vector3f const v010 = Lower(cell + makeVector3i(0, 1, 0));
    Vector3f const v011 = Lower(cell + makeVector3i(0, 1, 1));
    Vector3f const v100 = Lower(cell + makeVector3i(1, 0, 0));
    Vector3f const v101 = Lower(cell + makeVector3i(1, 0, 1));
    Vector3f const v110 = Lower(cell + makeVector3i(1, 1, 0));
    Vector3f const v111 = Lower(cell + makeVector3i(1, 1, 1));

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // ground plane
    debugRenderer->addLine(v000, v001, lineColor, lineColor, lineWidth);
    debugRenderer->addLine(v001, v011, lineColor, lineColor, lineWidth);
    debugRenderer->addLine(v011, v010, lineColor, lineColor, lineWidth);
    debugRenderer->addLine(v010, v000, lineColor, lineColor, lineWidth);
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // top plane
    debugRenderer->addLine(v100, v101, lineColor, lineColor, lineWidth);
    debugRenderer->addLine(v101, v111, lineColor, lineColor, lineWidth);
    debugRenderer->addLine(v111, v110, lineColor, lineColor, lineWidth);
    debugRenderer->addLine(v110, v100, lineColor, lineColor, lineWidth);
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // sides
    debugRenderer->addLine(v000, v100, lineColor, lineColor, lineWidth);
    debugRenderer->addLine(v001, v101, lineColor, lineColor, lineWidth);
    debugRenderer->addLine(v011, v111, lineColor, lineColor, lineWidth);
    debugRenderer->addLine(v010, v110, lineColor, lineColor, lineWidth);
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  debugRenderer->endRenderJob();
}

//----------------------------------------------------------------------
inline void TPSSolver::AddPoint(Vector3f const& point)
{
  Vector3i const cell = Cell(point);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (int32 z = -gridExtension_; z < gridExtension_ + 1; ++z)
  {
    for (int32 y = -gridExtension_; y < gridExtension_ + 1; ++y)
    {
      for (int32 x = -gridExtension_; x < gridExtension_ + 1; ++x)
      {
        Vector3i const offset = makeVector3i(x, y, z);

        AddGridEntry(cell + offset);
      }
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  int32 id = 0;

  grid_.getObject(cell, id);

  gridEntries_.at(id).occupied = true;
}

//----------------------------------------------------------------------
inline int32 TPSSolver::AddGridEntry(Vector3i const& cell)
{
  int32 id = 0;

  bool present = grid_.getObject(cell, id);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (!present)
  {
    id = int32(gridEntries_.size());

    grid_.setObject(cell, id);

    gridEntries_.push_back(GridEntry(cell, Center(cell)));
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return id;
}

} //namespace X4
