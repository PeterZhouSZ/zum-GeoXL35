//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "TPSSolver.h"
//----------------------------------------------------------------------
#include "SparseLinearAlgebra.inline.h"
#include "SparseGrid.inline.h"
//----------------------------------------------------------------------
#include "IterativeSolvers.h"
#include "IterativeSolvers.inline.h"
//----------------------------------------------------------------------
#include "DebugRenderer.h"
//----------------------------------------------------------------------
#include "ProgressWindow.h"
//----------------------------------------------------------------------

namespace X4
{

//======================================================================
// Local function
//----------------------------------------------------------------------
template <typename FloatType>
void SolveCG(
   SparseMatrix<FloatType> const& A,
  DynamicVector<FloatType>      & x,
  DynamicVector<FloatType> const& b,
  card32                        & maxIterations,
  FloatType                const  stopRes,
  bool                     const  printDebugOutput,
  CGStatistics                  * statistics,
  ProgressWindow                * progress)
{
  pAssert(A.getRows() == b.getDim());
	
  if (x.getDim() != b.getDim())
  {
	  x = nullDVector<FloatType>(b.getDim());
	}

  card32 const dim = A.getRows();

  DynamicVector<FloatType>     residual(dim);
  DynamicVector<FloatType> lastResidual(dim);

  DynamicVector<FloatType>       direction(dim);
  DynamicVector<FloatType> a_MUL_direction(dim);

  residual = b - A*x;
  
  FloatType residualNorm = norm(residual);
  FloatType resQuad      = residual * residual;
  
  direction = residual;
  
  if (progress)
  {
    progress->pushStep(true, "SolveCG");
  }

  card32 iteration = 0;

  while (residualNorm >= stopRes && iteration < maxIterations)
  {
    A.matVecMult(direction, a_MUL_direction);
    
    FloatType alpha = resQuad / (direction * a_MUL_direction);

    x += direction * alpha;

    lastResidual = residual;

    residual -= a_MUL_direction * alpha;

    FloatType newResQuad = residual * residual;

    FloatType beta = newResQuad / resQuad;

    direction = residual + direction * beta;

    ++iteration;

    residualNorm = sqrt(newResQuad);

    resQuad = newResQuad;

    if (printDebugOutput && ((iteration % 10) == 0       ||
                              iteration       == 0       ||
                              residualNorm < stopRes     ||
                              iteration == maxIterations))
    {
      debugOutput << "cg step " << iteration << " res. " << residualNorm << "\n";
	    debugOutput.flush();
    }

    if (progress)
    {
      progress->progressf(float32(iteration) / maxIterations);
    }
  }

  maxIterations = iteration;

  if (progress)
  {
    progress->popStep();
  }

  if (statistics)
  {
    statistics->lastResidualNorm = residualNorm;
    statistics->numIterations    = iteration;
  }
}

//======================================================================
// Member functions
//----------------------------------------------------------------------
void TPSSolver::Solve( card32               cgNumIterations,
                      float32        const& cgConvergenceThreshold,
                      bool           const  cgDebugOutput,
                      ProgressWindow      * progress)
{
//  static card32 counter = 0;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  DVectorF result = CollectDeformPos();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // store dense system to file for external processing
  //{
  //  DMatrixF dense;
  //
  //  dense.setDimension(quadraticTerms_.getNumCols(),
  //                     quadraticTerms_.getNumRows(), false);

  //  for (card32 i = 0; i < quadraticTerms_.getNumRows(); ++i)
  //  {
  //    SparseVectorF const& row = quadraticTerms_[i];

  //    for (card32 j = 0; j < row.entries.size(); ++j)
  //    {
  //      dense[row.entries.at(j).index][i] = row.entries.at(j).value;
  //    }
  //  }

  //  stringstream filename;
  //
  //  filename << "TPSSolver//system_" << counter << ".txt";

  //  std::ofstream file(filename.str().c_str());
  //  
  //  file << dense        << std::endl;
  //  file << linearTerms_ << std::endl;
  //  file << result       << std::endl;
  //}

  //X4_TIMER_START(SolveCG);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //conjugateGradientsSolve<float32>(quadraticTerms_,
  //                                 result,
  //                                 linearTerms_,
  //                                 cgNumIterations,
  //                                 cgConvergenceThreshold,
  //                                 cgDebugOutput);

  SolveCG<float32>(quadraticTerms_,
                   result,
                   linearTerms_,
                   cgNumIterations,
                   cgConvergenceThreshold,
                   cgDebugOutput,
                   0,
                   smoothed_ ? progress : 0);

  debugOutput << "CG: " << cgNumIterations << " iterations.\n";

  //X4_TIMER_STOP(SolveCG);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  DistributeDeformPos(result);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // store result
  lastResult_ = result;
 
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  debugRenderer->clearDebugData("Offset");
  debugRenderer->beginRenderJob("Offset", false, 2);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Vector3f const lineColor = makeVector3f(1.0f, 0.2f, 0.2f);

  float32 const lineWidth = 1.0f;
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (size_t i = 0; i < gridEntries_.size(); ++i)
  {
    GridEntry const& entry = gridEntries_.at(i);

    if (!entry.occupied) { continue; }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    Vector3f const v0 = Center(entry.gridCell);
    Vector3f const v1 = entry.deformPos;

    debugRenderer->addLine(v0, v1, lineColor, lineColor, lineWidth);
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  debugRenderer->endRenderJob();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  debugRenderer->clearDebugData("Deformed_grid");
  debugRenderer->beginRenderJob("Deformed_grid", true, 2);
    
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (size_t i = 0; i < gridEntries_.size(); ++i)
  {
    GridEntry const& entry = gridEntries_.at(i);

    if (!entry.occupied) { continue; }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    Vector3i const cell = entry.gridCell;

    Vector3f const v000 = Evaluate(Lower(cell + makeVector3i(0, 0, 0)));
    Vector3f const v001 = Evaluate(Lower(cell + makeVector3i(0, 0, 1)));
    Vector3f const v010 = Evaluate(Lower(cell + makeVector3i(0, 1, 0)));
    Vector3f const v011 = Evaluate(Lower(cell + makeVector3i(0, 1, 1)));
    Vector3f const v100 = Evaluate(Lower(cell + makeVector3i(1, 0, 0)));
    Vector3f const v101 = Evaluate(Lower(cell + makeVector3i(1, 0, 1)));
    Vector3f const v110 = Evaluate(Lower(cell + makeVector3i(1, 1, 0)));
    Vector3f const v111 = Evaluate(Lower(cell + makeVector3i(1, 1, 1)));
    
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
  debugRenderer->clearDebugData("Deformed_grid_full");
  debugRenderer->beginRenderJob("Deformed_grid_full", false, 2);
    
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (size_t i = 0; i < gridEntries_.size(); ++i)
  {
    GridEntry const& entry = gridEntries_.at(i);

    if (entry.occupied) { continue; }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  debugRenderer->endRenderJob();
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //++counter;
}

//----------------------------------------------------------------------
DVectorF TPSSolver::CollectDeformPos()
{
  mpcard const numElements = gridEntries_.size();
  
  DVectorF result;

  result.setDim(numElements * 3);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (mpcard i = 0; i < numElements; ++i)
  {
    result[3 * i    ] = gridEntries_[i].deformPos[0];
    result[3 * i + 1] = gridEntries_[i].deformPos[1];
    result[3 * i + 2] = gridEntries_[i].deformPos[2];
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}

//----------------------------------------------------------------------
void TPSSolver::DistributeDeformPos(DVectorF const& result)
{
  mpcard const numElements = gridEntries_.size();

  if (numElements * 3 != result.getDim())
  {
    PException("TPSSolver::DistributeDeformPos() - "
               "Size mismatch.");
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (mpcard i = 0; i < numElements; ++i)
  {
    gridEntries_[i].deformPos = makeVector3f(result[3 * i    ],
                                             result[3 * i + 1],
                                             result[3 * i + 2]);
  }
}

} //namespace X4
