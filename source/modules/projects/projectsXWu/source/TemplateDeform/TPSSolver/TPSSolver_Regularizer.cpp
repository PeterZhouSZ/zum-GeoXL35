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

namespace X4
{
//======================================================================
// Local structs
//----------------------------------------------------------------------
struct TPSStencil
{
  //====================================================================
  // Member functions
  //--------------------------------------------------------------------
  StaticVector<int32, 19> GetIndexMapping()
  {
    StaticVector<int32, 19> result;
    
    result[ 0] = ccc;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    result[ 1] = pcc;
    result[ 2] = cpc;
    result[ 3] = ccp;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    result[ 4] = mcc;
    result[ 5] = cmc;
    result[ 6] = ccm;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    result[ 7] = ppc;
    result[ 8] = cpp;
    result[ 9] = pcp;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    result[10] = mmc;
    result[11] = mcm;
    result[12] = mcp;
    result[13] = mpc;
    result[14] = cmm;
    result[15] = cmp;
    result[16] = pmc;
    result[17] = pcm;
    result[18] = cpm;
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    return result;
  }
  
  //====================================================================
  // Member variables
  //--------------------------------------------------------------------
	int32 ccc;
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  int32 pcc;
  int32 cpc;
  int32 ccp;
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  int32 ppc;
  int32 pcp;
  int32 cpp;
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  int32 mcc;
  int32 cmc;
  int32 ccm;
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  int32 mmc;
  int32 mcm;
  int32 cmm;
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  int32 pmc;
  int32 pcm;
  int32 cpm;
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  int32 mpc;
  int32 mcp;
  int32 cmp;
};

//======================================================================
// Local functions
//----------------------------------------------------------------------
StaticMatrix<float32, 19, 19> TPSStencilMatrix()
{
  typedef HessianTools<float32, 19> ADTool;
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  ADTool::DiffTwice ccc = ADTool::createVariable(0.0f,  0);
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  ADTool::DiffTwice pcc = ADTool::createVariable(0.0f,  1);
  ADTool::DiffTwice cpc = ADTool::createVariable(0.0f,  2);
  ADTool::DiffTwice ccp = ADTool::createVariable(0.0f,  3);
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  ADTool::DiffTwice mcc = ADTool::createVariable(0.0f,  4);
  ADTool::DiffTwice cmc = ADTool::createVariable(0.0f,  5);
  ADTool::DiffTwice ccm = ADTool::createVariable(0.0f,  6);
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  ADTool::DiffTwice ppc = ADTool::createVariable(0.0f,  7);
  ADTool::DiffTwice cpp = ADTool::createVariable(0.0f,  8);
  ADTool::DiffTwice pcp = ADTool::createVariable(0.0f,  9);
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  ADTool::DiffTwice mmc = ADTool::createVariable(0.0f, 10);
  ADTool::DiffTwice mcm = ADTool::createVariable(0.0f, 11);
  ADTool::DiffTwice mcp = ADTool::createVariable(0.0f, 12);
  ADTool::DiffTwice mpc = ADTool::createVariable(0.0f, 13);
  ADTool::DiffTwice cmm = ADTool::createVariable(0.0f, 14);
  ADTool::DiffTwice cmp = ADTool::createVariable(0.0f, 15);
  ADTool::DiffTwice pmc = ADTool::createVariable(0.0f, 16);
  ADTool::DiffTwice pcm = ADTool::createVariable(0.0f, 17);
  ADTool::DiffTwice cpm = ADTool::createVariable(0.0f, 18);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  ADTool::DiffTwice energy = sqr((ccc - mcc) - (pcc - ccc)) +
                             sqr((ccc - cmc) - (cpc - ccc)) +
                             sqr((ccc - ccm) - (ccp - ccc)) +
                             sqr((ppc - pmc) - (mpc - mmc)) + 
                             sqr((pcp - pcm) - (mcp - mcm)) + 
                             sqr((cpp - cpm) - (cmp - cmm));

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return ADTool::getHessian(energy);
}

//----------------------------------------------------------------------
inline bool GetTPSStencil(SparseGrid<3, int32, Object> const& grid,
                          Vector3i                     const& cell,
                          TPSStencil                        & stencil)
{
  return grid.getObject(cell + makeVector3i( 0,  0,  0), stencil.ccc) &&
         grid.getObject(cell + makeVector3i( 1,  0,  0), stencil.pcc) &&
         grid.getObject(cell + makeVector3i( 0,  1,  0), stencil.cpc) &&
         grid.getObject(cell + makeVector3i( 0,  0,  1), stencil.ccp) &&
         grid.getObject(cell + makeVector3i( 1,  1,  0), stencil.ppc) &&
         grid.getObject(cell + makeVector3i( 1,  0,  1), stencil.pcp) &&
         grid.getObject(cell + makeVector3i( 0,  1,  1), stencil.cpp) &&
         grid.getObject(cell + makeVector3i(-1,  0,  0), stencil.mcc) &&
         grid.getObject(cell + makeVector3i( 0, -1,  0), stencil.cmc) &&
         grid.getObject(cell + makeVector3i( 0,  0, -1), stencil.ccm) &&
         grid.getObject(cell + makeVector3i(-1, -1,  0), stencil.mmc) &&
         grid.getObject(cell + makeVector3i(-1,  0, -1), stencil.mcm) &&
         grid.getObject(cell + makeVector3i( 0, -1, -1), stencil.cmm) &&
         grid.getObject(cell + makeVector3i( 1, -1,  0), stencil.pmc) &&
         grid.getObject(cell + makeVector3i( 1,  0, -1), stencil.pcm) &&
         grid.getObject(cell + makeVector3i( 0,  1, -1), stencil.cpm) &&
         grid.getObject(cell + makeVector3i(-1,  1,  0), stencil.mpc) &&
         grid.getObject(cell + makeVector3i(-1,  0,  1), stencil.mcp) &&
         grid.getObject(cell + makeVector3i( 0, -1,  1), stencil.cmp);
}

//----------------------------------------------------------------------
template <typename FloatType, card32 SIZE>
inline static void ScatterAddMat3(
	SparseMatrix<FloatType>                  & sparse,
	StaticMatrix<FloatType, SIZE, SIZE> const& dense, 
	StaticVector<int32, SIZE>           const& mapping) 
{
  TASSERT(SIZE);
	
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (card32 d = 0; d < 3; ++d)
  {
    for (card32 row = 0; row < SIZE; ++row)
    {
      SparseVector<FloatType> rowVector;

      for (card32 col = 0; col < SIZE; ++col)
      {
        if (dense[col][row])
        {
          rowVector.setEntry(mapping[col] * 3 + d, dense[col][row]);
        }
      }

      sparse[mapping[row] * 3 + d] += rowVector;
    }
  }
}

//======================================================================
// Member functions
//----------------------------------------------------------------------
void TPSSolver::AddRegularizer()
{
  Clear();
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (gridEntries_.empty())
  {
    throw PException("TPSSolver::AddRegularizer() - Grid empty.");
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  linearTerms_.setDim(gridEntries_.size() * 3);
  linearTerms_.setZero();

  size_t numEntries = gridEntries_.size();

  quadraticTerms_.setRows(numEntries * 3);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  StaticMatrix<float32, 19, 19> const stencilMatrix = TPSStencilMatrix();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (size_t i = 0; i < numEntries; ++i)
  {
    TPSStencil stencil;

    if (GetTPSStencil(grid_, gridEntries_.at(i).gridCell, stencil))
    {
      if (stencil.ccc != i)
      {
        throw PException("TPSSolver::AddRegularizer() - Failed.");
      }

      ScatterAddMat3(quadraticTerms_,
                     stencilMatrix * (regularizerWeight_ * gridSpacing_),
                     stencil.GetIndexMapping());
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (size_t i = 0; i < numEntries; ++i)
  {
    Vector3f const center = Center(gridEntries_.at(i).gridCell);

    for (card32 d = 0; d < 3; ++d)
    {
      quadraticTerms_[3 * i + d].addEntry(3 * i + d, identityWeight_);
         linearTerms_[3 * i + d] = center[d] * identityWeight_;
    }
  }
}  

} //namespace X4
