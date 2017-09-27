//----------------------------------------------------------------------
#ifndef SymmSpaceNullSpace_h_
#define SymmSpaceNullSpace_h_
//----------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//----------------------------------------------------------------------
#include "SPSolver/SymmSpaceDirectSolver.h"
#include "SPSolver/SymmSpaceDirectNullSpaceSolver.h"
#include "Util\numerical\ClapackAdaptor.h"
#include "SToolBox/SymmSampleGraph.h"
//----------------------------------------------------------------------

class DefieldSymm;
class SymmSpaceNullSpace
{
public:
    typedef boost::shared_ptr< SymmSpaceNullSpace > Ptr;
    typedef boost::shared_ptr< const SymmSpaceNullSpace > ConstPtr;

    typedef std::deque< std::deque< unsigned > > OrbitQueue;

public:
    SymmSpaceNullSpace();

    //SparseMatrixD& getNS(void) { return NS; }
    const SparseMatrixD& getNS(void) const { return NS; }
    const SparseMatrixD& getNS_t(void) const { return NS_t; }
    const DVectorD& getCoord(void) const { return nsCoord; }

    SparseVectorD& operator[](const unsigned& index) { return NS[index]; }
    const SparseVectorD& operator[](const unsigned& index) const { return NS[index]; }

public:
    int BuildRotatedBasis(
        const SymmSampleGraph& symmGraph,
        const std::deque<unsigned>& fillInIndice,
        const unsigned& num_points
        );

public:
    void Solve(
        const SparseMatrixD& QT, const DVectorD& LT,
        const SparseMatrixD& SC, const DVectorD& SV,
        const SparseMatrixD& HC, const DVectorD& HV,
        DVectorD* pRES,
        const bool& bRefactor,
        const bool& debug_output
        );

private:
    void BuildLHSProjection(
        const SparseMatrixD& QT,
        const SparseMatrixD& SC,
        SparseMatrixD* pLHS,
        const SparseMatrixD& HC, SparseMatrixD* pHC_NS,
        const DVectorD& HV, DVectorD* pHV_NS,
        const bool& debug_output
        );

    void UpdateRHSProjection(
        const SparseMatrixD& QT, const DVectorD& LT,
        const SparseMatrixD& SC, const DVectorD& SV,
        DVectorD* pRHS,
        const bool& debug_output
        );

private:
    SparseMatrixD NS;
    SparseMatrixD NS_t;
    DVectorD nsCoord;
    size_t num_ns_basis;
    size_t num_var;

    bool isFullRankConst;
    SymmSpaceDirectSolver::Ptr rangeSpace_;
    SymmSpaceDirectNullSpaceSolver::Ptr directNullSpace_;
};

#endif
