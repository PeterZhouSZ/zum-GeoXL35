//----------------------------------------------------------------------
#ifndef SymmSpaceDirectNullSpaceSolver_h_
#define SymmSpaceDirectNullSpaceSolver_h_
//----------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//----------------------------------------------------------------------
#include "Util\numerical\LibraryAdaptor.h"
//----------------------------------------------------------------------

class SymmSpaceDirectNullSpaceSolver
{
public:
    typedef boost::shared_ptr< SymmSpaceDirectNullSpaceSolver > Ptr;
    typedef boost::shared_ptr< const SymmSpaceDirectNullSpaceSolver > ConstPtr;

public:
    SymmSpaceDirectNullSpaceSolver();
    virtual ~SymmSpaceDirectNullSpaceSolver() {}

    //const SparseMatrixD& getLHS(void) const { return LHS; }
    //const SparseMatrixD& getHC_Ut(void) const { return HC_Ut; }
    //const DVectorD& getRHS(void) const { return RHS; }
    const DVectorD& getSample(void) const { return samCoord; }

    int ReduceFactorize(
        const SparseMatrixD& HC,
        const DVectorD& HV,
        SparseMatrixD* QTQT,
        const bool& debug_output
        );

    void UpdateSolve(
        DVectorD* RHS,
        DVectorD* pRES,
        const bool& debug_output
        );

    void Solve(
        const SparseMatrixD& QT, const DVectorD& LT,
        const SparseMatrixD& SC, const DVectorD& SV,
        const SparseMatrixD& HC, const DVectorD& HV,
        DVectorD* pRES,
        const bool& bRefactor,
        const bool& debug_output
        );

protected:
    void BuildLHS(
        const SparseMatrixD& QT,
        const SparseMatrixD& SC,
        SparseMatrixD* pLHS,
        const bool& debug_output
        );

    int CombineReduceHC(
        const SparseMatrixD& HC,
        const DVectorD& HV,
        SparseMatrixD* QTQT,
        const bool& debug_output
        );

    void UpdateRHS(
        const SparseMatrixD& QT, const DVectorD& LT,
        const SparseMatrixD& SC, const DVectorD& SV,
        DVectorD* pRHS,
        const bool& debug_output
        );

    void CombineReduceHV(
        DVectorD* QTLT,
        const bool& debug_output
        );

protected:
    SparseMatrixD HC_N;
    SparseMatrixD HC_Nt;
    DVectorD HC_R_y;
    DVectorD HC_N_QTSC_y;
    DVectorD samCoord;

    //Clapack::ClapackFactorizationSymmetric::Ptr clapackFactorization_;
};

#endif
