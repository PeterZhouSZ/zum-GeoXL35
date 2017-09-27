//----------------------------------------------------------------------
#ifndef SymmSpaceDirectSolver_h_
#define SymmSpaceDirectSolver_h_
//----------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//----------------------------------------------------------------------
#include "Util\numerical\LibraryAdaptor.h"
//----------------------------------------------------------------------

class SymmSpaceDirectSolver
{
public:
    typedef boost::shared_ptr< SymmSpaceDirectSolver > Ptr;
    typedef boost::shared_ptr< const SymmSpaceDirectSolver > ConstPtr;

public:
    SymmSpaceDirectSolver();
    virtual ~SymmSpaceDirectSolver() {}

    //const SparseMatrixD& getLHS(void) const { return LHS; }
    //const SparseMatrixD& getHC_Ut(void) const { return HC_Ut; }
    //const DVectorD& getRHS(void) const { return RHS; }

    void ReduceFactorize(
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

    void CombineReduceHC(
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
    DVectorD HC_Ut_HV;

    //Clapack::ClapackFactorizationSymmetric::Ptr clapackFactorization_;
};

#endif
