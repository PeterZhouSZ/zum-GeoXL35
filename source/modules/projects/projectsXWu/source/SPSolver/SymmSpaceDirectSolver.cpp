//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "SPSolver/SymmSpaceDirectSolver.h"
//----------------------------------------------------------------------
//----------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

SymmSpaceDirectSolver::SymmSpaceDirectSolver()
    //: clapackFactorization_(new Clapack::ClapackFactorizationSymmetric())
{
}

void SymmSpaceDirectSolver::BuildLHS(
    const SparseMatrixD& QT,
    const SparseMatrixD& SC,
    SparseMatrixD* pLHS,
    const bool& debug_output
    )
{
    SparseMatrixD& LHS = *pLHS;

    // LHS = QT' * QT + SC' * SC;
    LHS = LibraryAdaptor::SparseMatrixSquare(QT);
    if (0 < SC.getNumRows()) {
        LHS += LibraryAdaptor::SparseMatrixOuterProduct(SC, QT.getNumRows());
    }
    LHS.removeNearZeros(std::numeric_limits<double>::epsilon());
}

void SymmSpaceDirectSolver::CombineReduceHC(
    const SparseMatrixD& HC,
    const DVectorD& HV,
    SparseMatrixD* QTQT,
    const bool& debug_output
    )
{
    if (0 == HC.getNumRows()) return;

    // HC = HC_Ut' * HC_R;
    SparseMatrixD HC_R;
    SparseMatrixD HC_Ut;
    std::vector<double> S;
    const double eps = .1f;
    //Clapack::ClapackAdaptor::ReduceSingular(HC, QTQT->getNumRows(), &HC_R, &HC_Ut, &S, eps, debug_output);

    // QTQT = [QTQT; HC_R];
    LibraryAdaptor::SymmetricSparseMatrixBinder(QTQT, HC_R); // use reduced anyway, for numerical stability

    // HC_Ut_HV = HC_Ut * HV;
    HC_Ut_HV = HC_Ut * HV;

    if (debug_output) debugOutput << "QTQT = " << QTQT->toMatString() << ";\n";
}

void SymmSpaceDirectSolver::UpdateRHS(
    const SparseMatrixD& QT, const DVectorD& LT,
    const SparseMatrixD& SC, const DVectorD& SV,
    DVectorD* pRHS,
    const bool& debug_output
    )
{
    DVectorD& RHS = *pRHS;

    // RHS = QT' * LT + SC' * SV;
    const size_t& varDim = QT.getNumRows();
    RHS = QT * LT;

    if (0 == SV.getDim()) return;
    RHS += LibraryAdaptor::SparseMatrixTranspose(SC, varDim) * SV;
}

void SymmSpaceDirectSolver::CombineReduceHV(
    DVectorD* QTLT,
    const bool& debug_output
    )
{
    // QTLT = [QTLT; HC_Ut_HV];
    if (0 == HC_Ut_HV.getDim()) return;
    QTLT->appendInPlace(HC_Ut_HV); // use reduced anyway, for numerical stability
}

void SymmSpaceDirectSolver::ReduceFactorize(
    const SparseMatrixD& HC,
    const DVectorD& HV,
    SparseMatrixD* QTQT,
    const bool& debug_output
    )
{
    CombineReduceHC(HC, HV, QTQT, debug_output);

    //clapackFactorization_->Factorize(
    //    *QTQT,
    //    debug_output
    //    );
}

void SymmSpaceDirectSolver::UpdateSolve(
    DVectorD* RHS,
    DVectorD* pRES,
    const bool& debug_output
    )
{
    const size_t& varDim = RHS->getDim();

    CombineReduceHV(RHS, debug_output);

    //clapackFactorization_->Solve(
    //    *RHS,
    //    pRES,
    //    debug_output
    //    );

    *pRES = pRES->subset(0, varDim);
}

void SymmSpaceDirectSolver::Solve(
    const SparseMatrixD& QT, const DVectorD& LT,
    const SparseMatrixD& SC, const DVectorD& SV,
    const SparseMatrixD& HC, const DVectorD& HV,
    DVectorD* pRES,
    const bool& bRefactor,
    const bool& debug_output
    )
{
    if (debug_output)
    {
        debugOutput << "\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n";

        if (bRefactor) {
            const unsigned num_c = QT.getNumRows();
            debugOutput << "QT = " << QT.toMatString(num_c) << ";\n";
            debugOutput << "LT = " << LT.toMatString() << ";\n";
            debugOutput << "SC = " << SC.toMatString(num_c) << ";\n";
            debugOutput << "SV = " << SV.toMatString() << ";\n";
            debugOutput << "HC = " << HC.toMatString(num_c) << ";\n";
            debugOutput << "HV = " << HV.toMatString() << ";\n";
            debugOutput << "\n";
        }
    }

    if (bRefactor) {
        Timer timer_fac; timer_fac.getDeltaValue();

        SparseMatrixD LHS;
        BuildLHS(
            QT, SC,
            &LHS,
            debug_output
            );

        ReduceFactorize(
            HC, HV, &LHS,
            debug_output
            );

        if (debug_output) debugOutput << "LHS = " << LHS.toMatString() << ";\n";

        debugOutput << "\nprefactorization time: " << convertTimeToString( timer_fac.getDeltaValue() ) << "\n";

        //if (debug_output)
        {
            debugOutput << "\n";
            debugOutput << "number of variables: " << QT.getNumRows() << ";\n";
            //debugOutput << "number of soft constraints: " << SC.getNumRows() << ";\n";
            debugOutput << "number of hard constraints: " << HC.getNumRows() << ";\n";
            debugOutput << "number of rows of reduced hard constraints: " << HC_Ut_HV.getDim() << ";\n";
            debugOutput << "\n";
        }
    }

    {
        DVectorD RHS;
        UpdateRHS(
            QT, LT,
            SC, SV,
            &RHS,
            debug_output
            );

        UpdateSolve(
            &RHS,
            pRES,
            debug_output
            );

        if (debug_output) debugOutput << "RHS = " << RHS.toMatString() << ";\n";
    }

    if (debug_output){
        debugOutput << "result = " << pRES->toMatString() << ";\n";

        debugOutput << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n";
    }
}
