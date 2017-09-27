//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "SPSolver/SymmSpaceDirectNullSpaceSolver.h"
//----------------------------------------------------------------------
//----------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

SymmSpaceDirectNullSpaceSolver::SymmSpaceDirectNullSpaceSolver()
    //: clapackFactorization_(new Clapack::ClapackFactorizationSymmetric())
{
}

void SymmSpaceDirectNullSpaceSolver::BuildLHS(
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

int SymmSpaceDirectNullSpaceSolver::CombineReduceHC(
    const SparseMatrixD& HC,
    const DVectorD& HV,
    SparseMatrixD* pQTQT,
    const bool& debug_output
    )
{
    if (0 == HC.getNumRows()) return 0;
    SparseMatrixD& QTQT = *pQTQT;

    const unsigned varDim = QTQT.getNumRows();
    const unsigned num_hc = HC.getNumRows();

    // HC = HC_Ut' * HC_R;
    SparseMatrixD HC_R;
    SparseMatrixD HC_Ut;
    std::vector<double> S;
    const double eps = .1f;
    //Clapack::ClapackAdaptor::SpaceDecomposition(HC, varDim, &HC_R, &HC_Ut, &HC_N, &S, eps, false);
    HC_Nt = LibraryAdaptor::SparseMatrixTranspose(HC_N, varDim);
    if (debug_output) debugOutput << "HC_R = " << HC_R.toMatString(varDim) << ";\n";
    if (debug_output) debugOutput << "HC_Ut = " << HC_Ut.toMatString(HC.getNumRows()) << ";\n";
    if (debug_output) debugOutput << "HC_N = " << HC_N.toMatString(varDim) << ";\n";

    if (0 == HC_N.getNumRows()) return -1; // full rank constraint set, no freedom to move

    // HC_R_y = HC_R' * (HC_R * HC_R' \ (HC_Ut * HV));
    HC_R_y = HC_Ut * HV;
    SparseMatrixD HC_Rt = LibraryAdaptor::SparseMatrixTranspose(HC_R, varDim);
    HC_R = LibraryAdaptor::SparseMatrixSquare(HC_R);
    //Clapack::ClapackAdaptor::dgesv(HC_R, varDim, &HC_R_y);
    HC_R_y = HC_Rt * HC_R_y;
    if (debug_output) debugOutput << "HC_R_y = " << HC_R_y.toMatString() << ";\n";

    // HC_N_QTSC_y = HC_N * (QTQT * HC_R_y);
    HC_N_QTSC_y = HC_N * (QTQT * HC_R_y);
    if (debug_output) debugOutput << "HC_N_QTSC_y = " << HC_N_QTSC_y.toMatString() << ";\n";

    // QTQT = HC_N * QTSC * HC_N';
    QTQT = LibraryAdaptor::SparseMatrixQuadratic(HC_N, QTQT);
    if (debug_output) debugOutput << "QTQT = " << QTQT.toMatString() << ";\n";

    return 0;
}

void SymmSpaceDirectNullSpaceSolver::UpdateRHS(
    const SparseMatrixD& QT, const DVectorD& LT,
    const SparseMatrixD& SC, const DVectorD& SV,
    DVectorD* pRHS,
    const bool& debug_output
    )
{
    DVectorD& RHS = *pRHS;

    // g = QT' * LT + SC' * SV;
    const size_t& varDim = QT.getNumRows();
    RHS = QT * LT;

    if (0 == SV.getDim()) return;
    RHS += LibraryAdaptor::SparseMatrixTranspose(SC, varDim) * SV;
}

void SymmSpaceDirectNullSpaceSolver::CombineReduceHV(
    DVectorD* pQTLT,
    const bool& debug_output
    )
{
    // RHS = HC_N * g - HC_N_QTSC_y;
    if (0 == HC_N_QTSC_y.getDim()) return;
    DVectorD& QTLT = *pQTLT;
    QTLT = HC_N * QTLT - HC_N_QTSC_y;
    if (debug_output) debugOutput << "QTLT = " << QTLT.toMatString() << ";\n";
}

int SymmSpaceDirectNullSpaceSolver::ReduceFactorize(
    const SparseMatrixD& HC,
    const DVectorD& HV,
    SparseMatrixD* QTQT,
    const bool& debug_output
    )
{
    if (0 != CombineReduceHC(HC, HV, QTQT, debug_output)) return -1;

    //clapackFactorization_->Factorize(
    //    *QTQT,
    //    debug_output
    //    );

    return 0;
}

void SymmSpaceDirectNullSpaceSolver::UpdateSolve(
    DVectorD* RHS,
    DVectorD* pRES,
    const bool& debug_output
    )
{
    CombineReduceHV(RHS, debug_output);

    //const size_t& varDim = RHS->getDim();
    DVectorD& result = *pRES;
    //clapackFactorization_->Solve(
    //    *RHS,
    //    &result,
    //    debug_output
    //    );

    //result = result.subset(0, varDim);
    if (0 < HC_Nt.getNumRows()) result = HC_Nt * result + HC_R_y;
}

void SymmSpaceDirectNullSpaceSolver::Solve(
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

        if (debug_output) debugOutput << "LHS = " << LHS.toMatString() << ";\n";

        ReduceFactorize(
            HC, HV, &LHS,
            debug_output
            );

        debugOutput << "\nprefactorization time: " << convertTimeToString( timer_fac.getDeltaValue() ) << "\n";

        //if (debug_output)
        {
            debugOutput << "\n";
            debugOutput << "number of variables: " << QT.getNumRows() << ";\n";
            //debugOutput << "number of soft constraints: " << SC.getNumRows() << ";\n";
            debugOutput << "number of hard constraints: " << HC.getNumRows() << ";\n";
            debugOutput << "dimension of null-space (reduced from hard constraints): " << LHS.getNumRows() << ";\n";
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

        if (debug_output) debugOutput << "RHS = " << RHS.toMatString() << ";\n";

        UpdateSolve(
            &RHS,
            pRES,
            debug_output
            );
        if (debug_output) debugOutput << "result = " << pRES->toMatString() << ";\n";
        samCoord = *pRES;
    }

    if (debug_output){
        debugOutput << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n";
    }
}
