//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "SPSolver/SymmSpaceNullSpace.h"
#include "SToolBox/DefieldSymm.h"
//----------------------------------------------------------------------
//----------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

SymmSpaceNullSpace::SymmSpaceNullSpace() :
    isFullRankConst(false),
    rangeSpace_( new SymmSpaceDirectSolver() ),
    directNullSpace_( new SymmSpaceDirectNullSpaceSolver() ),
    num_ns_basis(0),
    num_var(0)
{
}

void SymmSpaceNullSpace::BuildLHSProjection(
    const SparseMatrixD& QT,
    const SparseMatrixD& SC,
    SparseMatrixD* pLHS,
    const SparseMatrixD& HC, SparseMatrixD* pHC_NS,
    const DVectorD& HV, DVectorD* pHV_NS,
    const bool& debug_output
    )
{
    SparseMatrixD& LHS = *pLHS;
    // QTSC = QT' * QT + SC' * SC;
    LHS = LibraryAdaptor::SparseMatrixSquare(QT);
    if (0 < SC.getNumRows()) {
        LHS += LibraryAdaptor::SparseMatrixOuterProduct(SC, num_var);
    }
    {
        LHS.removeNearZeros(std::numeric_limits<double>::epsilon());
        const float ratio = 100 * (float)LibraryAdaptor::NumNonZero(LHS) / (LHS.getNumRows() * LHS.getNumRows());
        debugOutput << "non-zero ratio before projection: " << ratio << "%\n";
    }
    // LHS = NS * QTSC * NS';
    LHS = LibraryAdaptor::SparseMatrixQuadratic(NS, LHS);
    LHS.removeNearZeros(std::numeric_limits<double>::epsilon());
    {
        const float ratio = 100 * (float)LibraryAdaptor::NumNonZero(LHS) / (LHS.getNumRows() * LHS.getNumRows());
        debugOutput << "non-zero ratio after projection: " << ratio << "%\n";
    }

    // HC_NS = HC * NS'
    if (0 < HC.getNumRows()) { // project HC onto null-space
        SparseMatrixD& HC_NS = *pHC_NS;
        HC_NS.setRows(HC.getNumRows());
        {
            for (unsigned rr = 0; rr < HC.getNumRows(); ++rr) {
                for (unsigned cc = 0; cc < num_ns_basis; ++cc) {
                    HC_NS[rr].addEntryBinary(cc, HC[rr] * NS[cc]);
                }
            }
            HC_NS.removeNearZeros(std::numeric_limits<double>::epsilon());
        }
        // HV_NS = HV
        *pHV_NS = HV;
        if (debug_output) debugOutput << "HC_NS = " << HC_NS.toMatString(num_ns_basis) << ";\n";
        if (debug_output) debugOutput << "HV_NS = " << pHV_NS->toMatString() << ";\n";
    }
}

void SymmSpaceNullSpace::UpdateRHSProjection(
    const SparseMatrixD& QT, const DVectorD& LT,
    const SparseMatrixD& SC, const DVectorD& SV,
    DVectorD* pRHS,
    const bool& debug_output
    )
{
    DVectorD& RHS = *pRHS;

    // LTSV = QT' * LT + SC' * SV
    RHS = QT * LT;
    if (0 < SV.getDim()) {
        RHS += LibraryAdaptor::SparseMatrixTranspose(SC, num_var) * SV;
    }
    // RHS = NS * LTSV
    RHS = NS * RHS;
}

void SymmSpaceNullSpace::Solve(
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
            debugOutput << "NS = " << NS.toMatString(num_var) << ";\n";
            debugOutput << "NS_t = " << NS_t.toMatString(num_var) << ";\n";
            debugOutput << "\n";
        }
    }

    if (bRefactor) {
        Timer timer_fac; timer_fac.getDeltaValue();

        SparseMatrixD LHS;
        SparseMatrixD HC_NS;
        DVectorD HV_NS;
        BuildLHSProjection(
            QT, SC,
            &LHS,
            HC, &HC_NS,
            HV, &HV_NS,
            debug_output
            );

        //{
        //    SparseMatrixD QT(3);
        //    QT[0].addEntryBinary(0, 6);
        //    QT[0].addEntryBinary(1, 2);
        //    QT[0].addEntryBinary(2, 1);
        //    QT[1].addEntryBinary(0, 2);
        //    QT[1].addEntryBinary(1, 5);
        //    QT[1].addEntryBinary(2, 2);
        //    QT[2].addEntryBinary(0, 1);
        //    QT[2].addEntryBinary(1, 2);
        //    QT[2].addEntryBinary(2, 4);
        //    SparseMatrixD HC(2);
        //    HC[0].addEntryBinary(0, 1);
        //    HC[0].addEntryBinary(2, 1);
        //    HC[1].addEntryBinary(1, 1);
        //    HC[1].addEntryBinary(2, 1);
        //    DVectorD HV(2);
        //    HV.setZero();

        //    directNullSpace_->ReduceFactorize(
        //        HC, HV,
        //        &QT,
        //        true
        //        );
        //}

        if (debug_output) debugOutput << "QTSC = " << LHS.toMatString(num_ns_basis) << ";\n";
        if (!isFullRankConst) {
            if (0 != directNullSpace_->ReduceFactorize(
                HC_NS, HV_NS,
                &LHS,
                debug_output
                ))
            {
                isFullRankConst = true;
                rangeSpace_->ReduceFactorize(
                    HC_NS, HV_NS,
                    &LHS,
                    debug_output
                    );
            }
        }
        if (debug_output) debugOutput << "LHS = " << LHS.toMatString(num_ns_basis) << ";\n";

        debugOutput << "\nprefactorization time: " << convertTimeToString( timer_fac.getDeltaValue() ) << "\n";

        //if (debug_output)
        {
            debugOutput << "\n";
            debugOutput << "number of variables: " << QT.getNumRows() << ";\n";
            //debugOutput << "number of soft constraints: " << SC.getNumRows() << ";\n";
            debugOutput << "number of hard constraints: " << HC.getNumRows() << ";\n";
            debugOutput << "dimension of system projected onto null space: " << NS.getNumRows() << ";\n";
            debugOutput << "dimension of constaints reduced system: " << LHS.getNumRows() << ";\n";
            if (isFullRankConst) warning("full rank constraint set, no freedom to move\n");
            debugOutput << "\n";
        }
    }

    {
        DVectorD RHS;
        UpdateRHSProjection(
            QT, LT,
            SC, SV,
            &RHS,
            debug_output
            );

        //{
        //    DVectorD LT(3);
        //    LT.setZero();
        //    LT[0] = 8;
        //    LT[1] = 3;
        //    LT[2] = 3;
        //    directNullSpace_->UpdateSolve(
        //        &LT,
        //        &nsCoord,
        //        true
        //        );
        //    debugOutput << "nsCoord = " << nsCoord.toMatString() << ";\n";
        //}

        if (debug_output) debugOutput << "QTSV = " << RHS.toMatString() << ";\n";
        if (!isFullRankConst) {
            directNullSpace_->UpdateSolve(
                &RHS,
                &nsCoord,
                debug_output
                );
        } else {
            rangeSpace_->UpdateSolve(
                &RHS,
                &nsCoord,
                debug_output
                );
        }
        if (debug_output) debugOutput << "RHS = " << RHS.toMatString() << ";\n";
        if (debug_output) debugOutput << "res = " << nsCoord.toMatString() << ";\n";
    }
    *pRES = NS_t * nsCoord;
    //*pRES = nsCoord;

    if (debug_output){
        debugOutput << "nsCoord = " << pRES->toMatString() << ";\n";

        debugOutput << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n";
    }
}

int SymmSpaceNullSpace::BuildRotatedBasis(
    const SymmSampleGraph& symmGraph,
    const std::deque<unsigned>& fillInIndice,
    const unsigned& num_points
    )
{
    const bool debug_output = false;

    const unsigned Dim = 3;
    const unsigned varDim = Dim * num_points;

    typedef boost::property_map<SymmSampleGraph::graph_t, boost::vertex_index_t>::type IndexMap;
    const std::deque< SymmSampleGraph::graph_t >& orbitGraphVec = symmGraph.graphVec;
    const unsigned& num_orbit = orbitGraphVec.size();
    NS.setRows(num_orbit * Dim); /* num_orbit x num_sample */
    for (unsigned oi = 0; oi < num_orbit; ++oi) {
        unsigned num_entry = 0;
        std::set<unsigned> vertSet;
        //SparseMatrixD nullSpaceBasis(num_orbit * Dim); // one row for each orbit graph
        const SymmSampleGraph::graph_t& orbitGraph = orbitGraphVec[oi];
        IndexMap indMap = boost::get(boost::vertex_index, orbitGraph);
        boost::graph_traits<SymmSampleGraph::graph_t>::edge_iterator eit, eit_end;
        for (boost::tie(eit, eit_end) = edges(orbitGraph); eit != eit_end; ++eit)
        {
            vertSet.insert(indMap[boost::source(*eit, orbitGraph)]);
            vertSet.insert(indMap[boost::target(*eit, orbitGraph)]);
            const unsigned& index = indMap[boost::target(*eit, orbitGraph)];
            if (orbitGraph[*eit].groupId != orbitGraph[index].groupId) continue; // trick to add only one const for each overlapping vertex
            const Matrix4f& transformation = orbitGraph[*eit].rootTrans;
            const Matrix3f& R_t = shrink4To3(transformation).transpose();
            for (unsigned d = 0; d < Dim; ++d) {
                for (unsigned r = 0; r < 3; ++r) {
                    //nullSpaceBasis[Dim * oi + d].addEntryBinary(Dim * index + r, R_t[r][d]);
                    NS[Dim * oi + d].addEntryBinary(Dim * index + r, R_t[r][d]);
                }
            }
            if (debug_output) debugOutput << index << "\n";
            ++num_entry;
        }
        if (debug_output) debugOutput << num_entry << " matrix entries added on orbit " << oi << "\n";
        //if (vertSet.size() != num_entry) {
        //    std::ostringstream ss;
        //    ss << "SymmetryConstraints::BuildSymmetryConstraints - ";
        //    ss << "[" << oi << "]: " << vertSet.size() << " vertices, " << num_entry << " constraints";
        //    error(ss.str());
        //}
        //NS.appendRows(nullSpaceBasis);
    }

    // add degree of freedom for those "fill-in" vertice
    const unsigned num_fillin = fillInIndice.size();
    SparseMatrixD fillFree(num_fillin * Dim);
    for (unsigned ii = 0; ii < num_fillin; ++ii)
    {
        const unsigned& index = fillInIndice[ii];
        for (unsigned d = 0; d < Dim; ++d) {
            fillFree[Dim * ii + d].addEntryBinary(Dim * index + d, 1);
        }
    }
    NS.appendRows(fillFree);

    debugOutput << "number of orbits: " << num_orbit << "\n";
    debugOutput << "DoF by direct construction: " << NS.getNumRows() << "\n";
    { // must do SVD if points in an orbit are overlapping, e.g. when sampling on lines
        SparseMatrixD NS_R, NS_Ut;
        std::vector<double> S;
        const double eps = .1f;
        //Clapack::ClapackAdaptor::ReduceSingular(NS, varDim, &NS_R, &NS_Ut, &S, eps, false);
        // use reduced anyway, for numerical stability
        NS = NS_R;
        for (unsigned rr = 0; rr < NS_R.getNumRows(); ++rr) {
            NS[rr] /= S[rr];
        }
    }
    debugOutput << "DoF after SVD: " << NS.getNumRows() << "\n";
    debugOutput << "\n";

    NS_t = LibraryAdaptor::SparseMatrixTranspose(NS, varDim);
    num_ns_basis = NS.getNumRows();
    num_var = varDim;

    return 0;
}
