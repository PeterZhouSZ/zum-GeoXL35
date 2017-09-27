//#include "StdAfx.h"
////---------------------------------------------------------------------------
//#include "ClapackAdaptor.h"
//#include "f2c.h"
//#include "clapack.h"
////---------------------------------------------------------------------------
//
//using namespace Clapack;
//
////int dgesv_(integer *n, integer *nrhs, doublereal *a, integer 
////	*lda, integer *ipiv, doublereal *b, integer *ldb, integer *info);
//int ClapackAdaptor::dgesv(
//    const SparseMatrixD& LHS,
//    const unsigned& NUM_C,
//    DVectorD* pRHS
//    )
//{
//    DVectorD& RHS = *pRHS;
//
//    const integer NUM_R = LHS.getNumRows();
//    if (NUM_R != RHS.getDim()) {
//        error("SymmSpaceSolver::SolveDirect - system dimension mis-match");
//        return -1;
//    }
//
//    integer N = NUM_C;
//    integer NRHS = 1;
//    integer LDA = NUM_C;
//    std::vector<integer> IPIV(N);
//    integer LDB = NUM_C;
//    integer INFO;
//
//    std::vector<doublereal> A(N * N);
//    {
//        LibraryAdaptor::FlatSparseMatrixColumnMajor(LHS, N, N, &A);
//        for (integer rr = NUM_R; rr < N; ++rr) A[rr * N + rr] = 1;
//    }
//
//    std::vector<doublereal> B(N);
//    {
//        for (integer rr = 0; rr < NUM_R; ++rr) B[rr] = RHS[rr];
//        for (integer rr = NUM_R; rr < N; ++rr) B[rr] = 0;
//    }
//
//    dgesv_(&N, &NRHS, &A[0], &LDA, &IPIV[0], &B[0], &LDB, &INFO);
//    if (0 != INFO) {
//        error( str( boost::format("ClapackAdaptor::dgesv - failed with INFO: %1%.\n")
//            % INFO
//            ));
//        return -1;
//    }
//
//    RHS.setDim(N);
//    {
//        for (integer rr = 0; rr < N; ++rr) RHS[rr] = B[rr];
//    }
//
//    return 0;
//}
//
////int dgetrf_(integer *m, integer *n, doublereal *a, integer *
////	lda, integer *ipiv, integer *info);
////int dgetrs_(char *trans, integer *n, integer *nrhs, 
////	doublereal *a, integer *lda, integer *ipiv, doublereal *b, integer *
////	ldb, integer *info);
//int ClapackAdaptor::SolveLU(
//    const SparseMatrixD& LHS,
//    const DVectorD& RHS,
//    const unsigned& NUM_C,
//    DVectorD* pRES
//    )
//{
//    //DVectorD& RES = *pRES;
//
//    //integer M = LHS.getNumRows();
//    //integer N = NUM_C;
//    //integer LDA = M;
//    //std::vector<integer> IPIV(std::min<int>(M, N));
//    //integer INFO;
//
//    //std::vector<doublereal> A(M * N);
//    //LibraryAdaptor::FlatSparseMatrixColumnMajor(LHS, M, N, &A);
//
//    //dgetrf_(&M, &N, &A[0], &LDA, &IPIV[0], &INFO) ;
//    //if (0 != INFO) {
//    //    error( str( boost::format("ClapackAdaptor::SolveLU - factorization failed with INFO: %1%.\n")
//    //        % INFO
//    //        ));
//    //    return -1;
//    //}
//
//    //char TRANS = 'N';
//    //integer NRHS = 1;
//    //integer LDB = N;
//
//    //std::vector<doublereal> B(N);
//    //{
//    //    for (integer rr = 0; rr < N; ++rr) B[rr] = RHS[rr];
//    //}
//
//    //dgetrs_(&TRANS, &N, &NRHS, &A[0], &LDA, &IPIV[0], &B[0], &LDB, &INFO) ;
//    //if (0 != INFO) {
//    //    error( str( boost::format("ClapackAdaptor::SolveLU - solve failed with INFO: %1%.\n")
//    //        % INFO
//    //        ));
//    //    return -1;
//    //}
//
//    //{
//    //    for (integer rr = 0; rr < N; ++rr) RES[rr] = B[rr];
//    //}
//
//    return 0;
//}
//
////int dgesvd_(char *jobu, char *jobvt, integer *m, integer *n, 
////	doublereal *a, integer *lda, doublereal *s, doublereal *u, integer *
////	ldu, doublereal *vt, integer *ldvt, doublereal *work, integer *lwork, 
////	integer *info);
//int ClapackAdaptor::dgesvd(
//    std::vector<double>* pA,
//    std::vector<double>* pU,
//    std::vector<double>* pS,
//    std::vector<double>* pV_T,
//    const unsigned& NUM_R,
//    const unsigned& NUM_C,
//    const bool& debug_output
//    )
//{
//    std::vector<double>& A = *pA;
//    std::vector<double>& U = *pU;
//    std::vector<double>& S = *pS;
//    std::vector<double>& V_T = *pV_T;
//
//    char JOBU = 'A';
//    char JOBVT = 'A';
//    integer M = NUM_R;
//    integer N = NUM_C;
//    integer LDA = M;
//    integer LDU = M;
//    integer LDVT = N;
//    integer LWORK = std::max<int>(3 * std::min<int>(M, N) + std::max<int>(M, N), 5 * std::min<int>(M, N));
//    std::vector<doublereal> WORK(LWORK);
//    integer INFO;
//
//    U.resize(M * M);
//    S.resize(std::min<int>(M, N));
//    V_T.resize(N * N);
//
//    dgesvd_(&JOBU, &JOBVT, &M, &N,
//        &A[0], &LDA, &S[0], &U[0], &LDU, &V_T[0], &LDVT,
//        &WORK[0], &LWORK, &INFO);
//    if (0 != INFO) {
//        error( str( boost::format("ClapackAdaptor::dgesvd - failed with INFO: %1%.\n")
//            % INFO
//            ));
//        return -1;
//    }
//    if (debug_output)
//    {
//        // NOTE: singular values are unique, but singular vectors are not!
//        debugOutput << "U = " << LibraryAdaptor::ToMatlabString(U, M, M) << ";\n";
//        debugOutput << "S = " << LibraryAdaptor::ToMatlabString(S) << ";\n";
//        debugOutput << "V_T = " << LibraryAdaptor::ToMatlabString(V_T, N, N) << ";\n";
//    }
//    return 0;
//}
//
//int ClapackAdaptor::dgesvd(
//    const SparseMatrixD& LHS,
//    std::vector<double>* pU,
//    std::vector<double>* pS,
//    std::vector<double>* pV_T,
//    unsigned NUM_C,
//    const bool& debug_output
//    )
//{
//    const unsigned NUM_R = LHS.getNumRows();
//    if (0 == NUM_C) NUM_C = NUM_R;
//
//    std::vector<doublereal> A(NUM_R * NUM_C);
//    LibraryAdaptor::FlatSparseMatrixColumnMajor(LHS, NUM_R, NUM_C, &A);
//
//    if (debug_output)
//    {
//        debugOutput << "LHS = " << LHS.toMatString(NUM_C) << ";\n";
//    }
//    ClapackAdaptor::dgesvd(&A, pU, pS, pV_T, NUM_R, NUM_C, debug_output);
//
//    return 0;
//}
//
//void ClapackAdaptor::ReduceSingular(
//    const SparseMatrixD& A,
//    const unsigned& NUM_C,
//    SparseMatrixD* pA_R,
//    SparseMatrixD* pA_Ut,
//    std::vector<double>* pS,
//    double eps,
//    const bool& debug_output
//    )
//{
//    SparseMatrixD& A_R = *pA_R;
//    SparseMatrixD& A_Ut = *pA_Ut;
//    std::vector<double>& S = *pS;
//    const unsigned& NUM_R = A.getRows();
//
//    const double& eps_lim = std::numeric_limits<double>::epsilon();
//    if (eps < 0) eps = .1f;
//
//    std::vector<double> U, /*S,*/ V_T;
//    ClapackAdaptor::dgesvd(A, &U, &S, &V_T, NUM_C, debug_output);
//
//    unsigned num_positive = S.size();
//    for (; num_positive > 0; --num_positive) if (S[num_positive-1] > eps) break;
//    //if (debug_output)
//    //{
//    //    debugOutput << "svd cut-off at [" << num_positive << "]: ";
//    //    const unsigned svd_length = 3;
//    //    const unsigned svd_start = std::max<unsigned>(0, num_positive - svd_length);
//    //    const unsigned svd_end = std::min<unsigned>(S.size(), num_positive + svd_length);
//    //    unsigned svd_i = svd_start;
//    //    for (; svd_i < num_positive; ++svd_i) {
//    //        debugOutput << S[svd_i] << "; ";
//    //    }
//    //    debugOutput << "[" << S[num_positive] << "]; ";
//    //    for (++svd_i; svd_i < svd_end; ++svd_i) {
//    //        debugOutput << S[svd_i] << "; ";
//    //    }
//    //    debugOutput << /*S[svd_i] <<*/ "\n";
//    //}
//
//    // compute reduce matrix
//    A_R.clear();
//    A_R.setRows(num_positive);
//    for (unsigned cc = 0; cc < NUM_C; ++cc) {
//        for (unsigned rr = 0; rr < num_positive; ++rr) {
//            const double& value = V_T[cc * NUM_C + rr];
//            if (eps_lim > abs(value)) continue;
//            A_R[rr].addEntryBinary(cc, S[rr] * value);
//        }
//    }
//    if (debug_output) debugOutput << "A_R = " << A_R.toMatString(NUM_C) << ";\n";
//
//    // compute (croped) transpose of left singular matrix 
//    A_Ut.setRows(); // first clear everything up!
//    A_Ut.setRows(num_positive);
//    for (unsigned cc = 0; cc < num_positive; ++cc) {
//        for (unsigned rr = 0; rr < NUM_R; ++rr) {
//            const double& value = U[cc * NUM_R + rr];
//            if (eps_lim > abs(value)) continue;
//            A_Ut[cc].addEntryBinary(rr, value);
//        }
//    }
//    if (debug_output) debugOutput << "A_Ut = " << A_Ut.toMatString(NUM_R) << ";\n";
//}
//
//// A = A_Ut' * A_R; rank([A_R; A_N]);
//void ClapackAdaptor::SpaceDecomposition(
//    const SparseMatrixD& A,
//    const unsigned& NUM_C,
//    SparseMatrixD* pA_R,
//    SparseMatrixD* pA_Ut,
//    SparseMatrixD* pA_N,
//    std::vector<double>* pS,
//    double eps,
//    const bool& debug_output
//    )
//{
//    SparseMatrixD& A_R = *pA_R;
//    SparseMatrixD& A_Ut = *pA_Ut;
//    SparseMatrixD& A_N = *pA_N;
//    std::vector<double>& S = *pS;
//    const unsigned& NUM_R = A.getRows();
//
//    const double& eps_lim = std::numeric_limits<double>::epsilon();
//    if (eps < 0) eps = .1f;
//
//    std::vector<double> U, V_T;
//    ClapackAdaptor::dgesvd(A, &U, &S, &V_T, NUM_C, debug_output);
//
//    unsigned num_positive = S.size();
//    for (; num_positive > 0; --num_positive) if (S[num_positive-1] > eps) break;
//
//    // compute reduce matrix
//    A_R.clear();
//    A_R.setRows(num_positive);
//    for (unsigned cc = 0; cc < NUM_C; ++cc) {
//        for (unsigned rr = 0; rr < num_positive; ++rr) {
//            const double& value = V_T[cc * NUM_C + rr];
//            if (eps_lim > abs(value)) continue;
//            A_R[rr].addEntryBinary(cc, S[rr] * value);
//        }
//    }
//    if (debug_output) debugOutput << "A_R = " << A_R.toMatString(NUM_C) << ";\n";
//
//    // compute (croped) transpose of left singular matrix 
//    A_Ut.setRows(); // first clear everything up!
//    A_Ut.setRows(num_positive);
//    for (unsigned cc = 0; cc < num_positive; ++cc) {
//        for (unsigned rr = 0; rr < NUM_R; ++rr) {
//            const double& value = U[cc * NUM_R + rr];
//            if (eps_lim > abs(value)) continue;
//            A_Ut[cc].addEntryBinary(rr, value);
//        }
//    }
//    if (debug_output) debugOutput << "A_Ut = " << A_Ut.toMatString(NUM_R) << ";\n";
//
//    // compute null-space basis
//    A_N.clear();
//    A_N.setRows(NUM_C - num_positive);
//    for (unsigned cc = 0; cc < NUM_C; ++cc) {
//        for (unsigned rr = num_positive; rr < NUM_C; ++rr) {
//            const double& value = V_T[cc * NUM_C + rr];
//            if (eps_lim > abs(value)) continue;
//            A_N[rr - num_positive].addEntryBinary(cc, value);
//        }
//    }
//    if (debug_output) debugOutput << "A_N = " << A_N.toMatString(NUM_C) << ";\n";
//}
//
////int dsytrf_(char *uplo, integer *n, doublereal *a, integer *
////	lda, integer *ipiv, doublereal *work, integer *lwork, integer *info);
//void ClapackFactorizationSymmetric::Factorize(
//    const SparseMatrixD& LHS,
//    const bool& debug_output
//    )
//{
//    N = LHS.getNumRows();
//    LDA = N;
//    IPIV.resize(N, 0);
//
//    A.resize(N * N);
//    LibraryAdaptor::FlatSparseMatrixColumnMajor(LHS, N, N, &A);
//
//    integer ISPEC = 1;
//    integer dummy = -1;
//    integer NB = ilaenv_(&ISPEC, "dsytrf_", &UPLO, &LDA, &dummy, &dummy, &dummy);
//    integer LWORK = N * NB;
//    std::vector<doublereal> WORK(LWORK);
//    dsytrf_(&UPLO, &N, &A[0], &LDA, &IPIV[0], &WORK[0], &LWORK, &INFO);
//    //debugOutput << str( boost::format("optimal LWORK: %1%, the value given: %2%.\n")
//    //    % WORK[0] % LWORK
//    //    );
//    if (0 != INFO) {
//        error( str( boost::format("ClapackFactorizationSymmetric::Factorize - failed with INFO: %1%.\n")
//            % INFO
//            ));
//        return;
//    }
//}
//
////int dsytrs_(char *uplo, integer *n, integer *nrhs, 
////	doublereal *a, integer *lda, integer *ipiv, doublereal *b, integer *
////	ldb, integer *info);
//void ClapackFactorizationSymmetric::Solve(
//    const DVectorD& RHS,
//    DVectorD* pRES,
//    const bool& debug_output
//    )
//{
//    DVectorD& RES = *pRES;
//
//    NRHS = 1;
//    LDB = RHS.getDim();
//    B.resize(LDB);
//    std::copy(RHS.begin(), RHS.end(), B.begin());
//
//    if (LDA != LDB) {
//        error( str( boost::format("ClapackFactorizationSymmetric::Solve - system dimension mis-match: %1%, %2%")
//            % LDA % LDB
//            ));
//        return;
//    }
//
//    dsytrs_(&UPLO, &N, &NRHS, &A[0], &LDA, &IPIV[0], &B[0], &LDB, &INFO);
//    if (0 != INFO) {
//        error( str( boost::format("ClapackFactorizationSymmetric::Solve - failed with INFO: %1%.\n")
//            % INFO
//            ));
//        return;
//    }
//    RES.setDim(LDB);
//    for (integer rr = 0; rr < LDB; ++rr) RES[rr] = B[rr];
//}
//
////int dgetrf_(integer *m, integer *n, doublereal *a, integer *
////	lda, integer *ipiv, integer *info);
//void ClapackFactorizationGeneral::Factorize(
//    const SparseMatrixD& LHS,
//    const unsigned& NUM_C,
//    const bool& debug_output
//    )
//{
//    M = LHS.getNumRows();
//    N = NUM_C;
//    LDA = M;
//    IPIV.resize(min(M, N), 0);
//
//    A.resize(M * N);
//    LibraryAdaptor::FlatSparseMatrixColumnMajor(LHS, M, N, &A);
//
//    dgetrf_(&M, &N, &A[0], &LDA, &IPIV[0], &INFO);
//    if (0 != INFO) {
//        error( str( boost::format("ClapackFactorizationGeneral::Factorize - failed with INFO: %1%.\n")
//            % INFO
//            ));
//        return;
//    }
//}
//
////int dgetrs_(char *trans, integer *n, integer *nrhs, 
////	doublereal *a, integer *lda, integer *ipiv, doublereal *b, integer *
////	ldb, integer *info);
//void ClapackFactorizationGeneral::Solve(
//    const DVectorD& RHS,
//    DVectorD* pRES,
//    const bool& debug_output
//    )
//{
//    DVectorD& RES = *pRES;
//
//    NRHS = 1;
//    LDB = RHS.getDim();
//    B.resize(LDB);
//    std::copy(RHS.begin(), RHS.end(), B.begin());
//
//    if (LDA != LDB) {
//        error("ClapackFactorizationGeneral::Solve - system dimension mis-match");
//        return;
//    }
//
//    dgetrs_(&TRANS, &N, &NRHS, &A[0], &LDA, &IPIV[0], &B[0], &LDB, &INFO);
//    if (0 != INFO) {
//        error( str( boost::format("ClapackFactorizationGeneral::Solve - failed with INFO: %1%.\n")
//            % INFO
//            ));
//        return;
//    }
//    RES.setDim(LDB);
//    for (integer rr = 0; rr < LDB; ++rr) {
//        RES[rr] = B[rr];
//    }
//}
