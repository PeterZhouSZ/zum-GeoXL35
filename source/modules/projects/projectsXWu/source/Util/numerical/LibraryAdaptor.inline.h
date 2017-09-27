#ifdef LibraryAdaptorH
#ifndef LibraryAdaptor_Inline_H
#define LibraryAdaptor_Inline_H
//---------------------------------------------------------------------------
#include "Util\numerical\LibraryAdaptor.h"
//---------------------------------------------------------------------------

//! QT -> [QT, ST^t; ST, 0]
template <typename FloatType>
int LibraryAdaptor::SymmetricSparseMatrixBinder(
    SparseMatrix<FloatType>* pQT,
    const SparseMatrix<FloatType>& ST
    )
{
    //SparseMatrixD QT(2); // QT = [[2,-1,];[-1,2,];]
    //QT[0].addEntryBinary(0, 2.f);
    //QT[0].addEntryBinary(1, -1.f);
    //QT[1].addEntryBinary(0, -1.f);
    //QT[1].addEntryBinary(1, 2.f);
    //SparseMatrixD SC(1);
    //SC[0].addEntryBinary(0, 10.f); // SC = [10, 0]
    //SparseMatrixD HC(0);
    ////SparseMatrixD HC(3);
    ////HC[0].addEntryBinary(0, 1.f);
    ////HC[1].addEntryBinary(1, 1.f);
    ////HC[2].addEntryBinary(0, 1.f);
    ////LibraryAdaptor::SymmetricSparseMatrixBinder(QT, HC);
    //DVectorD B(2);
    //std::fill(B.begin(), B.end(), 1.f);
    //DVectorD SC_VALUE(1);
    //SC_VALUE[0] = 15;
    //DVectorD HC_VALUE(0);
    ////DVectorD HC_VALUE(2);
    ////std::fill(HC_VALUE.begin(), HC_VALUE.end(), 1.f);
    //debugOutput << QT.toMatString() << "\n";
    //debugOutput << B.toMatString() << "\n";
    //clapackFactorization_->Build(QT, SC, HC, B, SC_VALUE, HC_VALUE);
    //debugOutput << QT.toMatString() << "\n";
    //debugOutput << B.toMatString() << "\n";
    //clapackFactorization_->Factorize(QT, B);
    //clapackFactorization_->Solve(2, B);
    //debugOutput << B.toMatString() << "\n";
    //return;

    SparseMatrix<FloatType>& QT = *pQT;
    if (0 == ST.getNumRows()) return 0;

    const unsigned DIM = QT.getNumRows();
    QT.appendRows(ST);
    for (unsigned ri = 0; ri < ST.getNumRows(); ++ri) {
        const SparseVector<FloatType>& row = ST[ri];
        for (SparseVector<FloatType>::EIteratorConst it = row.begin(); it != row.end(); ++it) {
            QT[it->index].addEntryBinary(ri + DIM, it->value);
        }
    }

    return 0;
}

//! output flatened std::vector in column major order
template <typename FloatTypeIn, typename FloatTypeOut>
int LibraryAdaptor::FlatSparseMatrixColumnMajor(
    const SparseMatrix<FloatTypeIn>& A,
    const unsigned& NUM_R,
    const unsigned& NUM_C,
    std::vector<FloatTypeOut>* pRET
    )
{
    std::vector<FloatTypeOut>& RET = *pRET;

    const unsigned DIM = A.getNumRows();
    if (RET.size() != NUM_R * NUM_C) RET.resize(NUM_R * NUM_C);
    std::fill(RET.begin(), RET.end(), 0);
    for (unsigned ri = 0; ri < DIM; ++ri) {
        const SparseVector<FloatTypeIn>& row = A[ri];
        for (SparseVector<FloatTypeIn>::EIteratorConst it = row.begin(); it != row.end(); ++it) {
            mpcard ci = it->index;
            RET[ci * NUM_R + ri] = it->value;
        }
    }

    return 0;
}

//! sparse symmetric matrix addition
template <typename FloatType>
SparseMatrix<FloatType> LibraryAdaptor::SparseMatrixAddition(
    const SparseMatrix<FloatType>& A,
    const SparseMatrix<FloatType>& B
    )
{
    const size_t num_r = A.getNumRows();
    SparseMatrix<FloatType> ret(num_r);
    for (size_t rr = 0; rr < num_r; ++rr) {
        ret[rr] = A[rr] + B[rr];
    }
    return ret;
}

//! sparse symmetric matrix multiplication
template <typename FloatType>
SparseMatrix<FloatType> LibraryAdaptor::SymmetricSparseMatrixMultiplication(
    const SparseMatrix<FloatType>& A,
    const SparseMatrix<FloatType>& B
    )
{
    const size_t num_r = A.getNumRows();
    SparseMatrix<FloatType> B_t = LibraryAdaptor::SparseMatrixTranspose(B, num_r);
    const size_t num_c = B_t.getNumRows();
    SparseMatrix<FloatType> ret(num_r);
    for (size_t rr = 0; rr < num_r; ++rr) {
        for (size_t cc = 0; cc < num_c; ++cc) {
            ret[rr].addEntryBinary(cc, A[rr] * B_t[cc]);
        }
    }
    return ret;
}

//! sparse matrix-matrix multiplication
template <typename FloatType>
SparseMatrix<FloatType> LibraryAdaptor::SparseMatrixMultiplication(
    const SparseMatrix<FloatType>& A,
    const SparseMatrix<FloatType>& B,
    const size_t& num_c
    )
{
    const size_t num_r = A.getNumRows();
    SparseMatrix<FloatType> B_t = LibraryAdaptor::SparseMatrixTranspose(B, num_c);
    SparseMatrix<FloatType> ret(num_r);
    for (size_t rr = 0; rr < num_r; ++rr) {
        for (size_t cc = 0; cc < num_c; ++cc) {
            ret[rr].addEntryBinary(cc, A[rr] * B_t[cc]);
        }
    }
    return ret;
}

//! sparse matrix-matrix multiplication, with B transposed
template <typename FloatType>
SparseMatrix<FloatType> LibraryAdaptor::SparseMatrixMultiplicationTransposed(
    const SparseMatrix<FloatType>& A,
    const SparseMatrix<FloatType>& B
    )
{
    const size_t num_r = A.getNumRows();
    const size_t num_c = B.getNumRows();
    SparseMatrix<FloatType> ret(num_r);
    for (size_t rr = 0; rr < num_r; ++rr) {
        for (size_t cc = 0; cc < num_c; ++cc) {
            ret[rr].addEntryBinary(cc, A[rr] * B[cc]);
        }
    }
    return ret;
}

//! square of a symmetric matrix
template <typename FloatType>
SparseMatrix<FloatType> LibraryAdaptor::SparseMatrixSquare(
    const SparseMatrix<FloatType>& A
    )
{
    const size_t num_r = A.getNumRows();
    SparseMatrix<FloatType> ret(num_r);
    for (size_t rr = 0; rr < num_r; ++rr) {
        for (size_t cc = 0; cc < num_r; ++cc) {
            ret[rr].addEntryBinary(cc, A[rr] * A[cc]);
        }
    }
    return ret;
}

//! A' * A, where A: m x n, m < n
template <typename FloatType>
SparseMatrix<FloatType> LibraryAdaptor::SparseMatrixOuterProduct(
    const SparseMatrix<FloatType>& A,
    const size_t& num_c
    )
{
    return LibraryAdaptor::SparseMatrixSquare(
        LibraryAdaptor::SparseMatrixTranspose(A, num_c)
        );
}

//! compute A' * Q * A, where Q is symmetric
template <typename FloatType>
SparseMatrix<FloatType> LibraryAdaptor::SparseMatrixQuadratic(
    const SparseMatrix<FloatType>& A_t,
    const SparseMatrix<FloatType>& Q
    )
{
    const size_t dim_a = A_t.getNumRows();
    const size_t dim_q = Q.getNumRows();
    SparseMatrix<FloatType> A_t_Q(dim_a);
    for (size_t rr = 0; rr < dim_a; ++rr) {
        for (size_t cc = 0; cc < dim_q; ++cc) {
            A_t_Q[rr].addEntryBinary(cc, A_t[rr] * Q[cc]);
        }
    }
    SparseMatrix<FloatType> ret(dim_a);
    for (size_t rr = 0; rr < dim_a; ++rr) {
        for (size_t cc = 0; cc < dim_a; ++cc) {
            ret[rr].addEntryBinary(cc, A_t_Q[rr] * A_t[cc]);
        }
    }
    return ret;
}

//! compute A' * Q * A, where Q is symmetric
template <typename FloatType>
SparseMatrix<FloatType> LibraryAdaptor::SparseMatrixQuadratic(
    const SparseMatrix<FloatType>& Q,
    const SparseMatrix<FloatType>& A,
    const unsigned dim
    )
{
    return LibraryAdaptor::SparseMatrixQuadratic(
        LibraryAdaptor::SparseMatrixTranspose(A, dim),
        Q);
}

//! sparse vector - dense vector multiplication
template <typename FloatType>
FloatType SparseVectorMultiplication(
    const SparseVector<FloatType>& a,
    const DynamicVector<FloatType>& v
    )
{
    FloatType ret = 0;

    SparseVector<FloatType>::EIteratorConst it = a.begin();
    SparseVector<FloatType>::EIteratorConst itEnd = a.end();
    while (it != itEnd) {
        ret += it->value * v[it->index];
        ++it;
    }

    return ret;
}

//! sparse matrix - dense vector multiplication
template <typename FloatType>
DynamicVector<FloatType> LibraryAdaptor::SparseMatrixMultiplication(
    const SparseMatrix<FloatType>& A,
    const DynamicVector<FloatType>& v
    )
{
    const size_t num_r_A = A.getNumRows();
    DynamicVector<FloatType> ret(num_r_A);
    for (size_t rr = 0; rr < num_r_A; ++rr) {
        const SparseVector<FloatType>& row = A[rr];
        ret[rr] = SparseVectorMultiplication(row, v);
    }
    return ret;
}

//! sparse matrix transpose
template <typename FloatType>
SparseMatrix<FloatType> LibraryAdaptor::SparseMatrixTranspose(
    const SparseMatrix<FloatType>& A,
    const size_t& num_r
    )
{
    SparseMatrix<FloatType> ret(num_r);
    const size_t num_r_A = A.getNumRows();
    for (size_t rr = 0; rr < num_r_A; ++rr) {
        const SparseVector<FloatType>& row = A[rr];
        for (SparseVector<FloatType>::EIteratorConst it = row.begin(); it != row.end(); ++it) {
            if (it->index > num_r)
                error("LibraryAdaptor::SparseMatrixTranspose - index out of range");
            ret[it->index].addEntryBinary(rr, it->value);
        }
    }
    return ret;
}

//! compute -A
template <typename FloatType>
SparseMatrix<FloatType> LibraryAdaptor::SparseMatrixNegate(
    const SparseMatrix<FloatType>& A
    )
{
    const size_t num_r = A.getNumRows();
    SparseMatrix<FloatType> ret(num_r);
    for (size_t rr = 0; rr < num_r; ++rr) {
        const SparseVector<FloatType>& row = A[rr];
        for (SparseVector<FloatType>::EIteratorConst it = row.begin(); it != row.end(); ++it) {
            ret[rr].addEntryBinary(it->index, - it->value);
        }
    }
    return ret;
}

//! count number of non-0 entries
template <typename FloatType>
unsigned LibraryAdaptor::NumNonZero(const SparseVector<FloatType>& V)
{
    return V.entries.size();
}

//! count number of non-0 entries
template <typename FloatType>
unsigned LibraryAdaptor::NumNonZero(const SparseMatrix<FloatType>& M)
{
    unsigned ret = 0;
    const size_t num_r = M.getNumRows();
    for (size_t rr = 0; rr < num_r; ++rr) {
        ret += LibraryAdaptor::NumNonZero(M[rr]);
    }
    return ret;
}

template <typename ValueType>
std::string LibraryAdaptor::ToMatlabString(
    const std::vector<ValueType>& V
    )
{
    std::ostringstream ss;
    ss.precision(16);
    ss << std::scientific;
    ss << "[";
    for (size_t ii = 0; ii < V.size(); ++ii) {
        ss << V[ii] << ";";
    }
    ss << "]";
    return ss.str();
}

template <typename ValueType>
std::string LibraryAdaptor::ToMatlabString(
    const std::vector<ValueType>& V,
    const size_t NUM_C,
    const size_t NUM_R
    )
{
    if (V.size() != NUM_C * NUM_R) {
        error("LibraryAdaptor::ToMatlabString - dimension mis-match");
    }

    std::ostringstream ss;
    ss.precision(16);
    ss << std::scientific;
    ss << "[";
    for (size_t cc = 0; cc < NUM_C; ++cc) {
        ss << "[";
        for (size_t rr = 0; rr < NUM_C; ++rr) {
            ss << V[cc * NUM_C + rr] << ";";
        }
        ss << "],";
    }
    ss << "]";
    return ss.str();
}

#endif
#endif
