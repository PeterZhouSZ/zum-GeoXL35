#ifndef LibraryAdaptorH
#define LibraryAdaptorH
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "DynamicLinearAlgebra.h"
#include "SparseLinearAlgebra.h"
//---------------------------------------------------------------------------

class LibraryAdaptor
{
public:
    LibraryAdaptor(void);
    ~LibraryAdaptor(void);

public:
    //! QT -> [QT, ST^t; ST, 0]
    template <typename FloatType>
    static int SymmetricSparseMatrixBinder(
        SparseMatrix<FloatType>* pQT,
        const SparseMatrix<FloatType>& ST
        );

    //! output flatened std::vector in column major order
    template <typename FloatTypeIn, typename FloatTypeOut>
    static int FlatSparseMatrixColumnMajor(
        const SparseMatrix<FloatTypeIn>& A,
        const unsigned& NUM_R,
        const unsigned& NUM_C,
        std::vector<FloatTypeOut>* pRET
        );

    //! sparse symmetric matrix addition
    template <typename FloatType>
    static SparseMatrix<FloatType> SparseMatrixAddition(
        const SparseMatrix<FloatType>& A,
        const SparseMatrix<FloatType>& B
        );

    //! sparse symmetric matrix multiplication
    template <typename FloatType>
    static SparseMatrix<FloatType> SymmetricSparseMatrixMultiplication(
        const SparseMatrix<FloatType>& A,
        const SparseMatrix<FloatType>& B
        );

    //! sparse matrix-matrix multiplication
    template <typename FloatType>
    static SparseMatrix<FloatType> SparseMatrixMultiplication(
        const SparseMatrix<FloatType>& A,
        const SparseMatrix<FloatType>& B,
        const size_t& num_c
        );

    //! sparse matrix-matrix multiplication, with B transposed
    template <typename FloatType>
    static SparseMatrix<FloatType> SparseMatrixMultiplicationTransposed(
        const SparseMatrix<FloatType>& A,
        const SparseMatrix<FloatType>& B
        );

    //! square of a symmetric matrix
    template <typename FloatType>
    static SparseMatrix<FloatType> SparseMatrixSquare(
        const SparseMatrix<FloatType>& A
        );

    //! A' * A, where A: m x n, m < n
    template <typename FloatType>
    static SparseMatrix<FloatType> SparseMatrixOuterProduct(
        const SparseMatrix<FloatType>& A,
        const size_t& num_c
        );

    //! compute A' * Q * A, where Q is symmetric
    template <typename FloatType>
    static SparseMatrix<FloatType> SparseMatrixQuadratic(
        const SparseMatrix<FloatType>& A_t,
        const SparseMatrix<FloatType>& Q
        );

    template <typename FloatType>
    static SparseMatrix<FloatType> SparseMatrixQuadratic(
        const SparseMatrix<FloatType>& Q,
        const SparseMatrix<FloatType>& A,
        const unsigned dim
        );

    //! sparse matrix - dense vector multiplication
    template <typename FloatType>
    static DynamicVector<FloatType> SparseMatrixMultiplication(
        const SparseMatrix<FloatType>& A,
        const DynamicVector<FloatType>& v
        );

    //! sparse matrix transpose
    template <typename FloatType>
    static SparseMatrix<FloatType> SparseMatrixTranspose(
        const SparseMatrix<FloatType>& A,
        const size_t& num_r
        );

    //! compute -A
    template <typename FloatType>
    static SparseMatrix<FloatType> SparseMatrixNegate(
        const SparseMatrix<FloatType>& A
        );

    //! count number of non-0 entries
    template <typename FloatType>
    static unsigned NumNonZero(const SparseVector<FloatType>& V);
    template <typename FloatType>
    static unsigned NumNonZero(const SparseMatrix<FloatType>& M);

    //! output as Matlab representation
    template <typename ValueType>
    static std::string ToMatlabString(
        const std::vector<ValueType>& V
        );

    //! output as Matlab representation
    template <typename ValueType>
    static std::string ToMatlabString(
        const std::vector<ValueType>& V,
        const size_t NUM_C,
        const size_t NUM_R
        );
};

#include "LibraryAdaptor.inline.h"

#endif
