#ifndef CudaAdaptorH
#define CudaAdaptorH
#ifdef USE_CUDA
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
#include <cuda_runtime.h>
#include "cublas_v2.h"
#include "cusparse.h"
//---------------------------------------------------------------------------

//// row-major format
//template <typename FloatType>
//struct CudaSparseCOO
//{
//    int num_r;
//    int num_c;
//    int nnz;
//
//    int* rowInd;
//    int* colInd;
//    FloatType* spVal;
//
//    int* rowIndGpu;
//    int* colIndGpu;
//    FloatType* spValGpu;
//
//    void SendToGpu(void);
//
//    CudaSparseCOO(void);
//    ~CudaSparseCOO(void);
//};

// row-major format
template <typename FloatType>
struct CudaSparseCSR
{
    int num_r;
    int num_c;
    int nnz;

    //int* rowPtr;
    //int* colInd;
    //FloatType* spVal;
    std::deque<int> rowPtr;
    std::deque<int> colInd;
    std::deque<FloatType> spVal;

    int* rowPtrGpu;
    int* colIndGpu;
    FloatType* spValGpu;

    void SendToGpu(void);

    CudaSparseCSR(void);
    ~CudaSparseCSR(void);
};

class CudaAdaptor
{
public:
    ////! sparse matrix --> cuda coo format
    //template <typename FloatType>
    //static int SparseMatrix2CudaCOO(
    //    SparseMatrix<FloatType> const& SM,
    //    int const& num_c,
    //    CudaSparseCOO<FloatType>* coo_p
    //    );

    //! sparse matrix --> cuda csr format
    template <typename FloatType>
    static int SparseMatrix2CudaCSR(
        SparseMatrix<FloatType> const& SM,
        int const& num_c,
        CudaSparseCSR<FloatType>* csr_p
        );

    template <typename FloatType>
    static int CudaCSR2SparseMatrix(
        CudaSparseCSR<FloatType> const& csr,
        SparseMatrix<FloatType> * SM_p
        );

    //! csrmv(L2): y = a * op(A) * x + b * y
    template <typename FloatType>
    static int csrmv(
        cusparseOperation_t const& trans, CudaSparseCSR<FloatType> const& csr,
        FloatType const* const x, FloatType const& a, FloatType const& b,
        FloatType * const y
        );

    //! csrmv(L2): y = A * x
    template <typename FloatType>
    static int csrmv_01(
        CudaSparseCSR<FloatType> const& csr, FloatType const* const x,
        FloatType * const y
        );

    //! csrmv(L2): C = a * op(A) * op(B) + B * C
    template <typename FloatType>
    static int csrmm2(
        cusparseOperation_t const& transA, CudaSparseCSR<FloatType> const& csr,
        cusparseOperation_t const& transB, DynamicMatrix<FloatType> const& B,
        FloatType const& a,
        DynamicMatrix<FloatType>* C
        );

    //! csrmm2(L3): C_{m x n} = A * B
    template <typename FloatType>
    static int csrmm2_01(
        CudaSparseCSR<FloatType> const& A_csr, DynamicMatrix<FloatType> const B,
        DynamicMatrix<FloatType> * C_p
        );

    template <typename FloatType>
    static int csrgemm(
        cusparseOperation_t const& transA, CudaSparseCSR<FloatType> const& A_csr,
        cusparseOperation_t const& transB, CudaSparseCSR<FloatType> const& B_csr,
        CudaSparseCSR<FloatType> * C_csr_p, bool const& fetch_gpu = true
        );

    template <typename FloatType>
    static int sparseAtQA(
        const SparseMatrix<FloatType>& Q,
        const SparseMatrix<FloatType>& A,
        const unsigned dim,
        SparseMatrix<FloatType> * AtQA
        );

public:
    //template <typename FloatType>
    //static std::string ToStringCOO(
    //    CudaSparseCOO<FloatType> const& coo
    //    );

    template <typename FloatType>
    static std::string ToStringCSR(
        CudaSparseCSR<FloatType> const& csr
        );

public:
    static cublasHandle_t cuBlasHandle;
    static cusparseHandle_t cuSparseHandle;
    static cusparseMatDescr_t cudaMatDescr;
    static void Initialize(void);
    static void ShutDown(void);
    static void TestCudaAdaptor(void);
};

#endif
#endif

