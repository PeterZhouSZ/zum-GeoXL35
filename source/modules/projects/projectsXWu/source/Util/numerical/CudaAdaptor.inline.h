#ifndef CudaAdaptorInlineH
#define CudaAdaptorInlineH
#ifdef USE_CUDA
//---------------------------------------------------------------------------
#include "Util\numerical\CudaAdaptor.h"
#include <type_traits>
//---------------------------------------------------------------------------

//template <typename FloatType>
//CudaSparseCOO<FloatType>::CudaSparseCOO(void)
//{
//    num_r = -1;
//    num_c = -1;
//    nnz = -1;
//    rowInd = nullptr;
//    colInd = nullptr;
//    spVal = nullptr;
//    rowIndGpu = nullptr;
//    colIndGpu = nullptr;
//    spValGpu = nullptr;
//}
//
//template <typename FloatType>
//CudaSparseCOO<FloatType>::~CudaSparseCOO(void)
//{
//    if (rowInd) delete rowInd; rowInd = nullptr;
//    if (colInd) delete colInd; colInd = nullptr;
//    if (spVal) delete spVal; spVal = nullptr;
//    if (rowIndGpu) cudaFree(rowIndGpu); rowIndGpu = nullptr;
//    if (colIndGpu) cudaFree(colIndGpu); colIndGpu = nullptr;
//    if (spValGpu) cudaFree(spValGpu); spValGpu = nullptr;
//}
//
//template <typename FloatType>
//void CudaSparseCOO<FloatType>::SendToGpu(void)
//{
//    if (rowIndGpu) cudaFree(rowIndGpu); rowIndGpu = nullptr;
//    if (colIndGpu) cudaFree(colIndGpu); colIndGpu = nullptr;
//    if (spValGpu) cudaFree(spValGpu); spValGpu = nullptr;
//    if (
//        cudaSuccess != cudaMalloc((void**)&rowIndGpu, nnz * sizeof(rowIndGpu[0])) ||
//        cudaSuccess != cudaMalloc((void**)&colIndGpu, nnz * sizeof(colIndGpu[0])) ||
//        cudaSuccess != cudaMalloc((void**)&spValGpu, nnz * sizeof(spValGpu[0]))
//        ) {
//            error("CudaSparseCSR<FloatType>::SendToGpu - cudaMalloc failed");
//            return;
//    }
//
//    if (
//        cudaSuccess != cudaMemcpy(rowIndGpu, rowInd, (size_t)(nnz * sizeof(rowIndGpu[0])), cudaMemcpyHostToDevice) ||
//        cudaSuccess != cudaMemcpy(colIndGpu, colInd, (size_t)(nnz * sizeof(colIndGpu[0])), cudaMemcpyHostToDevice) ||
//        cudaSuccess != cudaMemcpy(spValGpu, spVal, (size_t)(nnz * sizeof(spValGpu[0])), cudaMemcpyHostToDevice)
//        ) {
//            error("CudaSparseCSR<FloatType> - cudaMemcpy failed");
//            return;
//    }
//}

template <typename FloatType>
CudaSparseCSR<FloatType>::CudaSparseCSR(void)
{
    num_r = -1;
    num_c = -1;
    nnz = -1;
    rowPtrGpu = nullptr;
    colIndGpu = nullptr;
    spValGpu = nullptr;
}

template <typename FloatType>
CudaSparseCSR<FloatType>::~CudaSparseCSR(void)
{
    if (rowPtrGpu) cudaFree(rowPtrGpu); rowPtrGpu = nullptr;
    if (colIndGpu) cudaFree(colIndGpu); colIndGpu = nullptr;
    if (spValGpu) cudaFree(spValGpu); spValGpu = nullptr;
}

template <typename FloatType>
void CudaSparseCSR<FloatType>::SendToGpu(void)
{
    if (rowPtrGpu) cudaFree(rowPtrGpu); rowPtrGpu = nullptr;
    if (colIndGpu) cudaFree(colIndGpu); colIndGpu = nullptr;
    if (spValGpu) cudaFree(spValGpu); spValGpu = nullptr;
    if (
        cudaSuccess != cudaMalloc((void**)&rowPtrGpu, (num_r + 1) * sizeof(rowPtrGpu[0])) ||
        cudaSuccess != cudaMalloc((void**)&colIndGpu, nnz * sizeof(colIndGpu[0])) ||
        cudaSuccess != cudaMalloc((void**)&spValGpu, nnz * sizeof(spValGpu[0]))
        ) {
            error("CudaSparseCSR<FloatType> - cudaMalloc failed");
            return;
    }

    int* rowPtr_host = new int[num_r + 1];
    int* colInd_host = new int[nnz];
    FloatType* spVal_host = new FloatType[nnz];
    std::copy(rowPtr.begin(), rowPtr.end(), rowPtr_host);
    std::copy(colInd.begin(), colInd.end(), colInd_host);
    std::copy(spVal.begin(), spVal.end(), spVal_host);
    if (
        cudaSuccess != cudaMemcpy(rowPtrGpu, rowPtr_host, (size_t)((num_r + 1) * sizeof(rowPtrGpu[0])), cudaMemcpyHostToDevice) ||
        cudaSuccess != cudaMemcpy(colIndGpu, colInd_host, (size_t)(nnz * sizeof(colIndGpu[0])), cudaMemcpyHostToDevice) ||
        cudaSuccess != cudaMemcpy(spValGpu, spVal_host, (size_t)(nnz * sizeof(spValGpu[0])), cudaMemcpyHostToDevice)
        ) {
            error("CudaSparseCSR<FloatType>::SendToGpu - cudaMemcpy failed");
            return;
    }
    delete rowPtr_host;
    delete colInd_host;
    delete spVal_host;

    //if (
    //    cudaSuccess != cudaMemcpy(rowPtrGpu, &rowPtr.front(), (size_t)((num_r + 1) * sizeof(rowPtrGpu[0])), cudaMemcpyHostToDevice) ||
    //    cudaSuccess != cudaMemcpy(colIndGpu, &colInd.front(), (size_t)(nnz * sizeof(colIndGpu[0])), cudaMemcpyHostToDevice) ||
    //    cudaSuccess != cudaMemcpy(spValGpu, &spVal.front(), (size_t)(nnz * sizeof(spValGpu[0])), cudaMemcpyHostToDevice)
    //    ) {
    //        error("CudaSparseCSR<FloatType>::SendToGpu - cudaMemcpy failed");
    //        return;
    //}
}

//template <typename FloatType>
//int CudaAdaptor::SparseMatrix2CudaCOO(
//    SparseMatrix<FloatType> const& SM,
//    int const& num_c,
//    CudaSparseCOO<FloatType>* coo_p
//    )
//{
//    CudaSparseCOO<FloatType>& coo = *coo_p;
//    int& nnz = coo.nnz;
//
//    std::deque<int> rowIndVec;
//    std::deque<int> colIndVec;
//    std::deque<FloatType> spValVec;
//
//    nnz = 0;
//    const size_t num_r = SM.getNumRows();
//    coo.num_r = num_r;
//    coo.num_c = num_c;
//    for (size_t rr = 0; rr < num_r; ++rr) {
//        const SparseVector<FloatType>& row = SM[rr];
//        SparseVector<FloatType>::EIteratorConst eit = row.begin();
//        SparseVector<FloatType>::EIteratorConst eit_end = row.end();
//        while (eit != eit_end) {
//            rowIndVec.push_back(rr);
//            colIndVec.push_back(eit->index);
//            spValVec.push_back(eit->value);
//            ++nnz;
//            ++eit;
//        }
//    }
//
//    coo.rowInd = new int[nnz];
//    coo.colInd = new int[nnz];
//    coo.spVal = new FloatType[nnz];
//    std::copy(rowIndVec.begin(), rowIndVec.end(), coo.rowInd);
//    std::copy(colIndVec.begin(), colIndVec.end(), coo.colInd);
//    std::copy(spValVec.begin(), spValVec.end(), coo.spVal);
//
//    coo.SendToGpu();
//
//    return 0;
//}

template <typename FloatType>
int CudaAdaptor::SparseMatrix2CudaCSR(
    SparseMatrix<FloatType> const& SM,
    int const& num_c,
    CudaSparseCSR<FloatType>* csr_p
    )
{
    CudaSparseCSR<FloatType>& csr = *csr_p;
    int& nnz = csr.nnz;

    csr.rowPtr.clear();
    csr.colInd.clear();
    csr.spVal.clear();

    nnz = 0;
    const size_t num_r = SM.getNumRows();
    csr.num_r = num_r;
    csr.num_c = num_c;
    for (size_t rr = 0; rr < num_r; ++rr) {
        csr.rowPtr.push_back(nnz);
        const SparseVector<FloatType>& row = SM[rr];
        SparseVector<FloatType>::EIteratorConst eit = row.begin();
        SparseVector<FloatType>::EIteratorConst eit_end = row.end();
        while (eit != eit_end) {
            csr.colInd.push_back(eit->index);
            csr.spVal.push_back(eit->value);
            ++nnz;
            ++eit;
        }
    }
    csr.rowPtr.push_back(nnz);

    csr.SendToGpu();

    return 0;
}

template <typename FloatType>
int CudaAdaptor::CudaCSR2SparseMatrix(
    CudaSparseCSR<FloatType> const& csr,
    SparseMatrix<FloatType> * SM_p
    )
{
    SparseMatrix<FloatType>& SM = *SM_p;
    SM.setRows(csr.num_r);
    SM.setZero();

    unsigned nzid = 0;
    for (int rr = 0; rr < csr.num_r; ++rr) {
        const int rr_f = rr + 1;
        for (int ii = csr.rowPtr[rr]; ii < csr.rowPtr[rr_f]; ++ii) {
            SM[rr].setEntryBinary(csr.colInd[nzid], csr.spVal[nzid]);
            ++nzid;
        }
    }

    return 0;
}

//! csrmv(L2): y = a * op(A) * x + b * y
template <typename FloatType>
int CudaAdaptor::csrmv(
    cusparseOperation_t const& trans, CudaSparseCSR<FloatType> const& csr,
    FloatType const* const x, FloatType const& a, FloatType const& b,
    FloatType * const y
    )
{
    if (std::is_same<FloatType, double>::value)
    {
        FloatType* yGpu = 0;
        FloatType* xGpu = 0;
        if (
            cudaSuccess != cudaMalloc((void**)&yGpu, csr.num_r * sizeof(yGpu[0])) ||
            cudaSuccess != cudaMalloc((void**)&xGpu, csr.num_c * sizeof(xGpu[0]))
            ) {
                error("CudaSparseCSR<FloatType>::SendToGpu - cudaMalloc failed");
                return 1;
        }
        if (
            cudaSuccess != cudaMemcpy(yGpu, y, (size_t)(csr.num_r * sizeof(yGpu[0])), cudaMemcpyHostToDevice) ||
            cudaSuccess != cudaMemcpy(xGpu, x, (size_t)(csr.num_c * sizeof(xGpu[0])), cudaMemcpyHostToDevice)
            ) {
                error("CudaSparseCSR<FloatType>::SendToGpu - cudaMemcpy failed");
                return 1;
        }

        if (CUSPARSE_STATUS_SUCCESS != cusparseDcsrmv(
            CudaAdaptor::cuSparseHandle, trans,
            csr.num_r, csr.num_c, csr.nnz, &a,
            CudaAdaptor::cudaMatDescr,
            csr.spValGpu, csr.rowPtrGpu, csr.colIndGpu, xGpu, &b, yGpu)
            )
        {
            error("CudaAdaptor::csrmv - matrix-vector multiplication failed");
            return 1;
        }

        if (
            cudaSuccess != cudaMemcpy(y, yGpu, (size_t)(csr.num_r * sizeof(yGpu[0])), cudaMemcpyDeviceToHost)
            ) {
                error("CudaSparseCSR<FloatType>::SendToGpu - cudaMemcpy failed");
                return 1;
        }

        cudaFree(yGpu);
        cudaFree(xGpu);
    } else {
        error("CudaAdaptor::csrmv - unsupported type");
        return 1;
    }

    return 0;
}

//! csrmv(L2): y = A * x
template <typename FloatType>
int CudaAdaptor::csrmv_01(
    CudaSparseCSR<FloatType> const& csr, FloatType const* const x,
    FloatType * const y
    )
{
    const double one = 1, zero = 0;
    return CudaAdaptor::csrmv<double>(
        CUSPARSE_OPERATION_NON_TRANSPOSE, csr,
        x, one, zero,
        y
        );
}

// C_{m x n} = a * op(A) + b * op(B), transpose matrix A by setting a = 1 & b = 0
template <typename FloatType>
int geam(
    cusparseOperation_t const& transA, FloatType const* const A,
    cusparseOperation_t const& transB, FloatType const* const B,
    unsigned const& m, unsigned const& n,
    FloatType const& a, FloatType const& b,
    FloatType * const C 
    )
{
    //// dense matrix: column-major, LD == num_r
    //unsigned const& ld_a = (CUBLAS_OP_N == transA) ? m : n;
    //unsigned const& ld_b = (CUBLAS_OP_N == transB) ? m : n;
    //cublasSetPointerMode(CudaAdaptor::cuBlasHandle, CUBLAS_POINTER_MODE_HOST);
    //cublasDgeam(CudaAdaptor::cuBlasHandle,
    //    CUBLAS_OP_T, CUBLAS_OP_T,
    //    m, n,
    //    &a, A, ld_a,
    //    &b, B, ld_b,
    //    C, m);
}

//! csrmm2(L3): C = a * op(A) * op(B) + b * C
template <typename FloatType>
int csrmm2(
    cusparseOperation_t const& transA, CudaSparseCSR<FloatType> const& csr,
    cusparseOperation_t const& transB, FloatType const* const B,
    FloatType const& a, FloatType const& b,
    unsigned const& ld_b,
    FloatType * const C
    )
{
    // step 1: Bt := transpose(B)
    FloatType* Bt_gpu;
    cudaMalloc((void**)&Bt_gpu, ld_b * csr.num_c * sizeof(Bt_gpu[0]));
    const double one = 1, zero = 0;
    cublasSetPointerMode(CudaAdaptor::cuBlasHandle, CUBLAS_POINTER_MODE_HOST);
    cublasDgeam(CudaAdaptor::cuBlasHandle,
        CUBLAS_OP_T, CUBLAS_OP_T,
        csr.num_c, ld_b,
        &one, B, ld_b,
        &zero, B, ld_b,
        Bt_gpu, csr.num_c);
    // step 2: perform C:=alpha*A*transpose(Bt) + beta*C
    cusparseDcsrmm2(CudaAdaptor::cuSparseHandle,
        transA, transB
        m, n, k, nnz, alpha,
        descrA, csrValA, csrRowPtrA, csrColIndA,
        Bt, ldb_Bt,
        beta, C, ldc);
}

//! csrmm2(L3): C_{m x n} = A * B
template <typename FloatType>
int CudaAdaptor::csrmm2_01(
    CudaSparseCSR<FloatType> const& A_csr, DynamicMatrix<FloatType> const B,
    DynamicMatrix<FloatType> * C_p
    )
{
    DynamicMatrix<FloatType>& C = *C_p;

    const unsigned& m = A_csr.num_r;
    const unsigned& k = A_csr.num_c;
    const unsigned& n = B.getColsDim();
    const double one = 1, zero = 0;

    if (std::is_same<FloatType, double>::value)
    {
        FloatType* Bt_host = new FloatType[n * k];
        for (unsigned cc = 0; cc < n; ++cc) {
            for (unsigned rr = 0; rr < k; ++rr) {
                Bt_host[rr * n + cc] = B[cc][rr];
            }
        }
        FloatType* Bt_gpu; // n x k
        cudaMalloc((void**)&Bt_gpu, n * k * sizeof(Bt_gpu[0]));
        cudaMemcpy(Bt_gpu, Bt_host, (size_t)(n * k * sizeof(Bt_gpu[0])), cudaMemcpyHostToDevice);

        FloatType* C_host = new FloatType[m * n];
        FloatType* C_gpu; // m x n
        cudaMalloc((void**)&C_gpu, m * n * sizeof(C_gpu[0]));

        cusparseDcsrmm2(CudaAdaptor::cuSparseHandle,
            CUSPARSE_OPERATION_NON_TRANSPOSE, CUSPARSE_OPERATION_TRANSPOSE,
            m, n, k,
            A_csr.nnz, &one, CudaAdaptor::cudaMatDescr,
            A_csr.spValGpu, A_csr.rowPtrGpu, A_csr.colIndGpu,
            Bt_gpu, n,
            &zero, C_gpu, m);

        cudaMemcpy(C_host, C_gpu, (size_t)(m * n * sizeof(C_gpu[0])), cudaMemcpyDeviceToHost);
        for (unsigned cc = 0; cc < n; ++cc) {
            for (unsigned rr = 0; rr < m; ++rr) {
                C[cc][rr] = C_host[cc * m + rr];
            }
        }

        delete Bt_host;
        cudaFree(Bt_gpu);
        delete C_host;
        cudaFree(C_gpu);
    } else {
        error("CudaAdaptor::csrmv - unsupported type");
        return 1;
    }

    return 0;
}

//! csrgemm: C = op(A) * op(B), all CSR format
template <typename FloatType>
int CudaAdaptor::csrgemm(
    cusparseOperation_t const& transA, CudaSparseCSR<FloatType> const& A_csr,
    cusparseOperation_t const& transB, CudaSparseCSR<FloatType> const& B_csr,
    CudaSparseCSR<FloatType> * C_csr_p, bool const& fetch_gpu
    )
{
    CudaSparseCSR<FloatType>& C_csr = *C_csr_p;
    //int* rowPtr;
    //int* colInd;
    //FloatType* spVal;

    const unsigned& m = (CUSPARSE_OPERATION_NON_TRANSPOSE == transA) ? A_csr.num_r : A_csr.num_c;
    const unsigned& k = (CUSPARSE_OPERATION_NON_TRANSPOSE == transA) ? A_csr.num_c : A_csr.num_r;
    const unsigned& n = (CUSPARSE_OPERATION_NON_TRANSPOSE == transB) ? B_csr.num_c : B_csr.num_r;
    const double one = 1, zero = 0;

    if (std::is_same<FloatType, double>::value)
    {
        int baseC, nnzC;
        int *nnzTotalDevHostPtr = &nnzC;
        cusparseSetPointerMode(CudaAdaptor::cuSparseHandle, CUSPARSE_POINTER_MODE_HOST);
        cudaMalloc((void**)&C_csr.rowPtrGpu, sizeof(C_csr.rowPtrGpu[0]) * (m + 1));
        cusparseXcsrgemmNnz(CudaAdaptor::cuSparseHandle,
            transA, transB,
            m, n, k,
            CudaAdaptor::cudaMatDescr, A_csr.nnz, A_csr.rowPtrGpu, A_csr.colIndGpu,
            CudaAdaptor::cudaMatDescr, B_csr.nnz, B_csr.rowPtrGpu, B_csr.colIndGpu,
            CudaAdaptor::cudaMatDescr, C_csr.rowPtrGpu, nnzTotalDevHostPtr
            );
        if (NULL != nnzTotalDevHostPtr){
            nnzC = *nnzTotalDevHostPtr;
        }else{
            cudaMemcpy(&nnzC, C_csr.rowPtrGpu + m, sizeof(int), cudaMemcpyDeviceToHost);
            cudaMemcpy(&baseC, C_csr.rowPtrGpu, sizeof(int), cudaMemcpyDeviceToHost);
            nnzC -= baseC;
        }
        C_csr.nnz = nnzC;
        C_csr.num_r = m;
        C_csr.num_c = n;

        cudaMalloc((void**)&C_csr.colIndGpu, sizeof(C_csr.colIndGpu[0]) * nnzC);
        cudaMalloc((void**)&C_csr.spValGpu, sizeof(C_csr.spValGpu[0]) * nnzC);
        cusparseDcsrgemm(CudaAdaptor::cuSparseHandle,
            transA, transB,
            m, n, k,
            CudaAdaptor::cudaMatDescr, A_csr.nnz,
            A_csr.spValGpu, A_csr.rowPtrGpu, A_csr.colIndGpu,
            CudaAdaptor::cudaMatDescr, B_csr.nnz,
            B_csr.spValGpu, B_csr.rowPtrGpu, B_csr.colIndGpu,
            CudaAdaptor::cudaMatDescr,
            C_csr.spValGpu, C_csr.rowPtrGpu, C_csr.colIndGpu
            );

        if (fetch_gpu)
        {
            int* rowPtr_host = new int[m + 1];
            int* colInd_host = new int[nnzC];
            FloatType* spVal_host = new FloatType[nnzC];
            cudaMemcpy(rowPtr_host, C_csr.rowPtrGpu, (size_t)((m + 1) * sizeof(C_csr.rowPtrGpu[0])), cudaMemcpyDeviceToHost);
            cudaMemcpy(colInd_host, C_csr.colIndGpu, (size_t)(nnzC * sizeof(C_csr.colIndGpu[0])), cudaMemcpyDeviceToHost);
            cudaMemcpy(spVal_host, C_csr.spValGpu, (size_t)(nnzC * sizeof(C_csr.spValGpu[0])), cudaMemcpyDeviceToHost);
            C_csr.rowPtr.resize(m + 1);
            C_csr.colInd.resize(nnzC);
            C_csr.spVal.resize(nnzC);
            std::copy(rowPtr_host, rowPtr_host + (m + 1), C_csr.rowPtr.begin());
            std::copy(colInd_host, colInd_host + nnzC, C_csr.colInd.begin());
            std::copy(spVal_host, spVal_host + nnzC, C_csr.spVal.begin());
            delete rowPtr_host;
            delete colInd_host;
            delete spVal_host;
        }
    } else {
        error("CudaAdaptor::csrmv - unsupported type");
        return 1;
    }

    return 0;
}

//! compute A' * Q * A, where Q is symmetric
template <typename FloatType>
int CudaAdaptor::sparseAtQA(
    const SparseMatrix<FloatType>& Q,
    const SparseMatrix<FloatType>& A,
    const unsigned dim,
    SparseMatrix<FloatType> * AtQA
    )
{
    const unsigned& m = Q.getNumRows();
    const double one = 1, zero = 0;

    CudaSparseCSR<FloatType> Q_csr;
    CudaAdaptor::SparseMatrix2CudaCSR(
        Q,
        m,
        &Q_csr
        );
    CudaSparseCSR<FloatType> A_csr;
    CudaAdaptor::SparseMatrix2CudaCSR(
        A,
        dim,
        &A_csr
        );

    if (std::is_same<FloatType, double>::value)
    {
        CudaSparseCSR<FloatType> QA_csr;
        CudaAdaptor::csrgemm(
            CUSPARSE_OPERATION_NON_TRANSPOSE, Q_csr,
            CUSPARSE_OPERATION_NON_TRANSPOSE, A_csr,
            &QA_csr, false
            );

        CudaSparseCSR<FloatType> AtQA_csr;
        CudaAdaptor::csrgemm(
            CUSPARSE_OPERATION_TRANSPOSE, A_csr,
            CUSPARSE_OPERATION_NON_TRANSPOSE, QA_csr,
            &AtQA_csr
            );

        CudaAdaptor::CudaCSR2SparseMatrix(AtQA_csr, AtQA);
    } else {
        error("CudaAdaptor::csrmv - unsupported type");
        return 1;
    }

    return 0;
}

//template <typename FloatType>
//std::string CudaAdaptor::ToStringCOO(
//    CudaSparseCOO<FloatType> const& coo
//    )
//{
//    std::ostringstream ss;
//    ss << "\n";
//    ss << "spVal = [ ";
//    for (int ii = 0; ii < coo.nnz; ++ii) {
//        ss << coo.spVal[ii] << " ";
//    }
//    ss << "]\n";
//    ss << "rowInd = [ ";
//    for (int ii = 0; ii < coo.nnz; ++ii) {
//        ss << coo.rowInd[ii] << " ";
//    }
//    ss << "]\n";
//    ss << "colInd = [ ";
//    for (int ii = 0; ii < coo.nnz; ++ii) {
//        ss << coo.colInd[ii] << " ";
//    }
//    ss << "]\n";
//    ss << "\n";
//    return ss.str();
//}

template <typename FloatType>
std::string CudaAdaptor::ToStringCSR(
    CudaSparseCSR<FloatType> const& csr
    )
{
    std::ostringstream ss;
    ss << "\n";
    ss << "spVal = [ ";
    for (int ii = 0; ii < csr.nnz; ++ii) {
        ss << csr.spVal[ii] << " ";
    }
    ss << "]\n";
    ss << "rowPtr = [ ";
    for (int ii = 0; ii < csr.num_r + 1; ++ii) {
        ss << csr.rowPtr[ii] << " ";
    }
    ss << "]\n";
    ss << "colInd = [ ";
    for (int ii = 0; ii < csr.nnz; ++ii) {
        ss << csr.colInd[ii] << " ";
    }
    ss << "]\n";
    ss << "\n";
    return ss.str();
}

#endif
#endif

