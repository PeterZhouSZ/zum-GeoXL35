#include "StdAfx.h"
#ifdef USE_CUDA
//---------------------------------------------------------------------------
#include "Util\numerical\CudaAdaptor.inline.h"
//---------------------------------------------------------------------------

cublasHandle_t CudaAdaptor::cuBlasHandle = 0;
cusparseHandle_t CudaAdaptor::cuSparseHandle = 0;
cusparseMatDescr_t CudaAdaptor::cudaMatDescr = 0;

void CudaAdaptor::Initialize(void)
{
    if (CUBLAS_STATUS_SUCCESS != cublasCreate(&CudaAdaptor::cuBlasHandle))
    {
        error("CudaAdaptor::Initialize - failed to creat cuBlas handle");
        return;
    }

    if (CUSPARSE_STATUS_SUCCESS != cusparseCreate(&CudaAdaptor::cuSparseHandle))
    {
        error("CudaAdaptor::Initialize - failed to creat cuSparse handle");
        return;
    }

    if (CUSPARSE_STATUS_SUCCESS != cusparseCreateMatDescr(&CudaAdaptor::cudaMatDescr))
    {
        error("CudaAdaptor::Initialize - matrix descriptor initialization failed");
        return;
    }
    cusparseSetMatType(CudaAdaptor::cudaMatDescr, CUSPARSE_MATRIX_TYPE_GENERAL);
    cusparseSetMatIndexBase(CudaAdaptor::cudaMatDescr, CUSPARSE_INDEX_BASE_ZERO);

    //CudaAdaptor::TestCudaAdaptor();
}

void CudaAdaptor::ShutDown(void)
{
    if (CUSPARSE_STATUS_SUCCESS != cusparseDestroyMatDescr(CudaAdaptor::cudaMatDescr))
    {
        error("CudaAdaptor::ShutDown - failed to destroy cuda matrix descriptor");
        return;
    }
    CudaAdaptor::cudaMatDescr = 0;

    if (CUSPARSE_STATUS_SUCCESS != cusparseDestroy(CudaAdaptor::cuSparseHandle))
    {
        error("CudaAdaptor::ShutDown - failed to destroy cuSparse handle");
        return;
    }
    CudaAdaptor::cuSparseHandle = 0;

    if (CUBLAS_STATUS_SUCCESS != cublasDestroy(CudaAdaptor::cuBlasHandle))
    {
        error("CudaAdaptor::ShutDown - failed to destroy cuBlas handle");
        return;
    }
    CudaAdaptor::cuBlasHandle = 0;
}

void CudaAdaptor::TestCudaAdaptor(void)
{
    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "########### CudaAdaptor Test ###########\n";
    debugOutput << "########################################\n";

    SparseMatrixD A; // A = [1 4 0 0 0; 0 2 3 0 0; 5 0 0 7 8; 0 0 9 0 6]
    A.setRows(4);
    A[0].setEntryBinary(0, 1.f);
    A[0].setEntryBinary(1, 4.f);
    A[1].setEntryBinary(1, 2.f);
    A[1].setEntryBinary(2, 3.f);
    A[2].setEntryBinary(0, 5.f);
    A[2].setEntryBinary(3, 7.f);
    A[2].setEntryBinary(4, 8.f);
    A[3].setEntryBinary(2, 9.f);
    A[3].setEntryBinary(4, 6.f);

    //CudaSparseCOO<double> cooA;
    //{
    //    CudaAdaptor::SparseMatrix2CudaCOO(
    //        A,
    //        5,
    //        &cooA
    //        );
    //    debugOutput << CudaAdaptor::ToStringCOO(
    //        cooA
    //        );
    //}

    CudaSparseCSR<double> csrA;
    {
        CudaAdaptor::SparseMatrix2CudaCSR(
            A,
            5,
            &csrA
            );
        debugOutput << CudaAdaptor::ToStringCSR(
            csrA
            );
    }

    {
        DVectorD v(5); // v = [0;3;2;0;6]
        v.setZero();
        v[1] = 3; v[2] = 2; v[4] = 6;
        DVectorD b;
        b.setDim(4);
        b.setZero();
        if (CudaAdaptor::csrmv_01(csrA, v.data(), b.data())) return;
        debugOutput << b << "\n\n"; // [ 12 12 48 54 ]
    }

    {
        DMatrixD B; // B = [1 6 ;2 5; 3 4; 4 3; 5 2]
        B.setDimension(2, 5, false);
        B[0][0] = 1; B[0][1] = 2; B[0][2] = 3; B[0][3] = 4; B[0][4] = 5;
        B[1][0] = 6; B[1][1] = 5; B[1][2] = 4; B[1][3] = 3; B[1][4] = 2;
        DMatrixD C;
        C.setDimension(2, 4, false);
        CudaAdaptor::csrmm2_01(
            csrA, B,
            &C
            );
        debugOutput << C << "\n\n"; // [ 9 13 73 57 | 26 22 67 48 ]'
    }

    SparseMatrixD B; // B = [0 5; 2 0; 7 0; 0 3; 6 0]
    B.setRows(5);
    B[0].setEntryBinary(1, 5);
    B[1].setEntryBinary(0, 2);
    B[2].setEntryBinary(0, 7);
    B[3].setEntryBinary(1, 3);
    B[4].setEntryBinary(0, 6);
    {
        CudaSparseCSR<double> csrB;
        CudaAdaptor::SparseMatrix2CudaCSR(
            B,
            2,
            &csrB
            );
        debugOutput << CudaAdaptor::ToStringCSR(
            csrB
            );

        CudaSparseCSR<double> csrC;
        CudaAdaptor::csrgemm(
            CUSPARSE_OPERATION_NON_TRANSPOSE, csrA,
            CUSPARSE_OPERATION_NON_TRANSPOSE, csrB,
            &csrC
            );
        debugOutput << CudaAdaptor::ToStringCSR(
            csrC
            );
        SparseMatrixD C;
        CudaAdaptor::CudaCSR2SparseMatrix(csrC, &C);
        debugOutput << C.toString(true, csrC.num_c) << "\n"; // [ 8 25 48 99 | 5 0 46 0 ]'
    }

    SparseMatrixD Q(A); // Q = [1 4 0 0 0; 0 2 3 0 0; 5 0 0 7 8; 0 0 9 0 6; 3 0 0 2 0]
    SparseVectorD Q1;
    Q1.setEntryBinary(0, 3);
    Q1.setEntryBinary(3, 2);
    Q.addRow(Q1);
    {
        SparseMatrixD AtBA;
        CudaAdaptor::sparseAtQA(
            Q, B,
            2, &AtBA
            );
        debugOutput << AtBA.toString() << "\n"; // [386 337 | 448 25]
    }

    debugOutput << "########################################\n";
    debugOutput << "######### CudaAdaptor Test END #########\n";
    debugOutput << "########################################\n";
    debugOutput << "\n";
}
#endif

