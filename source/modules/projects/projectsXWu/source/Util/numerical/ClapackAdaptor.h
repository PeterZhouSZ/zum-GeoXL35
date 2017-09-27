//#ifndef ClapackAdaptorH
//#define ClapackAdaptorH
////---------------------------------------------------------------------------
//#include "CommonHdrXWu.h"
//#include "Util\numerical\LibraryAdaptor.h"
////---------------------------------------------------------------------------
//#include "DynamicLinearAlgebra.h"
//#include "SparseLinearAlgebra.h"
////---------------------------------------------------------------------------
//
//namespace Clapack
//{
//    #include "f2c.h"
//    #include "clapack.h"
//
//    class ClapackAdaptor
//    {
//    public:
//        //! compute the solution to a real system of linear equations A * X = B, with N-by-N A
//        //! if rectangular, fill padding 1s.
//        static int dgesv(
//            const SparseMatrixD& LHS,
//            const unsigned& NUM_C,
//            DVectorD* pRHS
//            );
//
//        //! solve a system of linear equations  A * X = B M-by-N matrix A using the LU factorization
//        static int ClapackAdaptor::SolveLU(
//            const SparseMatrixD& LHS,
//            const DVectorD& RHS,
//            const unsigned& NUM_C,
//            DVectorD* pRES
//            );
//
//        //! compute the singular value decomposition (SVD) of a real M-by-N matrix A,
//        //! A = U * S * V^t
//        //! singular values are in descending order,
//        //! optionally computing the left and/or right singular vectors
//        static int dgesvd(
//            std::vector<double>* pA,
//            std::vector<double>* pU,
//            std::vector<double>* pS,
//            std::vector<double>* pV_T,
//            const unsigned& NUM_R,
//            const unsigned& NUM_C,
//            const bool& debug_output = false
//            );
//        static int dgesvd(
//            const SparseMatrixD& LHS,
//            std::vector<double>* pU,
//            std::vector<double>* pS,
//            std::vector<double>* pV_T,
//            unsigned NUM_C,
//            const bool& debug_output = false
//            );
//
//        //! compute reduced matrix which removes singular components
//        static void ReduceSingular(
//            const SparseMatrixD& A,
//            const unsigned& NUM_C,
//            SparseMatrixD* pA_R,
//            SparseMatrixD* pA_Ut,
//            std::vector<double>* pS,
//            double eps = .1f,
//            const bool& debug_output = false
//            );
//
//        //! compute reduced matrix which removes singular components
//        //! also return the null-space basis
//        static void SpaceDecomposition(
//            const SparseMatrixD& A,
//            const unsigned& NUM_C,
//            SparseMatrixD* pA_R,
//            SparseMatrixD* pA_Ut,
//            SparseMatrixD* pA_N,
//            std::vector<double>* pS,
//            double eps,
//            const bool& debug_output
//            );
//    };
//
//    class ClapackFactorizationSymmetric
//    {
//    public:
//        typedef boost::shared_ptr< ClapackFactorizationSymmetric > Ptr;
//        typedef boost::shared_ptr< const ClapackFactorizationSymmetric > ConstPtr;
//
//    public:
//        ClapackFactorizationSymmetric() : UPLO('L') {}
//
//        void Factorize(
//            const SparseMatrixD& LHS,
//            const bool& debug_output = false
//            );
//
//        void Solve(
//            const DVectorD& RHS,
//            DVectorD* pRES,
//            const bool& debug_output = false
//            );
//
//    public:
//        Clapack::integer N;
//        Clapack::integer NRHS;
//        Clapack::integer LDA;
//        std::vector<Clapack::integer> IPIV;
//        Clapack::integer LDB;
//        Clapack::integer INFO;
//
//        char UPLO;
//
//        std::vector<Clapack::doublereal> A;
//        std::vector<Clapack::doublereal> B;
//    };
//
//    class ClapackFactorizationGeneral
//    {
//    public:
//        typedef boost::shared_ptr< ClapackFactorizationGeneral > Ptr;
//        typedef boost::shared_ptr< const ClapackFactorizationGeneral > ConstPtr;
//
//    public:
//        ClapackFactorizationGeneral() : TRANS('N') {}
//
//        void Factorize(
//            const SparseMatrixD& LHS,
//            const unsigned& NUM_C,
//            const bool& debug_output = false
//            );
//
//        void Solve(
//            const DVectorD& RHS,
//            DVectorD* pRES,
//            const bool& debug_output = false
//            );
//
//    public:
//        Clapack::integer M;
//        Clapack::integer N;
//        Clapack::integer NRHS;
//        Clapack::integer LDA;
//        std::vector<Clapack::integer> IPIV;
//        Clapack::integer LDB;
//        Clapack::integer INFO;
//
//        char TRANS;
//
//        std::vector<Clapack::doublereal> A;
//        std::vector<Clapack::doublereal> B;
//    };
//}
//
////using namespace Clapack;
//
//#endif
