#ifndef EigenAdaptor_H
#define EigenAdaptor_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include <fstream>
//---------------------------------------------------------------------------
#include <Eigen/Dense>
//---------------------------------------------------------------------------

class EigenAdaptor
{
public:
    template<typename FloatType, int dim>
    inline static StaticVector<FloatType, dim>
        FromEigen(const Eigen::Matrix<FloatType, dim, 1>& value)
    {
        StaticVector<FloatType, dim> ret;
        for (int d = 0; d < dim; ++d) {
            ret[d] = value(d);
        }
        return ret;
    }

    template<typename FloatType, int columns, int rows>
    inline static StaticMatrix<FloatType, columns, rows>
        FromEigen(const Eigen::Matrix<FloatType, columns, rows>& value)
    {
        StaticMatrix<FloatType, columns, rows> ret;
        for (int c = 0; c < columns; ++c) {
            for (int r = 0; r < rows; ++r) {
                ret[c][r] = value(r, c);
            }
        }
        return ret;
    }

    template<typename FloatType, int dim>
    inline static Eigen::Matrix<FloatType, dim, 1>
        ToEigen(const StaticVector<FloatType, dim> value)
    {
        Eigen::Matrix<FloatType, dim, 1> ret;
        for (int d = 0; d < dim; ++d) {
            ret(d)= value[d];
        }
        return ret;
    }

    template<typename FloatType, int columns, int rows>
    inline static Eigen::Matrix<FloatType, columns, rows>
        ToEigen(const StaticMatrix<FloatType, columns, rows> value)
    {
        Eigen::Matrix<FloatType, rows, columns> ret;
        for (int c = 0; c < columns; ++c) {
            for (int r = 0; r < rows; ++r) {
                ret(r, c) = value[c][r];
            }
        }
        return ret;
    }

    template<typename FloatType>
    inline static Eigen::Matrix<FloatType, 4, 4>
        MakeTranslation(const Eigen::Matrix<FloatType, 3, 1> value)
    {
        Eigen::Matrix<FloatType, 4, 4> ret = Eigen::Matrix<FloatType, 4, 4>::Identity();
        ret.col(3).head<3>() = value;
        return ret;
    }

    template<typename FloatType>
    inline static Eigen::Matrix<FloatType, 4, 4>
        MakeScale(const FloatType& value)
    {
        Eigen::Matrix<FloatType, 4, 4> ret = Eigen::Matrix<FloatType, 4, 4>::Identity();
        for (int d = 0; d < 3; ++d) ret(d, d) = value;
        return ret;
    }

    template<typename FloatType>
    inline static Eigen::Matrix<FloatType, 4, 4>
        InvertFrame(const Eigen::Matrix<FloatType, 4, 4>& value)
    {
        Eigen::Matrix<FloatType, 4, 4> ret = Eigen::Matrix<FloatType, 4, 4>::Identity();
        ret.block<3, 3>(0, 0) = value.block<3, 3>(0, 0).transpose();
        ret.col(3) = - value.col(3);
        ret(3, 3) = 1;
        return ret;
    }

    template<typename FloatType>
    inline static Eigen::Matrix<FloatType, 3, 1>
        Transform(const Eigen::Matrix<FloatType, 4, 4>& m,
        const Eigen::Matrix<FloatType, 3, 1>& v)
    {
        return (m * v.homogeneous()).head<3>();
    }

    //template<typename FloatType, int NR, int NC>
    //static void WriteFile(const Eigen::Matrix<FloatType, NR, NC>& mat,
    //    const std::string& name = "M")
    //{
    //    std::ofstream oFile("C:\\Workstation\\temp\\matFile.txt",
    //        std::ofstream::out | std::ofstream::app);
    //    if (oFile.is_open())
    //    {
    //        oFile << name << " = [\n";
    //        for (int rr = 0; rr < NR; ++rr) {
    //            for (int cc = 0; cc < NC - 1; ++cc) {
    //                oFile << mat(rr, cc) << ", ";
    //            }
    //            oFile << mat(rr, NC - 1) << ";\n";
    //        }
    //        oFile << "];\n";
    //    }
    //    oFile.close();
    //}

    template<typename FloatType>
    static void WriteFile(
        const Eigen::Matrix<FloatType, Eigen::Dynamic, Eigen::Dynamic>& mat,
        const std::string& name = "M")
    {
        std::ofstream oFile("C:\\Workstation\\temp\\matFile.txt",
            std::ofstream::out | std::ofstream::app);
        if (oFile.is_open())
        {
            oFile << "\n" << name << " = [\n";
            const int NR = mat.rows();
            const int NC = mat.cols();
            for (int rr = 0; rr < NR; ++rr) {
                for (int cc = 0; cc < NC - 1; ++cc) {
                    oFile << mat(rr, cc) << ", ";
                }
                oFile << mat(rr, NC - 1) << ";\n";
            }
            oFile << "];\n";
        }
        oFile.close();
    }

    template<typename MatrixType>
    static std::string ToString(
        const MatrixType& mat
        )
    {
        std::ostringstream ss;
        ss << mat;
        return ss.str();
    }
};

namespace Eigen
{
    typedef Matrix<float, 6, 1> Vector6f;
    typedef Matrix<float, 6, 6> Matrix6f;
}

#endif
