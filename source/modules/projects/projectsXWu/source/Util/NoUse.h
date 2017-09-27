#ifndef NoUse_H
#define NoUse_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

namespace NU {
    bool CompTriNormal(
        array3vec3f& verts,
        Vector3f& normal);

    template< typename T_ >
    struct delete_ptr : public std::unary_function<bool, T_>
    {
        bool operator()(T_* pT) const { delete pT; return true; }
    };

    UnstructuredInCoreTriangleMeshPtr OpenMesh(
        UnstructuredInCoreTriangleMesh* mesh);

    bool IsSameDirection(const Vector3f& dir1, const Vector3f& dir2, const float& th = 1e-5);
    bool IsReverseDirection(const Vector3f& dir1, const Vector3f& dir2, const float& th = 1e-5);
    bool IsParallel(const Vector3f& dir1, const Vector3f& dir2, const float& th = 1e-5);
    bool IsParallelDeg(const Vector3f& dir1, const Vector3f& dir2, const float& th = 0.1);
    bool IsZeroMatrix(const Matrix4f& T, const float32& epsilon = 1e-2, const float32& normalizationFactor = 1.f, const bool& dout = false);
    bool IsZeroMatrix6DF(const Matrix4f& T, const float32& epsilon = 1e-6, const float32& normalizationFactor = 1.f);

    size_t CompressMatrixVec(std::vector< Matrix4f >& mat_vec, const float32& epsilon, const float32& normalizationFactor);

    Vector2f NormalToGauss(const Vector3f& N_);

    Vector6f MatrixToVec6DF(const Matrix4f& M_);
    Matrix4f VecToMatrix6DF(const Vector6f& V_);

    bool GenerateFrame(const Vector3f& v1, const Vector3f& v2, Matrix3f& frame);

    float PointlLineDist(const Vector3f& p, const Vector3f& o, const Vector3f& n);
    bool PointOnPlane(const Vector3f& p, const Vector3f& o, const Vector3f& n);

    std::string PrefixRootName(std::string const& name);
}

#endif
