#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Util/NoUse.h"
//---------------------------------------------------------------------------
#include "VertexArray.h"
//---------------------------------------------------------------------------

bool NU::CompTriNormal(
    array3vec3f& verts,
    Vector3f& normal)
{
    Vector3f edgeVec[3];
    unsigned edgeMaxIndex = 0;
    float edgeMaxLength = 0.f;
    for (unsigned j = 0; j < 3; ++j) {
        edgeVec[j] = verts[(j+1)%3] - verts[j];
        float tmp = norm(edgeVec[j]);
        if (tmp > edgeMaxLength) {
            edgeMaxIndex = j;
            edgeMaxLength = tmp;
        }
    }
    if (1e-6 > edgeMaxLength)
        return false;
    normal = edgeVec[(edgeMaxIndex+1)%3].crossProduct(edgeVec[(edgeMaxIndex+2)%3]);
    float length = norm(normal);
    if (1e-6 > length)
        return false;
    normal /= length;
    return true;
}

UnstructuredInCoreTriangleMeshPtr NU::OpenMesh(
    UnstructuredInCoreTriangleMesh* mesh)
{
    typedef std::set<int> KeyT_;
    typedef boost::unordered_map< KeyT_, mpcard > FaceTab;
    FaceTab ftab;
    std::vector<Vector3f> point;
    std::vector<Vector3f> normal;
    std::vector<Vector3i> index;
    {
        VertexArray va_vert(mesh->getPointSet());
        VertexArray va_face(mesh->getTriangles());
        for (mpcard fi = 0; fi < va_face.getNumElements(); ++fi) {
            Vector3i tri = va_face.getIndex3i(fi);
            KeyT_ key;
            for (mpcard j = 0; j < 3; ++j) {
                key.insert(tri[j]);
            }
            ftab[key] = fi;
        }

        FaceTab::iterator it = ftab.begin();
        for (int ii = 0; it != ftab.end(); ++it, ++ii) {
            Vector3i tri = va_face.getIndex3i(it->second);
            for (mpcard j = 0; j < 3; ++j) {
                point.push_back(va_vert.getPosition3f(tri[j]));
                normal.push_back(va_vert.getNormal3f(tri[j]));
            }
            index.push_back(makeVector3i(3*ii, 3*ii+1, 3*ii+2));
        }
    }

    UnstructuredInCoreTriangleMeshPtr omesh (new UnstructuredInCoreTriangleMesh);
    {
        omesh->clearAndSetupDefault(point.size(), index.size());
        VertexArray va_vert(omesh->getPointSet());
        VertexArray va_face(omesh->getTriangles());
        for (mpcard ii = 0; ii < point.size(); ++ii) {
            va_vert.setPosition3f(ii, point[ii]);
            va_vert.setNormal3f(ii, normal[ii]);
        }
        for (mpcard ii = 0; ii < index.size(); ++ii) {
            va_face.setIndex3i(ii, index[ii]);
        }
    }

    return omesh;
}

bool NU::IsSameDirection(const Vector3f& dir1, const Vector3f& dir2, const float& th)
{
    return fabs(dir1 * dir2 - 1) < th;
}

bool NU::IsReverseDirection(const Vector3f& dir1, const Vector3f& dir2, const float& th)
{
    return fabs(dir1 * dir2 + 1) < th;
}

bool NU::IsParallel(const Vector3f& dir1, const Vector3f& dir2, const float& th)
{
    return fabs(fabs(dir1 * dir2) - 1) < th;
}

bool NU::IsParallelDeg(const Vector3f& dir1, const Vector3f& dir2, const float& th)
{
    return fabs(dir1 * dir2) > cos(fabs((th * M_PI) / 180.f));
}

Vector2f NU::NormalToGauss(const Vector3f& N_)
{
    Vector2f angle;
    Vector3f normal = normalize(N_);
    Vector3f normal_xz = normal;
    normal_xz[1] = 0.f; // singular point at the polars?
    normal_xz = normalize(normal_xz);
    float zip = ZAXIS_VECTOR3F * normal_xz;
    if (zip > 1) zip = 1;
    else if (zip < -1) zip = -1;
    angle[0] = acos(zip);
    if (ZAXIS_VECTOR3F * normal < 0) angle[0] = 360.f - angle[0];
    float yip = YAXIS_VECTOR3F * normal;
    if (yip > 1) yip = 1;
    else if (yip < -1) yip = -1;
    angle[1] = acos(yip);
    return angle;
}

bool NU::IsZeroMatrix(const Matrix4f& T, const float32& epsilon/* = 1e-6*/, const float32& normalizationFactor/* = 1.f*/, const bool& dout)
{
    float sos = 0.f, sor = 0.f, sot = 0.f;
    for (unsigned ii = 0; ii < 3; ++ii) {
        sot += sqr(T[3][ii] / normalizationFactor);
    }
    for (unsigned ii = 0; ii < 3; ++ii) {
        for (unsigned jj = 0; jj < 3; ++jj) {
            sor += sqr(T[ii][jj]);
        }
    }
    sos = sqrt(fabs(sot + sor));
    if (dout) {
        debugOutput << str( boost::format("matrix difference test: %1% [%2%, %3%].\n")
            % sos % sot % sor
            );
    }
    return (sos < epsilon) ? true : false;
}

bool NU::IsZeroMatrix6DF(const Matrix4f& T, const float32& epsilon, const float32& normalizationFactor)
{
    Vector6f v6f = NU::MatrixToVec6DF(T);
    float sos = 0.f;
    for (unsigned ii = 0; ii < 3; ++ii) {
        v6f[ii] /= normalizationFactor;
    }
    for (unsigned ii = 0; ii < 6; ++ii) {
        sos += sqr(v6f[ii]);
    }
    return (sqrt(fabs(sos)) < epsilon) ? true : false;
}

size_t NU::CompressMatrixVec(std::vector< Matrix4f >& mat_vec, const float32& epsilon, const float32& normalizationFactor)
{
    const size_t num_mat = mat_vec.size();
    std::deque< Matrix4f > symm_unique;
    std::vector< bool > bmark(num_mat, false);
    for (size_t m1 = 0; m1 < num_mat; ++m1) {
        if (bmark[m1]) continue;
        symm_unique.push_back(mat_vec[m1]);
        bmark[m1] = true;
        for (size_t m2 = m1+1; m2 < num_mat; ++m2) {
            if (bmark[m2]) continue;
            if (NU::IsZeroMatrix(mat_vec[m1] - mat_vec[m2])) {
                bmark[m2] = true;
            }
            if (NU::IsZeroMatrix((mat_vec[m1]*mat_vec[m2])-IDENTITY4F, epsilon, normalizationFactor)) {
                bmark[m2] = true;
            }
        }
        mat_vec.clear();
        std::copy(symm_unique.begin(), symm_unique.end(), std::back_inserter(mat_vec));
    }
    return symm_unique.size();
}

Vector6f NU::MatrixToVec6DF(const Matrix4f& M_)
{
    Vector6f ret;
    ret[0] = M_[3][0];
    ret[1] = M_[3][1];
    ret[2] = M_[3][2];

    // remove scale factor (assume uniform scaling)
    Matrix3f mat3 = shrink4To3(M_);
    float scale = norm(mat3[0]);
    mat3 /= scale;

    float beta = atan2(
        -mat3[2][0],
        sqrt(mat3[0][0]*mat3[0][0] + mat3[1][0]*mat3[1][0])
        );
    float alpha, gamma;
    if (fabs(beta - M_PI/2.0f) < 1e-10f) {
        alpha = 0;
        gamma = atan2(mat3[0][1], mat3[1][1]);
    } else if (fabs(beta + M_PI/2.0f) < 1e-10f) {
        alpha = 0;
        gamma = -atan2(mat3[0][1], mat3[1][1]);
    } else {
        float cos_beta = cos(beta);
        alpha = atan2(
            mat3[1][0] / cos_beta,
            mat3[0][0] / cos_beta);
        gamma = atan2(
            mat3[2][1] / cos_beta,
            mat3[2][2] / cos_beta);
    }

    ret[3] = alpha;
    ret[4] = beta;
    ret[5] = gamma;

    return ret;
}

Matrix4f NU::VecToMatrix6DF(const Vector6f& V_)
{
    float alpha = V_[3];
    float beta = V_[4];
    float gamma = V_[5];
    Matrix3f mat3;

    if (fabs(beta - M_PI/2.0f) < 1e-10f) {
        mat3[0][0] = 0;
        mat3[0][1] = 0;
        mat3[0][2] = -1;

        mat3[1][0] = sin(gamma - alpha);
        mat3[1][1] = cos(gamma - alpha);
        mat3[1][2] = 0;

        mat3[2][0] = cos(gamma - alpha);
        mat3[2][1] = -sin(gamma - alpha);
        mat3[2][2] = 0;
    } else if (fabs(beta + M_PI/2.0f) < 1e-10f) {
        mat3[0][0] = 0;
        mat3[0][1] = 0;
        mat3[0][2] = 1;

        mat3[1][0] = -sin(gamma + alpha);
        mat3[1][1] = cos(gamma + alpha);
        mat3[1][2] = 0;

        mat3[2][0] = -cos(gamma + alpha);
        mat3[2][1] = -sin(gamma + alpha);
        mat3[2][2] = 0;
    } else {
        mat3[0][0] = cos(alpha) * cos(beta);
        mat3[0][1] = sin(alpha) * cos(beta);
        mat3[0][2] = -sin(beta);

        mat3[1][0] = cos(alpha) * sin(beta) * sin(gamma) - sin(alpha) * cos(gamma);
        mat3[1][1] = sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma);
        mat3[1][2] = cos(beta) * sin(gamma);

        mat3[2][0] = cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma);
        mat3[2][1] = sin(alpha) * sin(beta) * cos(gamma) - cos(alpha) * sin(gamma);
        mat3[2][2] = cos(beta) * cos(gamma);
    }

    Matrix4f ret = expand3To4(mat3);
    ret[3][0] = V_[0];
    ret[3][1] = V_[1];
    ret[3][2] = V_[2];

    return ret;
}

bool NU::GenerateFrame(const Vector3f& v1, const Vector3f& v2, Matrix3f& frame)
{
    if (1e-6 > norm(v1) || 1e-6 > norm(v2)) return false;
    if (IsParallel(v1, v2)) return false;
    frame[0] = normalize(v1.crossProduct(v2));
    frame[1] = normalize(v1);
    frame[2] = normalize(frame[0].crossProduct(frame[1]));
    return true;
}

float NU::PointlLineDist(const Vector3f& p, const Vector3f& o, const Vector3f& n)
{
    Vector3f v0 = o - p;
    Vector3f v1 = o + n - p;
    return norm(v0.crossProduct(v1)) / norm(n);
}

bool NU::PointOnPlane(const Vector3f& p, const Vector3f& o, const Vector3f& n)
{
    float d = (p - o) * normalize(n);
    return 1e-6 > d;
}

std::string NU::PrefixRootName(std::string const& name) { return "root/" + name; }
