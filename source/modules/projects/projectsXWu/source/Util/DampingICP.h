#ifndef DampingICP_H
#define DampingICP_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class DampingICP
{
public:
    DampingICP(PointCloud* pc);
    ~DampingICP(void);

    void setupParameters(
        unsigned max_num_icp = 10,
        float median_point_dist = 0.01f,
        float outlier_dist_factor = 10,
        float min_inlier_ratio =0.8f,
        float max_allowed_dist= 0.1f,
        float residual_tolerance= 0.01f,
        float converge_difference = 0.001f,
        bool visualization= true
        );

    ///returns remaining residual after ICP Alignment
    /// returns 1e20 if: moved too far or max steps reached and not converged
    /// if converged, overwrites parameter Matrix4f transformation 
    float calculateResidualAfterICP(
        PointSet* startPS, Matrix4f& originalTransformation,
        std::deque<unsigned>& inlier_indices,
        bool final_snap = false
        );

    float calculateResidualAfterICP(
        PointSet* startPS, Matrix4f& originalTransformation
        );

    void getClosestPointAndNormal( const Vector3f& transformedPos, Vector3f &closestPoint, Vector3f &closestPointNormal ) const;

private:
    bool checkAllowedMovement(const Matrix4f &trans);

private:

    unsigned  max_num_icp;
    float median_point_dist;
    float outlier_dist_factor;
    float min_inlier_ratio;
    float max_allowed_dist;
    float residual_tolerance;
    float converge_difference;
    bool    visualization;

    HierarchicalKNNIterator* hIt;
    AAT POSITION;
    AAT NORMAL;
    Matrix4f betterTransformation;
    Vector3f startPScenter;
    Vector3f startPScenterOrgTransformed;
};

#endif
