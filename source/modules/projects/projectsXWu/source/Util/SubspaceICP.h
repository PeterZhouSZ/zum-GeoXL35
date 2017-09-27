#ifndef SubspaceICP_H
#define SubspaceICP_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "Util\numerical\EigenAdaptor.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class SubspaceICP
{
public:
    typedef boost::shared_ptr< SubspaceICP > Ptr;
    typedef boost::shared_ptr< const SubspaceICP > ConstPtr;

    enum DebugLevel { None, Verbose, Graphical };
    enum AlignStatus { Aligned, NotAligned };
    enum AxisFreedom { Free = 0, Locked = 1 };

public:
    explicit SubspaceICP(UICPC* pc, const Eigen::Matrix4f& worldFrame);
    ~SubspaceICP(void);

    void setupParameters(
        // matching range multiplier, points outside of this range are considered unknown input
        float match_dist_factor = 10,
        // outlier distance multiplier, applied to the point pair-wise mean distance
        float outlier_dist_factor = 4,
        // minimal ratio of support, applied to the number of points
        float min_inlier_ratio = 0.8f,
        //// minimal ratio of matching, applied to the number of points
        //float min_match_ratio = 0.1f,
        // maximal moving distance multiplier, applied to the bounding box diameter
        float max_dist_factor = 0.1f,
        // #1 maximal number of iterations
        unsigned num_iter = 1,
        // #2 residual difference multiplier, applied to the point pair-wise mean distance 
        float residual_difference_factor = 0.01f,
        // #3 transformation difference
        float transformation_difference = 0.001f,
        // debug output level
        DebugLevel visualization = Verbose
        );

public:
    const std::deque<unsigned>& GetInlier(void) const { return inlier_indices_; }
    const Eigen::Matrix4f& GetUpdatedTransformation(void) const;
    const AlignStatus GetAlignStatus(void) const { return alignStatus_; }
    void SetAxisLocked(const unsigned axis);

public:
    float CalculateResidualAfterICP(
        PointSet* startPS, const Eigen::Vector3f& startCenter,
        const Eigen::Matrix4f& originalTransformation
        );

    unsigned FindInliers(
        PointSet* startPS, const Eigen::Matrix4f& originalTransformation
        );

private:
    bool checkAllowedMovement(
        const Eigen::Matrix4f &trans,
        const Eigen::Vector3f& startCenTrans
        );

    float gaussTruncate(const float& value, const float& sigma, const float& epsilon);

    float estimateRigidTransformationLLS(
        PointSet* startPS
        );

    float estimateRigidTransformationSVD(
        PointSet* startPS
        );

    void debugDrawLast(PointSet* startPS);

private:
    float median_point_dist_;
    float match_dist_;
    float outlier_dist_;
    float min_inlier_ratio_;
    //float min_match_ratio_;
    float max_dist_move_;
    unsigned num_iter_;
    float residual_difference_;
    float transformation_difference_;
    DebugLevel visualization_;
    AlignStatus alignStatus_;
    std::array<AxisFreedom, 6> axisFree_;
    float gauss_truncate_;

private:
    std::deque<float> residualVec_;
    std::deque<Eigen::Vector3f> corrSorc_;
    std::deque<Eigen::Vector3f> corrDest_;

    PointSet* PS_;
    PointSetANNQueryPtr KNN_;
    Eigen::Matrix4f updated_trans_;
    Eigen::Matrix4f last_trans_;
    Eigen::Matrix4f world2local_;
    Eigen::Matrix4f local2world_;
    Eigen::Vector3f start_center_;
    std::deque<unsigned> inlier_indices_;
    std::deque<unsigned> outlier_indices_;
    std::deque<unsigned> match_indices_;
};

#endif
