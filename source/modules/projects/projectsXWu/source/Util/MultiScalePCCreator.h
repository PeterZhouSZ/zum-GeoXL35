//---------------------------------------------------------------------------
#ifndef MultiScalePCCreatorH
#define MultiScalePCCreatorH
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
#include "Persistent.h"
#include "GaussianPointCloudSmoother.h"
#include "MultiScaleInCorePointCloud.h"
//---------------------------------------------------------------------------

class SimpleFeaturePointCloudSmoother : public GaussianPointCloudSmoother {
    DEFINE_CLASS(SimpleFeaturePointCloudSmoother)
public:
    virtual UnstructuredInCorePointCloud *createSmoothed(
        UnstructuredInCorePointCloud *input, 
        float32 currentGridSpacing, 
        float32 smoothingFactor) const;
};

///
///	convert point clouds, meshes to multi-scale point clouds
///   compute gradient / curvature features (tangents/curvature)
///
class MultiScalePCCreator : public Persistent {
    DEFINE_CLASS(MultiScalePCCreator)
public:
    MultiScalePCCreator();
    ~MultiScalePCCreator();


    MultiScaleInCorePointCloud *createPointCloudFromMesh(UnstructuredInCoreTriangleMesh *mesh);
    MultiScaleInCorePointCloud *createPointCloudFromPointCloud(UnstructuredInCorePointCloud *pc);

    MultiScaleInCorePointCloud* convert(PointCloud *pc);
    static void computeCurv2forPC(PointCloud* pc, float base_sigma, float top_ring_factor = 4,
        bool use_top_expansion = true, bool debugdraw = false);

public:
    float grid_spacing_factor;
    float top_ring_factor;
    MultiScalePCParams* params;
    PointCloudSmoother *smoother;
    bool use_top_expansion;
    bool debugdraw;
};

#endif
