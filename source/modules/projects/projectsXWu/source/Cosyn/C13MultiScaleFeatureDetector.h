//---------------------------------------------------------------------------
#ifndef C13MultiScaleFeatureDetectorH
#define C13MultiScaleFeatureDetectorH
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
#include "Persistent.h"
//---------------------------------------------------------------------------


namespace NAMESPACE_VERSION {

    class MultiScalePointCloud;


    ///
    ///	Detects Features in MultiScaleFeaturePointClouds
    ///
    class C13MultiScaleFeatureDetector : public Persistent {
        DEFINE_ABSTRACT_CLASS( C13MultiScaleFeatureDetector )
    public:
        /// creates features only at the finest resolution
        static const card32 MSMODE_MIN_LEVEL_ONLY = 0;
        /// creates features in the whole range of resolutions, betweeb minLevel and maxLevel
        static const card32 MSMODE_MIN_TO_MAX_SEPARATE = 1;

    protected:
        /// how to handle features at multiple scales
        card32 multiScaleMode;
        /// multi-resolution level from which the features are taken
        card32 minLevel;
        /// maximum multi-resolution level from which the features are taken
        card32 maxLevel;
        bool debugdraw;


    public:
        C13MultiScaleFeatureDetector();

        virtual void assign(const Object* obj, COPY_CONTEXT *context = nullptr);

        /// takes a MultiScalePointCloud as input
        /// outputs a point cloud with the following attributes (at least):
        ///   position(3f) - feature point position
        ///   flags(1i) - for further processing
        virtual PointCloud *detectFeatures(MultiScalePointCloud *source) = 0;

        card32 getMinLevel() const { return minLevel; }
        void setMinLevel(card32 val) { minLevel = val; }

        card32 getMaxLevel() const { return maxLevel; }
        void setMaxLevel(card32 val) { maxLevel = val; }

        card32 getMultiScaleMode() const { return multiScaleMode; }
        void setMultiScaleMode(card32 val) { multiScaleMode = val; }
    };


    ///
    ///	concrete implementation: take samples at cross point of crease line
    ///
    class C13MSFeatDetectCrossPoint : public C13MultiScaleFeatureDetector {
        DEFINE_CLASS( C13MSFeatDetectCrossPoint )
    private:
        /// minimal magnitude of the principal curvature. Points with less curvature are ignored.
        float32 curvThreshold;
        /// minimal ratio between principal curvature values
        float32 curvRatio;

        float32 msAngleTollerance;
        card32 msMaxNIter;
        float32 intersectionAngle;
        bool use1stPlane;
        float32 cluster_ratio;
        float32 intersect_ratio;
        float32 intersect_ring_ratio;

    public:
        C13MSFeatDetectCrossPoint();

        virtual PointCloud *detectFeatures(MultiScalePointCloud *source);
    };


}

#endif