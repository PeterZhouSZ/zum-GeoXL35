//---------------------------------------------------------------------------
#ifndef C13MultiScaleFeatureDescriptorH
#define C13MultiScaleFeatureDescriptorH
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
#include "Persistent.h"
//---------------------------------------------------------------------------


namespace NAMESPACE_VERSION {

    class MultiScalePointCloud;
    class FastSphereQuerry;
    class UnstructuredInCorePointCloud;
    class PointSet;
    class PointSetKNNQuery;
    class C13DescriptorAttachment;


    ///
    ///	Detects Features in MultiScaleFeaturePointClouds
    ///
    class C13MultiScaleFeatureDescriptor : public Persistent {
        DEFINE_ABSTRACT_CLASS( C13MultiScaleFeatureDescriptor )
    private:
    public:
        C13MultiScaleFeatureDescriptor() {}

        virtual void assign(const Object* obj, COPY_CONTEXT *context = nullptr);

        virtual void computeDescriptors(MultiScalePointCloud *geometry, PointCloud *featurePoints) = 0;
    };



    ///
    ///	concrete implementation: just resamples point cloud uniformly
    ///
    class C13MSFeatDescrHOG : public C13MultiScaleFeatureDescriptor {
        DEFINE_CLASS( C13MSFeatDescrHOG )
    public:
        /// multi-scale behavior: fixed bin layout, covering larger regions on coarser ms-levels
        static const card32 MSMODE_FIXED_NUMBINS_EXPANDING_SUPPORT = 0;	
        /// multi-scale behavior: fixed spatial support, using fewer bins on coarser ms-levels
        static const card32 MSMODE_FIXED_SUPPORT_SHRINKING_NUMBINS = 1;	
        /// multi-scale behavior: independent descriptors at different scales
        static const card32 MSMODE_INDEPENDENT_DESCRIPTORS_PER_LEVEL = 2;	

    private:
        /// operation mode w.r.t. multiple scales (see constants above).
        card32 multiScaleMode;
        /// subdivisions of the [0?..180°] range
        card32 numOrientationBins;
        /// subdivisions of the [-radius..+radius] range in the tangent plane
        card32 numSpatialBins;
        /// how many bins are added in the vertical (normal) direction; bins are cubes (side length 2*radius/numSpatialBins)
        card32 numVerticalBins;
        /// relative smoothing of Gaussian filters employed (default = 1.0)
        float32 smoothness;
        /// canonical upward direction
        Vector3f canonicalUpward;
        /// use canonical upward direction?
        bool useCanonicalUpward;
        /// normalize to mean of curvature magnitudes (default = true)
        bool normalizeDescriptor;
        /// dimension for used for PCA compression
        card32 pcaDimension;
        /// output some visualization on debug renderer?
        bool debugVis;
        /// Build histograms over curvature instead of normals
        bool useCurvature;
        /// use diffusion std::map
        bool useDiffusion;
        /// diffusion delta
        float32 diffusionDelta;

    public:
        C13MSFeatDescrHOG(const bool useCurvature = true); // If not curvature: uses normals

        virtual void computeDescriptors(MultiScalePointCloud *geometry, PointCloud *featurePoints);

        card32 getNumOrientationBins() const {return numOrientationBins;}
        void setNumOrientationBins(card32 val) {numOrientationBins = val;}
        card32 getNumSpatialBins() const {return numSpatialBins;}
        void setNumSpatialBins(card32 val) {numSpatialBins = val;}
        card32 getNumVerticalBins() const {return numVerticalBins;}
        void setNumVerticalBins(card32 val) {numVerticalBins = val;}
        float32 getSmoothness() const {return smoothness;}
        void setSmoothness(float32 val) {smoothness = val;}
        bool getNormalizeDescriptor() const {return normalizeDescriptor;}
        void setNormalizeDescriptor(bool val) {normalizeDescriptor = val;}
        bool getDebugVis() const { return debugVis; }
        void setDebugVis(bool val) { debugVis = val; }
        Vector3f getCanonicalUpward() const { return canonicalUpward; }
        void setCanonicalUpward(Vector3f val) { canonicalUpward = val; }
        bool getUseCanonicalUpward() const { return useCanonicalUpward; }
        void setUseCanonicalUpward(bool val) { useCanonicalUpward = val; }
        card32 getPcaDimension() const { return pcaDimension; }
        void setPcaDimension(card32 val) { pcaDimension = val; }

        ~C13MSFeatDescrHOG();

        // --- helper class
    private:
        class SingleScaleDetector {
        private:
            FastSphereQuerry *query;
            AttachedIndexedOctree *knnOctree;
            PointSetKNNQuery *knn;
            UnstructuredInCorePointCloud *upc;
            PointSet *ps;
            AAT positionAAT;
            AAT curv2AAT;
            AAT tangentUAAT;
            AAT normalAAT;
            bool singleWarning;
        public:	
            SingleScaleDetector(UnstructuredInCorePointCloud *upc);
            void getDescriptor(
                const Vector3f& pos, const Vector3f& normal, const Vector3f &tangentU,  float32 radius, 
                DVectorF &result, float32 &localRadius,
                card32 numSpatialBins, card32 numVerticalBins, card32 numOrientationBins,
                bool normalizeDescriptor, float32 smoothness,
                bool debugVis = false,
                bool useCurvature = true);
            /// does what it says; index must be the index of center position
            Vector3f getAverageNormal(const Vector3f &pos, const float32 fittingRadius);
            ~SingleScaleDetector();
        };
        struct MSHOGLevelInfo {
            UnstructuredInCorePointCloud *upc;
            SingleScaleDetector *detector;
        };	

        void computeDescriptors(
            const std::vector<MSHOGLevelInfo>& levelInfos, PointSet *featurePS, C13DescriptorAttachment* att,
            mpcard descrDim, mpcard descrStorageDim);
    };


}

#endif