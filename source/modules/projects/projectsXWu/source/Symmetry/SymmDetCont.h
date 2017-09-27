#ifndef SymmDetCont_H
#define SymmDetCont_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "Symmetry/SymmFeat.h"
#include "Util/TrimeshStatic.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class PROJECTSXWU_API SymmDetCont
{
public:
    typedef boost::shared_ptr< SymmDetCont > Ptr;
    typedef boost::shared_ptr< const SymmDetCont > ConstPtr;

    SymmDetCont();
    void Clear(void);

public:
    float diagonal_length;
    float relative_spatial_tolerance;
    float spatial_tolerance;

    float relative_grid_size;
    float grid_size;

    float angle_tolerance;
    float sameline_th;

    unsigned num_sample_th;
    unsigned num_iter;
    unsigned num_sample;
    unsigned num_sample_mult;

    float min_feat_length;
    float coplanarity_size;
    float coplanarity_angle;

public:
    TrimeshStatic::Ptr smesh;

    std::vector<SymmFeatLine::Ptr> featArr;
    std::vector<SymmFeatSetLine::Ptr> equivalentLines;
    std::vector<unsigned> lineClassID;

    HierarchicalKNNIteratorPtr hItFeatures;
    UnstructuredInCorePointCloudPtr lineFeatureUPC;
};

class PROJECTSXWU_API SymmDetContCCSG : public AttachedData
{
    DEFINE_CLASS(SymmDetContCCSG);
public:
    typedef boost::shared_ptr< SymmDetContCCSG > Ptr;
    typedef boost::shared_ptr< const SymmDetContCCSG > ConstPtr;
    SymmDetContCCSG();
    ~SymmDetContCCSG();

    static const char* getDefaultName() {return "ConsistentCrossShapeGrammarSettings";}

    int numCPUCores;

    float32 adjMaxDistance;
    float32 descriptorRadiusBBFactor;

    float32 proximityRelationFactor;

    float32 shapeSymmThreshold;

    int32 maxIcpIterations;
    float32 icpOutlierDistanceFactor;
    float32 icpMinPercentInliers;
    card32 minConnectedComponent;
    float32 distCompareSubpartThreshold;
    float32 symGroupCompareThreshold;
    float32 smallPartMinimalSizeFactor;

    int getNumAvailableThreads();

    float32 latticeMinGeneratorLength;
    float32 spatialTolerance;
    float32 angleTolerance;
    float32 detectorAngleTolerance;
    float32 symmetryCoveredPercentage;
    float32 symMeshSubsampling;
    float32 gridAngleTolerance;

    float32 rotGroupCompareSigma;
    float32 refGroupCompareSigma;
    float32 latGroupCompareSigma;
    float32 dihGroupCompareSigma;
    float32 angelGroupGroupCompareSigma;

    float32 minFeatureLineLength;

    float32 symGroupCombineAngleTolerance;

    bool useICPWhileDetectingBaseElements;
    bool reuseSegmentation;
    bool searchLattice;
    bool runPreliminaryPlausibilityCheck;
    bool clusterFoundGroupsBeforeExtractGeometry;
    bool performTopologyCheckOnFoundGroup;

    card32 minNumRotations;

    Vector4f gidSelectedColor;
    Vector4f gidSymmetricColor;
    Vector4f gidDefaultColor;
    Vector4f gidNonrelevantColor;

    //int32 embeddingDimensions;
    float32 embeddingCutoffEigenvalue;

    bool bDrawFeatureLines;
    float32 featureLineSize;
    bool bShowCluster;
};

#endif
