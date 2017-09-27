#include "StdAfx.h"
#include <QtCore/QtCore>
//---------------------------------------------------------------------------
#include "Symmetry/SymmDetCont.h"
//---------------------------------------------------------------------------
#include "PropertyTableProperty.h"
#include "SeparatorClassProperty.h"
//---------------------------------------------------------------------------

SymmDetCont::SymmDetCont()
{
    diagonal_length = 1e0;
    relative_spatial_tolerance = 5e-4;
    spatial_tolerance = 1e-6;
    relative_grid_size = 5e-3;
    grid_size = 1e-6;
    angle_tolerance = 1e-3;
    sameline_th = 2 * grid_size;
    num_sample_th = 1e5;
    num_iter = 1e5;
    num_sample = 1000;
    num_sample_mult = 4;
    min_feat_length = 10 * grid_size;
    coplanarity_size = 0.01f;
    coplanarity_angle = 5.f;
}

void SymmDetCont::Clear(void)
{
    featArr.clear();
    equivalentLines.clear();
    lineClassID.clear();
}

IMPLEMENT_CLASS(SymmDetContCCSG, 26)
{
    BEGIN_CLASS_INIT(SymmDetContCCSG);
    INIT_PROPERTY_TABLE();

    ADD_SEPARATOR("Settings");
    ADD_INT32_PROP(numCPUCores, 0);
    ADD_FLOAT32_PROP(adjMaxDistance, 1);
    ADD_VECTOR4F_PROP(gidDefaultColor, 4);
    ADD_VECTOR4F_PROP(gidSelectedColor, 4);
    ADD_VECTOR4F_PROP(gidSymmetricColor, 4);
    ADD_VECTOR4F_PROP(gidNonrelevantColor, 4);

    ADD_SEPARATOR("Matching");
    ADD_FLOAT32_PROP(descriptorRadiusBBFactor, 2);
    ADD_FLOAT32_PROP(shapeSymmThreshold, 2);
    ADD_FLOAT32_PROP(smallPartMinimalSizeFactor, 12);

    ADD_SEPARATOR("ICP");
    ADD_INT32_PROP(maxIcpIterations, 2);
    ADD_FLOAT32_PROP(icpOutlierDistanceFactor, 2);
    ADD_FLOAT32_PROP(icpMinPercentInliers, 2);

    ADD_SEPARATOR("Feature Lines");
    ADD_FLOAT32_PROP(detectorAngleTolerance, 5);
    ADD_FLOAT32_PROP(minFeatureLineLength, 19);

    ADD_SEPARATOR("Symmetries");
    ADD_FLOAT32_PROP(symMeshSubsampling, 8);
    ADD_FLOAT32_PROP(spatialTolerance, 3);
    ADD_FLOAT32_PROP(angleTolerance, 3);
    ADD_FLOAT32_PROP(symGroupCombineAngleTolerance, 16);
    ADD_FLOAT32_PROP(symmetryCoveredPercentage, 6);
    ADD_CARD32_PROP(minConnectedComponent, 7);
    ADD_BOOLEAN_PROP(useICPWhileDetectingBaseElements, 25);

    ADD_SEPARATOR("Comparing");
    ADD_FLOAT32_PROP(symGroupCompareThreshold, 10);
    ADD_FLOAT32_PROP(rotGroupCompareSigma, 17);
    ADD_FLOAT32_PROP(latGroupCompareSigma, 22);

    ADD_FLOAT32_PROP(refGroupCompareSigma, 22);
    SET_USER_EDIT(false);
    ADD_FLOAT32_PROP(dihGroupCompareSigma, 22);
    SET_USER_EDIT(false);

    ADD_FLOAT32_PROP(angelGroupGroupCompareSigma, 23);
    ADD_FLOAT32_PROP(distCompareSubpartThreshold, 9);
    SET_USER_EDIT(false);

    ADD_SEPARATOR("Graph");
    ADD_FLOAT32_PROP(proximityRelationFactor, 14);
    ADD_FLOAT32_PROP(embeddingCutoffEigenvalue, 26);
    //ADD_INT32_PROP(embeddingDimensions, 25);

    ADD_SEPARATOR("Stuff");
    ADD_CARD32_PROP(minNumRotations,20);
    ADD_FLOAT32_PROP(gridAngleTolerance, 11);
    ADD_FLOAT32_PROP(latticeMinGeneratorLength, 24);
    ADD_BOOLEAN_PROP(reuseSegmentation, 13);
    ADD_BOOLEAN_PROP(searchLattice, 15);
    ADD_BOOLEAN_PROP(runPreliminaryPlausibilityCheck, 18);
    SET_USER_EDIT(false);

    ADD_BOOLEAN_PROP(clusterFoundGroupsBeforeExtractGeometry, 20);
    ADD_BOOLEAN_PROP(performTopologyCheckOnFoundGroup, 21);

    ADD_BOOLEAN_PROP(bDrawFeatureLines, 0);
    ADD_FLOAT32_PROP(featureLineSize, 0);
    ADD_BOOLEAN_PROP(bShowCluster, 0);
}

//--------------------------------------------------------------------------------------
SymmDetContCCSG::SymmDetContCCSG()
{
    numCPUCores = -1;
    adjMaxDistance = 0.01f;
    descriptorRadiusBBFactor = 0.1f;
    gidSelectedColor = makeVector4f(1.0f, 0.0f, 0.0f, 1.0f);
    gidDefaultColor = makeVector4f(0.9f, 0.9f, 0.7f, 1.0f);
    gidSymmetricColor = makeVector4f(0.0f, 0.6f, 0.0f, 1.0f);
    gidNonrelevantColor = makeVector4f(0.6f, 0.6f, 0.4f, 0.25f);

    proximityRelationFactor = 5.0f;
    searchLattice = false;


    runPreliminaryPlausibilityCheck = false;
    performTopologyCheckOnFoundGroup = true;

    minFeatureLineLength = 0.03f;
    minNumRotations = 2;
    clusterFoundGroupsBeforeExtractGeometry = true;

    maxIcpIterations = 64;
    icpOutlierDistanceFactor = 3;
    icpMinPercentInliers = 0.4f;
    shapeSymmThreshold = 0.05f;
    distCompareSubpartThreshold = 0.075f;
    smallPartMinimalSizeFactor = 0.02f;
    useICPWhileDetectingBaseElements = false;

    symMeshSubsampling = 0.007f;
    spatialTolerance = 0.03f;
    angleTolerance = 0.075f;
    symGroupCombineAngleTolerance = 0.01f;

    gridAngleTolerance = 0.05f;
    latticeMinGeneratorLength = 0.1f;

    reuseSegmentation = true;

    detectorAngleTolerance = 0.9f; 
    symmetryCoveredPercentage = 0.9f;

    minConnectedComponent = 500;
    symGroupCompareThreshold = 0.5f;


    rotGroupCompareSigma = 0.15f;
    refGroupCompareSigma = 0.5f;
    latGroupCompareSigma = 0.3f;
    dihGroupCompareSigma = 0.5f;

    angelGroupGroupCompareSigma = 0.15f;

    //embeddingDimensions = -1;
    embeddingCutoffEigenvalue = 10.0f;

    bDrawFeatureLines = false;
    featureLineSize = 2.f;
    bShowCluster = false;
}

// -------------------------------------------------------------------------------
int SymmDetContCCSG::getNumAvailableThreads()
{
    int numthreads = QThread::idealThreadCount() + numCPUCores;
    if (numCPUCores > 0) 
        numthreads = numCPUCores;

    if (numthreads > QThread::idealThreadCount()) numthreads = QThread::idealThreadCount();
    if (numthreads <= 0) numthreads = 1;

    return numthreads;
}
//--------------------------------------------------------------------------------------
SymmDetContCCSG::~SymmDetContCCSG()
{
}
