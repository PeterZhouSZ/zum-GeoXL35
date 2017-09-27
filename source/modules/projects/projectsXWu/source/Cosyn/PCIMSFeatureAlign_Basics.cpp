//---------------------------------------------------------------------------
#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "PCIMSFeatureAlign.h"
#include "Util\NoUse.h"
#include "Util\IsometryGrid.h"
#include "Util\FunctionContainer.hpp"
#include "Cosyn\C13DescriptorAttachment.h"
#include "Util/NormalCluster.h"
#include "Util/PlaneCluster.h"
//---------------------------------------------------------------------------
#include "SeparatorClassProperty.h"
#include "SceneGraphTools.h"
#include "ProgressWindow.h"
#include "NormalEstimator.h"
#include "Timer.h"
#include "PCCTriangleMeshSampler.h"
#include "PointSetANNQuery.h"
#include "AnnSearch.h"
#include "MultiScaleInCorePointCloud.h"
#include "InCorePCTopologyGraph.h"
#include "PointCloudTopologyGraph.h"
#include "TopologyAttachment.h"
#include "FSIcp.h"
#include "FastSphereQuerry.h"
#include "LineFeature.h"
#include "LBase.h"
#include "LBaseSet.h"
#include "PCCComputeTopology.h"
#include "TopologyRangeSearch.h"

#include "PCCBoundaryDetector.h"
#include "PointToLineAlignment.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 1;
}

IMPLEMENT_CLASS( PCIMSFeatureAlignSettings , 0 )
{
    BEGIN_CLASS_INIT( PCIMSFeatureAlignSettings );

    ADD_SEPARATOR("ICL - ICP");
    ADD_BOOLEAN_PROP(show_icl_icp, 0);
    ADD_BOOLEAN_PROP(use_region_growing, 0);
    ADD_FLOAT32_PROP(icl_min_score, 0);
    ADD_FLOAT32_PROP(icp_init_radius_radio, 0);
    ADD_FLOAT32_PROP(icl_dist_ratio, 0);
    ADD_FLOAT32_PROP(support_ratio, 0);
    ADD_FLOAT32_PROP(outlier_dist_factor, 0);

    ADD_SEPARATOR("General");
    ADD_CARD32_PROP(knnTopology, 0);
}

IMPLEMENT_CLASS( PCIMSFeatureAlign , 0 )
{
    BEGIN_CLASS_INIT( PCIMSFeatureAlign );

    ADD_OBJECT_PROP(settings_, 0, PCIMSFeatureAlignSettings::getClass(), true);
    ADD_OBJECT_PROP(multiScalePCCreator, 0, MultiScalePCCreator::getClass(), true);
    ADD_OBJECT_PROP(featureDetector, 0, C13MultiScaleFeatureDetector::getClass(), true);
    ADD_OBJECT_PROP(featureDescriptor, 0, C13MultiScaleFeatureDescriptor::getClass(), true);

    //ADD_SEPARATOR("Directory");
    //ADD_STRING_PROP(dataFolder, 0);
    //ADD_STRING_PROP(muscFolder, 0);
    //ADD_STRING_PROP(featFolder, 0);
    //ADD_STRING_PROP(crosFolder, 0);

    //ADD_SEPARATOR("General");
    //ADD_CARD32_PROP(minLevel, 0);
    //ADD_CARD32_PROP(maxLevel, 0);

    ADD_SEPARATOR_METHOD("single");
    ADD_NOARGS_METHOD(createFolders);
    ADD_NOARGS_METHOD(convert2MSPC);
    ADD_NOARGS_METHOD(alignLBase);
    ADD_NOARGS_METHOD(geometricValidation);

    ADD_SEPARATOR_METHOD("execute");
    ADD_NOARGS_METHOD(execute);
    ADD_NOARGS_METHOD( CoarseSegmentation );
    ADD_NOARGS_METHOD(detectFeatures);
    //ADD_NOARGS_METHOD(extractSupports);
    ADD_NOARGS_METHOD(extractCrossPoints);
    //ADD_NOARGS_METHOD(detectSalientLines);
    ADD_NOARGS_METHOD(computeDescriptors);
    ADD_NOARGS_METHOD(computeCrossPointTpg);
    ADD_NOARGS_METHOD(CleanTransformations);

    ADD_SEPARATOR_METHOD("Debug");
    //ADD_NOARGS_METHOD(showBoundaryLines);
    ADD_NOARGS_METHOD(showPlanes);
    ADD_NOARGS_METHOD(showFeatureLineClusters);
    ADD_NOARGS_METHOD(showLBases);
    ADD_NOARGS_METHOD(showLBaseAlignment);
}

PCIMSFeatureAlignSettings::PCIMSFeatureAlignSettings()
{
    knnTopology = 50;

    show_icl_icp = true;
    use_region_growing = true;
    icl_dist_ratio = 0.005f;
    icl_min_score = 60;
    support_ratio = 0.01f;
    icp_init_radius_radio = 0.05f;
    outlier_dist_factor = 4; // note: topological expansion radius also depends on this
}

PCIMSFeatureAlignSettings::~PCIMSFeatureAlignSettings()
{
}

MultiScaleInCorePointCloud* PCIMSFeatureAlign::getMSPCFromListNode(SGListNode* node, const unsigned& id)
{
    SGObjectNode *objN = dynamic_cast<SGObjectNode*>(node->getChildNode(nullptr, id));
    if (!objN) return nullptr;
    SceneObject *sceneObj = objN->getSceneObject();
    if (!sceneObj) return nullptr;
    MultiScaleInCorePointCloud *featureMSPC = dynamic_cast<MultiScaleInCorePointCloud*>(sceneObj);
    return featureMSPC;
}

PCIMSFeatureAlign::PCIMSFeatureAlign()
{
    dataFolder = "root/data";
    muscFolder = "root/multiScale";
    featFolder = "root/msFeatures";
    crosFolder = "root/crossPoints";

    minLevel = 0;
    maxLevel = 2;

    settings_ = new PCIMSFeatureAlignSettings;
    multiScalePCCreator = new MultiScalePCCreator;
    featureDetector = new C13MSFeatDetectCrossPoint();
    featureDescriptor = new C13MSFeatDescrHOG();	
}

PCIMSFeatureAlign::~PCIMSFeatureAlign()
{
    delete settings_;
    delete multiScalePCCreator;
    delete featureDetector;
    delete featureDescriptor;
}

void PCIMSFeatureAlign::createFolders()
{
    {
        SGRelativeTimeAnimationNode *folder = new SGRelativeTimeAnimationNode();
        folder->setName("data");
        addSceneGraphNode(getScene(), "root", folder);
    }
    {
        SGRelativeTimeAnimationNode *folder = new SGRelativeTimeAnimationNode();
        folder->setName("multiScale");
        addSceneGraphNode(getScene(), "root", folder);
    }
    {
        SGRelativeTimeAnimationNode *folder = new SGRelativeTimeAnimationNode();
        folder->setName("msFeatures");
        addSceneGraphNode(getScene(), "root", folder);
    }
    {
        SGRelativeTimeAnimationNode *folder = new SGRelativeTimeAnimationNode();
        folder->setName("crossPoints");
        addSceneGraphNode(getScene(), "root", folder);
    }
}
