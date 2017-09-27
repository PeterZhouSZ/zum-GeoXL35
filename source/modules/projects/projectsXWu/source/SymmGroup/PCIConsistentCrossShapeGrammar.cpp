#include "StdAfx.h"
#include <QtCore/QtCore>

#include "PCIConsistentCrossShapeGrammar.h"

#include <CopyObjectProperties.h>

#include <SGObjectNode.h>
#include <SGListNode.h>
#include <UnstructuredInCorePointCloudManifold.h>
#include <UnstructuredInCoreTriangleMeshManifold.h>
#include <Ransac.h>
#include <ObjectClassProperty.h>
#include "GLMaterial.h"
#include "GLImage.h"
#include "GLRenderTextureCache.h"
#include "GLHelper.h"
#include "GLContext.h"
#include "ProgressWindow.h"
#include <HighQualityRenderer.h>
#include <SlippageFeatureDetector.h>
#include <ManifoldCorrespondences.h>
#include <LandmarkCoordinate.h>
#include "RegularInCoreVolume.h"
#include "RegularVolume.h"
#ifdef XGRT_3_0
#include "RegularVolume_templCode.h"
#elif GEOXL_3_5
#include "RegularVolume.inline.h"
#endif


#include <PCCSmoothing.h>
#include "Random.h"
#include "MeshExporter.h"
#include "PointCloudExporter.h"

#include "SceneEditorWidget.h"
#include "ColorTools.h"
#include "PickFromPointSet.h"
#include "SGRelativeTimeAnimationNode.h"
#include <GeometricTools.h>
#include "TopologyAttachment.h"
#include "PCCResampler.h"
#include "SeparatorClassProperty.h"
#include "FastSphereQuerry.h"
#include "VertexArray.h"
#include "PropertyTableProperty.h"

#include "Statistics.h"

#include "AttachedFloatSignatures.h"
#include <PCCResampleByCurvature.h>
#include <SpinImageDescriptor.h>
#include <SimpleAttachment.h>
#include <PCAGeneral.h>
#include <FSIcp.h>

#include <FileDialogs.h>
#include <InterfaceGlobals.h>
#include <detector/FeatureLine.h>
#include <group/Lattice.h>
#include <group/Rotation.h>
#include <group/Reflection.h>
#include <group/Dihedral.h>
#include <detector/CreaseLineFeatureDetector.h>
#include <DVectorObject.h>
#include <RigidMotion.h>
#include <PCCComputeTopology.h>

#include <ImportanceSampling.h>
#include <GeometricTools.h>
#include <GLShaderMaterial.h>
#include <FileDialogs.h>

#include <TopologicalKNNIterator.h>
#include <PCCTriangleMeshSampler.h>
#include <InverseDynamicKMedianClustering.h>

#include <ctime>
#include <omp.h>
#include <fstream>
#include <qthread.h>
#include <bitset>


#include <MultiDimensionalScaling.h>
#ifdef XGRT_3_0
#include <MultiDimensionalScaling_templCode.h>
#elif GEOXL_3_5
#include <MultiDimensionalScaling.inline.h>
#endif
#include <SGTransformationNode.h>
//#include "MgcEigen.h"

namespace {
    int DR_FRAME = 0;
}

//--------------------------------------------------------------------------------------
IMPLEMENT_CLASS(ConsistentCrossShapeGrammarSettings, 27)
{
    BEGIN_CLASS_INIT(ConsistentCrossShapeGrammarSettings);
    INIT_PROPERTY_TABLE();

    ADD_SEPARATOR("Settings");
    ADD_INT32_PROP(numCPUCores, 0);
    ADD_FLOAT32_PROP(adjMaxDistance, 1);
    SET_USER_EDIT(false);
    ADD_STRING_PROP(shapeDBPath, 0);
    SET_USER_EDIT(false);

    ADD_VECTOR4F_PROP(gidDefaultColor, 4);
    SET_USER_EDIT(false);
    ADD_VECTOR4F_PROP(gidSelectedColor, 4);
    SET_USER_EDIT(false);
    ADD_VECTOR4F_PROP(gidSymmetricColor, 4);
    SET_USER_EDIT(false);
    ADD_VECTOR4F_PROP(gidNonrelevantColor, 4);
    SET_USER_EDIT(false);

    ADD_SEPARATOR("Matching");
    SET_USER_EDIT(false);
    ADD_FLOAT32_PROP(descriptorRadiusBBFactor, 2);
    SET_USER_EDIT(false);
    ADD_FLOAT32_PROP(shapeSymmThreshold, 2);
    SET_USER_EDIT(false);
    ADD_FLOAT32_PROP(smallPartMinimalSizeFactor, 12);
    SET_USER_EDIT(false);

    ADD_SEPARATOR("ICP");
    SET_USER_EDIT(false);
    ADD_INT32_PROP(maxIcpIterations, 2);
    SET_USER_EDIT(false);
    ADD_FLOAT32_PROP(icpOutlierDistanceFactor, 2);
    SET_USER_EDIT(false);
    ADD_FLOAT32_PROP(icpMinPercentInliers, 2);
    SET_USER_EDIT(false);

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
    SET_USER_EDIT(false);

    ADD_BOOLEAN_PROP(performTopologyCheckOnFoundGroup, 21);
    SET_USER_EDIT(false);


    ADD_BOOLEAN_PROP(doVeryConservativeRegionGrowing, 27);
}

//--------------------------------------------------------------------------------------
ConsistentCrossShapeGrammarSettings::ConsistentCrossShapeGrammarSettings()
{
    numCPUCores = -1;
    shapeDBPath = "D:\\tevs\\Stanford\\princeton_benchmark\\db";
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

    symMeshSubsampling = 0.03f;
    spatialTolerance = 0.03f;
    angleTolerance = 0.075f;
    symGroupCombineAngleTolerance = 0.02f;

    gridAngleTolerance = 0.05f;
    latticeMinGeneratorLength = 0.07f;

    reuseSegmentation = true;

    detectorAngleTolerance = 0.9f; 
    symmetryCoveredPercentage = 0.8f;

    minConnectedComponent = 500;
    symGroupCompareThreshold = 0.5f;


    rotGroupCompareSigma = 0.15f;
    refGroupCompareSigma = 0.5f;
    latGroupCompareSigma = 0.3f;
    dihGroupCompareSigma = 0.5f;

    angelGroupGroupCompareSigma = 0.15f;

    doVeryConservativeRegionGrowing = false;

    //embeddingDimensions = -1;
    embeddingCutoffEigenvalue = 10.0f;
}

// -------------------------------------------------------------------------------
int ConsistentCrossShapeGrammarSettings::getNumAvailableThreads()
{
    int numthreads = QThread::idealThreadCount() + numCPUCores;
    if (numCPUCores > 0) 
        numthreads = numCPUCores;

    if (numthreads > QThread::idealThreadCount()) numthreads = QThread::idealThreadCount();
    if (numthreads <= 0) numthreads = 1;

    return numthreads;
}
//--------------------------------------------------------------------------------------
ConsistentCrossShapeGrammarSettings::~ConsistentCrossShapeGrammarSettings()
{
}


//--------------------------------------------------------------------------------------
IMPLEMENT_CLASS(CCSGClassification, 4)
{
    BEGIN_CLASS_INIT(CCSGClassification);
    INIT_PROPERTY_TABLE();

    ADD_OBJECT_LIST_PROP(Group, 1, CCSGSymmetryGroup::getClass());
    ADD_OBJECT_LIST_PROP(symGroup, 2, sym::SymmetryGroupAttachment::getClass());

    ADD_STRING_PROP(loggedOutput, 3);
    ADD_OBJECT_PROP(stats, 3, Statistics::getClass(), true);
}
//--------------------------------------------------------------------------------------
CCSGClassification::CCSGClassification()
{
    stats = new Statistics;
}

//--------------------------------------------------------------------------------------
CCSGClassification::~CCSGClassification()
{
    if (stats) delete stats;
    for (unsigned i=0; i < m_Group.size(); i++) if (m_Group[i]) delete m_Group[i];
    for (unsigned i=0; i < m_symGroup.size(); i++) if (m_symGroup[i]) delete m_symGroup[i];
}

//--------------------------------------------------------------------------------------
void CCSGClassification::write(OutputObjectStream *s) const
{
    AttachedData::write(s);

    //shape std::map
    card32 size = (card32)shapeMap.size();
    s->write<card32>(size);
    for (std::map<std::string, std::vector<int> >::const_iterator it = shapeMap.begin(); it != shapeMap.end(); it++)
    {
        s->writeString(it->first);
        card32 subsize = (card32)it->second.size();
        s->write<card32>(subsize);
        for (std::vector<int>::const_iterator jt = it->second.begin(); jt != it->second.end(); jt ++)
            s->write<int32>(*jt);
    }

    // symmetryT
    size = (card32)symmetryT.size();
    s->write<card32>(size);
    for (std::map<unsigned, std::vector<Matrix4f> >::const_iterator it = symmetryT.begin(); it != symmetryT.end(); it++)
    {
        s->write<card32>(it->first);
        card32 subsize = (card32)it->second.size();
        s->write<card32>(subsize);
        for (std::vector<Matrix4f>::const_iterator jt = it->second.begin(); jt != it->second.end(); jt ++)
            s->writeBuffer(jt->data(), 16 * sizeof(float32));
    }

    // symmetryid
    size = (card32)symmetryId.size();
    s->write<card32>(size);
    for (std::map<unsigned, std::vector<unsigned> >::const_iterator it = symmetryId.begin(); it != symmetryId.end(); it++)
    {
        s->write<card32>(it->first);
        card32 subsize = (card32)it->second.size();
        s->write<card32>(subsize);
        for (std::vector<unsigned>::const_iterator jt = it->second.begin(); jt != it->second.end(); jt ++)
            s->write<card32>(*jt);
    }

    // regions, i.e. disjoint regions hashed over all symmetries
    size = (card32)regions.size();
    s->write<card32>(size);
    for (std::map<unsigned, std::vector<unsigned> >::const_iterator it = regions.begin(); it != regions.end(); it++)
    {
        s->write<card32>(it->first);
        card32 subsize = (card32)it->second.size();
        s->write<card32>(subsize);
        for (std::vector<unsigned>::const_iterator jt = it->second.begin(); jt != it->second.end(); jt ++)
            s->write<card32>(*jt);
    }
    for (std::map<unsigned, std::vector<unsigned> >::const_iterator it = regionGroupsId.begin(); it != regionGroupsId.end(); it++)
    {
        s->write<card32>(it->first);
        card32 subsize = (card32)it->second.size();
        s->write<card32>(subsize);
        for (std::vector<unsigned>::const_iterator jt = it->second.begin(); jt != it->second.end(); jt ++)
            s->write<card32>(*jt);
    }


}

//--------------------------------------------------------------------------------------
void CCSGClassification::read(InputObjectStream *s)
{
    AttachedData::read(s);

    //shape std::map
    card32 size = 0;
    s->read<card32>(size);
    for (unsigned i=0; i < size; i++)
    {
        std::string id;
        s->readString(id);
        card32 subsize = 0;
        s->read<card32>(subsize);
        for (unsigned j=0; j < subsize; j++)
        {
            int32 k = 0;
            s->read<int32>(k);
            shapeMap[id].push_back(k);
        }
    }

    // symmetryT
    s->read<card32>(size);
    for (unsigned i=0; i < size; i++)
    {
        card32 id;
        s->read<card32>(id);
        card32 subsize = 0;
        s->read<card32>(subsize);
        for (unsigned j=0; j < subsize; j++)
        {
            Matrix4f T;
            s->readBuffer(T.data(), 16 * sizeof(float32));
            symmetryT[id].push_back(T);
        }
    }

    // symmetryid
    s->read<card32>(size);
    for (unsigned i=0; i < size; i++)
    {
        card32 id;
        s->read<card32>(id);
        card32 subsize = 0;
        s->read<card32>(subsize);
        for (unsigned j=0; j < subsize; j++)
        {
            card32 k=0;
            s->read<card32>(k);
            symmetryId[id].push_back(k);
        }
    }

    // regions did came after version 4
    if (s->getClassVersion(CCSGClassification::getClass()) >= MetaClass::FIRST_VERSION + 4)
    {
        regions.clear();
        regionGroupsId.clear();

        s->read<card32>(size);
        for (unsigned i=0; i < size; i++)
        {
            card32 id;
            s->read<card32>(id);
            card32 subsize = 0;
            s->read<card32>(subsize);
            for (unsigned j=0; j < subsize; j++)
            {
                card32 k=0;
                s->read<card32>(k);
                regions[id].push_back(k);
            }
        }
        for (unsigned i=0; i < size; i++)
        {
            card32 id;
            s->read<card32>(id);
            card32 subsize = 0;
            s->read<card32>(subsize);
            for (unsigned j=0; j < subsize; j++)
            {
                card32 k=0;
                s->read<card32>(k);
                regionGroupsId[id].push_back(k);
            }
        }

    }
}

//--------------------------------------------------------------------------------------
IMPLEMENT_CLASS(PCIConsistentCrossShapeGrammar, 0)
{
    BEGIN_CLASS_INIT(PCIConsistentCrossShapeGrammar);
    INIT_PROPERTY_TABLE();

    ADD_SEPARATOR("Settings");

    ADD_OBJECT_PROP(m_Settings, 0, ConsistentCrossShapeGrammarSettings::getClass(), true);

    ADD_VECTOR4F_PROP(symColorDetected, 0);
    ADD_VECTOR4F_PROP(symColorVirtual,0);
    ADD_VECTOR4F_PROP(symRotColorAxis,0);
    ADD_FLOAT32_PROP(symRotAxisThickness,0);

    //ADD_VECTOR3F_PROP(m_2DPattern_U, 0);
    //ADD_VECTOR3F_PROP(m_2DPattern_V, 0);
    //ADD_VECTOR2I_PROP(m_2DPattern_Size, 0);
    ADD_INT32_PROP(showCluster, 0);
    ADD_BOOLEAN_PROP(doRenderSelected, 0);
    ADD_FLOAT32_PROP(selectedSymDistanceThreshold, 0);
    ADD_BOOLEAN_PROP(doScreenshotWhileSelecting, 0);

    ADD_BOOLEAN_PROP(drawFeatureLines, 0);
    ADD_FLOAT32_PROP(featureLineSize, 0);
    ADD_BOOLEAN_PROP(showSymmetryGraph, 0);
    //ADD_BOOLEAN_PROP(showFoundGroups, 0);
    //ADD_BOOLEAN_PROP(extractSubgroupsWhileSelecting, 0);
    //ADD_BOOLEAN_PROP(recordSubgroupsWhileSelecting, 0);
    ADD_BOOLEAN_PROP(loadMatchForEmbedding, 0);
    ADD_BOOLEAN_PROP(regionBasedCorrespondences, 0);
    //ADD_FLOAT32_PROP(embDistanceResultSigma, 0);

    //ADD_BOOLEAN_PROP(useSpectralEmbedding, 0);
    ADD_BOOLEAN_PROP(showBackwardMatching, 0);

    //ADD_SEPARATOR_METHOD("Princeton Benchmark");
    //ADD_NOARGS_METHOD_DESC(loadPrincetonBenchmarkClasses, "Loads a classification file from the princeton shape benchmark. Classes are added as list nodes to the scene");
    //ADD_NOARGS_METHOD_DESC(loadSelectedClass, "Load all shapes for the selected class nodes");

    ADD_SEPARATOR_METHOD("Prework");
    ADD_NOARGS_METHOD_DESC(addEmptyClassification, "Just add empty classification to be able to work with self-imported objects.");
    ADD_NOARGS_METHOD_DESC(importMesh, "Import a mesh into a selected class.");
    ADD_NOARGS_METHOD_DESC(segmentObjectsFromClass, "Segment all objects in selected classes.");
    //ADD_NOARGS_METHOD_DESC(computeGraphs, "Create a graph for all shapes of one class.");
    //ADD_NOARGS_METHOD_DESC(alignTwoSelectedShapes, "Align two selected shapes based on their graph");
    //ADD_NOARGS_METHOD_DESC(computeSymmetryBetweenParts, "Check every part to which parts it is symmetric.");


    ADD_SEPARATOR_METHOD("Symmetries");
    ADD_NOARGS_METHOD_DESC(detectFeatureLines, "Try to find all feature lines within the selected shape.");
    ADD_NOARGS_METHOD_DESC(clusterFeatureLines, "Cluster all detected feature lines to identify identical one.");
    //ADD_NOARGS_METHOD_DESC(findLatticeGroups, "Find lattice structures within found symmetries");
    ADD_NOARGS_METHOD_DESC(findPreliminarySymmetryGroups, "Find groups within the set of line features");
    ADD_NOARGS_METHOD_DESC(extractFinalSymmetricGroups, "Extract final symmetric groups from the geometry with according geometry");
    ADD_NOARGS_METHOD_DESC(constructDisjointRegions, "Find disjoint regions and extract them as point clouds");
    ADD_NOARGS_METHOD_DESC(execute, "Automatic symmetry detection");

    ADD_SEPARATOR_METHOD("Export");
    ADD_NOARGS_METHOD_DESC(exportSymmetries, "Export selected symmetries to a directory for further processing");
    ADD_NOARGS_METHOD_DESC(computeRelationMatrices, "Read exported data to compute matrices of relationsships for each of the exported shape pair");
    ADD_NOARGS_METHOD_DESC(exportTransformations, "Export transformations for a selected symmetry for further processing");



    ADD_SEPARATOR_METHOD("Matching");
    ADD_NOARGS_METHOD_DESC(showMatchingForSelectedNode, "Show matching result for the selected node");
    ADD_NOARGS_METHOD_DESC(showRegionMatchForSelectedNode, "Show matching result for the selected region.");


    ADD_SEPARATOR_METHOD("Debug");
    ADD_NOARGS_METHOD_DESC(showStatisticsAndLogs, "Show statistics and logs stored for the selected point cloud.");
    ADD_NOARGS_METHOD_DESC(debugShowAllPreliminaryGroups, "Show all preliminary detected groups");
    //ADD_NOARGS_METHOD_DESC(generate2DPattern, "Copy selected object in a 2D pattern.");
    //ADD_NOARGS_METHOD_DESC(debugRenderCluster, "Show all clusters.");
    ADD_NOARGS_METHOD_DESC(loadAndShowExportedSymmetries, "Load exported symmetries and show the corresponding graph");
    ADD_NOARGS_METHOD_DESC(applyColorsToSelectedSymmetries, "Just different test methods.");
}

//--------------------------------------------------------------------------------------
PCIConsistentCrossShapeGrammar::PCIConsistentCrossShapeGrammar()
{
    m_Settings = new ConsistentCrossShapeGrammarSettings();
    m_2DPattern_Size = makeVector2i(8,8);
    m_2DPattern_U = makeVector3f(2.6f,0,0);
    m_2DPattern_V = makeVector3f(0,1.8f,0);

    symColorDetected = makeVector4f(1,1,0,0.35f);
    symColorVirtual = makeVector4f(1,0,1,0.35f);
    symRotColorAxis = makeVector4f(1,0,0,1.0f);
    symRotAxisThickness = 3.0f;

    doRenderSelected = true;
    selectedSymDistanceThreshold = 10.0f;
    doScreenshotWhileSelecting = false;

    //useSpectralEmbedding = false;
    showBackwardMatching = false;

    recordSubgroupsWhileSelecting = false;
    m_selectSymmetryPC = nullptr;
    m_State = IDLE;
    showCluster = -1;
    drawFeatureLines = false;
    showSymmetryGraph = false;
    showGroupGraph = nullptr;
    extractSubgroupsWhileSelecting = true;
    showFoundGroups = false;

    featureLineSize = 2.0f;

    embCurrentlySelectedNode = -1;
    loadMatchForEmbedding = false;
    embDistanceResultSigma = 0.2f;
    regionBasedCorrespondences = true;
}

//--------------------------------------------------------------------------------------
PCIConsistentCrossShapeGrammar::~PCIConsistentCrossShapeGrammar()
{
    if (m_Settings) delete m_Settings;
}

//----------------------------------------------------------------------------------
#include "Util\FunctionContainer.hpp"
void PCIConsistentCrossShapeGrammar::execute() 
{
    typedef void(PCIConsistentCrossShapeGrammar::*VoidFuncPtr)(void);
    FunctionContainer<VoidFuncPtr> func_list;
    func_list.push_back(&PCIConsistentCrossShapeGrammar::detectFeatureLines);
    func_list.push_back(&PCIConsistentCrossShapeGrammar::clusterFeatureLines);
    //func_list.push_back(&PCIConsistentCrossShapeGrammar::findLatticeGroups);
    func_list.push_back(&PCIConsistentCrossShapeGrammar::findPreliminarySymmetryGroups);
    func_list.push_back(&PCIConsistentCrossShapeGrammar::extractFinalSymmetricGroups);

    const size_t num_func = func_list.size();
    progressWindow->pushStep(true, "execute");
    for (size_t fi = 0; fi < num_func; ++fi) {
        (this->*(func_list[fi]))();
        progressWindow->progressf((float)fi/(float)(num_func-1));
    }
    progressWindow->popStep();
}

//----------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::connectToSceneImpl(Scene * scene,OnSceneChangeCallback *onChange, QWidget *sceneEditorWidget, QWidget* toolWidget)
{
    dynamic_cast<NAMESPACE_VERSION::SceneEditorWidget*>(sceneEditorWidget)->on_action_Auto_Layout_triggered(true);
    sceneEditorW = dynamic_cast<SceneEditorWidget*>(sceneEditorWidget);

    this->scene = this->scene;

    // load scene settings
    if (scene->getAttachments()->getData(ConsistentCrossShapeGrammarSettings::getDefaultName()))
    {
        m_Settings = dynamic_cast<ConsistentCrossShapeGrammarSettings*>(scene->getAttachments()->getData(ConsistentCrossShapeGrammarSettings::getDefaultName()));
    }else
    {
        m_Settings->setup(ConsistentCrossShapeGrammarSettings::getDefaultName(), AttachedData::ADF_PERSISTENT);
        scene->getAttachments()->attachData(m_Settings);
    }
}

//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::attachSettingsToScene()
{
    if (getScene() && m_Settings)
    {
        getScene()->getAttachments()->attachData(m_Settings);
    }
}

//--------------------------------------------------------------------------------------
bool PCIConsistentCrossShapeGrammar::drawGLSceneRendering()
{
    if (m_State == SELECT_GENERATOR) return false;
    if (showCluster >= 0) return false;
    if (!(selectedSymDistanceThreshold > 1 || selectedSymDistanceThreshold < 0)) return false;

    return true;
}

//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::glDrawToolImplementation(GLContext* glContext)
{
    SceneObject* objToRender = nullptr;

    if (tmp_correpondenceMatrix.getColsDim())
    {
        std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
        if (selection.size())
        {
            SGObjectNode* srcnode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[0]));
            if (srcnode && SimpleAttachment::exists(srcnode->getSceneObject()->getAttachments(),"EmbeddedIndex"))
            {
                card32 idx = SimpleAttachment::get1i(srcnode->getSceneObject()->getAttachments(),"EmbeddedIndex");
                if (idx != tmp_currentlySelectedMatchingIdx)
                {
                    tmp_currentlySelectedMatchingIdx = idx;
                    showMatchingForSelectedNode();
                }
            }else if (srcnode && SimpleAttachment::exists(srcnode->getSceneObject()->getAttachments(),"RegionID"))
            {
                card32 idx = SimpleAttachment::get1i(srcnode->getSceneObject()->getAttachments(),"RegionID");
                if (idx != tmp_currentlySelectedMatchingIdx)
                {
                    tmp_currentlySelectedMatchingIdx = idx;
                    showRegionMatchForSelectedNode();
                }
            }
        }
    }

    // render graph representation
    if (showSymmetryGraph && showGroupGraph)
    {
        glPushAttrib(GL_ALL_ATTRIB_BITS);

        // draw a circle around each element in the group
        for (unsigned i=0; i < showGroupGraph->getNumChildNodes(nullptr); i++)
        {
            SGObjectNode* sgobj = (SGObjectNode*)showGroupGraph->getChildNode(nullptr, i);
            if (!sgobj->visible()) continue;

            SceneObject* obj = (sgobj)->getSceneObject();			
            UnstructuredInCorePointCloud* pc = (UnstructuredInCorePointCloud*)obj;
            Matrix4f T = obj->getTransformation();

            BoundingBox3f bbox = pc->getPointSet()->getBoundingBox();

            Sphere<float,3> bsphere = bbox.getBoundingSphere();
            bsphere.radius *= 1.1f;
            bsphere.center += shrink4To3(T[3]);

            glBegin(GL_LINE_STRIP);
            for (unsigned k=0; k <= 100; k++)
            {
                Vector3f pos = makeVector3f(bsphere.radius * cos(2.0 * M_PI * float(k) / 100.0f), bsphere.radius * sin(2.0 * M_PI * float(k) / 100.0f), 0);
                pos += bsphere.center;
                glVertex3fv(pos.data());
            }
            glEnd();
        }

        // draw a line for an edge
        for (unsigned i=0; i < groupEdgeMap.size(); i++)
        {
            SGObjectNode* sgobjA = (SGObjectNode*)showGroupGraph->getChildNode(nullptr, groupEdgeMap[i].A);
            SGObjectNode* sgobjB = (SGObjectNode*)showGroupGraph->getChildNode(nullptr, groupEdgeMap[i].B);
            if (!sgobjA->visible() || !sgobjB->visible()) continue;

            UnstructuredInCorePointCloud* pcA = (UnstructuredInCorePointCloud*)(sgobjA->getSceneObject());			
            UnstructuredInCorePointCloud* pcB = (UnstructuredInCorePointCloud*)(sgobjB->getSceneObject());			
            Matrix4f Ta = pcA->getTransformation();
            Matrix4f Tb = pcB->getTransformation();

            Sphere<float,3> bsphereA = pcA->getPointSet()->getBoundingBox().getBoundingSphere();
            Sphere<float,3> bsphereB = pcB->getPointSet()->getBoundingBox().getBoundingSphere();
            bsphereA.radius *= 1.1f; bsphereA.center += shrink4To3(Ta[3]);
            bsphereB.radius *= 1.1f; bsphereB.center += shrink4To3(Tb[3]);

            Vector3f dir = normalize(bsphereB.center - bsphereA.center);
            Vector3f s = bsphereA.center + dir * bsphereA.radius;
            Vector3f e = bsphereB.center - dir * bsphereB.radius;

            glBegin(GL_LINES);
            glVertex3fv(s.data());
            glVertex3fv(e.data());
            glEnd();
        }
        glPopAttrib();
    }

    //if (m_State == SELECT_GENERATOR)
    //	objToRender = m_selectSymmetryPC;

    if (doRenderSelected)
    {
        std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
        if (selection.size() != 1) return;
        SGObjectNode* selnode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[0]));
        if (!selnode) return;
        SceneObject* selcloud = selnode->getSceneObject();
        if (selcloud == nullptr) return;
        objToRender = selcloud;
        FeatureSet* fs = dynamic_cast<FeatureSet*>(selcloud->getAttachments()->getData("FeatureLines"));

        if (objToRender)
        {
            glPushMatrix();
            glPushAttrib(GL_ALL_ATTRIB_BITS);

            // render object
            {
                GLMaterial* mat = getScene()->getGLMaterial(objToRender->getMaterialIndex());
                SceneObjectIterator* sIt = objToRender->createIterator(SOIT::CAP_BASIC_PC);
                mat->draw(getScene(), getScene()->getRootState(), objToRender, sIt, glContext);
                delete sIt;
            }

            // render attachments
            {
                AttachedDataContainer *attachments = objToRender->getAttachments();
                std::vector<AttachedData*> allAtt;
                attachments->getAll(allAtt);
                size_t n = allAtt.size();
                for (size_t i=0; i<n; i++)
                {
                    RenderableObject* renderableObj = dynamic_cast<RenderableObject*>(allAtt[i]);
                    if (renderableObj != nullptr)
                        renderableObj->renderGL(objToRender);
                }
            }

            glPopAttrib();
            glPopMatrix();
        }
    }

    // show clusters
    if (showCluster >= 0)
    {
        static int32 lastShownCluster = -10;

        using namespace NAMESPACE_VERSION::sym;

        std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
        if (selection.size() != 1) return;
        SGObjectNode* selnode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[0]));
        if (!selnode) return;
        SceneObject* selcloud = selnode->getSceneObject();
        if (selcloud == nullptr) return;

        DVectorUObjectList* list = dynamic_cast<DVectorUObjectList*>(selcloud->getAttachments()->getData("LineClusters"));
        if (!list) goto next;
        if (list->getNumLists() <= showCluster) goto next;
        DVectorUObject* cluster = dynamic_cast<DVectorUObject*>(list->getList(showCluster));
        if (!cluster) goto next;

        FeatureSet* fs = dynamic_cast<FeatureSet*>(selcloud->getAttachments()->getData("FeatureLines"));
        if (!fs) goto next;

        // draw lines
        glBegin(GL_LINES);
        glPushAttrib(GL_ALL_ATTRIB_BITS);
        glEnable(GL_DEPTH_TEST);
        glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);
        glDepthMask(0);
        glLineWidth(3.0f);
        glPolygonOffset(0, 1e-3f);
        for (unsigned i=0; i < cluster->getDim(); i++)
        {
            FeatureLine* line = dynamic_cast<FeatureLine*>(fs->m_Features[(*cluster)[i]]);
            if (!line) continue;
            Vector3f pos0 = line->m_Position;
            Vector3f pos1 = line->m_Position + line->m_Direction * line->m_Length;

            glVertex3f(pos0[0], pos0[1], pos0[2]);
            glVertex3f(pos1[0], pos1[1], pos1[2]);

            //if (lastShownCluster != showCluster)
            //	debugOutput << (*cluster)[i] << " - " << line->m_Length << " \n";
        }
        glDepthMask(1);
        glPopAttrib();
        glEnd();

        lastShownCluster = showCluster;
    }next: {}


    // debug render selected symmetries as well
    if (selectedSymDistanceThreshold < 1.0001)
    {
        std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();

        std::map<std::string, Matrix4f> nodeTmap;

        // render all objects
        SGListNode* root = dynamic_cast<SGListNode*>(getScene()->getRootNode());
        for (unsigned i=0; i < root->getNumChildNodes(nullptr); i++)
        {
            SGObjectNode* sgobj = dynamic_cast<SGObjectNode*>(root->getChildNode(nullptr, i));			
            if (!sgobj || !sgobj->visible()) continue;

            SceneObject* obj = sgobj->getSceneObject();			
            UnstructuredInCorePointCloud* pc = dynamic_cast<UnstructuredInCorePointCloud*>(obj);
            if (!obj) continue;

            Matrix4f T = obj->getTransformation();

            nodeTmap[sgobj->getName()] = T;

            // render all objects first
            glPushMatrix();
            glMultMatrixf(T.data());
            glPushAttrib(GL_ALL_ATTRIB_BITS);

            // render object
            {
                GLMaterial* mat = getScene()->getGLMaterial(obj->getMaterialIndex());
                SceneObjectIterator* sIt = obj->createIterator(SOIT::CAP_BASIC_PC);
                mat->draw(getScene(), getScene()->getRootState(), obj, sIt, glContext);
                delete sIt;
            }

            // render attachments
            {
                AttachedDataContainer *attachments = obj->getAttachments();
                std::vector<AttachedData*> allAtt;
                attachments->getAll(allAtt);
                size_t n = allAtt.size();
                for (size_t i=0; i<n; i++)
                {
                    RenderableObject* renderableObj = dynamic_cast<RenderableObject*>(allAtt[i]);
                    if (renderableObj != nullptr)
                        renderableObj->renderGL(obj);
                }
            }

            glPopAttrib();
            glPopMatrix();
        }

        // render all selected symmetries
        for (unsigned i=0; i < root->getNumChildNodes(nullptr); i++)
        {
            SGStateChangeNode* stnode = dynamic_cast<SGStateChangeNode*>(root->getChildNode(nullptr,i)); if (!stnode) continue;
            SGListNode* _lsnode = dynamic_cast<SGListNode*>(stnode->getChildNode(nullptr,0)); if (!_lsnode) continue;
            card32 ls_sid; if (!strToInt(_lsnode->getName(), ls_sid)) continue;

            const Matrix4f& T = nodeTmap[std::string("main_") + intToStr(ls_sid)];

            SGListNode* listNode = dynamic_cast<SGListNode*>(_lsnode->getChildNode(nullptr,0)); if (!listNode) continue;

            for (unsigned j=0; j < listNode->getNumChildNodes(nullptr); j++)
            {
                SGObjectNode* objnode = dynamic_cast<SGObjectNode*>(listNode->getChildNode(nullptr,j));
                SceneObject* obj = objnode->getSceneObject();

                if (SimpleAttachment::exists(obj->getAttachments(),"EmbeddedIndex"))
                {
                    int idx = SimpleAttachment::get1i(obj->getAttachments(), "EmbeddedIndex");

                    std::map<card32,float32>::const_iterator it = selectedSymDistanceMap.find(idx);
                    if (it == selectedSymDistanceMap.end()) continue;
                    if (it->second < selectedSymDistanceThreshold) continue;

                    RenderableObject* att = dynamic_cast<RenderableObject*>(obj->getAttachments()->getData("SymmetryGroup"));
                    if (!att) continue;
                    if (dynamic_cast<sym::SymmetryGroupAttachment*>(att)) dynamic_cast<sym::SymmetryGroupAttachment*>(att)->visible = true;
                    if (dynamic_cast<CCSGSymmetryGroup*>(att)) dynamic_cast<CCSGSymmetryGroup*>(att)->visible = true;

                    // render all objects first
                    glPushMatrix();
                    glMultMatrixf(T.data());
                    glPushAttrib(GL_ALL_ATTRIB_BITS);

                    att->renderGL(obj);

                    glPopAttrib();
                    glPopMatrix();

                }
            }

        }

    }

}


//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::loadPrincetonBenchmarkClasses()
{

    // load classification file and parse it
    std::string filename = FileDialogs::getOpenFileName(InterfaceGlobals::getMainWindow(), "Open Princeton CLA files...",
        "Files (*.cla)", nullptr, "", FileDialogs::IMPORT_DIR);
    if (filename != "")
    {		
        QFile file(QString(filename.c_str()));
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            error(std::string("Cannot read file ") + filename + std::string(". File not found"));
            return;
        }

        QTextStream in(&file);

        // only V1 supports now
        unsigned numclasses = 0;
        unsigned numshapes = 0;
        {
            QString line = in.readLine();
            if (line != "PSB 1")
            {
                error(std::string("The loaded file does not contain a valid and supported classification format"));
                return;
            }


            line = in.readLine();
            QStringList list = line.split(" ", QString::SkipEmptyParts);
            if (list.size() != 2)
            {
                error(std::string("No numbers about amount of classes and shapes found"));
                return;
            }

            numclasses = list.front().toInt(); list.pop_front();
            numshapes = list.front().toInt(); 
        }

        struct Class
        {
            std::string name;
            std::string parent;
            std::vector<int> ids;
        };

        std::vector<Class> classes;

        // read classes
        int mode = 0;
        Class* lastClass = nullptr;
        int lastShapeIdx = 0;
        while (!in.atEnd())
        {
            QString line = in.readLine();

            switch (mode)
            {
            case 0:  // read class definition
                if (line.length() > 4)
                {
                    QStringList list = line.split(" ", QString::SkipEmptyParts);
                    if (list.size() != 3)
                    {
                        error(std::string("Invalid line: ") + line.toStdString() + std::string(" expected a valid class definition"));
                        return;
                    }

                    Class cls;

                    cls.name = list.front().toLocal8Bit().constData(); 
                    list.pop_front();
                    cls.parent = list.front().toLocal8Bit().constData(); 
                    list.pop_front();
                    cls.ids.resize(list.front().toInt());

                    classes.push_back(cls);

                    if( cls.ids.size() > 0)
                    {
                        mode = 1;
                        lastClass = &classes[classes.size()-1];
                        lastShapeIdx = 0;
                    }
                }
                break;

            case 1: // read shape ids of a class
                if (line.length() > 0)
                {
                    lastClass->ids[lastShapeIdx++] = line.toInt();

                    if (lastShapeIdx >= lastClass->ids.size())
                        mode = 0;
                }
            }
        }

        CCSGClassification* ccsg = new CCSGClassification;
        ccsg->setup(CCSGClassification::getDefaultName(), AttachedData::ADF_PERSISTENT);
        scene->getAttachments()->attachData(ccsg);

        // create hierarchical representation in the scene graph correspondign to the classification
        struct MagicHierarchicalSort
        {
            void operator()(const std::vector<Class>& classes, const std::string& parent, SGListNode* node, CCSGClassification* ccsg, int depth)
            {
                if (node == nullptr) return;
                for (std::vector<Class>::const_iterator it=classes.begin(); it != classes.end(); it++)
                {
                    if (it->parent == parent)
                    {
                        SGListNode* child = (/*depth != 0 ? new SGRelativeTimeAnimationNode :*/ new SGListNode);
                        child->setName(intToStr(depth) + std::string("_") + it->name);
                        ccsg->shapeMap[child->getName()] = it->ids;
                        node->addChildNode(child);

                        (*this)(classes, it->name, child, ccsg, depth+1);
                    }
                }
            };
        };

        MagicHierarchicalSort sortme;
        sortme(classes, "0", dynamic_cast<SGListNode*>(scene->getRootNode()), ccsg, 0);
    }
}

#if 0
//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::computeGraphs()
{
    debugRenderer->clearDebugData("graphs");
    debugRenderer->beginRenderJob("graphs");

    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    ProgressWindow* progress = getProgressWindow();
    for (unsigned i=0; i < selection.size(); i++)
    {
        SGListNode* node = dynamic_cast<SGListNode*>(getSceneGraphNodeByName(getScene(), selection[i]));
        if (!node) continue;

        debugOutput << "process: " << node->getName() << " (" << node->getNumChildNodes(nullptr) << "):\n";

        SGListNode* parentnode = dynamic_cast<SGListNode*>(node->getParent(0));
        SGListNode* graphnode = new SGListNode();
        graphnode->setName(node->getName() + std::string("_graph"));
        parentnode->addNode(graphnode);

        progress->pushStep(true, std::string("process ") + node->getName());
        for (unsigned j=0; j < node->getNumChildNodes(nullptr); j++)
        {
            progress->progressf(float(j) / float(node->getNumChildNodes(nullptr)));

            // for each connected component we check what are the nearest other components
            SGObjectNode* objnode = dynamic_cast<SGObjectNode*>(node->getChildNode(nullptr, j));
            UnstructuredInCorePointCloud* mesh = dynamic_cast<UnstructuredInCorePointCloud*>(objnode->getSceneObject());

            AAT cidAAT = mesh->getAAT("cid");
            AAT posAAT = mesh->getAAT("position");

            unsigned maxcid = 0;
            std::map<int32, std::set<int32> > segmentAdjMap;
            std::map<int32, std::vector<DVectorF> > dataForPCA;
            for (unsigned k=0; k < mesh->getNumPoints(); k++)
            {
                Vector3f startpos = mesh->getPointSet()->get3f(k, posAAT);
                int cid = mesh->getPointSet()->get2i(k, cidAAT)[0];
                if (cid < 0) continue;

                (dataForPCA[cid]).push_back(PCAGeneral::convertVec3fToDVec(std::vector<Vector3f>(1,startpos)));

                HierarchicalKNNIterator query(mesh, 32, nullptr);
                std::vector<mpcard> indices;
                mpcard num = 0;
                query.setSeekPointAndReset(startpos); //query.next();
                while (!query.atEnd())
                {
                    if (norm(query.get3f(posAAT) - startpos) > m_Settings->adjMaxDistance) break;
                    indices.push_back(query.getCurrentPointIndex());
                    query.next();
                }

                num = indices.size();
                maxcid = cid > maxcid ? cid : maxcid;

                // for each point in the neighborhood add it's segment to the set of neighbor segments
                for (unsigned l=0; l < num; l++)
                    segmentAdjMap[cid].insert(mesh->getPointSet()->get2i(indices[l], cidAAT)[0]);
            }

            // now create a structure which can be attached
            InCorePCTopologyGraph* topo = new InCorePCTopologyGraph;
            {
                VertexDescriptor vd;
                vd.pushAttrib( VAD("vindex", 2, VAD::DATA_FORMAT_INT32));
                topo->clearAndSetup(&vd);
            }

            UnstructuredInCorePointCloud* topopc = new UnstructuredInCorePointCloud;
            topopc->clearAndSetup(maxcid+1, true, false, false, true);
            TopologyAttachment* tatt = new TopologyAttachment;
            tatt->setTopology(topo);
            tatt->setup(TopologyAttachment::getDefaultName(), AttachedData::ADF_PERSISTENT);
            topopc->getAttachments()->attachData(tatt);
            tatt->setLineColor(makeVector4f(0,0,0,0.5));
            tatt->setVisible(true);
            topopc->setMaterialIndex(4);
            AAT topPosAAT = topopc->getAAT("position");


            // compute PCA for each of the segments
            checkAttribute(topopc, "axes", 9, VAD::DATA_FORMAT_FLOAT32);
            AAT axesAAT = topopc->getAAT("axes");
            for (unsigned c=0; c < maxcid+1; c++)
            {
                if (dataForPCA[c].size() < 3) continue;

                PCAGeneral pca;
                pca.calculatePCA(dataForPCA[c], -1, 3);
                DMatrixF pcamatr = pca.getEigenVecs();
                Matrix3f matr;
                Vector3f values = PCAGeneral::convertDVtoVec3f(pca.getEigenValues())[0];

                matr[0] = values[0] * PCAGeneral::convertDVtoVec3f(pcamatr[0])[0];
                matr[1] = values[1] * PCAGeneral::convertDVtoVec3f(pcamatr[1])[0];
                matr[2] = values[2] * PCAGeneral::convertDVtoVec3f(pcamatr[2])[0];

                topopc->getPointSet()->set9f(c, axesAAT, matr);
            }

            for (std::map<int32, std::set<int32> >::const_iterator it=segmentAdjMap.begin(); it != segmentAdjMap.end(); it++)
            {
                Vector3f avgPos = makeVector3f(0,0,0);
                unsigned num = 0;
                for (unsigned l=0; l < mesh->getNumPoints(); l++)
                    if (mesh->getPointSet()->get2i(l, cidAAT)[0] == it->first)
                    {
                        avgPos += mesh->getPointSet()->get3f(l, posAAT);
                        num ++;
                    }
                    avgPos /= float(num);

                    topopc->getPointSet()->set3f(it->first, topPosAAT, avgPos);

                    for (set<int32>::const_iterator jt=it->second.begin(); jt != it->second.end(); jt++)
                        if (it->first != *jt) topo->addEdge(it->first, *jt);
            }

            SGObjectNode* graph = new SGObjectNode(topopc);
            graph->setName(objnode->getName());
            graphnode->addNode(graph);

            debugOutput << "\t" << objnode->getName() << " done\n";
        }
        progress->popStep();
    }
    debugRenderer->endRenderJob();
}
#endif 

#if 0
//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::loadSelectedClass()
{
    CCSGClassification* ccsg = dynamic_cast<CCSGClassification*>(scene->getAttachments()->getData(CCSGClassification::getDefaultName()));
    if (ccsg == nullptr)
    {
        error("Need to load classification first\n" );
        return;
    }

    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    ProgressWindow* progress = getProgressWindow();
    for (unsigned i=0; i < selection.size(); i++)
    {
        // if a list node, then load it 
        SGListNode* node = dynamic_cast<SGListNode*>(getSceneGraphNodeByName(getScene(), selection[i]));
        if (!node) continue;

        std::vector<int> idx = ccsg->shapeMap[node->getName()];
        debugOutput << "load: " << node->getName() << " (" << idx.size() << "):\n";

        progress->pushStep(true, std::string("load ") + node->getName());
        for (unsigned j=0; j < idx.size(); j++)
        {
            UnstructuredInCoreTriangleMesh* tri = ImportMeshOpenAsset::import(m_Settings->shapeDBPath + std::string("\\m") + intToStr(idx[j]) + std::string("\\m") + intToStr(idx[j]) + std::string(".off"));
            if (tri == nullptr)
                debugOutput << "\tfailed to read " << idx[j] << "\n"; 
            else{
                SGObjectNode* obj = new SGObjectNode(tri);
                obj->setName(intToStr(idx[j]));
                node->addNode(obj);
                debugOutput << "\tok " << idx[j] << "\n";
                tri->setMaterialIndex(3);
                tri->computeTopologyAttachment();			

                TopologyAttachment* tatt = dynamic_cast<TopologyAttachment*>(tri->getAttachments()->getData(TopologyAttachment::getDefaultName()));
                tatt->setLineColor(makeVector4f(0,0,0,1.5));
                tatt->setVisible(false);
                //tatt->setBlending(true);

            }
            progress->progressf(float(j) / float(idx.size()));
        }
        progress->popStep();
    }

}
#endif

//--------------------------------------------------------------------------------------

std::vector<SGObjectNode*> PCIConsistentCrossShapeGrammar::segmentObject(UnstructuredInCoreTriangleMesh* mesh, InCorePCTopologyGraph* top, int& cid)
{
    std::vector<SGObjectNode*> segments;

    std::vector<std::vector<mpcard>> components;
    if (findConnectedComponents(mesh, top, components) > 0)
    {
        checkAttribute(mesh, "cid", 2, VAD::DATA_FORMAT_INT32);  // [0] - local cid, [1] - global cid
        AAT cidAtt = mesh->getAAT("cid");

        for (unsigned c=0; c < components.size(); c++)
        {
            UnstructuredInCoreTriangleMesh* part = mesh->extractSubMesh(components[c]);
            SGObjectNode* partnode = new SGObjectNode(part);
            partnode->setName(intToStr(c));
            segments.push_back(partnode);

            for (unsigned p=0; p < components[c].size(); p++)
                mesh->getPointSet()->set2i(components[c][p], cidAtt, makeVector2i(c, cid));

            cid++;
        }

        SimpleAttachment::set1i(mesh->getAttachments(), "numsegments", (int)components.size());

        // colorify components
        AAT colAtt = mesh->getPointSet()->getAAT("color");
        for (unsigned c=0; c < mesh->getNumPoints(); c++)
        {
            srandom(mesh->getPointSet()->get2i(c, cidAtt)[1]);
            mesh->getPointSet()->set3f(c, colAtt, makeVector3f(rnd01(),rnd01(),rnd01()));
        }

        //debugOutput << "\tdone " << objnode->getName() << " " << components.size() << " segments\n";
    }//else
    //	debugOutput << "\tno segments found " << objnode->getName() << "\n";

    return segments;
}

//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::segmentObjectsFromClass()
{
    debugRenderer->clearDebugData("groups"); 
    debugRenderer->beginRenderJob("groups");

    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    ProgressWindow* progress = getProgressWindow();
    for (unsigned i=0; i < selection.size(); i++)
    {
        int cid = 0;

        SGListNode* node = dynamic_cast<SGListNode*>(getSceneGraphNodeByName(getScene(), selection[i]));
        if (!node) continue;

        debugOutput << "segment: " << node->getName() << " (" << node->getNumChildNodes(nullptr) << "):\n";

        progress->pushStep(true, std::string("segment ") + node->getName());
        for (unsigned j=0; j < node->getNumChildNodes(nullptr); j++)
        {
            progress->progressf(float(j) / float(node->getNumChildNodes(nullptr)));

            // we do the segmentation just by looking for connected components
            SGObjectNode* objnode = dynamic_cast<SGObjectNode*>(node->getChildNode(nullptr, j));
            UnstructuredInCoreTriangleMesh* mesh = dynamic_cast<UnstructuredInCoreTriangleMesh*>(objnode->getSceneObject());

            if (mesh == nullptr)
            {
                error(std::string("Can't work with ") + objnode->getName() + " because currently only triangle meshes are supported");
                continue;
            }

            // every node must contain a topology graph
            TopologyAttachment* tatt = dynamic_cast<TopologyAttachment*>(mesh->getAttachments()->getData(TopologyAttachment::getDefaultName()));
            if (!tatt)
            {
                debugOutput << "\tnode " << objnode->getName() << " has no topology graph attached, ignore\n";
                continue;
            }
            InCorePCTopologyGraph* top = tatt->getTopology();

            std::vector<SGObjectNode*> sgmts = segmentObject(mesh, top, cid);

            SGListNode* segments = new SGListNode();
            segments->setName(node->getName() + "_" + objnode->getName() + std::string("_segments"));
            getScene()->getRootNode()->addChildNode(segments);

            for (unsigned c=0; c < sgmts.size(); c++)
                segments->addChildNode(sgmts[c]);

            debugOutput << "\tdone " << objnode->getName() << " " << sgmts.size() << " segments\n";

            /*std::vector<std::vector<mpcard>> components;
            if (findConnectedComponents(mesh, top, components) > 0)
            {
            checkAttribute(mesh, "cid", 2, VAD::DATA_FORMAT_INT32);  // [0] - local cid, [1] - global cid
            AAT cidAtt = mesh->getAAT("cid");

            for (unsigned c=0; c < components.size(); c++)
            {
            UnstructuredInCoreTriangleMesh* part = mesh->extractSubMesh(components[c]);
            SGObjectNode* partnode = new SGObjectNode(part);
            partnode->setName(intToStr(c));
            segments->addChildNode(partnode);

            for (unsigned p=0; p < components[c].size(); p++)
            {
            mesh->getPointSet()->set2i(components[c][p], cidAtt, makeVector2i(c, cid));
            }
            cid++;
            }

            SimpleAttachment::set1i(mesh->getAttachments(), "numsegments", components.size());

            // colorify components
            AAT colAtt = mesh->getPointSet()->getAAT("color");
            for (unsigned c=0; c < mesh->getNumPoints(); c++)
            {
            srandom(mesh->getPointSet()->get2i(c, cidAtt)[1]);
            mesh->getPointSet()->set3f(c, colAtt, makeVector3f(rnd01(),rnd01(),rnd01()));
            }

            debugOutput << "\tdone " << objnode->getName() << " " << components.size() << " segments\n";
            }else
            debugOutput << "\tno segments found " << objnode->getName() << "\n";*/

        }
        progress->popStep();
    }

    debugRenderer->endRenderJob();
}

//--------------------------------------------------------------------------------------
UnstructuredInCorePointCloud* PCIConsistentCrossShapeGrammar::getGraphNode(SGObjectNode* node)
{
    SGListNode* parentnode = dynamic_cast<SGListNode*>(node->getParent(0));
    SGListNode* grandparentnode = dynamic_cast<SGListNode*>(parentnode->getParent(0));

    for (unsigned i=0; i < grandparentnode->getNumChildNodes(nullptr); i++)
        if (grandparentnode->getChildNode(nullptr, i)->getName() == parentnode->getName() + "_graph")
        {
            SGListNode* siblingparent = dynamic_cast<SGListNode*>(grandparentnode->getChildNode(nullptr, i));

            for (unsigned j=0; j < siblingparent->getNumChildNodes(nullptr); j++)
                if (siblingparent->getChildNode(nullptr, j)->getName() == node->getName())
                    return dynamic_cast<UnstructuredInCorePointCloud*>(dynamic_cast<SGObjectNode*>(siblingparent->getChildNode(nullptr, j))->getSceneObject());
        }

        return nullptr;
}

#if 0
//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::alignTwoSelectedShapes()
{
    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    if (selection.size() != 2)
    {
        error("You need to select two shapes");
        return;
    }

    debugRenderer->clearDebugData("align");
    debugRenderer->beginRenderJob("align");

    ProgressWindow* progress = getProgressWindow();

    SGObjectNode* nodeA = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[0]));
    SGObjectNode* nodeB = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[1]));
    if (!nodeA || !nodeB) return;

    debugOutput << "align: " << nodeA->getName() << " with " << nodeB->getName() << "\n";

    // access to the graphs describing the shapes. these need to be computed before
    SGListNode* parentnodeA = dynamic_cast<SGListNode*>(nodeA->getParent(0));
    SGListNode* parentnodeB = dynamic_cast<SGListNode*>(nodeB->getParent(0));

    UnstructuredInCorePointCloud* pcA = dynamic_cast<UnstructuredInCorePointCloud*>(nodeA->getSceneObject());
    UnstructuredInCorePointCloud* pcB = dynamic_cast<UnstructuredInCorePointCloud*>(nodeB->getSceneObject());

    UnstructuredInCorePointCloud* gcA = getGraphNode(nodeA);
    UnstructuredInCorePointCloud* gcB = getGraphNode(nodeB);

    if (!gcA || !gcB)
    {
        error("No graph for the selected nodes found\n");
        return;
    }

    AAT AposAAT = pcA->getAAT("position");
    AAT BposAAT = pcB->getAAT("position");

    // perform RANSAC:
    //   take 3 points on shape A
    //   compute spin image signatures for them
    //   randomly sample points on shape B, where spin images are similar
    //   compute transformation and try to align both graphs of shape A and shape B together
    //   use node's pca distributions for scale invariant alignment

    float radiusA = pcA->getPointSet()->getBoundingBox().getMaxSideLength() * m_Settings->descriptorRadiusBBFactor;
    float radiusB = pcB->getPointSet()->getBoundingBox().getMaxSideLength() * m_Settings->descriptorRadiusBBFactor;

    // compute spin image descriptors for each point on A and B
    SpinImageDescriptor spinImageDescriptor;
    spinImageDescriptor.setSpinSize(32);
    std::vector<SpinImageSignature*> spinA(pcA->getNumPoints());
    std::vector<SpinImageSignature*> spinB(pcB->getNumPoints());

    struct ComputeSpinImages
    {
        ComputeSpinImages(SpinImageDescriptor& descr, UnstructuredInCorePointCloud* pc, std::vector<SpinImageSignature*>& spin, float radius)
        {
            AAT posAAT = pc->getAAT("position");
            AAT norAAT = pc->getAAT("normal");
            spin.resize(pc->getNumPoints());
            for (unsigned i=0; i < pc->getNumPoints(); i++)
                spin[i] = dynamic_cast<SpinImageSignature*>(descr.computeSignature(pc->getPointSet()->get3f(i,posAAT),pc->getPointSet()->get3f(i,norAAT),radius,pc));
        }
    };

    debugOutput << "\tcompute descriptors\n";

    ComputeSpinImages(spinImageDescriptor, pcA, spinA, radiusA);
    ComputeSpinImages(spinImageDescriptor, pcB, spinB, radiusB);

    debugOutput << "\tdo ransac\n";

    float minres = 1e20;
    Matrix4f minT; minT.setIdentity();

    // perform ransac
    for (unsigned rsac=0; rsac < 1; rsac++)
    {
        const int NUMC = 3;

        // sample 3 points on shape A
        LotteryDraw draw;
        draw.setup(pcA->getNumPoints());
        unsigned pts[NUMC]; for (unsigned k=0; k < NUMC; k++) pts[k] = draw.draw();
        unsigned sampled[NUMC]; memset(&sampled[0], 0, sizeof(unsigned) * NUMC);

        ImportanceSampler sampler;
        std::vector<ImportanceSample*> smpl(pcB->getNumPoints());
        for (unsigned k=0; k < NUMC; k++)
        {
            for (unsigned j=0; j < pcB->getNumPoints(); j++)
            {
                smpl[j] = new ImportanceSample(spinImageDescriptor.compareLinear(spinA[pts[k]], spinB[j]));
                smpl[j]->setSampleID(j);
                sampler.addSample(smpl[j]);
            }

            ImportanceSample* sam = sampler.sample();
            sampled[k] = sam->getSampleID();

            debugRenderer->addLine(pcA->getPointSet()->get3f(pts[k], AposAAT), pcB->getPointSet()->get3f(sampled[k], BposAAT), rainbowGradient(sam->getProbability()), rainbowGradient(sam->getProbability()), 3.0f);

            for (unsigned j=0; j < pcB->getNumPoints(); j++) delete smpl[j];
            sampler.clearSamples();
        }


        // compute rigid transformation using the matches
        Matrix4f T;
        {
            Vector3f centroidA = NULL_VECTOR3F;
            Vector3f centroidB = NULL_VECTOR3F;
            for( unsigned i=0;i<NUMC;i++ )
            {
                //debugRenderer->addSphere(pcA->getPointSet()->get3f(pts[i], AposAAT), 0.01, makeVector3f(1, 0, float(i)/float(NUMC)));
                //debugRenderer->addSphere(pcB->getPointSet()->get3f(sampled[i], BposAAT), 0.01, makeVector3f(0, 1, float(i)/float(NUMC)));
                centroidA += pcA->getPointSet()->get3f(pts[i], AposAAT);
                centroidB += pcB->getPointSet()->get3f(sampled[i], BposAAT);
            }

            centroidA /= (float)NUMC;
            centroidB /= (float)NUMC;


            Matrix3f C = IDENTITY3F * 0;
            for( unsigned i=0;i<NUMC;i++ )
                C += outerProduct(pcA->getPointSet()->get3f(pts[i], AposAAT) - centroidA, pcB->getPointSet()->get3f(sampled[i], BposAAT) - centroidB);
            C /= (float)NUMC;

            // Prepare OpenCV Datastructure CvMat
            Matrix3f U,V;
            Vector3f eigenValues;
            CvMat cvMatInput, cvMat_U, cvMat_W, cvMat_Vt;
            cvInitMatHeader( &cvMatInput, 3, 3, CV_32FC1, C.data());
            cvInitMatHeader( &cvMat_W, 3, 1, CV_32FC1, eigenValues.data() );
            cvInitMatHeader( &cvMat_U, 3, 3, CV_32FC1, U.data() );
            cvInitMatHeader( &cvMat_Vt, 3, 3, CV_32FC1, V.data() );

            // Do SVD
            cvSVD(&cvMatInput, &cvMat_W, &cvMat_U, &cvMat_Vt, CV_SVD_MODIFY_A | CV_SVD_U_T );

            Matrix3f rotation = U*V;
            T = makeTranslation4f(centroidA) * expand3To4(rotation) * makeTranslation4f(-centroidB);			
        }

        pcB->setTransformation(T);
        gcB->setTransformation(T);
        updateSceneView();

        // perform ICP between both shapes in order to find final alignment
        {
            FSIcp* icp = new FSIcp;
            icp->setupData(pcA, getScene());
            icp->setupParameters(m_Settings->maxIcpIterations, 2, getCachedMedianPointDistance(pcB) /*medDist*/, m_Settings->icpOutlierDistanceFactor, m_Settings->icpMinPercentInliers, 1e20f, -1, 0.01f, false);			
            float residual = icp->calculateResidualAfterICP(pcB->getPointSet(), T);
            delete icp;

            if (residual < minres)
            {
                minres = residual;
                minT = T;
            }
        }

        pcB->setTransformation(T);
        gcB->setTransformation(T);
        updateSceneView();
    }

    // apply best transofmration found
    pcB->setTransformation(minT);
    gcB->setTransformation(minT);
    updateSceneView();

    for (unsigned i=0; i < spinA.size(); i++) delete spinA[i];
    for (unsigned i=0; i < spinB.size(); i++) delete spinB[i];

    debugRenderer->endRenderJob();
}
#endif 

//--------------------------------------------------------------------------------------
UnstructuredInCorePointCloud* PCIConsistentCrossShapeGrammar::extractSegment(UnstructuredInCorePointCloud* pc, int cid, bool localcid)
{
    UnstructuredInCorePointCloud* npc = (UnstructuredInCorePointCloud*)pc->copy();

    AAT cidAAT = npc->getAAT("cid");
    std::vector<mpcard> keep;
    for (unsigned i=0; i < pc->getNumPoints(); i++)
    {
        if (npc->getPointSet()->get2i(i, cidAAT)[localcid ? 0 : 1] == cid)
            keep.push_back(i);
    }

    TopologyAttachment* tatt = dynamic_cast<TopologyAttachment*>(npc->getAttachments()->getData(TopologyAttachment::getDefaultName()));
    InCorePCTopologyGraph* tpg = nullptr;
    if (tatt) tpg = tatt->getTopology();

    if (dynamic_cast<UnstructuredInCoreTriangleMesh*>(pc))
    {
        UnstructuredInCoreTriangleMesh* tpc = dynamic_cast<UnstructuredInCoreTriangleMesh*>(npc);
        UnstructuredInCoreTriangleMesh* ppc = tpc->extractSubMesh(keep);
        if(tatt) ppc->computeTopologyAttachment();
        delete npc;
        npc = ppc;
    }else
    {
        InCorePCTopologyGraph* newtpg = nullptr;
        keepPointsInPointCloud(npc, keep, tpg, &newtpg);
        if (tatt) tatt->swapTopology(newtpg); 
        delete tpg;
    }
    return npc;
}


//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::addEmptyClassification()
{
    CCSGClassification* ccsg = dynamic_cast<CCSGClassification*>(scene->getAttachments()->getData(CCSGClassification::getDefaultName()));
    if (ccsg != nullptr)
    {
        error("Classification already found. I don't want to override it, hence delete it first" );
        return;
    }

    ccsg = new CCSGClassification();
    ccsg->setup(CCSGClassification::getDefaultName(), AttachedData::ADF_PERSISTENT);
    scene->getAttachments()->attachData(ccsg);
}

#if 0
//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::computeSymmetryBetweenParts()
{
    CCSGClassification* ccsg = dynamic_cast<CCSGClassification*>(scene->getAttachments()->getData(CCSGClassification::getDefaultName()));
    if (ccsg == nullptr)
    {
        error("Need to load classification first\n" );
        return;
    }

    debugRenderer->clearDebugData("symmetries");
    debugRenderer->beginRenderJob("symmetries");

    ccsg->symmetryT.clear();
    ccsg->symmetryId.clear();

    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    ProgressWindow* progress = getProgressWindow();
    for (unsigned i=0; i < selection.size(); i++)
    {
        SGListNode* node = dynamic_cast<SGListNode*>(getSceneGraphNodeByName(getScene(), selection[i]));
        if (!node) continue;

        debugOutput << "find symmetries in class: " << node->getName() << " (" << node->getNumChildNodes(nullptr) << "):\n";

        struct Part
        {
            UnstructuredInCorePointCloud* parentMesh;
            int parentChildIndex;

            unsigned cid;
            unsigned gcid;

            UnstructuredInCorePointCloud* geometry;
            Vector3f pos;
            Matrix3f frame;
            Vector3f extension;
            float size;
        };

        std::vector<Part> allparts;

        // first extract all parts out of the whole class
        debugOutput << "\tgather all parts from each instance\n";
        for (unsigned j=0; j < node->getNumChildNodes(nullptr); j++)
        {
            progress->progressf(float(j) / float(node->getNumChildNodes(nullptr)));

            SGObjectNode* objnode = dynamic_cast<SGObjectNode*>(node->getChildNode(nullptr, j));
            UnstructuredInCorePointCloud* mesh = dynamic_cast<UnstructuredInCorePointCloud*>(objnode->getSceneObject());
            UnstructuredInCorePointCloud* graph = getGraphNode(objnode);

            BoundingBox3f meshBB = mesh->getPointSet()->getBoundingBox();
            float32 meshRadius = meshBB.getBoundingSphere().radius;

            AAT axesAAT = graph->getAAT("axes");
            AAT gposAAT = graph->getAAT("position");

            SGRelativeTimeAnimationNode* list = new SGRelativeTimeAnimationNode();
            list->setName(node->getName() + std::string("_") + objnode->getName() + "_parts");
            getScene()->getRootNode()->addChildNode(list);

            for (unsigned k=0; k < graph->getNumPoints(); k++)
            {
                Part part;

                part.frame = graph->getPointSet()->get9f(k, axesAAT);
                part.pos = graph->getPointSet()->get3f(k, gposAAT);

                part.extension = makeVector3f(part.frame[0].getNorm(), part.frame[1].getNorm(), part.frame[2].getNorm());
                part.size = std::max(part.extension[0], std::max(part.extension[1], part.extension[2]));

                part.frame[0].normalize();
                part.frame[1].normalize();
                part.frame[2].normalize();

                part.geometry = extractSegment(mesh, k, true);
                if (part.geometry->getNumPoints() < 6 || part.geometry->getPointSet()->getBoundingBox().getBoundingSphere().radius / meshRadius < m_Settings->smallPartMinimalSizeFactor)
                {
                    delete part.geometry;
                    continue;
                }
                part.geometry->setMaterialIndex(5);
                //detectFeatureLines(dynamic_cast<UnstructuredInCoreTriangleMesh*>(part.geometry));
                //clusterFeatureLines(dynamic_cast<UnstructuredInCoreTriangleMesh*>(part.geometry));

                AAT partPosAAT = part.geometry->getAAT("position");

                for (unsigned t=0; t < part.geometry->getNumPoints(); t++)
                    part.geometry->getPointSet()->set3f(t, partPosAAT, part.geometry->getPointSet()->get3f(t, partPosAAT) - part.pos);

                part.parentMesh = mesh;
                part.parentChildIndex = j;
                part.cid = k;
                part.gcid = part.geometry->getPointSet()->get2i(0, part.geometry->getAAT("cid"))[1];

                allparts.push_back(part);

                // copy data into a list, for later access
                SGObjectNode* nn = new SGObjectNode(part.geometry);
                nn->setName(intToStr(part.gcid));
                list->addChildNode(nn);
            }
        }

        debugOutput << "\tcompute symmetric parts\n";
        progress->pushStep(true, std::string("match"));
        //unsigned k=0, j=0;
        for (unsigned k=0, j=0; k < allparts.size(); k++)
        {
            const Matrix3f& frameA = allparts[k].frame;
            const Vector3f& posA = allparts[k].pos;
            UnstructuredInCorePointCloud* partA = allparts[k].geometry;
            //addPointCloud(getScene(), (PointCloud*)partA->copy(), std::string("partA"));
            //debugRenderer->addCoordSystem(posA, frameA, allparts[k].size);

            //unsigned l=1;
            for (unsigned l=k+1; l < allparts.size(); l++, j++)
            {
                progress->progressf(float(j) / float(allparts.size() * allparts.size() * 0.5));

                const Matrix3f& frameB = allparts[l].frame;
                const Vector3f& posB = allparts[l].pos;
                UnstructuredInCorePointCloud* partB = allparts[l].geometry;
                //addPointCloud(getScene(), (PointCloud*)partA->copy(), std::string("partB"));
                //debugRenderer->addCoordSystem(posB, frameB, allparts[k].size);

                // compute transformation to transform frame B into frame A
                // then for each of the mirroring cases check the geometrical match
                Matrix4f Tatob;
                Matrix4f Tbtoa;
                Vector3f TposA, TposB;
                float dist = 1e20f;
                for (unsigned t=0; t < 8; t++)
                {
                    float x = (t & 1) ? -1.0 : 1.0;
                    float y = (t & 2) ? -1.0 : 1.0;
                    float z = (t & 4) ? -1.0 : 1.0;

                    Matrix3f scale; scale.setIdentity();
                    scale[0][0] *= x;
                    scale[1][1] *= y;
                    scale[2][2] *= z;

                    Matrix4f btoa = expand3To4(frameA * frameB.transpose() * scale);

                    // now we ICP partB with partA to remove the last uncertainty in localization
                    //PointCloud* partC = (PointCloud*)partB->copy(); partC->setTransformation(btoa);
                    //addPointCloud(getScene(), partC, std::string("before_T_") + intToStr(t));
                    {
                        FSIcp* icp = new FSIcp;
                        icp->setupData(partA, getScene());
                        icp->setupParameters(m_Settings->maxIcpIterations, 2, getCachedMedianPointDistance(partB), m_Settings->icpOutlierDistanceFactor, m_Settings->icpMinPercentInliers, 1e20f, -1, 1e-5f, false);			
                        float residual = icp->calculateResidualAfterICP(partB->getPointSet(), btoa);
                        delete icp;
                    }

                    // compute inverse matrix for aligning a to b
                    Matrix4f atob = expand3To4(shrink4To3(btoa).transpose());
                    atob[3] = expand3To4(shrink4To3(-btoa[3]));

                    //partC = (PointCloud*)partB->copy(); partC->setTransformation(btoa);
                    //addPointCloud(getScene(), partC, std::string("after_T_") + intToStr(t));

                    float d = computeHausdorffDistance(partA, partB, atob, btoa);
                    if (d < dist)
                    {
                        dist = d;

                        Tbtoa = btoa;
                        Tatob = atob;
                        TposA = posA;
                        TposB = posB;
                    }

                    if (dist < m_Settings->shapeSymmThreshold)
                        break;
                }

                //debugOutput << "\tdist " << k << "->" << l << " : " << dist;
                if (dist < m_Settings->shapeSymmThreshold)
                {
                    //debugRenderer->addLine(posA, posB, rainbowGradient(dist), rainbowGradient(dist), 3.0f);
                    //addPointCloud(getScene(), (PointCloud*)partB->copy(), intToStr(l));

                    //debugOutput << " (symmetric) " << allparts[k].gcid << " - " << allparts[l].gcid;

                    // store this symmetry information 
                    ccsg->symmetryT[allparts[k].gcid].push_back(Tatob + makeTranslation4f(TposB - TposA));
                    ccsg->symmetryId[allparts[k].gcid].push_back(allparts[l].gcid);

                    ccsg->symmetryT[allparts[l].gcid].push_back(Tbtoa + makeTranslation4f(TposA - TposB));
                    ccsg->symmetryId[allparts[l].gcid].push_back(allparts[k].gcid);
                }
                //debugOutput << "\n";
            }
        }
        progress->popStep();

    }
    debugOutput << "done\n";
    debugRenderer->endRenderJob();
}
#endif 

#include <detector/SymmetryGroupDetector.h>

//--------------------------------------------------------------------------------------

std::vector<sym::SymmetryGroupAttachment*> PCIConsistentCrossShapeGrammar::findLatticeGroups(FeatureSet* fs, DVectorUObjectList* clusters, ProgressWindow* progress)
{
    debugOutput << "find lattice structures in line clusters\n";

    std::vector<sym::SymmetryGroupAttachment*> groupsResult;
    {
        sym::SymmetryGroupDetector symDetector;
        symDetector.setAngleTolerance(m_Settings->angleTolerance);
        symDetector.setSpatialTolerance(m_Settings->spatialTolerance);
        symDetector.setMinNumRotations(m_Settings->minNumRotations);
        symDetector.setSymmetryCoveredPercentage(m_Settings->symmetryCoveredPercentage);
        symDetector.setLatticeMinGeneratorLength(m_Settings->latticeMinGeneratorLength);
        symDetector.setGridAngleTolerance(m_Settings->gridAngleTolerance);
        symDetector._doVeryConservativeRegionGrowing = m_Settings->doVeryConservativeRegionGrowing;

        symDetector.setUseDebugOutput(true);

        groupsResult = symDetector.findLatticeGroups(fs, nullptr /*clusters*/, progress);
    }


    debugOutput << "found " << groupsResult.size() << " lattice groups\n";

    return groupsResult;
}

//--------------------------------------------------------------------------------------

std::vector<sym::SymmetryGroupAttachment*> PCIConsistentCrossShapeGrammar::findOtherGroups(FeatureSet* fs, DVectorUObjectList* clusters, ProgressWindow* progress)
{
    std::vector<sym::SymmetryGroupAttachment*> groupsResult;
    {
        sym::SymmetryGroupDetector symDetector;
        symDetector.setAngleTolerance(m_Settings->angleTolerance);
        symDetector.setSpatialTolerance(m_Settings->spatialTolerance);
        symDetector.setMinNumRotations(m_Settings->minNumRotations);
        symDetector.setSymmetryCoveredPercentage(m_Settings->symmetryCoveredPercentage);
        symDetector.setUseDebugOutput(true);

        //std::vector<sym::SymmetryGroupAttachment*> groups =  
        groupsResult = symDetector.findOnePointGroups(fs, clusters, progress, showCluster);
        //for (unsigned i=0; i < groups.size(); i++)
        //	groupsResult.push_back(CCSGSymmetryGroup::buildFromSymmetryGroupAttachment(groups[i]));
    }

#if 0
    // DEBUG show groups
    for (unsigned i=0; i < groupsResult.size() && showFoundGroups; i++)
    {
        CCSGSymmetryGroup* group = groupsResult[i];

        sym::Reflection* ref = dynamic_cast<sym::Reflection*>(group->group);
        sym::Rotation* rot = dynamic_cast<sym::Rotation*>(group->group);

        debugRenderer->beginRenderJob_OneFrame(std::string("group_"), i);

        if (ref)
        {
            BoundingBox3f bbox;
            sym::ReflectionGenerator* gen = dynamic_cast<sym::ReflectionGenerator*>(ref->getGenerator());
            for (unsigned si=0; si < group->lines.size(); si++)
            {
                sym::FeatureLine* line1 = dynamic_cast<sym::FeatureLine*>(fs->m_Features[group->lines[si]]);
                debugRenderer->addLine(line1->getPosition(), line1->getEndPosition(),
                    gen->getParameterCoordinate(line1->getPosition())[0] < 0 ? makeVector3f(1,0,1) : makeVector3f(0,1,1),
                    gen->getParameterCoordinate(line1->getEndPosition())[0] < 0 ? makeVector3f(1,0,1) : makeVector3f(0,1,1),
                    3.0f);
                bbox.addPoint(line1->getPosition());
                bbox.addPoint(line1->getEndPosition());
            }
            debugRenderer->addPlane(gen->getPlane(), bbox.getMaxSideLength(), makeVector3f(1,1,0));
        }

        if (rot)
        {
            sym::RotationGenerator* gen = dynamic_cast<sym::RotationGenerator*>(rot->getGenerator());
            for (unsigned si=0; si < group->lines.size(); si++)
            {
                sym::FeatureLine* line3 = dynamic_cast<sym::FeatureLine*>(fs->m_Features[group->lines[si]]);
                debugRenderer->addLine(line3->m_Position, line3->m_Position + line3->m_Length * line3->m_Direction, makeVector3f(1,0,1), makeVector3f(1,0,1), 3.0f);
            }

            debugRenderer->addLine(gen->getRotationAxisCenter(), gen->getRotationAxisCenter() + gen->getRotationAxis(), makeVector3f(1,0,0), makeVector3f(1,0,0), 3.05f);

            sym::FeatureLine* line1 = dynamic_cast<sym::FeatureLine*>(fs->m_Features[group->lines[0]]);
            (dynamic_cast<sym::RotationGenerator*>(rot->getGenerator()))->getRigidMotion()->draw(line1->getCenterPosition(), makeVector3f(1,0,0), 20);

            //gen->rigidMotion->draw(line1->getCenterPosition(), makeVector3f(1,0,0), 20);
        }

        debugOutput << "[" << i << "] - " << " : "   << group->key << "\n";

        debugRenderer->endRenderJob();
    }
#endif

    return groupsResult;
}

#if 0
//--------------------------------------------------------------------------------------

std::vector<UnstructuredInCorePointCloud*> PCIConsistentCrossShapeGrammar::extractLatticeGroupElements(
    std::vector<sym::SymmetryGroupAttachment*>& groups, 
    UnstructuredInCorePointCloud* shape, 
    InCorePCTopologyGraph* tpg, 
    FeatureSet* fs, 
    DVectorUObjectList* clusters, 
    std::vector<char>& assigned)
{
    ProgressWindow* progress = getProgressWindow();

    std::vector<UnstructuredInCorePointCloud*> resultShapes;

    HierarchicalKNNIterator hIt(shape, 32, nullptr);
    hIt.setMaxDistanceToSeekPoint(m_Settings->spatialTolerance);

    std::vector<int32> pointGroupMap(shape->getNumPoints(), -1);

    AAT flagsAAT = shape->getAAT("flags");
    AAT posAAT = shape->getAAT("position");

    for (unsigned gc=0; gc < groups.size(); gc++)
    {
        sym::SymmetryGroupAttachment*& group = groups[gc];
        group->group->setSymCoverage(m_Settings->symmetryCoveredPercentage);
        group->group->setSpatialTolerance(m_Settings->spatialTolerance);
        group->group->setAngleTolerance(m_Settings->angleTolerance);

        debugOutput << "\t[" << gc << "] - check group [" << group->id << "] " << group->key << "\n";	

        // before we start detection of base elements, we need to check if the lines are already covered
        // a coverage means that there was already region growing performed where the points of those lines
        // were already assigned to another group. in case if the assignment happened for the same type of group
        // then we don't have to extract elements again and can ignore these group of lines
        bool alreadyCovered = false;
        for (std::vector<mpcard>::iterator it=group->lines.begin(); it != group->lines.end(); )
        {
            sym::FeatureLine* line = dynamic_cast<sym::FeatureLine*>(fs->m_Features[*it]);
            if (!line)
            {
                it = group->lines

                    // TODO: take care when taking center point. in case of triangulated shape, center point might not have a correct vertex in the near
                    hIt.setSeekPointAndReset(line->getCenterPosition());
                while (!hIt.atEnd() && !alreadyCovered)
                {
                    alreadyCovered = pointGroupMap[hIt.getCurrentPointIndex()] >= 0;
                    hIt.next();
                }
            }
            if (alreadyCovered)
            {
                debugOutput << "\t[" << gc << "] - no need, already covered before\n";
                delete group;
                group = nullptr;
                continue;
            }

            // detect generator and the rest
            std::vector<mpcard> invalidPoints;
            pair<std::vector<mpcard>, std::vector<mpcard>> result = group->group->detectBaseElements_Improved(shape, tpg, fs, group->lines, m_Settings->useICPWhileDetectingBaseElements, &invalidPoints);				
            if (result.first.size() < m_Settings->minConnectedComponent) 
            {
                delete group;
                group = nullptr;
                continue;
            }

            // put found group points on a side for later check
            std::vector<mpcard>& generatorPoints = group->symmetryPoints;
            {
                generatorPoints = result.first;
                sort(generatorPoints.begin(), generatorPoints.end());
                generatorPoints.resize(unique(generatorPoints.begin(), generatorPoints.end()) - generatorPoints.begin());
            }

            // HACK: removes the possibility to detect group within lattice elements
            //result.second.insert(result.second.end(), generatorPoints.begin(), generatorPoints.end()); 
            {
                sort(result.second.begin(), result.second.end());
                result.second.resize(unique(result.second.begin(), result.second.end()) - result.second.begin());
            }

            // mark all points found within the grid with the group id
            for (unsigned i=0; i < result.second.size(); i++)
            {
                pointGroupMap[result.second[i]] = group->id;
                assigned[result.second[i]]++;
            }
            /*for (unsigned i=0; i < generatorPoints.size(); i++)
            {
            pointGroupMap[generatorPoints[i]] = group->id;
            assigned[generatorPoints[i]]++;
            }*/

            UnstructuredInCorePointCloud* redundantGeometry = new UnstructuredInCorePointCloud;
            redundantGeometry->setPointSet(shape->getPointSet()->subset(result.second));
            redundantGeometry->clearAttachments();

            HierarchicalKNNIterator cIt(redundantGeometry, 32, nullptr);
            cIt.setMaxDistanceToSeekPoint(m_Settings->spatialTolerance);

            // mark all points within the symmetric region with invalid flag, so that they are not further processed
            {
                std::vector<char> marked(shape->getNumPoints(), 0);
                for (unsigned i=0; i < result.second.size(); i++)
                {
                    if (marked[result.second[i]]) continue;
                    PointCloudIterator* titer = tpg->createIterator(shape->getPointSet());
                    TopologicalKNNIterator_Fast* tIt = new TopologicalKNNIterator_Fast(shape, titer);
                    tIt->setStartPointAndReset(result.second[i]);
                    while(!tIt->atEnd())
                    {
                        marked[tIt->getPointSetPointNumber()] = 1;
                        tIt->next();
                    }
                    delete tIt;
                    delete titer;
                }
                for (unsigned i=0; i < shape->getNumPoints(); i++)
                {
                    if (!marked[i]) continue;
                    cIt.setSeekPointAndReset(shape->getPointSet()->get3f(i,posAAT));
                    if (cIt.atEnd()) continue;
                    shape->getPointSet()->set1i(i, flagsAAT, PF_FLAG_INVALID);
                }
            }

            for (unsigned i=0; i < generatorPoints.size(); i++)
                shape->getPointSet()->set1i(generatorPoints[i], flagsAAT, 0);

            // remove all feature lines, which are covered by the points in the symmetry (not by the generator)
            for (unsigned cl=0; cl < clusters->getNumLists(); cl++)
            {
                //DVectorUObject* elems = dynamic_cast<DVectorUObject*>(clusters->getList(cl));

                std::vector<unsigned> elems = dynamic_cast<DVectorUObject*>(clusters->getList(cl))->toStd();

                for (std::vector<unsigned>::iterator it = elems.begin(); it != elems.end(); )
                {
                    sym::FeatureLine* line = dynamic_cast<sym::FeatureLine*>(fs->m_Features[*it]);
                    if (!line)
                    {
                        it++;
                        continue;
                    }

                    // TODO: take care when taking center point. in case of triangulated shape, center point might not have a correct vertex in the near
                    cIt.setSeekPointAndReset(line->getCenterPosition());
                    if (cIt.atEnd())
                    {
                        it++;
                        continue;
                    }

                    delete line; 
                    fs->m_Features[*it] = nullptr;
                    it = elems.erase(it);
                }
                dynamic_cast<DVectorUObject*>(clusters->getList(cl))->fromStd(elems);

            }
            addPointCloud(scene, redundantGeometry, std::string("red_") + intToStr(group->id));
            //delete redundantGeometry;

        } // end for each lattice group

        // extract all base elements
        for (unsigned j=0; j < groups.size(); j++)
        {
            if (!groups[j]) continue;

            // perform some clean-up on the found data (remove small unconnected components)
            UnstructuredInCorePointCloud* pc = nullptr;
            if (groups[j]->clearGeometrically(shape, nullptr, fs, getCachedMedianPointDistance(shape), m_Settings->minConnectedComponent, &pc, false, false, false, nullptr))
            {
                delete groups[j];
                groups[j] = nullptr;
                continue;
            }

            sym::SymmetryGroupAttachment* group = (sym::SymmetryGroupAttachment*)groups[j]->copy();
            group->setup("SymmetryGroup", AttachedData::ADF_PERSISTENT);
            pc->getAttachments()->attachData(group);
            resultShapes.push_back(pc);
            debugOutput << "\tfound new grid [" << group->id <<"] " << group->key << "\n";
        }		
        debugOutput << "done\n";

        return resultShapes;
    }
}
#endif

//--------------------------------------------------------------------------------------

std::vector<UnstructuredInCorePointCloud*> PCIConsistentCrossShapeGrammar::extractOtherGroupElements(
    std::vector<sym::SymmetryGroupAttachment*>& groups,
    UnstructuredInCorePointCloud* shape, 
    InCorePCTopologyGraph* tpg,
    FeatureSet* fs, 
    DVectorUObjectList* clusters, 
    std::vector<char>& assigned)
{
    ProgressWindow* progress = getProgressWindow();

    srand(42);
    srandom(42);

    debugOutput << "analyze and consolidate groups\n";

    //std::vector<sym::SymmetryGroupAttachment*> _groups;
    sym::SymmetryGroupDetector symDetector;
    {
        symDetector.setAngleTolerance(m_Settings->angleTolerance);
        symDetector.setSpatialTolerance(m_Settings->spatialTolerance);
        symDetector.setMinNumRotations(m_Settings->minNumRotations);
        symDetector.setSymmetryCoveredPercentage(m_Settings->symmetryCoveredPercentage);
        symDetector.setLatticeMinGeneratorLength(m_Settings->latticeMinGeneratorLength);
        symDetector.setGridAngleTolerance(m_Settings->gridAngleTolerance);
        symDetector.setNumAvailableThreads(m_Settings->getNumAvailableThreads());

        symDetector._rotGroupCompareSigma = m_Settings->rotGroupCompareSigma;
        symDetector._refGroupCompareSigma = m_Settings->refGroupCompareSigma;

        symDetector._symGroupCompareThreshold = m_Settings->symGroupCompareThreshold;
        symDetector._useICPWhileDetectingBaseElements = m_Settings->useICPWhileDetectingBaseElements;
        symDetector._doVeryConservativeRegionGrowing = m_Settings->doVeryConservativeRegionGrowing;
        symDetector._minConnectedComponent = m_Settings->minConnectedComponent;
        symDetector._symGroupCombineAngleTolerance = m_Settings->symGroupCombineAngleTolerance;

        symDetector.setUseDebugOutput(true);

        symDetector.consolidateOnePointGroups(groups, shape, tpg, fs, progress);
    }

    debugOutput << "combine to complex groups and extract geometry\n";

    for (std::vector<sym::SymmetryGroupAttachment*>::iterator it = groups.begin(); it != groups.end(); )
    {
        if (*it == nullptr) it = groups.erase(it);
        else it++;
    }

    struct SortPredicate
    {
        static bool sort(const sym::SymmetryGroupAttachment* a, const sym::SymmetryGroupAttachment* b)  { return a->symmetryPoints.size() < b->symmetryPoints.size(); }
    };
    sort(groups.begin(), groups.end(), SortPredicate::sort);

    std::vector<sym::SymmetryGroupAttachment*> _groups(groups.size());
    std::copy(groups.begin(), groups.end(), _groups.begin());
    groups.resize(0);

    float meddist = getCachedMedianPointDistance(shape);


    // extract symmetric geometry from all groups in parallel (remove very tiny groups)
    std::vector<UnstructuredInCorePointCloud*> resultShapes;
    omp_set_num_threads(m_Settings->getNumAvailableThreads());
#pragma omp parallel for shared(resultShapes, _groups, assigned) ordered schedule(dynamic)
    for (int j=0; j < _groups.size(); j++)
    {
        UnstructuredInCorePointCloud* pc = nullptr;

#pragma omp critical
        {
            debugOutput << j << "/" << _groups.size()-1 << " - symmetric check of [" << _groups[j]->id << "] : " << _groups[j]->key << "\n";
        }

        if (_groups[j]->clearGeometrically(shape, tpg, fs, meddist, m_Settings->minConnectedComponent, &pc, true/*true*/, false, true, m_Settings->doVeryConservativeRegionGrowing, nullptr, false))
        {
#pragma omp critical
            {
                debugOutput << "\t[" << _groups[j]->id << "] cleared geometrically, non-symmetric\n";
            }
            delete _groups[j];
            continue;
        }
        if (pc->getPointSet()->getBoundingBox().getMaxSideLength() < shape->getPointSet()->getBoundingBox().getMaxSideLength() * 0.1f)
        {
#pragma omp critical
            {
                debugOutput << "\t[" << _groups[j]->id << "] small bounding box\n";
            }
            delete pc;
            delete _groups[j];
            continue;
        }

        pc->clearAttachments();
        _groups[j]->update(shape);

#pragma omp ordered
        {
            resultShapes.push_back(pc);
            groups.push_back(_groups[j]);
        }
    }

    groups.erase(std::remove(groups.begin(), groups.end(), nullptr), groups.end());
    // perform final test, where we remove groups which are spurious
    _groups = symDetector.removeSpuriousGroups(getScene(), shape, groups);
    //_groups = symDetector.removeCoveredSimplerGroups(groups);
    for (int j=0; j < groups.size(); j++)
    {
        // just based on the pointer address we check if a group was removed from the given list or not
        if (std::find(_groups.begin(), _groups.end(), groups[j]) == _groups.end())
        {
            delete resultShapes[j];
            resultShapes[j] = nullptr;
            groups[j] = nullptr;
        }
    }

    // extract all base elements
    std::vector<UnstructuredInCorePointCloud*> _resultShapes;
    for (int j=0,i=0; j < groups.size(); j++)
    {
        if (groups[j] == nullptr) continue;

        UnstructuredInCorePointCloud* pc = resultShapes[j];
        sym::SymmetryGroupAttachment* resgroup = (sym::SymmetryGroupAttachment*)groups[j]->copy();
        resgroup->setup("SymmetryGroup", AttachedData::ADF_PERSISTENT);
        pc->getAttachments()->attachData(resgroup);
        _resultShapes.push_back(pc);

        if (pc->providesAttribute("flags"))
        {
            for (unsigned k=0; k < pc->getNumPoints(); k++)
                pc->getPointSet()->set1i(k, pc->getAAT("flags"), 0);
        }

        for (unsigned k=0; k < resgroup->symmetryPoints.size(); k++)
        {
            assigned[resgroup->symmetryPoints[k]] ++;
        }

        debugOutput << i++ << "  -  found new symmetric element in group [" << resgroup->id <<"] " << resgroup->key << "\n";
    }		

    // remove invalid shapes and groups
    groups = _groups;
    resultShapes = _resultShapes;

    std::reverse(resultShapes.begin(), resultShapes.end());
    std::reverse(groups.begin(), groups.end());

    debugOutput << "done\n";


    return std::vector<UnstructuredInCorePointCloud*>(resultShapes);
}


//--------------------------------------------------------------------------------------
pair<UnstructuredInCorePointCloud*, std::vector<UnstructuredInCorePointCloud*>> PCIConsistentCrossShapeGrammar::detectAndExtractFinalSymmetries(
    const std::string& shapeName,
    UnstructuredInCoreTriangleMesh* mesh, 
    FeatureSet* fs, 
    DVectorUObjectList* cluster)
{
    pair<UnstructuredInCorePointCloud*, CCSGClassification*> shapeccsg = findPreliminarySymmetryGroups(shapeName, mesh, fs, cluster);

    UnstructuredInCorePointCloud* shape = shapeccsg.first;
    CCSGClassification* ccsg = shapeccsg.second;

    FeatureSet* _fs = dynamic_cast<FeatureSet*>(shape->getAttachments()->getData("FeatureLines"));
    DVectorUObjectList* _clusters = dynamic_cast<DVectorUObjectList*>(shape->getAttachments()->getData("LineClusters"));
    InCorePCTopologyGraph* tpg = dynamic_cast<TopologyAttachment*>(shape->getAttachments()->getData(TopologyAttachment::getDefaultName()))->getTopology();

    return extractFinalSymmetricGroups(shape, fs, cluster, ccsg, tpg);
}

//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::showStatisticsAndLogs()
{
    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    if (selection.size() != 1) 
    {
        error("Please select only one node for which symmetries were computed");
        return;
    }
    SGObjectNode* ssnode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[0]));

    SGListNode* root = dynamic_cast<SGListNode*>(getScene()->getRootNode());
    SGObjectNode* selnode = dynamic_cast<SGObjectNode*>(root->getChildNode(nullptr, root->getChildIndex(std::string("__sym_work_shape_") + ssnode->getName())));
    if (!selnode)
    {
        error(std::string("No temporary processing node (") + std::string("__sym_work_shape_") + ssnode->getName() + ") found.");
        return;
    }

    UnstructuredInCorePointCloud* shape = dynamic_cast<UnstructuredInCorePointCloud*>(selnode->getSceneObject());
    CCSGClassification* ccsg = dynamic_cast<CCSGClassification*>(shape->getAttachments()->getData(CCSGClassification::getDefaultName()));

    debugOutput << "------------- LOGS --------------\n";
    debugOutput << ccsg->loggedOutput << "\n";

    debugOutput << "------------- STATS --------------\n";
    debugOutput << ccsg->stats->outputAll();	
}

//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::extractFinalSymmetricGroups()
{
    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    if (selection.size() != 1) 
    {
        error("Please select only one node for which symmetries were computed");
        return;
    }
    SGObjectNode* ssnode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[0]));

    SGListNode* root = dynamic_cast<SGListNode*>(getScene()->getRootNode());
    SGObjectNode* selnode = dynamic_cast<SGObjectNode*>(root->getChildNode(nullptr, root->getChildIndex(std::string("__sym_work_shape_") + ssnode->getName())));
    if (!selnode)
    {
        error(std::string("No temporary processing node (") + std::string("__sym_work_shape_") + ssnode->getName() + ") found. You have first to find preliminary groups");
        return;
    }

    if (ssnode->getParent(0))
        selectSymmetryGroupName = ssnode->getParent(0)->getName() + std::string("_") + ssnode->getName() + "_groups";
    else
        selectSymmetryGroupName = "data_0_groups";

    UnstructuredInCorePointCloud* shape = dynamic_cast<UnstructuredInCorePointCloud*>(selnode->getSceneObject());

    FeatureSet* _fs = dynamic_cast<FeatureSet*>(shape->getAttachments()->getData("FeatureLines"));
    DVectorUObjectList* _clusters = dynamic_cast<DVectorUObjectList*>(shape->getAttachments()->getData("LineClusters"));
    CCSGClassification* ccsg = dynamic_cast<CCSGClassification*>(shape->getAttachments()->getData(CCSGClassification::getDefaultName()));
    InCorePCTopologyGraph* tpg = dynamic_cast<TopologyAttachment*>(shape->getAttachments()->getData(TopologyAttachment::getDefaultName()))->getTopology();

    //debugOutput.clearLoggedOutput();
    //debugOutput.enableLogging();
    pair<UnstructuredInCorePointCloud*, std::vector<UnstructuredInCorePointCloud*>> result = extractFinalSymmetricGroups(shape, _fs, _clusters, ccsg, tpg);
    //ccsg->loggedOutput = debugOutput.getLoggedOutput();
    //debugOutput.disableLogging();

    SGRelativeTimeAnimationNode* listnode = new SGRelativeTimeAnimationNode;
    listnode->setName(selectSymmetryGroupName);
    {
        SGListNode* sublist = listnode;
        //SGListNode* sublist = new SGListNode;
        //sublist->setName("0");
        //listnode->addChildNode(sublist);


        if (result.first)
        {
            SGObjectNode* nonsym = new SGObjectNode(result.first);
            nonsym->setName("nonsym");
            sublist->addChildNode(nonsym);
        }

        for(unsigned i=0; i < result.second.size(); i++)
        {
            SGObjectNode* sym = new SGObjectNode(result.second.at(i));
            sym->setName(intToStr(i + (result.first ? 1 : 0)));
            sublist->addChildNode(sym);
        }
    }
    getScene()->getRootNode()->addChildNode(listnode);

    //if (result.first) addPointCloud(getScene(), result.first, "nonsymmetric");

    //for(unsigned i=0; i < result.second.size(); i++)
    //	addPointCloud(getScene(), result.second.at(i), std::string("element_") + intToStr(i));
}

//--------------------------------------------------------------------------------------
pair<UnstructuredInCorePointCloud*, std::vector<UnstructuredInCorePointCloud*>> PCIConsistentCrossShapeGrammar::extractFinalSymmetricGroups(
    UnstructuredInCorePointCloud* shape,
    FeatureSet* _fs,
    DVectorUObjectList* _clusters,
    CCSGClassification* ccsg,
    InCorePCTopologyGraph* tpg
    )
{
    std::vector<CCSGSymmetryGroup*>& _groups = ccsg->m_Group;
    std::vector<sym::SymmetryGroupAttachment*>& _symGroups = ccsg->m_symGroup;
    for (unsigned i=0; i < _groups.size(); i++)
    {
        _symGroups.push_back(CCSGSymmetryGroup::buildSymmetryGroupAttachment(_groups[i]));
        delete _groups[i];
    }
    _groups.clear();


    // split groups into lattice and normal groups, because due to historical reasons we have two methods to extract main elements from there
    // we create copies of each object with which we further work with
    std::vector<sym::SymmetryGroupAttachment*> latticeGroup;
    std::vector<sym::SymmetryGroupAttachment*> otherGroup;
    for (unsigned i=0; i < _symGroups.size(); i++) 
    {
        if (dynamic_cast<sym::Lattice*>(_symGroups[i]->group)) latticeGroup.push_back((sym::SymmetryGroupAttachment*)_symGroups[i]->copy());
        else otherGroup.push_back((sym::SymmetryGroupAttachment*)_symGroups[i]->copy());
    }

    FeatureSet* fs = (FeatureSet*)_fs->copy();
    DVectorUObjectList* clusters = (DVectorUObjectList*)_clusters->copy();
    std::vector<char> assigned(shape->getNumPoints(), 0);

    checkAttribute(shape, "flags", 1, VAD::DATA_FORMAT_INT32);
    AAT flagsAAT = shape->getAAT("flags");
    for (unsigned i=0; i < shape->getNumPoints(); i++)
        shape->getPointSet()->set1i(i, flagsAAT, 0);

    ccsg->stats->clearTimer("Extract_Lattice");
    ccsg->stats->startTimer("Extract_Lattice");
    std::vector<UnstructuredInCorePointCloud*> results = extractOtherGroupElements(latticeGroup, shape, tpg, fs, clusters, assigned);
    ccsg->stats->stopTimer("Extract_Lattice");


    // mark all points found in the lattice groups as invalid, so that they are not considered in the subsequent search
    // might be easier just to mark points stored in the group attachments for each of the group
    {
        HierarchicalKNNIterator hIt(shape, 32, nullptr);
        hIt.setMaxDistanceToSeekPoint(getCachedMedianPointDistance(shape) * 2.0f);
        PointCloudIterator* titer = tpg->createIterator(shape->getPointSet());
        TopologicalKNNIterator_Fast* tIt = new TopologicalKNNIterator_Fast(shape, titer);
        AAT shapePosAAT = shape->getAAT("position");
        AAT flagsAAT = shape->getAAT("flags");

        for (unsigned i=0; i < results.size(); i++)
        {
            AAT posAAT = results[i]->getAAT("position");
            for (unsigned j=0; j < results[i]->getNumPoints(); j++)
            {
                Vector3f pos = results[i]->getPointSet()->get3f(j, posAAT);
                hIt.setSeekPointAndReset(pos);
                if (hIt.atEnd()) continue;
                tIt->setStartPointAndReset(hIt.getCurrentPointIndex());
                while(!tIt->atEnd())
                {
                    if (norm(tIt->get3f(shapePosAAT) - pos) > m_Settings->spatialTolerance) break;
                    shape->getPointSet()->set1i(tIt->getPointSetPointNumber(), flagsAAT, PF_FLAG_INVALID);
                    tIt->next();
                }
            }
        }
        delete tIt;
        delete titer;
    }

    ccsg->stats->clearTimer("Extract_OnePoint");
    ccsg->stats->startTimer("Extract_OnePoint");
    std::vector<UnstructuredInCorePointCloud*> resultsR = extractOtherGroupElements(otherGroup, shape, tpg, fs, clusters, assigned);
    ccsg->stats->stopTimer("Extract_OnePoint");

    for (unsigned i=0; i < shape->getNumPoints(); i++)
        shape->getPointSet()->set1i(i, flagsAAT, 0);

    results.insert(results.end(), resultsR.begin(), resultsR.end());

    // compute the non-symmetric part
    std::vector<mpcard> nonsymmetric;
    for (unsigned i=0; i < shape->getNumPoints(); i++)
        if (!assigned[i]) nonsymmetric.push_back(i);
    UnstructuredInCorePointCloud* nonsym = shape->createCopyWithNewPointSet(shape->getPointSet()->subset(nonsymmetric));


    // cleanup the non-symmetric part
    {
        nonsym->clearAttachments();
        PCCComputeTopology cmd;
        cmd.setup(PCCComputeTopology::TOPTYPE_EPS, m_Settings->spatialTolerance);
        InCorePCTopologyGraph* _tpg = cmd.computeEpsTopology(nonsym->getPointSet(), nonsym);
        removeSmallPatches(nonsym, _tpg, nullptr, m_Settings->minConnectedComponent);
        delete _tpg;
    }

    if (nonsym->getNumPoints() < m_Settings->minConnectedComponent)
    {
        delete nonsym;
        nonsym = nullptr;
    }


    for (unsigned i=0; i < latticeGroup.size(); i++) delete latticeGroup[i];
    for (unsigned i=0; i < otherGroup.size(); i++) delete otherGroup[i];
    delete fs;
    delete clusters;

    return pair<UnstructuredInCorePointCloud*, std::vector<UnstructuredInCorePointCloud*>>(nonsym, results);
}

//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::debugShowAllPreliminaryGroups()
{
    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    if (selection.size() != 1) 
    {
        error("Please select only one node for which symmetries were computed");
        return;
    }
    SGObjectNode* ssnode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[0]));

    SGListNode* root = dynamic_cast<SGListNode*>(getScene()->getRootNode());
    SGObjectNode* selnode = dynamic_cast<SGObjectNode*>(root->getChildNode(nullptr, root->getChildIndex(std::string("__sym_work_shape_") + ssnode->getName())));
    if (!selnode)
    {
        error(std::string("No temporary processing node (") + std::string("__sym_work_shape_") + ssnode->getName() + ") found. You have first to find preliminary groups");
        return;
    }
    UnstructuredInCorePointCloud* shape = dynamic_cast<UnstructuredInCorePointCloud*>(selnode->getSceneObject());

    CCSGClassification* ccsg = dynamic_cast<CCSGClassification*>(shape->getAttachments()->getData(CCSGClassification::getDefaultName()));

    getScene()->getRootNode()->removeChildNode("preliminary_symmetries");
    SGRelativeTimeAnimationNode* list = new SGRelativeTimeAnimationNode;
    list->setName("preliminary_symmetries_1");
    for (unsigned i=0; i < ccsg->m_Group.size(); i++)
    {
        UnstructuredInCoreTriangleMesh* _shape = new UnstructuredInCoreTriangleMesh;;
        _shape->setPointSet((PointSet*)shape->getPointSet()->copy());
        CCSGSymmetryGroup* group = (CCSGSymmetryGroup*)ccsg->m_Group[i]->copy();
        _shape->getAttachments()->attachData(group);

        debugOutput << i << " - " << "[" << ccsg->m_Group[i]->id << "] " << ccsg->m_Group[i]->key << "\n";

        SGObjectNode* node = new SGObjectNode(_shape);
        node->setName(intToStr(group->id));
        list->addChildNode(node);
    }
    SGRelativeTimeAnimationNode* list2 = new SGRelativeTimeAnimationNode;
    list2->setName("preliminary_symmetries_2");
    for (unsigned i=0; i < ccsg->m_symGroup.size(); i++)
    {
        UnstructuredInCoreTriangleMesh* _shape = new UnstructuredInCoreTriangleMesh;;
        _shape->setPointSet((PointSet*)shape->getPointSet()->copy());
        sym::SymmetryGroupAttachment* group = (sym::SymmetryGroupAttachment*)ccsg->m_symGroup[i]->copy();
        _shape->getAttachments()->attachData(group);

        debugOutput << i + ccsg->m_Group.size() << " - " << "[" << ccsg->m_symGroup[i]->id << "] " << ccsg->m_symGroup[i]->key << "\n";

        SGObjectNode* node = new SGObjectNode(_shape);
        node->setName(intToStr(group->id));
        list2->addChildNode(node);
    }

    debugOutput << "done, found " << ccsg->m_symGroup.size() + ccsg->m_Group.size() << "\n";

    getScene()->getRootNode()->addChildNode(list);
    getScene()->getRootNode()->addChildNode(list2);
}

//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::findPreliminarySymmetryGroups()
{
    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    if (selection.size() != 1) 
    {
        error("Please select only one node for which symmetries should be detected");
        return;
    }
    SGObjectNode* selnode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[0]));
    UnstructuredInCoreTriangleMesh* selcloud = dynamic_cast<UnstructuredInCoreTriangleMesh*>(selnode->getSceneObject());
    if (!selcloud)
    {
        error("This method works with triangle meshes only");
        return;
    }

    FeatureSet* fs = dynamic_cast<FeatureSet*>(selcloud->getAttachments()->getData("FeatureLines"));
    if (fs == nullptr)
    {
        error("the shape must have feature lines computed on it first");
        return;
    }

    DVectorUObjectList* clusters = dynamic_cast<DVectorUObjectList*>(selcloud->getAttachments()->getData("LineClusters"));
    if (clusters == nullptr)
    {
        error("You need to cluster feature lines first");
        return;
    }

    findPreliminarySymmetryGroups(selnode->getName(), selcloud, fs, clusters);
}

//--------------------------------------------------------------------------------------
pair<UnstructuredInCorePointCloud*,CCSGClassification*> PCIConsistentCrossShapeGrammar::findPreliminarySymmetryGroups(const std::string& shapeName, UnstructuredInCoreTriangleMesh* selcloud, FeatureSet* _fs, DVectorUObjectList* _clusters)
{
    CCSGClassification* ccsg = new CCSGClassification();
    ccsg->setup(CCSGClassification::getDefaultName(), AttachedData::ADF_PERSISTENT);

    srand(42);
    srandom(42);

    FeatureSet* fs = (FeatureSet*)_fs->copy();
    DVectorUObjectList* clusters = (DVectorUObjectList*)_clusters->copy();

    ProgressWindow* progress = getProgressWindow();

    std::vector<sym::SymmetryGroupAttachment*> groups;

    // lattice groups
    if (m_Settings->searchLattice)
    {
        ccsg->stats->startTimer("Detect_Lattice");

        debugOutput << "search lattice groups\n";
        const std::vector<sym::SymmetryGroupAttachment*> _groups = findLatticeGroups(fs, clusters, progress);
        for (unsigned i=0; i < _groups.size(); i++)groups.push_back(_groups[i]);

        ccsg->stats->stopTimer("Detect_Lattice");
    }

    // rotation, reflection groups
    ccsg->stats->startTimer("Detect_OnePoint");
    {
        debugOutput << "look for other group types\n";
        const std::vector<sym::SymmetryGroupAttachment*> _groups = findOtherGroups(fs, clusters, progress);
        for (unsigned i=0; i < _groups.size(); i++) groups.push_back(_groups[i]);
    }
    ccsg->stats->stopTimer("Detect_OnePoint");

    for (unsigned i=0; i < groups.size(); i++)
    {
        groups[i]->featureSet = fs;
        groups[i]->group->setSymCoverage(m_Settings->symmetryCoveredPercentage);
        groups[i]->group->setSpatialTolerance(m_Settings->spatialTolerance);
        groups[i]->group->setAngleTolerance(m_Settings->angleTolerance);
        groups[i]->id = i;
        groups[i]->update();
    }


    debugOutput << "upsample shape for later element extraction and group validation\n";
    UnstructuredInCorePointCloud* shape = nullptr;
    InCorePCTopologyGraph* tpg = nullptr;
    {
        UnstructuredInCoreTriangleMesh* _shape = (UnstructuredInCoreTriangleMesh*)selcloud->copy();
        _shape->calculateVertexNormals();
        shape = PCCTriangleMeshSampler::sampleMeshPoisson(_shape, m_Settings->symMeshSubsampling, 0);
        delete _shape;
        checkAttribute(shape, "flags", 1, VAD::DATA_FORMAT_INT32);
        shape->clearAttachments();

        AAT cidAAT = NULL_AAT;
        if (shape->providesAttribute("cid"))
            cidAAT = shape->getAAT("cid");

        // add end-points from line features and assign them to the nearest vertex
        {
            const VertexDescriptor *destVertexDescr = shape->getDescr();
            char *destBuffer = new char[destVertexDescr->getSize()];
            for (unsigned i=0; i < fs->m_Features.size(); i++)
            {
                sym::FeatureLine* line = dynamic_cast<sym::FeatureLine*>(fs->m_Features[i]);
                if (!line) continue;

                char* vbuff = selcloud->getPointSet()->getDataPointer(line->m_nearestPoint[0]);
                memcpy(destBuffer, vbuff, destVertexDescr->getSize());
                shape->getPointSet()->addPoint(destBuffer);

                vbuff = selcloud->getPointSet()->getDataPointer(line->m_nearestPoint[1]);
                memcpy(destBuffer, vbuff, destVertexDescr->getSize());
                shape->getPointSet()->addPoint(destBuffer);
            }
            delete destBuffer;
        }

        fs->setup("FeatureLines", AttachedData::ADF_PERSISTENT);
        clusters->setup("LineClusters", AttachedData::ADF_PERSISTENT);
        shape->getAttachments()->attachData(fs);
        shape->getAttachments()->attachData(clusters);
        shape->getAttachments()->attachData(ccsg);

        PCCComputeTopology cmd;
        cmd.setup(PCCComputeTopology::TOPTYPE_EPS, m_Settings->symMeshSubsampling * 2.5f);//m_Settings->spatialTolerance);
        tpg = cmd.computeEpsTopology(shape->getPointSet(), shape);

        // HACK: reuse segmentation if provided
        if (m_Settings->reuseSegmentation)
        {
            const PointSet* edges = tpg->getEdges();
            AAT vindex = edges->getAAT("vindex");

            InCorePCTopologyGraph* tpgnew = new InCorePCTopologyGraph;
            tpgnew->clearAndSetup((VertexDescriptor*)tpg->getDescr()->copy());

            for (unsigned i=0; i < tpg->getNumEdges(); i++)
            {
                Vector2i vi = edges->get2i(i, vindex);
                if (cidAAT == NULL_AAT || shape->getPointSet()->get2i(vi[0], cidAAT)[1] == shape->getPointSet()->get2i(vi[1], cidAAT)[1])
                    tpgnew->addEdge(vi[0],vi[1]);
            }
            delete tpg;
            tpg = tpgnew;
        }

        getScene()->getRootNode()->removeChildNode(std::string("__sym_work_shape_") + shapeName);
        addPointCloud(scene, shape, std::string("__sym_work_shape_") + shapeName);

        TopologyAttachment* tatt= new TopologyAttachment;
        tatt->setup(TopologyAttachment::getDefaultName(), AttachedData::ADF_PERSISTENT);
        tatt->setTopology(tpg);
        tatt->setVisible(false);
        shape->getAttachments()->attachData(tatt);

        SGListNode* root = dynamic_cast<SGListNode*>(getScene()->getRootNode());
        SGObjectNode* node = dynamic_cast<SGObjectNode*>(root->getChildNode(nullptr, root->getChildIndex(std::string("__sym_work_shape_") + shapeName)));
        node->setVisible(false);
    }


    for (std::vector<sym::SymmetryGroupAttachment*>::iterator it = groups.begin(); it != groups.end(); )
    {
        if (*it /*&& (*it)->group->getNumTransformations() == 6*/) it++;
        else it = groups.erase(it);
    }


    for (unsigned i=0; i < groups.size(); i++)
    {
        debugOutput << i << " - " << "[" << groups[i]->id << "] " << groups[i]->key << "\n";
        ccsg->addsymGroup(groups[i]);
    }
    debugOutput << "done, found " << groups.size() << "\n";

    return pair<UnstructuredInCorePointCloud*, CCSGClassification*>(shape, ccsg);
}

#include <PCCNormalizeAnimation.h>
#include <PointCloudImporterReaderSMFOBJ.h>
#include <PointCloudImporterAdder.h>
//----------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::importMesh()
{
    /*std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    if (selection.size() != 1)
    {
    error("You have to select a group node (i.e. class of shapes) where the mesh will be imported");
    return;

    }

    SGListNode* node = dynamic_cast<SGListNode*>(getSceneGraphNodeByName(getScene(), selection[0]));
    if (!node)
    {
    error("Selected node is not a list node\n");
    return;
    }

    card32 id = node->getNumChildNodes(nullptr);*/

    // ask user for a directory
    std::string path;
    path = FileDialogs::getOpenFileName(InterfaceGlobals::getMainWindow(), "Open Mesh File",
        "Mesh Files (*.obj; *.off; *.dae)", nullptr, "", FileDialogs::IMPORT_DIR);
    //{
    //    struct AskPath : public OperatorNoArgsLockedResult<std::string>
    //    {
    //        void operator()()
    //        {
    //            set(FileDialogs::getOpenFileName(InterfaceGlobals::getMainWindow(), "Open Mesh File",
    //                "Mesh Files (*.obj; *.off; *.dae)", nullptr, "", FileDialogs::IMPORT_DIR));
    //        }
    //    };
    //    AskPath msg; execOperatorNoArgs(&msg);
    //    path = msg.get();
    //}; 

    UnstructuredInCoreTriangleMesh* tri = nullptr;
    {
        PointCloudImporterReaderSMFOBJ importer;
        if (importer.beginImport(path, true))
        {
            tri = new UnstructuredInCoreTriangleMesh;
            tri->clearAndSetupTriangles(importer.getTriangleDescriptor());
            tri->clearAndSetup(importer.getVertexDescriptor());

            PointCloudImporterAdderMesh _adder; _adder.doImport(&importer, tri);
        }
    }

    // import file
    //UnstructuredInCoreTriangleMesh* tri = ImportMeshOpenAsset::import(path);
    if (tri == nullptr)
    {
        warning(std::string("Cannot read file: ") + path );
        return;
    }else
    {
        SGListNode* node = new SGListNode;
        node->setName("data");
        node->setVisible(false);
        getScene()->getRootNode()->addChildNode(node);

        tri->closeMesh();
        tri->cleanTopology();

        SGObjectNode* obj = new SGObjectNode(tri);
        obj->setName(intToStr(0));
        node->addNode(obj);
        debugOutput << "imported " << path << "\n";
        tri->setMaterialIndex(2);
        tri->computeTopologyAttachment();			

        TopologyAttachment* tatt = dynamic_cast<TopologyAttachment*>(tri->getAttachments()->getData(TopologyAttachment::getDefaultName()));
        tatt->setLineColor(makeVector4f(0,0,0,1.5));
        tatt->setVisible(false);

        PCCNormalizeAnimation normCmd;
        normCmd.unifySizeAndPosition(node);

        int cid = 0;
        std::vector<SGObjectNode*> sgmts = segmentObject(tri, tatt->getTopology(), cid);
        for (unsigned i=0; i < sgmts.size(); i++) delete sgmts[i];

        getScene()->rebuildRenderObjects();
    }
}


//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::constructDisjointRegions()
{
    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    if (selection.size() != 1)
    {
        error("This function can only be performed on one shape at a time");
        return;
    }

    SGObjectNode* objnode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[0]));
    if (!objnode)
    {
        warning("You need to select a shape first");
        return;
    }

    SGListNode* root = dynamic_cast<SGListNode*>(getScene()->getRootNode());

    std::string groupname;
    if (objnode->getParent(0))
        groupname = std::string("root/") + objnode->getParent(0)->getName() + std::string("_") + objnode->getName() + "_groups";

    SGListNode* groupnode = dynamic_cast<SGListNode*>(getSceneGraphNodeByName(getScene(), groupname));
    if (!groupnode)
    {
        error(std::string("I wasn't able to find a list node (") + groupname + std::string(") in the scene which contains all the symmetry groups of the selected cloud"));
        return;
    }

    SGObjectNode* selnode = dynamic_cast<SGObjectNode*>(root->getChildNode(nullptr, root->getChildIndex(std::string("__sym_work_shape_") + objnode->getName())));
    if (!selnode)
    {
        error(std::string("I can't find processing point cloud: ") + std::string("root/__sym_work_shape_") + objnode->getName());
        return;
    }
    UnstructuredInCorePointCloud* selpc = dynamic_cast<UnstructuredInCorePointCloud*>(selnode->getSceneObject());
    CCSGClassification* ccsg = dynamic_cast<CCSGClassification*>(selpc->getAttachments()->getData(CCSGClassification::getDefaultName()));
    if (!ccsg)
    {
        error("Selected point cloud must have a CCSGClassification attached. Did you performed the symmetry detection before that?");
        return;
    }

    // very simple dynamic bitset datastructure (works only for bitsets of the same size) 
    struct DynamicBitset
    {
        std::vector<card64> bits;
        DynamicBitset() { }
        explicit DynamicBitset(int size){ bits.resize(size/64+1, 0); }
        void set(int pos, bool val)
        {
            if (val)
                bits[pos/64] |= 1ULL << card64(pos%64);
            else
                bits[pos/64] &= ~(1ULL << card64(pos%64));
        }
        bool operator<(const DynamicBitset& b) const
        {
            bool result = bits[0] < b.bits[0];
            for (int i=1; i < bits.size() && result; i++)
                result = result && (bits[i] < b.bits[i]);
            return result;
        }
    };

    std::vector<DynamicBitset> sets(selpc->getNumPoints());
    debugOutput << "Hash points by their group affiliation ... ";

    // iterate over all points and hash points by the set of groups it belongs to
    for (unsigned g=0; g < groupnode->getNumChildNodes(nullptr); g++)
    {
        SGListNode* group = dynamic_cast<SGListNode*>(groupnode->getChildNode(nullptr, g));
        if (group)
        {
            debugOutput << "This shape was created with old symmetry groups. Please refactor them into new groups or recompute them!\n";
            return;
        }

        { // new version (without subgroups)

            // extract geometry and symmetry group
            SGObjectNode* node = dynamic_cast<SGObjectNode*>(groupnode->getChildNode(nullptr, g));
            UnstructuredInCorePointCloud* data = (UnstructuredInCorePointCloud*)node->getSceneObject();
            sym::SymmetryGroupAttachment* symgroup = nullptr;
            if (data->getAttachments()->getData("SymmetryGroup"))
            {
                symgroup = dynamic_cast<sym::SymmetryGroupAttachment*>(data->getAttachments()->getData("SymmetryGroup"));
            }

            if (!symgroup && node->getName() != "nonsym")
            {
                debugOutput << "node " << group->getName() << "/" << node->getName() << " does not contain valid symmetry information. Output might be incorrect\n";
                continue;
            }else if (symgroup)
            {
                // set for each point covered by this symmetry group its affiliation to the symmetry group
                for (size_t i=0, size = symgroup->symmetryPoints.size(); i < size; i++)
                {
                    if (sets[symgroup->symmetryPoints[i]].bits.size() == 0)
                    {
                        sets[symgroup->symmetryPoints[i]] = DynamicBitset(groupnode->getNumChildNodes(nullptr));
                    }
                    sets[symgroup->symmetryPoints[i]].set(g,1);
                }
            }
        }
    }
    debugOutput << "DONE\ncreate regions as point clouds ...\n";

    // we use now the hashes to separate the points
    std::map<DynamicBitset, std::vector<card32>> regions;
    for (size_t i=0, size = sets.size(); i < size; i++)
    {
        if (sets[i].bits.size())
        {
            regions[sets[i]].push_back(i);
        }
    }

    SGRelativeTimeAnimationNode* listnode = new SGRelativeTimeAnimationNode;
    listnode->setName(objnode->getParent(0)->getName() + std::string("_") + objnode->getName() + "_regions");
    getScene()->getRootNode()->addChildNode(listnode);

    // store all the regions in the shape attachment
    for (std::map<DynamicBitset, std::vector<card32>>::iterator jt = regions.begin(); jt != regions.end(); jt++)
    {
        int id = std::distance(regions.begin(), jt);

        debugOutput << "region [" << id << "] -> groups: ";

        // mark region
        DVectorI groupids;
        ccsg->regions[id] = jt->second;
        for (unsigned i=0; i < jt->first.bits.size(); i++)
        {
            card64 bits = jt->first.bits[i];
            for (card64 j=0; j < 64 && bits; j++)
            {
                card64 tmp = 1ULL << j;
                if (bits & tmp)
                {
                    ccsg->regionGroupsId[id].push_back(i*64+j);
                    DVectorI group(1);
                    group[0] = i*64+j;
                    groupids.appendInPlace(group);
                    debugOutput << group[0] << ", ";
                    bits &= ~(1 << j);
                }
            }
        }
        debugOutput << "\n";

        // extract point cloud
        UnstructuredInCorePointCloud* pc = new UnstructuredInCorePointCloud();
        pc->setPointSet(selpc->getPointSet()->subset(jt->second));

        SimpleAttachment::setDVector(pc->getAttachments(), "GroupIDs", groupids);

        SGObjectNode* sym = new SGObjectNode(pc);
        sym->setName(intToStr(id));
        listnode->addChildNode(sym);
    }
    debugOutput << "DONE\nfound " << regions.size() << " regions";
}

#include "SToolBox/DefieldSymm.h"
#include "Eigen/Dense"
void PCIConsistentCrossShapeGrammar::exportTransformations()
{
    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    if (selection.size() != 1)
    {
        error("This function can only be performed on one shape at a time");
        return;
    }

    SGObjectNode* objnode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[0]));
    if (!objnode)
    {
        warning("You need to select a shape first");
        return;
    }

    UnstructuredInCorePointCloud* symPC = (UnstructuredInCorePointCloud*)objnode->getSceneObject();
    if (nullptr == symPC) {
        error("selected node is not a symmetry UICPC.");
        return;
    }

    if (!symPC->getAttachments()->getData("SymmetryGroup")) {
        error("no symmetry group attachment found.");
        return;
    }

    sym::SymmetryGroupAttachment* symgroup = dynamic_cast<sym::SymmetryGroupAttachment*>
        (symPC->getAttachments()->getData("SymmetryGroup"));

    DefieldSymm* defSymm;
    if (0 > DefieldSymm::CreateFromSG(symgroup->group, &defSymm)) {
        error("symmetry group can not be created");
        return;
    };

    std::string filename = FileDialogs::getSaveFileName(InterfaceGlobals::getMainWindow(), "save symmetry transformations", "*.box");
    if (filename != "")
    {
        std::ofstream fs;
        std::string line;
        fs.open(filename.c_str());

        {
            //const unsigned numTrans = symgroup->group->getNumTransformations();
            //fs << "# " << numTrans << " matrices" << std::endl;
            //for (unsigned t = 0; t < numTrans; ++t) {
            //    const Matrix4f trans = symgroup->group->getGenerator()->getWorldTransformation(t);
            //    Eigen::Matrix4f teig;
            //    teig <<
            //        trans[0][0], trans[1][0], trans[2][0], trans[3][0],
            //        trans[0][1], trans[1][1], trans[2][1], trans[3][1],
            //        trans[0][2], trans[1][2], trans[2][2], trans[3][2],
            //        trans[0][3], trans[1][3], trans[2][3], trans[3][3];

            //    debugOutput << trans << "\n";
            //    fs << "# " << t << std::endl;
            //    fs << teig << std::endl;
            //}
        }

        {
            const unsigned numTrans = defSymm->GetNumTransformation();
            fs << "# " << numTrans << " matrices" << std::endl;
            for (unsigned t = 0; t < numTrans; ++t) {
                const Matrix4f trans = defSymm->GetTransformation(t);
                Eigen::Matrix4f teig;
                teig <<
                    trans[0][0], trans[1][0], trans[2][0], trans[3][0],
                    trans[0][1], trans[1][1], trans[2][1], trans[3][1],
                    trans[0][2], trans[1][2], trans[2][2], trans[3][2],
                    trans[0][3], trans[1][3], trans[2][3], trans[3][3];

                debugOutput << trans << "\n";
                fs << "# " << t << std::endl;
                fs << teig << std::endl;
            }
        }

        fs.close();
    }
}

#include "BinaryObjectStreams.h"
//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::exportSymmetries()
{
    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    if (selection.size() != 1)
    {
        error("This function can only be performed on one shape at a time");
        return;
    }

    SGObjectNode* objnode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[0]));
    if (!objnode)
    {
        warning("You need to select a shape first");
        return;
    }

    SGListNode* root = dynamic_cast<SGListNode*>(getScene()->getRootNode());

    std::string groupname;
    if (objnode->getParent(0))
        groupname = std::string("root/") + objnode->getParent(0)->getName() + std::string("_") + objnode->getName() + "_groups";

    SGListNode* groupnode = dynamic_cast<SGListNode*>(getSceneGraphNodeByName(getScene(), groupname));
    if (!groupnode)
    {
        error(std::string("I wasn't able to find a list node (") + groupname + std::string(") in the scene which contains all the symmetry groups of the selected cloud"));
        return;
    }


    // ask for a directory where to export whole information to
    std::string path;
    path = FileDialogs::getExistingDirectory(InterfaceGlobals::getMainWindow(), "Select directory to export", "", FileDialogs::SAVE_DIR);
    //{
    //    struct AskPath : public OperatorNoArgsLockedResult<std::string>
    //    {
    //        void operator()()
    //        {
    //            set(FileDialogs::getExistingDirectory(InterfaceGlobals::getMainWindow(), "Select directory to export", "", FileDialogs::SAVE_DIR));
    //        }
    //    };

    //    // ask user to specify type of the symmetry
    //    AskPath msg; execOperatorNoArgs(&msg);
    //    path = msg.get();
    //}; // ask path

    if (path.length() == 0) return;

    ProgressWindow* progress = getProgressWindow();

    // write out all parts to the directory
    struct PC
    {
        std::vector<card32> pts;
        UnstructuredInCorePointCloud* data;
        sym::SymmetryGroup* group;
    };
    std::vector<PC> pcs;

    std::vector<RegionElement> regions;

    std::vector<UnstructuredInCorePointCloud*> allparts;
    UnstructuredInCorePointCloud* pointcloud;

    card32 sumAllPoints = 0;
    std::vector<card32> subNumPoints;


    {
        SGObjectNode* selnode = dynamic_cast<SGObjectNode*>(root->getChildNode(nullptr, root->getChildIndex(std::string("__sym_work_shape_") + objnode->getName())));
        if (!selnode)
        {
            error(std::string("I can't find processing point cloud: ") + std::string("root/__sym_work_shape_") + objnode->getName());
            return;
        }
        UnstructuredInCorePointCloud* selpc = dynamic_cast<UnstructuredInCorePointCloud*>(selnode->getSceneObject());
        subNumPoints.push_back(selpc->getNumPoints());
        sumAllPoints += selpc->getNumPoints();
        allparts.push_back(selpc);

        pointcloud = selpc;
    }

    debugOutput << "prepare meshes for export\n";
    progress->pushStep(true, "prepare");
    for (unsigned g=0; g < groupnode->getNumChildNodes(nullptr); g++)
    {
        progress->progressf(float(g) / float(groupnode->getNumChildNodes(nullptr)));
        SGListNode* group = dynamic_cast<SGListNode*>(groupnode->getChildNode(nullptr, g));
        if (group) // old version - using subgroups
        {
            error ("Old versioned symmetry export was deprecated, but you have symmetries computed with an old version, recompute!");
            return;
#if 0
            SGObjectNode* selnode = dynamic_cast<SGObjectNode*>(root->getChildNode(nullptr, root->getChildIndex(std::string("__sym_work_shape__tmp_") + group->getName())));
            if (!selnode)
            {
                error(std::string("I can't find processing point cloud: ") + std::string("root/__sym_work_shape__tmp_") + group->getName());
                return;
            }
            UnstructuredInCorePointCloud* selpc = dynamic_cast<UnstructuredInCorePointCloud*>(selnode->getSceneObject());
            subNumPoints.push_back(selpc->getNumPoints());
            sumAllPoints += selpc->getNumPoints();
            allparts.push_back(selpc);

            for (unsigned i=0; i < group->getNumChildNodes(nullptr); i++)
            {
                SGListNode* subgroup = dynamic_cast<SGListNode*>(group->getChildNode(nullptr, i));
                if (!subgroup || subgroup->getName() != "subgroups") continue;

                for (unsigned j=0; j < subgroup->getNumChildNodes(nullptr); j++)
                {
                    PC output;

                    SGObjectNode* node = dynamic_cast<SGObjectNode*>(subgroup->getChildNode(nullptr, j));
                    output.data = (UnstructuredInCorePointCloud*)node->getSceneObject();
                    output.group = nullptr;
                    if (output.data->getAttachments()->getData("SymmetryGroup"))
                    {
                        output.group = dynamic_cast<CCSGSymmetryGroup*>(output.data->getAttachments()->getData("SymmetryGroup"))->group;
                    }

                    if (!output.group && node->getName() != "nonsym")
                    {
                        debugOutput << "node " << group->getName() << "/" << node->getName() << " does not contain valid symmetry information. Output might be incorrect\n";
                        continue;
                    }

                    /*if (output.group && output.group->getInstanceClass() == sym::Lattice::getClass())
                    {
                    sym::Lattice* lat = dynamic_cast<sym::Lattice*>(output.group);
                    Vector2i mincoord = makeVector2i(lat->getMinParameterCoordinate()[0], lat->getMinParameterCoordinate()[1]);
                    Vector2i maxcoord = makeVector2i(lat->getMaxParameterCoordinate()[0], lat->getMaxParameterCoordinate()[1]);
                    output.data = lat->applyLattice(output.data, mincoord, maxcoord);
                    }*/

                    pcs.push_back(output);
                }
            }
#endif

        }else // new version - no subgroups
        {
            PC output;

            // Symmetry group and its point cloud
            SGObjectNode* node = dynamic_cast<SGObjectNode*>(groupnode->getChildNode(nullptr, g));
            output.data = (UnstructuredInCorePointCloud*)node->getSceneObject();
            output.group = nullptr;
            sym::SymmetryGroupAttachment* symgroup = nullptr;
            if (output.data->getAttachments()->getData("SymmetryGroup"))
            {
                symgroup = dynamic_cast<sym::SymmetryGroupAttachment*>(output.data->getAttachments()->getData("SymmetryGroup"));
                output.group = symgroup->group;
                /*if (!output.group)
                {
                CCSGSymmetryGroup* ccsymgroup = dynamic_cast<CCSGSymmetryGroup*>(output.data->getAttachments()->getData("SymmetryGroup"));
                output.pts = ccsymgroup->symmetryPoints;
                output.group = ccsymgroup->group;
                }*/

                output.pts.resize(symgroup->symmetryPoints.size());
                for (size_t k=0, size = symgroup->symmetryPoints.size(); k < size; k++)
                    output.pts[k] = (card32)symgroup->symmetryPoints[k];
            }

            if (!output.group && node->getName() != "nonsym")
            {
                debugOutput << "node " << group->getName() << "/" << node->getName() << " does not contain valid symmetry information. Output might be incorrect\n";
                continue;
            }

            /*if (output.group && output.group->getInstanceClass() == sym::Lattice::getClass())
            {
            sym::Lattice* lat = dynamic_cast<sym::Lattice*>(output.group);
            Vector2i mincoord = makeVector2i(lat->getMinParameterCoordinate()[0], lat->getMinParameterCoordinate()[1]);
            Vector2i maxcoord = makeVector2i(lat->getMaxParameterCoordinate()[0], lat->getMaxParameterCoordinate()[1]);
            output.data = lat->applyLattice(output.data, mincoord, maxcoord);
            }*/

            pcs.push_back(output);
        }
    }
    progress->popStep();


    // -------------------------------------
    // Extract regions
    // -------------------------------------
    {
        std::string regionsname;
        if (objnode->getParent(0))
            regionsname = std::string("root/") + objnode->getParent(0)->getName() + std::string("_") + objnode->getName() + "_regions";

        SGListNode* regionsnode = dynamic_cast<SGListNode*>(getSceneGraphNodeByName(getScene(), regionsname));
        if (!regionsnode)
        {
            error(std::string("I wasn't able to find a list node (") + regionsname + std::string(") in the scene which contains all the disjoint regions of the selected cloud"));
            return;
        }

        for (unsigned g=0; g < regionsnode->getNumChildNodes(nullptr); g++)
        {
            RegionElement output;

            SGObjectNode* node = dynamic_cast<SGObjectNode*>(regionsnode->getChildNode(nullptr, g));
            output.pc = (UnstructuredInCorePointCloud*)node->getSceneObject();

            DVectorI ids = SimpleAttachment::getDVector<int>(output.pc->getAttachments(), "GroupIDs");
            for (int j=0; j < ids.getDim(); j++)
            {
                output.groups.push_back(ids[j]);
            }

            regions.push_back(output);
        }
    }



    // compute volume of the whole model (orient model based on pca analyzis and then compute bounding box size)
    Vector3f pcaValues;
    float bboxLength = 0;
    {
        // analyze it and compute pca
        //debugRenderer->beginRenderJob("bbox");

        // build set of all available points
        std::vector<Vector3f> allpoints;
        for (unsigned i=0; i < allparts.size(); i++)
            for (unsigned j=0; j < allparts[i]->getNumPoints(); j++)
                allpoints.push_back(allparts[i]->getPointSet()->get3f(j, allparts[i]->getAAT("position")));


        std::vector<Vector3f> points;
        //float meddist = m_Settings->spatialTolerance;//
        //float meddist = getMedianPointDistance(allparts[0].data);
        //debugOutput << "meddist : "  << meddist << "\n";
        //meddist *= meddist;
        std::vector<DVectorF> dataForPCA;
        LotteryDraw draw;
        draw.setup(allpoints.size());
        for (unsigned i=0; i < 10000 && !draw.empty(); i++)
        {
            Vector3f pos = allpoints[draw.draw()];
            bool found = false;
            for (unsigned j=0; j < points.size() && !found; j++)
                found = (points[j] - pos).getSqrNorm() < m_Settings->spatialTolerance * m_Settings->spatialTolerance;
            if(found) continue;

            dataForPCA.push_back(PCAGeneral::convertVec3fToDVec(std::vector<Vector3f>(1,pos)));
            points.push_back(pos);
        }

        PCAGeneral pca; pca.calculatePCA(dataForPCA, -1, 3);
        DMatrixF pcamatr = pca.getEigenVecs();

        //pcaValues[0] = pca.getEigenValues()[0];
        //pcaValues[1] = pca.getEigenValues()[1];
        //pcaValues[2] = pca.getEigenValues()[2];
        Vector3f mean = pca.getMeanVecAsVec3f()[0];

        Matrix3f T;
        T[0] = normalize(PCAGeneral::convertDVtoVec3f(pcamatr[0])[0]);
        T[1] = normalize(PCAGeneral::convertDVtoVec3f(pcamatr[1])[0]);
        T[2] = normalize(PCAGeneral::convertDVtoVec3f(pcamatr[2])[0]);

        Matrix4f Tb = makeTranslation4f(mean) * expand3To4(T);

        BoundingBox3f bbox(makeVector3f(0,0,0));
        for (unsigned i=0; i < allparts.size(); i++)
        {
            for (unsigned j=0; j < allparts[i]->getNumPoints(); j++)
            {
                Vector3f pos = allparts[i]->getPointSet()->get3f(j, allparts[i]->getAAT("position"));
                pos -= mean;
                pos = shrink4To3(invertFrame(expand3To4(T))) * pos;
                //pos += mean;
                bbox.addPoint(pos);
                //debugRenderer->addPoint(pos, makeVector3f(0,1,0));
            }
        }
        pcaValues[0] = bbox.getSideLength(0);
        pcaValues[1] = bbox.getSideLength(1);
        pcaValues[2] = bbox.getSideLength(2);

        bboxLength = std::max(pcaValues[0], std::max(pcaValues[1], pcaValues[2]));

        /*debugRenderer->addCoordSystem(mean, T, 1.0f);

        debugRenderer->addFastSphere(mean, 0.05f, makeVector3f(1,0,0));
        debugRenderer->addBoundingBox(bbox, makeVector3f(1,0,0), 3.0f);
        debugRenderer->endRenderJob();*/
    }

    // ----------------------
    // Export information about the data
    // ----------------------
    debugOutput << "export data info to disk: pca = " << pcaValues << "\n";
    {
        std::ofstream metaf((path + std::string("/") + "data.dat").c_str());
        metaf << sumAllPoints << "\n";
        metaf << pcaValues[0] << " " << pcaValues[1] << " " << pcaValues[2] << "\n";
        metaf << subNumPoints.size() << "\n";
        for (unsigned i=0; i < subNumPoints.size(); i++)
            metaf << subNumPoints[i] << "\n";
    }

    debugOutput << "export group info to disk\n";
    {
        std::ofstream metagroups((path + std::string("/") + "data.grp").c_str());
        for (unsigned i=0; i < pcs.size(); i++)
        {
            sym::SymmetryGroup* group = pcs[i].group;
            if (group == nullptr)
                metagroups << i << " NONSYM : _\n";
            else
            {
                metagroups << i << " " << group->getInstanceClass()->getClassName() << "\n";// : " << group->serialize() << "\n";

                BinaryOutputObjectStream io((path + std::string("/") + intToStr(i) + "_group.object").c_str());
                io.writeObject(group);
            }

            // export list of point indices into the original point cloud
            if(pcs[i].pts.size())
            {
                FILE* file = fopen((path + std::string("/") + intToStr(i) + ".idx").c_str(), "wb");
                card32 size = pcs[i].pts.size();
                fwrite(&size, sizeof(card32),1, file);
                fwrite(&pcs[i].pts[0], sizeof(card32), pcs[i].pts.size(), file);
                fclose(file);
            }
        }
    }

    struct CacheValue
    {
        card32 A;
        card32 B;
        float dist;
        //bool allptswithin;
        //bool allptswithin_rev;
        bool numOverlapBPoints;
        bool numOverlapAPoints;
    };
    struct CacheKey
    {
        CacheKey() : a(-1), b(-1) {}
        CacheKey(mpcard a, mpcard b) : a(a), b(b) {}
        mpcard a;
        mpcard b;
        bool operator < (const CacheKey& key) const
        { 
            if (a > key.a) return false;
            else return b < key.b;
        }
    };
    std::map<CacheKey, CacheValue> cache;

    float meddist = getCachedMedianPointDistance(allparts[0]);



    // ----------------------
    // Export edges - binary relations
    // ----------------------
    // test each pair of exported elements, which are near to each other and add them as an edge
    groupEdgeMap.clear();
    debugOutput << "find proximity relationships, treshold " << m_Settings->spatialTolerance * m_Settings->proximityRelationFactor << ", meddist = "  <<  meddist << "\n";
    progress->pushStep("proximity");
    int _k = 0;
    //#pragma omp parallel for
    for (unsigned i=0; i < pcs.size(); i++)
    {
        UnstructuredInCorePointCloud* pcA = pcs[i].data;
        UnstructuredInCorePointCloud* _pcA = nullptr;

        // don't add any edge to non-symmetric parts
        if (!pcs[i].group) continue;

        //debugRenderer->beginRenderJob_OneFrame(std::string("graph_"),i);

        _pcA = (UnstructuredInCorePointCloud*)pcA->copy();
        {
            //addPointCloud(getScene(), (PointCloud*)_pcA->copy(), intToStr(i));
            //debugOutput << "check " << i << " (" << _pcA->getNumPoints() << ") points and group : " << (pcs[i].group ? pcs[i].group->getName() : "NOSYM") << "\n";
        }

        BoundingBox3f bboxA = pcA->getPointSet()->getBoundingBox();

        _pcA->clearAttachments();
        HierarchicalKNNIterator hItA(_pcA, 32, nullptr);
        hItA.setMaxDistanceToSeekPoint(m_Settings->spatialTolerance * m_Settings->proximityRelationFactor);
        AAT posAAAT = _pcA->getAAT("position");

        for (unsigned j=i+1; j < pcs.size(); j++)
        {
#pragma omp atomic
            _k++;

#pragma omp critical
            {
                progress->progressf(float(_k) / float(pcs.size() * pcs.size()));
            }
            UnstructuredInCorePointCloud* pcB = nullptr;

            // don't add any edge to non-symmetric parts
            if (!pcs[j].group) continue;

            // In case if a symmetry group of this data is a grid,
            // we then need to add generated points to the data, making the data same size as it was in original data
            /*if (pcs[j].group && pcs[j].group->getInstanceClass() == sym::Lattice::getClass())
            {
            sym::Lattice* lat = dynamic_cast<sym::Lattice*>(pcs[j].group);
            Vector2i mincoord = makeVector2i(lat->getMinParameterCoordinate()[0], lat->getMinParameterCoordinate()[1]);
            Vector2i maxcoord = makeVector2i(lat->getMaxParameterCoordinate()[0], lat->getMaxParameterCoordinate()[1]);
            pcB = lat->applyLattice(pcs[j].data,mincoord,maxcoord);

            }else*/
            pcB = (UnstructuredInCorePointCloud*)pcs[j].data->copy();

            BoundingBox3f bboxB = pcB->getPointSet()->getBoundingBox();

            //mpcard pA = i;//(mpcard)pcA;
            //mpcard pB = j;//(mpcard)pcB;
            CacheKey key(std::min(i,j),std::max(i,j));

            //bool allptswithin = true;
            //bool allptswithin_rev = true;
            card32 numOverlapBPoints = 0;
            card32 numOverlapAPoints = 0;
            float dist = 1e20f;

            bool foundincache = false;

#pragma omp critical
            {
                foundincache = false;//(cache.find(key) != cache.end());
            }

            if (!foundincache)
            {
                if (bboxA.intersectBoundingBox(bboxB) == BVQueryDisjoint && 
                    (bboxA.getBoundingSphere().center - bboxB.getBoundingSphere().center).getNorm() - bboxA.getBoundingSphere().radius - bboxB.getBoundingSphere().radius > m_Settings->spatialTolerance * m_Settings->proximityRelationFactor)
                {
                    delete pcB;
                    continue;
                }

                AAT posBAAT = pcB->getAAT("position");

                numOverlapBPoints = 0;

                // points of A -> B
                for (unsigned p=0; p < pcB->getNumPoints(); p++)
                {
                    Vector3f posb = pcB->getPointSet()->get3f(p, posBAAT);
                    hItA.setSeekPointAndReset(posb);
                    if (hItA.atEnd()) continue;

                    float d = (posb - hItA.get3f(posAAAT)).getNorm();

                    if (d < meddist * 3.f) numOverlapBPoints++;
                    if (d < dist) dist = d;
                }

                // point of B -> A
                numOverlapAPoints = 0;
                HierarchicalKNNIterator hItB(pcB, 32, nullptr);
                hItB.setMaxDistanceToSeekPoint(m_Settings->spatialTolerance * m_Settings->proximityRelationFactor);
                for (unsigned p=0; p < pcA->getNumPoints(); p++)
                {
                    Vector3f posa = pcA->getPointSet()->get3f(p, posAAAT);
                    hItB.setSeekPointAndReset(posa);
                    if (hItB.atEnd()) continue;

                    float d = (posa - hItB.get3f(posBAAT)).getNorm();

                    if (d < meddist * 3.f) numOverlapAPoints++;
                    if (d < dist) dist = d;
                }

#pragma omp critical
                {
                    CacheValue val;
                    val.dist = dist;
                    //val.allptswithin = allptswithin;
                    //val.allptswithin_rev = allptswithin_rev;
                    val.A = i;
                    val.B = j;
                    val.numOverlapAPoints = numOverlapAPoints;
                    val.numOverlapBPoints = numOverlapBPoints;
                    cache[key] = val;

                    //debugOutput << "put " << key.a << ", " << key.b << " : " << dist << "\n";
                }
            }else
            {
#pragma omp critical
                {
                    const CacheValue& val = cache[key];
                    dist = val.dist;
                    //allptswithin = val.allptswithin;
                    //allptswithin_rev = val.allptswithin_rev;

                    if (val.A == i)
                    {
                        numOverlapAPoints = val.numOverlapAPoints;
                        numOverlapBPoints = val.numOverlapBPoints;
                    }else
                    {
                        numOverlapAPoints = val.numOverlapBPoints;
                        numOverlapBPoints = val.numOverlapAPoints;
                    }

                    //debugOutput << "get " << key.a << ", " << key.b << " : " << dist << "\n";
                }
            }

            card32 edgeTypeAtoB = 0;
            card32 edgeTypeBtoA = 0;
            float32 edgeWeightAtoB = 0;
            float32 edgeWeightBtoA = 0;

            if (dist < m_Settings->spatialTolerance * m_Settings->proximityRelationFactor)
            {
                bool bSubsetA = (float)numOverlapBPoints / (float)pcB->getNumPoints() > 0.9f;
                bool aSubsetB = (float)numOverlapAPoints / (float)pcA->getNumPoints() > 0.9f;

                // directed subset edges
                if (bSubsetA)
                {
                    edgeTypeBtoA |= GroupEdge::SUBSET;
                    edgeWeightBtoA = (float)numOverlapBPoints / (float)pcB->getNumPoints();
                }
                if (aSubsetB)
                {
                    edgeTypeAtoB |= GroupEdge::SUBSET;
                    edgeWeightAtoB = (float)numOverlapAPoints / (float)pcA->getNumPoints();
                }

                // if we don't have subset relation ship, then add undirected edges
                if (!bSubsetA && !aSubsetB)
                {
                    edgeWeightBtoA = 1.0 - dist / (m_Settings->spatialTolerance * m_Settings->proximityRelationFactor);
                    edgeWeightAtoB = 1.0 - dist / (m_Settings->spatialTolerance * m_Settings->proximityRelationFactor);

                    // check for overlap or subset condition B->A
                    if ((float)numOverlapBPoints / (float)pcB->getNumPoints() > 0.05f 
                        ||  (float)numOverlapAPoints / (float)pcA->getNumPoints() > 0.05f)
                    {
                        edgeTypeBtoA |= GroupEdge::OVERLAP | GroupEdge::SYMREL;
                        edgeTypeAtoB |= GroupEdge::OVERLAP | GroupEdge::SYMREL;
                    }else
                    {
                        edgeTypeBtoA |= GroupEdge::PROXIMITY | GroupEdge::SYMREL;
                        edgeTypeAtoB |= GroupEdge::PROXIMITY | GroupEdge::SYMREL;
                    }
                }

                // add also an edge if both symmetry groups coincides, that means that they can be measured in a later computeRelationMatricesStep
                if (bSubsetA || aSubsetB)
                {
                    sym::SymmetryGroup* grA = pcs[i].group;
                    sym::SymmetryGroup* grB = pcs[j].group;

                    sym::Rotation* rotA = dynamic_cast<sym::Rotation*>(grA);
                    sym::Reflection* refA = dynamic_cast<sym::Reflection*>(grA);
                    sym::Dihedral* dihA = dynamic_cast<sym::Dihedral*>(grA);
                    sym::Lattice* latA = dynamic_cast<sym::Lattice*>(grA);
                    Vector3f structVecA;
                    if (rotA) structVecA = rotA->getGenerator()->getRotationAxis();
                    if (refA) structVecA = refA->getGenerator()->plane.getNormal();
                    if (dihA) structVecA = dihA->getBaseRotation()->getGenerator()->getRotationAxis();
                    if (latA) structVecA = latA->getGenerator()->u();

                    sym::Rotation* rotB = dynamic_cast<sym::Rotation*>(grB);
                    sym::Reflection* refB = dynamic_cast<sym::Reflection*>(grB);
                    sym::Dihedral* dihB = dynamic_cast<sym::Dihedral*>(grB);
                    sym::Lattice* latB = dynamic_cast<sym::Lattice*>(grB);
                    Vector3f structVecB;
                    if (rotB) structVecB = rotB->getGenerator()->getRotationAxis();
                    if (refB) structVecB = refB->getGenerator()->plane.getNormal();
                    if (dihB) structVecB = dihB->getBaseRotation()->getGenerator()->getRotationAxis();
                    if (latB) structVecB = latB->getGenerator()->u();

                    bool spatCoincide = false;
                    if (rotA && rotB) spatCoincide = rotA->sameRotationAxis(rotB, m_Settings->spatialTolerance, m_Settings->angleTolerance);
                    if (refA && refB) spatCoincide = fabs(refA->getGenerator()->plane.calculateSignedDistance(refB->getGenerator()->plane.getPoint())) < m_Settings->spatialTolerance;
                    if (dihA && dihB) spatCoincide = dihA->getBaseRotation()->sameRotationAxis(dihB->getBaseRotation(), m_Settings->spatialTolerance, m_Settings->angleTolerance);

                    // if structural vectors do conicide
                    if (spatCoincide && fabs(normalize(structVecA) * normalize(structVecB)) >= 1.0f -  m_Settings->angleTolerance)
                    {
                        // A->B
                        {
                            edgeTypeAtoB |= GroupEdge::SYMREL;
                            edgeWeightAtoB = fabs(normalize(structVecA) * normalize(structVecB));
                        }
                        // A->B
                        {
                            edgeTypeBtoA |= GroupEdge::SYMREL;
                            edgeWeightBtoA = fabs(normalize(structVecA) * normalize(structVecB));
                        }
                    }
                }
            }


            // add edge if requested
            if (edgeTypeAtoB != 0)
            {
                GroupEdge e;
                e.A = i;
                e.B = j;
                e.type = edgeTypeAtoB;
                e.weight = edgeWeightAtoB;

#pragma omp critical
                groupEdgeMap.push_back(e);

                std::string type;
                if (edgeTypeAtoB & GroupEdge::OVERLAP) type += " OVERLAP";
                if (edgeTypeAtoB & GroupEdge::PROXIMITY) type += " PROXIMITY";
                if (edgeTypeAtoB & GroupEdge::SUBSET) type += " SUBSET";
                if (edgeTypeAtoB & GroupEdge::SYMREL) type += " SYMREL";

#pragma omp critical
                {
                    debugOutput << i << "->" << j << " " << type << "\n";
                }
                //debugRenderer->addFineArrow(pcA->getPointSet()->getBoundingBox().getCenter(), 
                //	pcB->getPointSet()->getBoundingBox().getCenter(), makeVector3f(1,0,0), 0.01f);
            }

            if (edgeTypeBtoA != 0)
            {
                GroupEdge e;
                e.A = j;
                e.B = i;
                e.type = edgeTypeBtoA;
                e.weight = edgeWeightBtoA;

#pragma omp critical
                groupEdgeMap.push_back(e);


                std::string type;
                if (edgeTypeBtoA & GroupEdge::OVERLAP) type += " OVERLAP";
                if (edgeTypeBtoA & GroupEdge::PROXIMITY) type += " PROXIMITY";
                if (edgeTypeBtoA & GroupEdge::SUBSET) type += " SUBSET";
                if (edgeTypeBtoA & GroupEdge::SYMREL) type += " SYMREL";

#pragma omp critical
                {
                    debugOutput << j << "->" << i << " " << type << "\n";
                }
                //debugRenderer->addFineArrow(pcB->getPointSet()->getBoundingBox().getCenter(), 
                //	pcA->getPointSet()->getBoundingBox().getCenter(), makeVector3f(1,0,0), 0.01f);
            }

            delete pcB;
        }

        //debugRenderer->endRenderJob();

        delete _pcA;
    }
    progress->popStep();

    // save relationship to file
    std::vector<char> exported(pcs.size(), 0);
    debugOutput << "export data to disk\n";
    {
        // edges
        std::ofstream metaf((path + std::string("/") + "data.edg").c_str());
        for (unsigned i=0; i < groupEdgeMap.size(); i++)
        {
            progress->progressf(float(i) / float(groupEdgeMap.size()));

            const GroupEdge& e = groupEdgeMap[i];
            metaf << e.A << " " << e.B << " " << e.type << " " << e.weight << "\n";
        }

        // point cloud of the whole shape
        {
            UnstructuredInCorePointCloud* copypc = new UnstructuredInCorePointCloud();
            copypc->clearAndSetup(pointcloud->getNumPoints(), true, false, false, false);
            AAT posAAT = copypc->getAAT("position");
            for (unsigned k=0; k < copypc->getNumPoints(); k++)
            {
                copypc->getPointSet()->set3f(k, posAAT, pointcloud->getPointSet()->get3f(k, pointcloud->getAAT("position")));
            }

            BinaryOutputObjectStream io((path + std::string("/data.object")).c_str());
            io.writeObject(copypc);
        }

        // point cloud of each of the groups
        for (unsigned i=0; i < pcs.size(); i++)
        {
            UnstructuredInCorePointCloud* copypc = new UnstructuredInCorePointCloud();
            copypc->clearAndSetup(pcs[i].data->getNumPoints(), true, true, false, false);
            AAT posAAT = copypc->getAAT("position");
            AAT nrmAAT = copypc->getAAT("normal");
            for (unsigned k=0; k < copypc->getNumPoints(); k++)
            {
                copypc->getPointSet()->set3f(k, posAAT, pcs[i].data->getPointSet()->get3f(k, pcs[i].data->getAAT("position")));
                copypc->getPointSet()->set3f(k, nrmAAT, normalize(pcs[i].data->getPointSet()->get3f(k, pcs[i].data->getAAT("normal"))));
            }

            BinaryOutputObjectStream io((path + std::string("/") + intToStr(i) + std::string(".pts.object")).c_str());
            io.writeObject(copypc);
            delete copypc;
        }
    }


    // ---------------------------------------------------------------
    // Regions - Edges and data
    // ---------------------------------------------------------------
    // save regions to files
    debugOutput << "export regions to disk\n";
    {
        // file containing the information about the groups of a region
        std::ofstream metaf((path + std::string("/") + "data.rgr").c_str());
        for (unsigned i=0; i < regions.size(); i++)
        {
            metaf << i << " [" << regions[i].groups.size() << "]\n\t";
            for (int j=0; j < regions[i].groups.size(); j++)
            {
                metaf << regions[i].groups[j] << " ";
            }
            metaf << "\n";

            // file containing the region point cloud
            UnstructuredInCorePointCloud* copypc = new UnstructuredInCorePointCloud();
            copypc->clearAndSetup(regions[i].pc->getNumPoints(), true, true, false, false);
            AAT posAAT = copypc->getAAT("position");
            AAT nrmAAT = copypc->getAAT("normal");
            for (unsigned k=0; k < copypc->getNumPoints(); k++)
            {
                copypc->getPointSet()->set3f(k, posAAT, regions[i].pc->getPointSet()->get3f(k, regions[i].pc->getAAT("position")));
                copypc->getPointSet()->set3f(k, nrmAAT, normalize(regions[i].pc->getPointSet()->get3f(k, regions[i].pc->getAAT("normal"))));
            }

            BinaryOutputObjectStream io((path + std::string("/") + intToStr(i) + std::string(".region.object")).c_str());
            io.writeObject(copypc);

            // export regions also as an ASCII point cloud, to import in another tool
            std::ofstream outf((path + std::string("/") + intToStr(i) + std::string(".pts")).c_str());
            PointCloudExporter exporter(copypc, &outf);
            exporter.exportPointCloud();

            delete copypc;
        }
    }


    // export graph of the regions, i.e. compute edges between regions and export them
    std::vector<GroupEdge> regionEdges;
    float maxdist = 0;
    {
        float threshold = m_Settings->spatialTolerance * m_Settings->proximityRelationFactor;
        debugOutput << "find region edges, treshold " << threshold << ", meddist = "  <<  meddist << "\n";
        progress->pushStep("proximity regions");

        for (int i=0, regionsize = regions.size(), _k = 0; i < regionsize; i++)
        {
            UnstructuredInCorePointCloud* pcA = regions[i].pc;
            //BoundingBox3f bboxA = pcA->getPointSet()->getBoundingBox();
            //HierarchicalKNNIterator hItA(pcA, 32, nullptr);
            //hItA.setMaxDistanceToSeekPoint(threshold);
            //AAT posAAAT = pcA->getAAT("position");

            //#pragma omp parallel for
            for (int j=i+1; j < regionsize; j++)
            {
#pragma omp atomic
                _k++;

#pragma omp critical
                {
                    progress->progressf(2.0f * float(_k) / float(regions.size() * regions.size()));
                }

                //UnstructuredInCorePointCloud* pcB = (UnstructuredInCorePointCloud*)regions[j].pc;
                //BoundingBox3f bboxB = pcB->getPointSet()->getBoundingBox();
                //AAT posBAAT = pcB->getAAT("position");

                // default edge to add
                GroupEdge e;
                e.type = 0;
                e.A = i;
                e.B = j;

                // compute shortest distance between points in B and A
                float dist = RegionElement::minimalDistance(regions[i], regions[j]);
                maxdist = std::max(dist, maxdist);
                e.weight = dist;

                // if the distance between both regions is certainly above our threshold, then no connection
                /*if (bboxA.intersectBoundingBox(bboxB) == BVQueryDisjoint && 
                (bboxA.getBoundingSphere().center - bboxB.getBoundingSphere().center).getNorm() - bboxA.getBoundingSphere().radius - bboxB.getBoundingSphere().radius > threshold)
                {

                }else*/
                {

                    // compute shortest distance between points in B and A
                    //float dist = RegionElement::minimalDistance(regions[i], regions[j]);
                    /*for (unsigned p=0; p < pcB->getNumPoints() ; p++)
                    {
                    Vector3f posb = pcB->getPointSet()->get3f(p, posBAAT);
                    hItA.setSeekPointAndReset(posb);
                    if (hItA.atEnd()) continue;

                    float d = (posb - hItA.get3f(posAAAT)).getNorm();
                    if (d < dist) dist = d;
                    }*/

                    // distance below a threshold is marked as PROXIMITY-edge
                    if (dist < threshold)
                    {
                        e.type = GroupEdge::PROXIMITY;
                        //e.weight = (threshold - dist) / threshold;

                        // distance above threshold is an OFFGRAPH edge - representing loosy rigid connection
                    }else
                    {
                        e.type = GroupEdge::OFFGRAPH;
                    }

                }

                // add edge
#pragma omp critical
                {
                    regionEdges.push_back(e);
                }

            }
        }
        progress->popStep();	
    }

    // store compute region edges in a graph file
    debugOutput << "export region edges to disk\n";
    {
        // edges
        std::ofstream metaf((path + std::string("/") + "region.edg").c_str());
        for (unsigned i=0; i < regionEdges.size(); i++)
        {
            GroupEdge& e = regionEdges[i];
            e.weight /= maxdist;
            metaf << e.A << " " << e.B << " " << e.type << " " << e.weight << "\n";
            metaf << e.B << " " << e.A << " " << e.type << " " << e.weight << "\n";
            debugOutput << e.A << "<->" << e.B << " type = " << e.type << ", dist = "  << e.weight << "\n";
        }
    }
    debugOutput << "DONE\n";
}

#include "PointCloudImporterReaderCustomASCII.h"
#include <PCCApplySOTransformation.h>

//--------------------------------------------------------------------------------------
PCIConsistentCrossShapeGrammar::ShapeData PCIConsistentCrossShapeGrammar::parseSymmetryDataFile(const std::string& path)
{
    ShapeData data;
    data.numpoints = 0;
    data.pcaValues = makeVector3f(0,0,0);
    data.shape = nullptr;

    std::ifstream datfile((path + "\\data.dat").c_str());
    if (!datfile.is_open()) return data;

    datfile >> data.numpoints;
    datfile >> data.pcaValues[0] >> data.pcaValues[1] >> data.pcaValues[2];


    {
        BinaryInputObjectStream io((path + std::string("/data.object")).c_str());
        io.readAnyObject(data.shape);
        if (data.shape == nullptr)
        {
            debugOutput << "Cannot read object from " << io.getFileName() << "\n";
        }else
        {
            //checkAttribute(data.shape, "flags", 1, VAD::DATA_FORMAT_INT32);
            //checkAttribute(data.shape, "color", 4, VAD::DATA_FORMAT_FLOAT32);
            //data.shape->setMaterialIndex(6);
        }
    }

    return data;
}

//--------------------------------------------------------------------------------------

std::vector<PCIConsistentCrossShapeGrammar::SymmetricElement> PCIConsistentCrossShapeGrammar::parseGroupFromFile(const std::string& path, bool loadData)
{
    std::vector<SymmetricElement> shape;

    card32 numlines = 0;
    {
        std::ifstream grpfile((path + "\\data.grp").c_str());
        if (!grpfile.is_open()) return shape;

        while (!grpfile.eof())
        {
            std::vector<char> linebuf(2048,0); 
            grpfile.getline(&linebuf[0], linebuf.size());
            numlines++;
        };
    }
    std::ifstream grpfile((path + "\\data.grp").c_str());

    ProgressWindow* progress = getProgressWindow();
    progress->pushStep(true, "import");
    card32 linenum = 0;
    while (!grpfile.eof())
    {
        progress->progressf(float(linenum++) / float(numlines));

        std::vector<char> linebuf(2048,0); 
        std::vector<char> tmpbuf(128,0);
        std::vector<char> serbuf(linebuf.size() - tmpbuf.size(),0);
        grpfile.getline(&linebuf[0], linebuf.size());
        if (strlen(&linebuf[0]) < 5) continue;

        SymmetricElement elem;
        sscanf(&linebuf[0], "%d %s : %[^\t\n]", & elem.index, &tmpbuf[0], &serbuf[0]);
        elem.nonsymmetric = (strcmp(&tmpbuf[0], "NONSYM") == 0);

        // if not symmetric, then parse the class
        elem.group = nullptr;
        if (!elem.nonsymmetric && loadData)
        {
            const std::string symgroupfile = (path + std::string("/") + intToStr(elem.index) + "_group.object");
            BinaryInputObjectStream io(symgroupfile.c_str());
            io.readAnyObject(elem.group);
            if (elem.group == nullptr)
            {
                debugOutput << "Cannot parse class name: " << std::string(&tmpbuf[0]) << " in file " << symgroupfile << "\n";
                return shape;
            }
        }
        elem.path = path + "\\" + intToStr(elem.index) + ".pts";
        elem.pc = nullptr;

        if (loadData)
        {
            try
            {
                BinaryInputObjectStream io((elem.path + ".object").c_str());
                io.readAnyObject(elem.pc);
            }catch(...)
            {
                PointCloudImporterReaderCustomASCII importer;
                importer.setCommentMarkers(std::vector<std::string>(1, "#"));
                importer.beginImport(elem.path);
                importer.parseHeader();
                elem.pc = new UnstructuredInCorePointCloud();
                elem.pc->clearAndSetup(importer.getImportVD(), 0);
                while(true)
                {
                    const card8* pt = importer.getNextPoint();
                    if (pt == nullptr) break;
                    elem.pc->getPointSet()->addPoint((const char*)pt);
                }
                importer.endImport();
            }
            checkAttribute(elem.pc, "flags", 1, VAD::DATA_FORMAT_INT32);
            checkAttribute(elem.pc, "color", 4, VAD::DATA_FORMAT_FLOAT32);
            elem.pc->setMaterialIndex(6);
        }

        // estimate volume of this part by orienting it with pca and then taking bbox volume
        if (loadData)
        {
            LotteryDraw draw;
            draw.setup(elem.pc->getNumPoints());
            std::vector<DVectorF> dataForPCA;
            for (unsigned k=0; k < 1000 && !draw.empty(); k++)
            {
                Vector3f startpos = elem.pc->getPointSet()->get3f(draw.draw(), elem.pc->getAAT("position"));
                dataForPCA.push_back(PCAGeneral::convertVec3fToDVec(std::vector<Vector3f>(1,startpos)));
            }

            PCAGeneral pca; pca.calculatePCA(dataForPCA, -1, 3);
            DMatrixF pcamatr = pca.getEigenVecs();

            Matrix3f T;
            T[0] = normalize(PCAGeneral::convertDVtoVec3f(pcamatr[0])[0]);
            T[1] = normalize(PCAGeneral::convertDVtoVec3f(pcamatr[1])[0]);
            T[2] = normalize(PCAGeneral::convertDVtoVec3f(pcamatr[2])[0]);

            Vector3f mean = pca.getMeanVecAsVec3f()[0];

            //debugRenderer->beginRenderJob_OneFrame("bbox", linenum);
            BoundingBox3f bbox(makeVector3f(0,0,0));
            for (unsigned i=0; i < elem.pc->getNumPoints(); i++)
            {
                Vector3f pos = elem.pc->getPointSet()->get3f(i, elem.pc->getAAT("position"));
                pos -= mean;
                pos = shrink4To3(invertFrame(expand3To4(T))) * pos;
                bbox.addPoint(pos);
                //debugRenderer->addPoint(pos, makeVector3f(0,1,0));
            }

            elem.pcaValues[0] = bbox.getSideLength(0);
            elem.pcaValues[1] = bbox.getSideLength(1);
            elem.pcaValues[2] = bbox.getSideLength(2);

            //debugRenderer->addCoordSystem(mean, T, 1.0f);

            //debugRenderer->addFastSphere(mean, 0.05f, makeVector3f(1,0,0));
            //debugRenderer->addBoundingBox(bbox, makeVector3f(1,0,0), 3.0f);
            //debugRenderer->endRenderJob();

            //Vector3f localAxis = normalize(PCAGeneral::convertDVtoVec3f(pcamatr[2])[0]);
            //Vector3f rotAxis = normalize(makeVector3f(0,0,1).crossProduct(localAxis));
            //Matrix3f T = makeRotVector3f(rotAxis, acos(localAxis[2]));
            //elem.pc->setTransformation(expand3To4(T));
            //PCCApplySOTransformation::applyTransformation(elem.pc);
        }

        // load point indices
        //if (loadData)
        {
            FILE* file = fopen((path + "\\" + intToStr(elem.index) + ".idx").c_str(), "rb");
            if (file)
            {
                card32 size = 0;
                fread(&size, sizeof(card32), 1, file);
                elem.pts.resize(size);
                fread(&elem.pts[0], sizeof(card32), size, file);
                fclose(file);
            }
        }
        shape.push_back(elem);
    }
    progress->popSteps();

    return shape;
}

//--------------------------------------------------------------------------------------

std::vector<PCIConsistentCrossShapeGrammar::RegionElement> PCIConsistentCrossShapeGrammar::parseRegionsFromFile(const std::string& path, bool loadData)
{
    std::vector<RegionElement> shape;

    card32 numlines = 0;
    {
        std::ifstream grpfile((path + "\\data.rgr").c_str());
        if (!grpfile.is_open()) return shape;

        while (!grpfile.eof())
        {
            std::vector<char> linebuf(2048,0); 
            grpfile.getline(&linebuf[0], linebuf.size());
            numlines++;
        };
    }
    std::ifstream grpfile((path + "\\data.rgr").c_str());

    ProgressWindow* progress = getProgressWindow();
    progress->pushStep(true, "import");
    card32 linenum = 0;
    while (!grpfile.eof())
    {
        progress->progressf(float(linenum++) / float(numlines/2));

        std::vector<char> linebuf(2048,0); 
        grpfile.getline(&linebuf[0], linebuf.size());
        if (strlen(&linebuf[0]) < 3) continue;

        RegionElement elem;
        int numgroups;
        sscanf(&linebuf[0], "%d [%d]", &elem.index, &numgroups);
        elem.groups.reserve(numgroups);

        // parse group ids
        grpfile.getline(&linebuf[0], linebuf.size());
        if (strlen(&linebuf[0]) < 1) continue;
        std::string s(&linebuf[0]);
        std::istringstream iss(s);
        for (int i=0; i < numgroups; i++)
        {
            int id;
            iss >> id; 
            elem.groups.push_back(id); 
        }

        // parse data
        elem.path = path + "\\" + intToStr(elem.index) + ".region";
        elem.pc = nullptr;
        if (loadData)
        {
            try
            {
                BinaryInputObjectStream io((elem.path + ".object").c_str());
                io.readAnyObject(elem.pc);
            }catch(...)
            {
                PointCloudImporterReaderCustomASCII importer;
                importer.setCommentMarkers(std::vector<std::string>(1, "#"));
                importer.beginImport(elem.path);
                importer.parseHeader();
                elem.pc = new UnstructuredInCorePointCloud();
                elem.pc->clearAndSetup(importer.getImportVD(), 0);
                while(true)
                {
                    const card8* pt = importer.getNextPoint();
                    if (pt == nullptr) break;
                    elem.pc->getPointSet()->addPoint((const char*)pt);
                }
                importer.endImport();
            }
            checkAttribute(elem.pc, "flags", 1, VAD::DATA_FORMAT_INT32);
            checkAttribute(elem.pc, "color", 4, VAD::DATA_FORMAT_FLOAT32);
            elem.pc->setMaterialIndex(6);
        }
        shape.push_back(elem);
    }
    progress->popSteps();

    return shape;
}

//--------------------------------------------------------------------------------------

std::vector<PCIConsistentCrossShapeGrammar::GroupEdge> PCIConsistentCrossShapeGrammar::parseGraphFromFile(const std::string& path, const std::string& filename)
{
    std::vector<GroupEdge> graph;

    std::ifstream file((path + "\\" + filename).c_str());
    if (!file.is_open()) return graph;

    while (!file.eof())
    {
        std::vector<char> linebuf(2048,0); 
        file.getline(&linebuf[0], linebuf.size());
        if (strlen(&linebuf[0]) < 7) continue;

        GroupEdge edge;

        sscanf(&linebuf[0], "%d %d %d %f", &edge.A, &edge.B, &edge.type, &edge.weight);
        graph.push_back(edge);
    }
    return graph;
}

//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::loadAndShowExportedSymmetries()
{	
    std::string path;
    path = FileDialogs::getExistingDirectory(InterfaceGlobals::getMainWindow(), "Open extracted symmetries directory", "", FileDialogs::SAVE_DIR);
    //{
    //    struct AskPath : public OperatorNoArgsLockedResult<std::string>
    //    {
    //        void operator()()
    //        {
    //            set(FileDialogs::getExistingDirectory(InterfaceGlobals::getMainWindow(), "Open extracted symmetries directory", "", FileDialogs::SAVE_DIR));
    //        }
    //    };

    //    // ask user to specify type of the symmetry
    //    AskPath msg; execOperatorNoArgs(&msg);
    //    path = msg.get();
    //}; // ask path

    if (path.length() == 0) return;

    debugOutput << "Load from: " << path << "\n";
    std::string name;
    for (int i=path.length()-1; i >=0; i--)
        if (path[i] == '\\') break;
        else name += path[i];
        name = std::string(name.rbegin(), name.rend());

        // read the groups
        std::vector<SymmetricElement> shape = parseGroupFromFile(path);
        std::vector<GroupEdge> graph = parseGraphFromFile(path);
        if (shape.size() == 0 || graph.size() == 0)
        {
            debugOutput << "Cannot find valid symmetries within the given path\n";
            return;
        }


        SGListNode* list = new SGListNode();
        list->setName(name);
        for (unsigned i=0; i < shape.size(); i++)
        {
            SGObjectNode* node = new SGObjectNode(shape[i].pc);
            node->setName(intToStr(shape[i].index) + (shape[i].nonsymmetric ? "_nonsym" : ""));
            sym::SymmetryGroupAttachment* group = new sym::SymmetryGroupAttachment();
            group->group = shape[i].group;
            group->setup("SymmetryGroup", AttachedData::ADF_PERSISTENT);
            node->getSceneObject()->getAttachments()->attachData(group);
            list->addChildNode(node);
        }
        getScene()->getRootNode()->addChildNode(list);

        debugOutput << "loaded symmetries with " << shape.size() << " elements and a graph of " << graph.size() << " edges\n";

        showGroupGraph = list;
        groupEdgeMap = graph;
        showSymmetryGraph = true;
}


#define STITCH_32_TO_U64(msb,lsb) ( ((card64)(card32(msb)) << 32) | ((card32)lsb) )
#define GET_MSB_FROM_U64(u64) ( (int32)((u64 >> 32L) & 0xFFFFFFFFL) )
#define GET_LSB_FROM_U64(u64) ( (int32)(u64 & 0xFFFFFFFFL) )

//--------------------------------------------------------------------------------------

std::vector<std::vector<card32>> PCIConsistentCrossShapeGrammar::computeRegionCorrespondence(
    const std::vector<RegionElement>& shapeARegions, 
    const std::vector<SymmetricElement>& shapeB,
    const ShapeData& shapeBdata,
    const DMatrixF& correspondence,
    std::vector<float>& regionMatchScores
    )
{
    // iterate over all given regions (they belong to the shapeA)
    // and identify the matching set of points on shapeB
    // for this we first construct a set of points in shapeB as the union of all symmetric elements
    // then based on the correspondences between the symmetric elements we vote on the shapeB 
    // for the points wich do match for the region in A
    // point with most votes win the race as a region which is matched towards A
    // we return a std::vector with indices into shapeB for each of the point of each region of shapeA

    std::vector<std::vector<card32>> result(shapeARegions.size());
    regionMatchScores.clear();
    regionMatchScores.resize(shapeARegions.size(), 0);

    std::vector<float> scores(shapeBdata.numpoints, 0);
    for (size_t r=0, rsize = shapeARegions.size(); r < rsize; r++)
    {
        float maxscore = 0;
        float norm = 0;

        // for each group to which this region does belong to 
        for (size_t g=0, gsize = shapeARegions[r].groups.size(); g < gsize; g++)
        {
            // get group correspondence on shapeB
            for (size_t gj=0, gjsize = shapeB.size(); gj < gjsize; gj++)
            {
                float score = correspondence[shapeARegions[r].groups[g]][gj];
                norm += 1.0f;

                // the score of each point of the group in shapeB is increased
                // we thus mark points on shapeB that they match with the region on shapeA
                for (size_t p=0, psize = shapeB[gj].pts.size(); p < psize; p++)
                {
                    scores[shapeB[gj].pts[p]] += score;
                    maxscore = std::max(maxscore, scores[shapeB[gj].pts[p]]);
                }
            }
        }

        // in very rare cases a region might not have a match at all, hence just put an empty solution 
        if (maxscore < 1e-5f)
        {
            continue;
        }

        // a region on shapeB with the maximal score is the region where the region on shapeA matches to
        std::vector<card32>& regionB = result[r];
        for (size_t i=0; i < scores.size(); i++)
        {
            if (scores[i] + 1e-5f > maxscore)
            {
                regionB.push_back(i);
            }
        }
        regionMatchScores[r] = maxscore / norm;
        memset(&scores[0], 0, sizeof(float) * scores.size());
    }

    return result;
}

//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::checkAndSetTrianglePointAssignmentCache()
{
    // in this method we iterate over all objects in our scene graph
    // on top level of our graph we have statechangenodes, which do contain a shape
    // each of this shapes is mapped to some of triangle meshes in the scene graph
    // we attach to this triangle meshes vectors indicating which points
    // from symmetry shapes do triangle meshes points match to

    // we now precache assignments of all vertices of triangle shapes 
    for (unsigned i=0; i < getScene()->getRootNode()->getNumChildNodes(nullptr); i++)
    {
        SGStateChangeNode* stnode = dynamic_cast<SGStateChangeNode*>(getScene()->getRootNode()->getChildNode(nullptr,i)); if (!stnode) continue;
        SGListNode* _lsnode = dynamic_cast<SGListNode*>(stnode->getChildNode(nullptr,0)); if (!_lsnode) continue;
        card32 ls_sid; if (!strToInt(_lsnode->getName(), ls_sid)) continue;
        SGObjectNode* trinode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), std::string("root/main_") + intToStr(ls_sid))); if (!trinode) continue;
        SGListNode* listNode = dynamic_cast<SGListNode*>(_lsnode->getChildNode(nullptr,0)); if (!listNode) continue;
        SGListNode* regListNode = dynamic_cast<SGListNode*>(_lsnode->getChildNode(nullptr,1)); if (!regListNode) continue;
        SGObjectNode* pcNode = dynamic_cast<SGObjectNode*>(_lsnode->getChildNode(nullptr,2)); if (!pcNode) continue;
        UnstructuredInCorePointCloud* shape = dynamic_cast<UnstructuredInCorePointCloud*>(pcNode->getSceneObject());

        UnstructuredInCorePointCloud* tripc = dynamic_cast<UnstructuredInCorePointCloud*>(trinode->getSceneObject());
        ProgressWindow* progress = getProgressWindow();

        // cache symmetry node maps
        if (!regionBasedCorrespondences && !SimpleAttachment::exists(tripc->getAttachments(), "nid_cached"))
            //|| SimpleAttachment::get1i(tripc->getAttachments(), "nid_cached") != tmp_embeddedVectorsNum )
                //|| !SimpleAttachment::exists(tripc->getAttachments(), "point_map_size"))
        {
            debugOutput << "precache assignment of triangle mesh " << trinode->getName() << " to loaded data\n";
            debugOutput.flush();

            std::vector<std::vector<card32>> nidCache(listNode->getNumChildNodes(nullptr));
            std::vector<int32> pointIdCache(shape->getNumPoints(), -1);

            AAT posAAT = tripc->getAAT("position");
            progress->pushStep(true, std::string("cache_") + intToStr(ls_sid));

            HierarchicalKNNIterator* hIt = new HierarchicalKNNIterator(shape, 32, nullptr);
            hIt->setMaxDistanceToSeekPoint(m_Settings->spatialTolerance);

            std::vector<UnstructuredInCorePointCloud*> cachePartPCs(listNode->getNumChildNodes(nullptr), nullptr);
            std::vector<HierarchicalKNNIterator*> cacheHits(listNode->getNumChildNodes(nullptr), nullptr);
            std::vector<card32> cacheNids(listNode->getNumChildNodes(nullptr), -1);
            for (int j=0; j < listNode->getNumChildNodes(nullptr); j++)
            {
                SGObjectNode* objnode = dynamic_cast<SGObjectNode*>(listNode->getChildNode(nullptr,j));
                if (!objnode) continue;
                cachePartPCs[j] = dynamic_cast<UnstructuredInCorePointCloud*>(objnode->getSceneObject()->copy());
                cacheHits[j] = new HierarchicalKNNIterator(cachePartPCs[j], 32, nullptr);
                cacheHits[j]->setMaxDistanceToSeekPoint(m_Settings->spatialTolerance);
                cacheNids[j] = atoi(objnode->getName().c_str());
            }


            for (unsigned k=0; k < tripc->getNumPoints(); k++)
            {
                Vector3f pointPos = tripc->getPointSet()->get3f(k,posAAT);
                progress->progressf((float)k / (float)tripc->getNumPoints());

                // point wise match
                hIt->setSeekPointAndReset(pointPos);
                while(!hIt->atEnd())
                {
                    if (pointIdCache[hIt->getCurrentPointIndex()] == -1) pointIdCache[hIt->getCurrentPointIndex()] = k;
                    hIt->next();
                }

                // node-wise match
#pragma omp parallel for
                for (int j=0; j < listNode->getNumChildNodes(nullptr); j++)
                {
                    if (!cacheHits[j]) continue;
                    cacheHits[j]->setSeekPointAndReset(pointPos);
                    if (!cacheHits[j]->atEnd())
                    {
#pragma omp critical
                        nidCache[j].push_back(k);
                    }
                }
            }

            for (int j=0; j < listNode->getNumChildNodes(nullptr); j++)
            {
                delete cachePartPCs[j];
                delete cacheHits[j];
            }
            delete hIt;
            progress->popSteps();

            tripc->getAttachments()->clear();

            // attache the assigned nids to the triangle mesh
            for (unsigned j=0; j < nidCache.size(); j++)
            {
                tripc->getAttachments()->removeData(std::string("nid_pc_map_") + intToStr(cacheNids[j]));
                tripc->getAttachments()->removeData(std::string("nid_pc_map_size_") + intToStr(cacheNids[j]));

                SimpleAttachment::set1i(tripc->getAttachments(), std::string("nid_pc_map_size_") + intToStr(cacheNids[j]), nidCache[j].size());
                if (nidCache[j].size())
                    SimpleAttachment::set(tripc->getAttachments(), std::string("nid_pc_map_") + intToStr(cacheNids[j]), &nidCache[j][0], sizeof(card32) * nidCache[j].size());
            }
            SimpleAttachment::set1i(tripc->getAttachments(), "nid_cached", tmp_embeddedVectorsNum);

#if 0
            // attach point std::map of the whole shape
            tripc->getAttachments()->removeData(std::string("point_map"));
            tripc->getAttachments()->removeData(std::string("point_map_size"));
            SimpleAttachment::set1i(tripc->getAttachments(), std::string("point_map_size"), pointIdCache.size());
            if (pointIdCache.size())
                SimpleAttachment::set(tripc->getAttachments(), std::string("point_map"), &pointIdCache[0], sizeof(int32) * pointIdCache.size());
#endif
        }


        // cache symmetry node maps
        if (regionBasedCorrespondences && !SimpleAttachment::exists(tripc->getAttachments(), "rid_cached"))
            //|| SimpleAttachment::get1i(tripc->getAttachments(), "rid_cached") != tmp_embeddedVectorsNum)
        {
            debugOutput << "precache assignment of triangle mesh " << trinode->getName() << " to loaded region data\n";
            debugOutput.flush();

            std::vector<std::vector<card32>> ridCache(regListNode->getNumChildNodes(nullptr));

            AAT posAAT = tripc->getAAT("position");
            progress->pushStep(true, std::string("cache_") + intToStr(ls_sid));

            std::vector<UnstructuredInCorePointCloud*> cachePartPCs(regListNode->getNumChildNodes(nullptr), nullptr);
            std::vector<HierarchicalKNNIterator*> cacheHits(regListNode->getNumChildNodes(nullptr), nullptr);
            std::vector<card32> cacheRids(regListNode->getNumChildNodes(nullptr), -1);
            for (int j=0; j < regListNode->getNumChildNodes(nullptr); j++)
            {
                SGObjectNode* objnode = dynamic_cast<SGObjectNode*>(regListNode->getChildNode(nullptr,j));
                if (!objnode) continue;
                cachePartPCs[j] = dynamic_cast<UnstructuredInCorePointCloud*>(objnode->getSceneObject()->copy());
                cacheHits[j] = new HierarchicalKNNIterator(cachePartPCs[j], 32, nullptr);
                cacheHits[j]->setMaxDistanceToSeekPoint(m_Settings->spatialTolerance);
                cacheRids[j] = atoi(objnode->getName().c_str());
            }


            for (unsigned k=0; k < tripc->getNumPoints(); k++)
            {
                Vector3f pointPos = tripc->getPointSet()->get3f(k,posAAT);
                progress->progressf((float)k / (float)tripc->getNumPoints());

                // region-wise match
#pragma omp parallel for
                for (int j=0; j < regListNode->getNumChildNodes(nullptr); j++)
                {
                    if (!cacheHits[j]) continue;
                    cacheHits[j]->setSeekPointAndReset(pointPos);
                    if (!cacheHits[j]->atEnd())
                    {
#pragma omp critical
                        ridCache[j].push_back(k);
                    }
                }
            }

            for (int j=0; j < regListNode->getNumChildNodes(nullptr); j++)
            {
                delete cachePartPCs[j];
                delete cacheHits[j];
            }
            progress->popSteps();

            tripc->getAttachments()->clear();

            // attache the assigned nids to the triangle mesh
            for (unsigned j=0; j < ridCache.size(); j++)
            {
                tripc->getAttachments()->removeData(std::string("rid_pc_map_") + intToStr(cacheRids[j]));
                tripc->getAttachments()->removeData(std::string("rid_pc_map_size_") + intToStr(cacheRids[j]));

                SimpleAttachment::set1i(tripc->getAttachments(), std::string("rid_pc_map_size_") + intToStr(cacheRids[j]), ridCache[j].size());
                if (ridCache[j].size())
                    SimpleAttachment::set(tripc->getAttachments(), std::string("rid_pc_map_") + intToStr(cacheRids[j]), &ridCache[j][0], sizeof(card32) * ridCache[j].size());
            }
            SimpleAttachment::set1i(tripc->getAttachments(), "rid_cached", ridCache.size());
        }

    }

}

//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::showRegionMatchForSelectedNode()
{
    checkAndSetTrianglePointAssignmentCache();


    // ---------------------------------
    // prerequisites
    // ---------------------------------
    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    if (selection.size() != 1)
    {
        error("Select node first");
        return;
    }

    SGObjectNode* srcnode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[0]));
    if (!srcnode || !srcnode->getParent(0))
    {
        warning("Select a valid shape, with a valid parent");
        return;
    }

    if (!SimpleAttachment::exists(srcnode->getSceneObject()->getAttachments(),"RegionID"))
    {
        debugOutput << "This is not a region node\n";
        return;
    }

    card32 idx = SimpleAttachment::get1i(srcnode->getSceneObject()->getAttachments(), "RegionID");
    if (idx >= tmp_embeddedVectors.size())//nid == (card32)-1 || sid == (card32)-1)
    {
        warning("Select a valid shape, with a valid name, maybe you have loaded the correspondencies without regionBaseCorrespondences flag active");
        return;
    }

    // ---------------------------------
    // Mark matched points
    // ---------------------------------
    for (unsigned i=0; i < getScene()->getRootNode()->getNumChildNodes(nullptr); i++)
    {
        // get triangle meshes associated with the other shapes
        SGStateChangeNode* stnode = dynamic_cast<SGStateChangeNode*>(getScene()->getRootNode()->getChildNode(nullptr,i)); if (!stnode) continue;
        SGListNode* _lsnode = dynamic_cast<SGListNode*>(stnode->getChildNode(nullptr,0)); if (!_lsnode) continue;
        card32 ls_sid; if (!strToInt(_lsnode->getName(), ls_sid)) continue;

        // list of regions
        SGListNode* listNode = dynamic_cast<SGListNode*>(_lsnode->getChildNode(nullptr,2)); if (!listNode) continue;

        UnstructuredInCorePointCloud* tripc = nullptr;
        SGObjectNode* trinode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), std::string("root/main_") + intToStr(ls_sid)));
        if (trinode)
        {
            tripc = dynamic_cast<UnstructuredInCorePointCloud*>(trinode->getSceneObject());
            if (!SimpleAttachment::exists(tripc->getAttachments(), "rid_cached"))
                //|| !SimpleAttachment::exists(tripc->getAttachments(), "point_map_size"))
            {
                debugOutput << "NO CACHE FOUND, ERROR\n";
                continue;
            }
        }else
        {
            debugOutput << "NO MAIN TRIANGLE MESHES FOUND, ERROR!\n";
            continue;
        }
#if 1
        // compute distance from the selected node to all found in this shape
        std::map<card32, float32> distanceMap;
        std::map<card32, char> selectedMap;

        for (int j=0; j < listNode->getNumChildNodes(nullptr); j++)
        {
            SGObjectNode* objnode = dynamic_cast<SGObjectNode*>(listNode->getChildNode(nullptr,j)); if (!objnode) continue;
            card32 ls_rid; if (!strToInt(objnode->getName(), ls_rid)) continue;

            float dist = 0;
            card32 lsidx = SimpleAttachment::get1i(objnode->getSceneObject()->getAttachments(), "EmbeddedIndex");
            dist = !showBackwardMatching ? tmp_correpondenceMatrix[idx][lsidx] : tmp_correpondenceMatrix[lsidx][idx];

            //if (dist > 1) dist = 1;
            selectedMap[ls_rid] = (idx == lsidx) ? 1 : 0;
            distanceMap[ls_rid] = dist;
            debugOutput << ls_sid << "," << ls_rid << "[" << idx << "," << lsidx << "] = " << dist << "\n";

            selectedSymDistanceMap[lsidx] = (idx == lsidx) ? 1 : dist;
        }

        // assign distances to points in triangle mesh
        std::vector<float32> ptDistance(tripc->getNumPoints(), 0);
        std::vector<char> nonsymFlag(tripc->getNumPoints(), 1);
        std::vector<char> selectedFlag(tripc->getNumPoints(), 0);
        for (std::map<card32, float32>::iterator it = distanceMap.begin(); it != distanceMap.end(); it++)
        {
            card32 rid = it->first;
            float32 dist = it->second;

            // go to every child
            std::vector<card32> ridmap;
            ridmap.resize(SimpleAttachment::get1i(tripc->getAttachments(), std::string("rid_pc_map_size_") + intToStr(rid)));
            memcpy(&ridmap[0], SimpleAttachment::get(tripc->getAttachments(), std::string("rid_pc_map_") + intToStr(rid)), sizeof(card32) * ridmap.size());

            // assigned lowest distance over all nids to each point
            for (unsigned j=0; j < ridmap.size(); j++)
            {
                //if (dist < ptDistance[nidmap[j]]) ptDistance[nidmap[j]] = dist;
                ptDistance[ridmap[j]] += dist;
                nonsymFlag[ridmap[j]] = 0;
                selectedFlag[ridmap[j]] |= selectedMap[rid];
            }
        }

        // no iterate over all points which are mapped to this nid and colorify them
        AAT colorAAT = tripc->getAAT("color");
        for (unsigned j=0; j < tripc->getNumPoints(); j++)
        {
            Vector4f color = makeVector4f(ptDistance[j], nonsymFlag[j] ? 0 : 1, selectedFlag[j] ? 1 : 0, 0);
            tripc->getPointSet()->set3f(j, colorAAT, shrink4To3(color));			
        }
#endif
#if 0
        // get for the selected node, which does represent a region the 
        // list of points where it does match to on shapeB
        std::vector<card32> matchmap;
        matchmap.resize(SimpleAttachment::get1i(srcnode->getSceneObject()->getAttachments(), std::string("region_map_size_") + intToStr(ls_sid)));
        if (matchmap.size() == 0) continue; // the selected region does not have any match into the shape ls_sid
        memcpy(&matchmap[0], SimpleAttachment::get(srcnode->getSceneObject()->getAttachments(), std::string("region_map_") + intToStr(ls_sid)), sizeof(card32) * matchmap.size());

        float score = SimpleAttachment::get1f(srcnode->getSceneObject()->getAttachments(), std::string("region_map_score_") + intToStr(ls_sid));

        // get mapping of the shape points to triangle points		
        std::vector<int32> pointMap;
        pointMap.resize(SimpleAttachment::get1i(tripc->getAttachments(), std::string("point_map_size")));
        memcpy(&pointMap[0], SimpleAttachment::get(tripc->getAttachments(), std::string("point_map")), sizeof(int32) * pointMap.size());

        // assign distances to points in triangle mesh
        std::vector<float32> ptDistance(tripc->getNumPoints(), 0);
        std::vector<char> nonsymFlag(tripc->getNumPoints(), 1);
        std::vector<char> selectedFlag(tripc->getNumPoints(), 0);
        for (size_t mm=0; mm < matchmap.size(); mm++)
        {
            int k = pointMap[matchmap[mm]];
            if (k < 0) continue;

            ptDistance[k] = score; //exp(-score*score*5.0f);
            nonsymFlag[k] = 0;
            //selectedFlag[k] |= selectedMap[nid];
        }

        // no iterate over all points which are mapped to this nid and colorify them
        AAT colorAAT = tripc->getAAT("color");
        for (unsigned j=0; j < tripc->getNumPoints(); j++)
        {
            Vector4f color = makeVector4f(ptDistance[j], nonsymFlag[j] ? 0 : 1, selectedFlag[j] ? 1 : 0, 0);
            tripc->getPointSet()->set3f(j, colorAAT, shrink4To3(color));			
        }
#endif 
    }

    scene->rebuildRenderObjects();
    //if (doScreenshotWhileSelecting) updateSceneView();
    //writeFrameToDisk();

}

//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::showMatchingForSelectedNode()
{
    checkAndSetTrianglePointAssignmentCache();


    // load stored embedded vectors
    if (SimpleAttachment::exists(getScene()->getAttachments(), "EmbeddingVectors"))
    {
        card32 dim = SimpleAttachment::get1i(getScene()->getAttachments(), "DimEmbeddingVectors");
        card32 num = SimpleAttachment::get1i(getScene()->getAttachments(), "NumEmbeddingVectors");

        if (tmp_embeddedVectorsDim != dim || tmp_embeddedVectorsNum != num)
        {
            debugOutput << "get cached eigenstuff\n";

            tmp_embeddedVectorsDim = dim;
            tmp_embeddedVectorsNum = num;
            char* buffer = (char*)SimpleAttachment::get(getScene()->getAttachments(), "EmbeddingVectors");

            tmp_embeddedVectors.resize(num);
            for(unsigned i=0; i < tmp_embeddedVectors.size(); i++)
            {
                tmp_embeddedVectors[i].setDim(dim);
                memcpy(tmp_embeddedVectors[i].dataPtr(), &buffer[i * dim * sizeof(float)], dim * sizeof(float));
            }

            tmp_embeddedValues.resize(num);
            buffer = (char*)SimpleAttachment::get(getScene()->getAttachments(), "EmbeddingValues");
            memcpy(&tmp_embeddedValues[0], buffer, sizeof(float) * num);
        }

        //tmp_correpondenceMatrix.copyDataFromArray((float*)SimpleAttachment::get(getScene()->getAttachments(), "Correspondence_M"));
    }else
    {
        error("You have first to perform embedding\n");
        return;
    }

    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    if (selection.size() != 1)
    {
        error("Select node first");
        return;
    }

    SGObjectNode* srcnode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[0]));
    if (!srcnode || !srcnode->getParent(0))
    {
        warning("Select a valid shape, with a valid parent");
        return;
    }

    if (!SimpleAttachment::exists(srcnode->getSceneObject()->getAttachments(),"EmbeddedIndex"))
    {
        error("You have to run embedding first\n");
        return;
    }


    // precache each point of triangle mesh to sid/nid pair of loaded meshes
    card32 idx = SimpleAttachment::get1i(srcnode->getSceneObject()->getAttachments(), "EmbeddedIndex");
    if (idx >= tmp_embeddedVectors.size())//nid == (card32)-1 || sid == (card32)-1)
    {
        warning("Select a valid shape, with a valid name");
        return;
    }


    float time = m_Settings->embeddingCutoffEigenvalue;
    if (time < 1) time = 1;

    // check if there is a main point cloud exists, which will just show maximal scores over all points
    ProgressWindow* progress = getProgressWindow();

    selectedSymDistanceMap.clear();

    // compute all distances 
    for (unsigned i=0; i < getScene()->getRootNode()->getNumChildNodes(nullptr); i++)
    {
        SGStateChangeNode* stnode = dynamic_cast<SGStateChangeNode*>(getScene()->getRootNode()->getChildNode(nullptr,i)); if (!stnode) continue;
        SGListNode* _lsnode = dynamic_cast<SGListNode*>(stnode->getChildNode(nullptr,0)); if (!_lsnode) continue;
        card32 ls_sid; if (!strToInt(_lsnode->getName(), ls_sid)) continue;
        SGListNode* listNode = dynamic_cast<SGListNode*>(_lsnode->getChildNode(nullptr,0)); if (!listNode) continue;


        // get corresponding triangle mesh, it must be cached already
        UnstructuredInCorePointCloud* tripc = nullptr;
        SGObjectNode* trinode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), std::string("root/main_") + intToStr(ls_sid)));
        if (trinode)
        {
            tripc = dynamic_cast<UnstructuredInCorePointCloud*>(trinode->getSceneObject());
            if (!SimpleAttachment::exists(tripc->getAttachments(), "nid_cached"))
            {
                debugOutput << "NO CACHE FOUND, ERROR\n";
                continue;
            }
        }else
        {
            debugOutput << "NO MAIN TRIANGLE MESHES FOUND, ERROR!\n";
            continue;
        }



        // compute distance from the selected node to all found in this shape
        std::map<card32, float32> distanceMap;
        std::map<card32, char> selectedMap;

        for (int j=0; j < listNode->getNumChildNodes(nullptr); j++)
        {
            SGObjectNode* objnode = dynamic_cast<SGObjectNode*>(listNode->getChildNode(nullptr,j)); if (!objnode) continue;
            card32 ls_nid; if (!strToInt(objnode->getName(), ls_nid)) continue;

            float dist = 0;
            card32 lsidx = SimpleAttachment::get1i(objnode->getSceneObject()->getAttachments(), "EmbeddedIndex");
#if 0
            if (useSpectralEmbedding)
            {
                // get embedded point 
                //DVectorF pos(tmp_embeddedVectors.size());
                //for (unsigned i=0; i < tmp_embeddedVectors.size(); i++)
                //	pos[i] = (tmp_embeddedVectors[i])[idx] * pow(tmp_embeddedValues[i], time);

                // get embedded point 
                //DVectorF ls_pos(tmp_embeddedVectors.size());
                for (unsigned k=0; k < tmp_embeddedVectors.size(); k++)
                    dist += pow(tmp_embeddedValues[k], 2.f * time) * pow(tmp_embeddedVectors[k][lsidx] - tmp_embeddedVectors[k][idx],2.f);

                //dist = (pos - ls_pos).getNorm();
                dist = 1.0 - dist;
            }else
#endif
            {
                dist = !showBackwardMatching ? tmp_correpondenceMatrix[idx][lsidx] : tmp_correpondenceMatrix[lsidx][idx];
            }

            //if (dist > 1) dist = 1;
            selectedMap[ls_nid] = (idx == lsidx) ? 1 : 0;
            distanceMap[ls_nid] = dist;
            debugOutput << ls_sid << "," << ls_nid << "[" << idx << "," << lsidx << "] = " << dist << "\n";

            selectedSymDistanceMap[lsidx] = (idx == lsidx) ? 1 : dist;
        }

        // assign distances to points in triangle mesh
        std::vector<float32> ptDistance(tripc->getNumPoints(), 0);
        std::vector<char> nonsymFlag(tripc->getNumPoints(), 1);
        std::vector<char> selectedFlag(tripc->getNumPoints(), 0);
        for (std::map<card32, float32>::iterator it = distanceMap.begin(); it != distanceMap.end(); it++)
        {
            card32 nid = it->first;
            float32 dist = it->second;

            // go to every child
            std::vector<card32> nidmap;
            nidmap.resize(SimpleAttachment::get1i(tripc->getAttachments(), std::string("nid_pc_map_size_") + intToStr(nid)));
            memcpy(&nidmap[0], SimpleAttachment::get(tripc->getAttachments(), std::string("nid_pc_map_") + intToStr(nid)), sizeof(card32) * nidmap.size());

            // assigned lowest distance over all nids to each point
            for (unsigned j=0; j < nidmap.size(); j++)
            {
                //if (dist < ptDistance[nidmap[j]]) ptDistance[nidmap[j]] = dist;
                ptDistance[nidmap[j]] += dist;
                nonsymFlag[nidmap[j]] = 0;
                selectedFlag[nidmap[j]] |= selectedMap[nid];
            }
        }

        // no iterate over all points which are mapped to this nid and colorify them
        AAT colorAAT = tripc->getAAT("color");
        for (unsigned j=0; j < tripc->getNumPoints(); j++)
        {
            Vector4f color = makeVector4f(ptDistance[j], nonsymFlag[j] ? 0 : 1, selectedFlag[j] ? 1 : 0, 0);
            tripc->getPointSet()->set3f(j, colorAAT, shrink4To3(color));			
        }
    }

    scene->rebuildRenderObjects();
    //if (doScreenshotWhileSelecting) updateSceneView();
    //writeFrameToDisk();
}

//--------------------------------------------------------------------------------------
float PCIConsistentCrossShapeGrammar::RegionElement::minimalDistance(const RegionElement& regionA, const RegionElement& regionB)
{
    // region A
    UnstructuredInCorePointCloud* pcA = regionA.pc;
    BoundingBox3f bboxA = pcA->getPointSet()->getBoundingBox();
    HierarchicalKNNIterator hItA(pcA, 32, nullptr);
    AAT posAAAT = pcA->getAAT("position");

    // region B
    UnstructuredInCorePointCloud* pcB = (UnstructuredInCorePointCloud*)regionB.pc;
    BoundingBox3f bboxB = pcB->getPointSet()->getBoundingBox();
    AAT posBAAT = pcB->getAAT("position");

    // compute shortest distance between points in B and A
    float dist = 1e20f;
    for (unsigned p=0; p < pcB->getNumPoints() ; p++)
    {
        Vector3f posb = pcB->getPointSet()->get3f(p, posBAAT);
        hItA.setSeekPointAndReset(posb);
        if (hItA.atEnd()) continue;

        float d = (posb - hItA.get3f(posAAAT)).getSqrNorm();
        if (d < dist) dist = d;
    }

    return sqrt(dist);
}

//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::computeRelationMatrices()
{
    // in this method we create the input required for the spectral matcher
    // here we load the exported data and compute the node-node and edge-edge scores
    // we then export the data into a binary format 

    typedef std::vector<RegionElement> Regions;
    typedef std::vector<SymmetricElement> Shape;
    typedef std::vector<GroupEdge> Graph;

    std::vector<Shape> shapes;
    std::vector<std::string> paths;
    std::vector<Graph> graphs;
    std::vector<ShapeData> shapeData;

    std::vector<Regions> regions;
    std::vector<Graph> regionGraph;
    {
        std::string path;
        path = FileDialogs::getOpenFileName(InterfaceGlobals::getMainWindow(), "Open result info file", "*.txt");
        //{
        //    struct AskPath : public OperatorNoArgsLockedResult<std::string>
        //    {
        //        void operator()()
        //        {
        //            set(FileDialogs::getOpenFileName(InterfaceGlobals::getMainWindow(), "Open result info file", "*.txt"));
        //        }
        //    };

        //    // ask user to specify type of the symmetry
        //    AskPath msg; execOperatorNoArgs(&msg);
        //    path = msg.get();
        //}; // ask path

        debugOutput << "Load from: " << path << "\n";

        std::ifstream file(path.c_str());
        if (!file.is_open())
        {
            error("Not a valid config file given\n");
            return;
        }

        std::string datapath;
        std::string prefix;
        card32 num = 0;
        std::string pairwisematches;
        file >> datapath >> prefix >> num >> pairwisematches;

        for (unsigned g=0; g < num; g++)
        {
            std::string filename = datapath + "\\" + prefix + "_" + intToStr(g);
            debugOutput << "Load from: " << filename << "\n";

            Regions regs    = parseRegionsFromFile(filename);
            Graph reggraph  = parseGraphFromFile(filename, "region.edg");
            Shape shape     = parseGroupFromFile(filename);
            Graph graph     = parseGraphFromFile(filename);
            ShapeData size  = parseSymmetryDataFile(filename);

            graphs.push_back(graph);
            shapes.push_back(shape);
            paths.push_back(filename);
            shapeData.push_back(size);

            regions.push_back(regs);
            regionGraph.push_back(reggraph);
        }
    }

    if (shapes.size() < 2)
    {
        debugOutput << "There must be at least two groups loaded\n";
        return;
    }

    debugOutput << "Compute matrices\n";
    struct Matr
    {
        unsigned a;
        unsigned b;
        DMatrixF M;
    };
    std::vector<Matr> matrices;
    std::vector<Matr> edgematrices;

    // -------------------------------------
    // Compute scores for vertex-vertex and edge-edge
    // -------------------------------------
    omp_set_num_threads(m_Settings->getNumAvailableThreads());
    for (unsigned i=0; i < graphs.size(); i++)
    {
        const std::vector<SymmetricElement>& shapeA = shapes[i];
        const std::vector<GroupEdge>& graphA = graphs[i];
        float sizeA = fabs(shapeData[i].pcaValues[0]) * fabs(shapeData[i].pcaValues[1]) * fabs(shapeData[i].pcaValues[2]);

        for (unsigned j=0 /* i+1 */; j < graphs.size(); j++)
        {
            const std::vector<SymmetricElement>& shapeB = shapes[j];
            const std::vector<GroupEdge>& graphB = graphs[j];
            float sizeB = fabs(shapeData[j].pcaValues[0]) * fabs(shapeData[j].pcaValues[1]) * fabs(shapeData[j].pcaValues[2]);

            // vertex-vertex matrix
            Matr vvMatr;
            {
                DMatrixF M(shapeA.size(), shapeB.size(),false);
                for (unsigned c=0; c < shapeA.size(); c++)
                {
                    const SymmetricElement& elemA = shapeA[c];
                    float relSizeA = fabs(elemA.pcaValues[0]) * fabs(elemA.pcaValues[1]) * fabs(elemA.pcaValues[2]) / sizeA;
                    if (relSizeA > 1) relSizeA = 1;

                    //#pragma omp parallel for
                    for (int r=0; r < shapeB.size(); r++)
                    {
                        const SymmetricElement& elemB = shapeB[r];
                        float relSizeB = fabs(elemB.pcaValues[0]) * fabs(elemB.pcaValues[1]) * fabs(elemB.pcaValues[2]) / sizeB;
                        if (relSizeB > 1) relSizeB = 1;

                        if (elemA.group == nullptr || elemB.group == nullptr)
                        {
#pragma omp critical
                            {
                                M[c][r] = 0;
                            }
                            continue;
                        }

                        float sigmaA = dynamic_cast<sym::Lattice*>(elemA.group) ? m_Settings->latGroupCompareSigma : m_Settings->rotGroupCompareSigma;
                        float sigmaB = dynamic_cast<sym::Lattice*>(elemB.group) ? m_Settings->latGroupCompareSigma : m_Settings->rotGroupCompareSigma;

                        float score = sym::SymmetryGroup::compareIntergroupwise(*elemA.group, *elemB.group, m_Settings->spatialTolerance, m_Settings->angleTolerance, sigmaA, sigmaB, false, true);

#pragma omp critical
                        {
                            float sca = pow(relSizeA, 0.5f);
                            float scb = pow(relSizeB, 0.5f);

                            float scale = 1.0f; //std::min<float>(sca,scb) / std::max<float>(sca,scb);
                            float m = score * scale;
                            M[c][r] = m;
                        }
                    }
                }

                vvMatr.a = i;
                vvMatr.b = j;
                vvMatr.M = M;

                matrices.push_back(vvMatr);
            }

            // edge-edge matrix (pairwise score)
            {
                DMatrixF M(graphA.size(), graphB.size(),false);
                for (unsigned c=0; c < graphA.size(); c++)
                {
                    const GroupEdge& elemA = graphA[c];

                    const SymmetricElement& elemA1 = shapeA[elemA.A];
                    const SymmetricElement& elemA2 = shapeA[elemA.B];

#pragma omp parallel for
                    for (int r=0; r < graphB.size(); r++)
                    {
                        const GroupEdge& elemB = graphB[r];

                        const SymmetricElement& elemB1 = shapeB[elemB.A];
                        const SymmetricElement& elemB2 = shapeB[elemB.B];

                        // match of the group is computed always
                        float groupScore = computeGroup2GroupScore(
                            std::pair<SymmetricElement, SymmetricElement>(elemA1, elemA2), pair<card32,card32>(elemA.A, elemA.B),
                            std::pair<SymmetricElement, SymmetricElement>(elemB1, elemB2), pair<card32,card32>(elemB.A, elemB.B),
                            vvMatr.M);

                        // next we check how much of the structural property is preserved
                        float32 structWeight = 0;
                        if ((elemA.type & GroupEdge::PROXIMITY) && (elemB.type & GroupEdge::PROXIMITY)) structWeight += 2. / 10.;
                        if ((elemA.type & GroupEdge::OVERLAP) && (elemB.type & GroupEdge::OVERLAP))     structWeight += 2. / 10.;
                        if ((elemA.type & GroupEdge::SUBSET) && (elemB.type & GroupEdge::SUBSET))       structWeight += 2. / 10.;
                        if ((elemA.type & GroupEdge::SYMREL) && (elemB.type & GroupEdge::SYMREL))       structWeight += 8. / 10.;

                        if (structWeight > 0)
                            groupScore *= structWeight;//(float)structOverlap / 4.0f;

#pragma omp critical
                        {
                            //M[c][r] = groupScore;
                            float scl = sqrt(vvMatr.M[elemA.A][elemB.A] * vvMatr.M[elemA.B][elemB.B]);
                            groupScore *= scl;
                            M[c][r] = groupScore;
                        }
                    }
                }

                Matr eeMatr;

                eeMatr.a = i;
                eeMatr.b = j;
                eeMatr.M = M;

                edgematrices.push_back(eeMatr);
            }
        }
    }

    // -------------------------------------
    // Compute scores for vertex-vertex and edge-edge - REGION based
    // -------------------------------------
    debugOutput << "Compute region based matrices\n";
    std::vector<Matr> regionVV;
    std::vector<Matr> regionEE;
    for (unsigned i=0; i < graphs.size(); i++)
    {
        const std::vector<RegionElement>& regionsA = regions[i];
        const std::vector<GroupEdge>& graphA = regionGraph[i];
        const std::vector<SymmetricElement>& groupsA = shapes[i];

        for (unsigned j=0 /* i+1 */; j < graphs.size(); j++)
        {
            const std::vector<RegionElement>& regionsB = regions[j];
            const std::vector<GroupEdge>& graphB = regionGraph[j];
            const std::vector<SymmetricElement>& groupsB = shapes[j];

            // get vertex matrix containing all weights between the symmetric elements for this pair of shapes
            DMatrixF vvM;
            DMatrixF eeM;
            {
                unsigned k = 0;
                for (; k < matrices.size(); k++)
                {
                    if (matrices[k].a == i && matrices[k].b == j) break;
                }

                if (k == matrices.size())
                {
                    error("While extracting relation matrices, I wasn't able to find vertex-vertex scores for certain shapes. This can't be. Give up.");
                    return;
                }
                vvM = matrices[k].M;
                eeM = edgematrices[k].M;
                pAssert(edgematrices[k].a == matrices[k].a && edgematrices[k].b == matrices[k].b);
            }


            // region-region matrix
            Matr rrMatr;
            {
                float maxentry = 0;

                rrMatr.M = DMatrixF(regionsA.size(), regionsB.size(), false);
                for (unsigned m=0; m < regionsA.size(); m++)
                {
                    const RegionElement& elemA = regionsA[m];

                    for (int n=0; n < regionsB.size(); n++)
                    {
                        const RegionElement& elemB = regionsB[n];
                        float score = 0.0f;

                        // both regions are covered by symmetric groups
                        // the score between two regions is the sum of all similiarity scores
                        // between all the groups of both regions
                        for (unsigned ga = 0; ga < elemA.groups.size(); ga++)
                        {
                            float mx = 0;
                            for (unsigned gb = 0; gb < elemB.groups.size(); gb++)
                            {
                                mx = std::max(mx, vvM[elemA.groups[ga]][elemB.groups[gb]]);
                            }
                            score += mx;
                        } 

#pragma omp critical
                        {
                            rrMatr.M[m][n] = score;
                            maxentry = std::max(maxentry, score);
                        }
                    }
                }
                rrMatr.a = i;
                rrMatr.b = j;

                // normalize the matrix
                if (maxentry > 1e-5f)
                    rrMatr.M /= maxentry;

            }
            regionVV.push_back(rrMatr);
            debugOutput << "Rvv(" << i << "," << j << "):\n" << rrMatr.M << "\n\n";

            // edge-edge matrix, i.e. scores between edges of the regions
            Matr eeMatr;
            {
                eeMatr.M = DMatrixF(graphA.size(), graphB.size(), false);

                float maxentry = 0;

                const float diffSigmaSqr = 0.3f * 0.3f;

                // first we prefill the matrix with differences of distances between nodes
                // these do always exists for edges which are of OFFGRAPH type (constraining the "rigidness" of a shape)
                for (unsigned c=0; c < graphA.size(); c++)
                {
                    const GroupEdge& elemA = graphA[c];
                    //if (elemA.type != GroupEdge::OFFGRAPH) continue;
                    for (int r=0; r < graphB.size(); r++)
                    {
                        const GroupEdge& elemB = graphB[r];
                        //if (elemB.type != GroupEdge::OFFGRAPH) continue;

                        float diff  = elemA.weight - elemB.weight;
                        float score = 10.0f * exp(- diff * diff / diffSigmaSqr );
                        maxentry = std::max(maxentry, score);

                        eeMatr.M[c][r] = score;
                    }
                }


                // PROXIMITY edges
                const std::vector<GroupEdge>& symNodeGraphA = graphs[i];
                const std::vector<GroupEdge>& symNodeGraphB = graphs[j];

                for (unsigned c=0; c < graphA.size(); c++)
                {
                    const GroupEdge& elemA = graphA[c];
                    if (elemA.type != GroupEdge::PROXIMITY) continue;

                    const RegionElement& elemA1 = regionsA[elemA.A];
                    const RegionElement& elemA2 = regionsA[elemA.B];

                    // construct list of all edges between overlapping groups of regions in A
                    std::vector<int> edgesA;
                    for (int k=0; k < elemA1.groups.size(); k++)
                    {
                        for (int l=0; l < elemA2.groups.size(); l++)
                        {
                            // check if an edge between these two groups exists (it does probably all the time, at least a proximity edge)
                            for (int gg=0; gg < symNodeGraphA.size(); gg++)
                            {
                                if (   (symNodeGraphA[gg].A == elemA1.groups[k] && symNodeGraphA[gg].B == elemA2.groups[l])
                                    || (symNodeGraphA[gg].B == elemA1.groups[k] && symNodeGraphA[gg].A == elemA2.groups[l]) )
                                {
                                    edgesA.push_back(gg);
                                }
                            }
                        }
                    }


                    //#pragma omp parallel for
                    for (int r=0; r < graphB.size(); r++)
                    {
                        const GroupEdge& elemB = graphB[r];
                        if (elemB.type != GroupEdge::PROXIMITY) continue;

                        const RegionElement& elemB1 = regionsB[elemB.A];
                        const RegionElement& elemB2 = regionsB[elemB.B];

                        // construct list of all edges between overlapping groups of regions in B
                        std::vector<int> edgesB;
                        for (int k=0; k < elemB1.groups.size(); k++)
                        {
                            for (int l=0; l < elemB2.groups.size(); l++)
                            {
                                // check if an edge between these two groups exists (it does probably all the time, at least a proximity edge)
                                for (int gg=0; gg < symNodeGraphB.size(); gg++)
                                {
                                    if (   (symNodeGraphB[gg].A == elemB1.groups[k] && symNodeGraphB[gg].B == elemB2.groups[l])
                                        || (symNodeGraphB[gg].B == elemB1.groups[k] && symNodeGraphB[gg].A == elemB2.groups[l]) )
                                    {
                                        edgesB.push_back(gg);
                                    }
                                }
                            }
                        }

                        // each edge connects two regions, each region is overlapped by multiple symmetric nodes
                        // the edge similarity for two regions is computed as the sum of all similarities of the
                        // overlapped symmetric node edges, i.e. we use the edge scores computed previously
                        float score = 0.0f;

                        for (int ea = 0; ea < edgesA.size(); ea++)
                        {
                            float mx = 0;
                            for (int eb = 0; eb < edgesB.size(); eb++)
                            {
                                mx = std::max(mx, eeM[edgesA[ea]][edgesB[eb]]);
                            }
                            score += mx;
                        }


#pragma omp critical
                        {
                            eeMatr.M[c][r] += score;
                            maxentry = std::max(maxentry, eeMatr.M[c][r]);
                        }
                    }
                }

                eeMatr.a = i;
                eeMatr.b = j;

                // normalize matrix
                if (maxentry > 1e-5f)
                    eeMatr.M /= maxentry;
            }
            regionEE.push_back(eeMatr);
            debugOutput << "Ree(" << i << "," << j << "):\n" << eeMatr.M << "\n\n";
        }
    }




    // store matrices
    std::string path;
    path = FileDialogs::getExistingDirectory(InterfaceGlobals::getMainWindow(), "Where store matrices", "", FileDialogs::SAVE_DIR);
    //{
    //    struct AskPath : public OperatorNoArgsLockedResult<std::string>
    //    {
    //        void operator()()
    //        {
    //            set(FileDialogs::getExistingDirectory(InterfaceGlobals::getMainWindow(), "Where store matrices", "", FileDialogs::SAVE_DIR));
    //        }
    //    };

    //    // ask user to specify type of the symmetry
    //    AskPath msg; execOperatorNoArgs(&msg);
    //    path = msg.get();
    //}; // ask path

    if (path.length() == 0) error("Need to specify path\n");

    struct SaveMatrix
    {
        static void doit(const DMatrixF &mat, FILE *file_ptr)
        {
            int n = mat.getRowsDim(), m = mat.getColsDim();
            fwrite(&n, sizeof(int), 1, file_ptr);
            fwrite(&m, sizeof(int), 1, file_ptr);
            if (n > 0 && m > 0) {
                int dim = n*m;
                std::vector<float> vals;
                vals.resize(dim);
                for (int i = 0; i < m; ++i)
                    for (int j = 0; j < n; ++j)
                        vals[i*n + j] = mat[i][j];
                fwrite(&vals[0], sizeof(float)*dim, 1, file_ptr);
            }
        }
    };

    for (unsigned i=0; i < regionVV.size(); i++)
    {
        const Matr& m = regionVV[i];
        std::string nameA, nameB;
        for (int k=paths[m.a].size()-1; k >= 0; k--)
            if (paths[m.a][k] == '\\' || paths[m.a][k] == '/') break;
            else nameA += paths[m.a][k];
            for (int k=paths[m.b].size()-1; k >= 0; k--)
                if (paths[m.b][k] == '\\' || paths[m.b][k] == '/') break;
                else nameB += paths[m.b][k];

                FILE* file = fopen((path + "\\" + std::string(nameA.rbegin(), nameA.rend()) + "-" + std::string(nameB.rbegin(), nameB.rend())).c_str(), "wb");
                SaveMatrix::doit(m.M, file);
                fclose(file);
    }

    for (unsigned i=0; i < regionEE.size(); i++)
    {
        const Matr& m = regionEE[i];
        std::string nameA, nameB;
        for (int k=paths[m.a].size()-1; k >= 0; k--)
            if (paths[m.a][k] == '\\' || paths[m.a][k] == '/') break;
            else nameA += paths[m.a][k];
            for (int k=paths[m.b].size()-1; k >= 0; k--)
                if (paths[m.b][k] == '\\' || paths[m.b][k] == '/') break;
                else nameB += paths[m.b][k];

                FILE* file = fopen((path + "\\" + std::string(nameA.rbegin(), nameA.rend()) + "-" + std::string(nameB.rbegin(), nameB.rend()) + ".edgm").c_str(), "wb");
                SaveMatrix::doit(m.M, file);
                fclose(file);
    }

    debugOutput << "DONE\n";
}

//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::rebuildMaterials()
{
    for ( unsigned int n = 0; n < scene->getNumGLMaterials(); n++ )
    {
        GLShaderMaterial* mat = dynamic_cast<GLShaderMaterial*>( scene->getGLMaterial( n ) );
        if ( mat != nullptr ) mat->increaseVersion();
    }
}

//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::detectFeatureLines()
{
    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    if (selection.size() != 1) 
    {
        error("Please select only one cloud for testing");
        return;
    }
    SGObjectNode* selnode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[0]));
    UnstructuredInCoreTriangleMesh* selcloud = dynamic_cast<UnstructuredInCoreTriangleMesh*>(selnode->getSceneObject());
    if (selcloud == nullptr)
    {
        error("Work only on tri meshes");
        return;
    }

    UnstructuredInCoreTriangleMesh* copy = (UnstructuredInCoreTriangleMesh*)selcloud->copy();
    copy->calculateVertexNormals();
    FeatureSet* fs = detectFeatureLines(copy);
    fs->setup("FeatureLines", AttachedData::ADF_PERSISTENT);
    selcloud->getAttachments()->attachData(fs);
    delete copy;
}

//--------------------------------------------------------------------------------------
FeatureSet* PCIConsistentCrossShapeGrammar::detectFeatureLines(UnstructuredInCoreTriangleMesh* selcloud)
{
    using namespace NAMESPACE_VERSION::sym;

    debugOutput << "detect feature lines ... "; debugOutput.flush();
    //X4_TIMER_START(lines);
    FeatureSet* fs = nullptr;
    {
        if (drawFeatureLines)
        {
            debugRenderer->clearDebugData("fslines");
            debugRenderer->beginRenderJob("fslines");
        }

        CreaseLineFeatureDetector detector;
        detector.setSpatialTolerance(m_Settings->spatialTolerance);
        detector.setAngleTolerance(m_Settings->detectorAngleTolerance);
        detector.setMinimalLength(m_Settings->minFeatureLineLength);
        fs = detector.detect(selcloud);

        if (drawFeatureLines)
        {
            for (unsigned i=0; i < fs->m_Features.size(); i++)
            {
                FeatureLine* line = dynamic_cast<FeatureLine*>(fs->m_Features[i]);
                debugRenderer->addLine(line->m_Position, line->getEndPosition(), makeVector3f(0.8,0,0), makeVector3f(0.8,0,0), featureLineSize);
                debugRenderer->addLine(line->getCenterPosition(), line->getCenterPosition() + line->m_Normal * 0.08f, makeVector3f(0.8,0,0), makeVector3f(0.8,0,0), featureLineSize);
            }
            debugRenderer->endRenderJob();
        }
    }
    debugOutput << "done - " << fs->m_Features.size() << "\n";
    //X4_TIMER_STOP(lines);
    return fs;
}

//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::clusterFeatureLines()
{
    using namespace NAMESPACE_VERSION::sym;

    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    if (selection.size() != 1) 
    {
        error("Please select only one cloud for testing");
        return;
    }
    SGObjectNode* selnode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[0]));
    UnstructuredInCoreTriangleMesh* selcloud = dynamic_cast<UnstructuredInCoreTriangleMesh*>(selnode->getSceneObject());
    if (selcloud == nullptr)
    {
        error("Work only on tri meshes");
        return;
    }

    DVectorUObjectList* list = clusterFeatureLines(dynamic_cast<FeatureSet*>(selcloud->getAttachments()->getData("FeatureLines")));
    list->setup("LineClusters", AttachedData::ADF_PERSISTENT);
    selcloud->getAttachments()->attachData(list);

}

//--------------------------------------------------------------------------------------
DVectorUObjectList* PCIConsistentCrossShapeGrammar::clusterFeatureLines(FeatureSet* fs)
{
    using namespace NAMESPACE_VERSION::sym;

    debugOutput << "cluster feature lines ... "; debugOutput.flush();
    //X4_TIMER_START(lines);
    CreaseLineFeatureDetector detector;
    detector.setSpatialTolerance(m_Settings->spatialTolerance);
    detector.setAngleTolerance(m_Settings->angleTolerance);
    std::vector<std::vector<card32> > clusters = detector.clusterLines(fs, true);
    debugOutput << "done\n";
    //X4_TIMER_STOP(lines);

    card32 numClusters = 0;
    DVectorUObjectList* list = new DVectorUObjectList();
    for (unsigned i=0; i < clusters.size(); i++)
    {
        if (clusters[i].size() <= 3) continue;
        numClusters++;
        DVectorUObject* data = new DVectorUObject;
        data->data.setDim(clusters[i].size());
        for (unsigned j=0; j < clusters[i].size(); j++)
            (*data)[j] = clusters[i][j];
        list->addList(data);
    }

    debugOutput << "found " << numClusters << " clusters\n";
    return list;

}

#if 0
//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::debugRenderCluster()
{
    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    if (selection.size() != 1) 
    {
        error("Please select only one cloud for testing");
        return;
    }
    SGObjectNode* selnode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[0]));
    UnstructuredInCoreTriangleMesh* selcloud = dynamic_cast<UnstructuredInCoreTriangleMesh*>(selnode->getSceneObject());
    if (selcloud == nullptr)
    {
        error("Work only on tri meshes");
        return;
    }

    using namespace NAMESPACE_VERSION::sym;

    DVectorUObjectList* list = dynamic_cast<DVectorUObjectList*>(selcloud->getAttachments()->getData("LineClusters"));
    FeatureSet* fs = dynamic_cast<FeatureSet*>(selcloud->getAttachments()->getData("FeatureLines"));
    if (!list || !fs) return;

    debugRenderer->clearDebugData("clusters");
    debugRenderer->beginRenderJob("clusters");
    for (unsigned i=0; i < list->getNumLists(); i++)
    {
        DVectorUObject* data = dynamic_cast<DVectorUObject*>(list->getList(i));

        srand(i);
        Vector3f color = makeVector3f(rnd01(), rnd01(), rnd01());

        for (unsigned j=0; j < data->getDim(); j++)
        {
            FeatureLine* line = dynamic_cast<FeatureLine*>(fs->m_Features[(*data)[j]]);
            debugRenderer->addFastCylinder(line->m_Position, line->m_Direction, line->m_Length, featureLineSize, color);
        }
    }
    debugRenderer->endRenderJob();
    debugRenderer->instanciateMesh(getScene(), "clusters", 5);
}
#endif 

#if 0
//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::generate2DPattern()
{
    using namespace NAMESPACE_VERSION::sym;

    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    if (selection.size() != 1) 
    {
        error("Please select only one cloud for testing");
        return;
    }
    SGObjectNode* selnode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[0]));
    UnstructuredInCoreTriangleMesh* selcloud = dynamic_cast<UnstructuredInCoreTriangleMesh*>(selnode->getSceneObject());
    if (selcloud == nullptr)
    {
        error("Work only on tri meshes");
        return;
    }

    // generate a test set consisting of a box

    UnstructuredInCoreTriangleMesh* copy = (UnstructuredInCoreTriangleMesh*)selcloud->copy();
    AAT posAAT = copy->getAAT("position");

    for (int i=0; i < (int)m_2DPattern_Size[1]; i++)
    {
        Vector3f g = m_2DPattern_V * float(i);

        for (int j=0; j < (int)m_2DPattern_Size[0]; j++)
        {
            UnstructuredInCoreTriangleMesh* _copy = (UnstructuredInCoreTriangleMesh*)selcloud->copy();
            for (unsigned k=0; k < _copy->getNumPoints(); k++)
                _copy->getPointSet()->set3f(k, posAAT, _copy->getPointSet()->get3f(k, posAAT) + g + m_2DPattern_U * (float)j);
            copy->joinOtherMesh(_copy);
            delete _copy;
        }
    }
    addSceneObject(getScene(), copy, selnode->getName() + "_pattern");
}
#endif

//--------------------------------------------------------------------------------------
void PCIConsistentCrossShapeGrammar::applyColorsToSelectedSymmetries()
{
    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();

    for (unsigned i=0; i < selection.size(); i++)
    {
        SGObjectNode* node = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[i]));
        if (!node) continue;
        UnstructuredInCorePointCloud* pc = dynamic_cast<UnstructuredInCorePointCloud*>(node->getSceneObject());
        if (!pc) continue;

        sym::SymmetryGroupAttachment* sym = dynamic_cast<sym::SymmetryGroupAttachment*>(pc->getAttachments()->getData("SymmetryGroup"));
        if (!sym) continue;

        SimpleAttachment::set4f(pc->getAttachments(), "symColorDetected", symColorDetected);
        SimpleAttachment::set4f(pc->getAttachments(), "symColorVirtual", symColorVirtual);
        SimpleAttachment::set4f(pc->getAttachments(), "symRotAxisColor", symRotColorAxis);
        SimpleAttachment::set1f(pc->getAttachments(), "symRotAxisThickness", symRotAxisThickness);
    }

#if 0
    if (selection.size() == 1)
    {
        SGObjectNode* nodeA = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[0]));
        UnstructuredInCorePointCloud* cloudA = dynamic_cast<UnstructuredInCorePointCloud*>(nodeA->getSceneObject());

        using namespace sym;


        for (unsigned r=0; r < 5; r++)
        {
            Plane3f p1(normalize(makeVector3f(rnd01(),rnd01(),rnd01())), normalize(makeVector3f(rnd01(),rnd01(),rnd01())));
            Plane3f p2(normalize(makeVector3f(rnd01(),rnd01(),rnd01())), normalize(makeVector3f(rnd01(),rnd01(),rnd01())));

            Line line;
            try{
                line = p1.calculateIntersection(p2);
            }catch(...){}

            debugRenderer->beginRenderJob_OneFrame("test",r);
            debugRenderer->addPlane(p1, 1.0f, makeVector3f(1,0,0));
            debugRenderer->addPlane(p2, 1.0f, makeVector3f(0,1,0));
            debugRenderer->addLine(line.getPoint(), line.getPoint() + line.getDirection(), makeVector3f(1,1,0), makeVector3f(1,1,0), 3.0f);
            debugRenderer->endRenderJob();
        }

        std::vector<SymmetryGroup*> groups;
        {
            Reflection* group = new Reflection;
            group->getGenerator()->setup(makeVector3f(0,-1,0), makeVector3f(0,1,0.5));
            groups.push_back(group);			
        }
        {
            Rotation* group = new Rotation(4);
            group->getGenerator()->setup(makeVector3f(0,0,0), makeVector3f(0,1,1), makeVector3f(1,0,0));
            groups.push_back(group);
        }
        {
            RotationH* group = new RotationH(4);
            group->getGenerator()->setup(makeVector3f(0,0,0), makeVector3f(0,1,1), makeVector3f(1,0,0));

            Reflection* ref = new Reflection;
            ref->getGenerator()->setup(group->getGenerator()->getRotationAxisCenter(), group->getGenerator()->getRotationAxis());
            group->setReflection(ref);

            groups.push_back(group);
        }
        {
            RotationV* group = new RotationV(2);
            group->getGenerator()->setup(makeVector3f(0,0,0), makeVector3f(0,1,1), makeVector3f(1,0,0));


            Reflection* ref = new Reflection;
            ref->getGenerator()->setup(group->getGenerator()->getRotationAxisCenter(), shrink4To3(group->getGenerator()->getRigidMotion()->getFrame()[0]));
            group->addReflection(ref);

            group->consolidate();

            groups.push_back(group);
        }
        {
            Dihedral* group = new Dihedral(4);
            group->getGenerator()->setup(makeVector3f(0,0,0), makeVector3f(0,1,1), makeVector3f(1,0,0));

            Rotation* rot = new Rotation(2);
            rot->getGenerator()->setup(group->getGenerator()->getRotationAxisCenter(), shrink4To3(group->getGenerator()->getRigidMotion()->getFrame()[0]), makeVector3f(1,0,0));
            group->addRotation(rot);

            group->consolidate();

            groups.push_back(group);
        }
        {
            DihedralH* group = new DihedralH(4);
            group->getGenerator()->setup(makeVector3f(0,0,0), makeVector3f(0,1,1), makeVector3f(1,0,0));

            Rotation* rot = new Rotation(2);
            rot->getGenerator()->setup(group->getGenerator()->getRotationAxisCenter(), shrink4To3(group->getGenerator()->getRigidMotion()->getFrame()[0]), makeVector3f(1,0,0));
            group->addRotation(rot);

            Reflection* ref = new Reflection;
            ref->getGenerator()->setup(group->getGenerator()->getRotationAxisCenter(), group->getGenerator()->getRotationAxis());
            group->setReflection(ref);

            group->consolidate();

            groups.push_back(group);
        }

        SGRelativeTimeAnimationNode* results = new SGRelativeTimeAnimationNode;
        results->setName("groups");
        for (unsigned i=0; i < groups.size(); i++)
        {
            {
                UnstructuredInCorePointCloud* copypc = (UnstructuredInCorePointCloud*)cloudA->copy();
                CCSGSymmetryGroup* gr = new CCSGSymmetryGroup;
                gr->group = groups[i];
                gr->setup("SymmetryGroup", AttachedData::ADF_PERSISTENT, nullptr);
                copypc->getAttachments()->attachData(gr);
                addPointCloud(getScene(), copypc, intToStr(i));
            }

            SGListNode* list = new SGListNode;
            list->setName(groups[i]->getName());
            for (unsigned t=0; t < groups[i]->getNumTransformations(); t++)
            {
                Matrix4f T = groups[i]->getGenerator()->getWorldTransformation(t);
                UnstructuredInCorePointCloud* copypc = (UnstructuredInCorePointCloud*)cloudA->copy();
                copypc->setTransformation(T);
                SGObjectNode* node = new SGObjectNode(copypc);
                node->setName(intToStr(t));
                list->addNode(node);
            }
            results->addNode(list);

            //delete groups[i];
        }
        getScene()->getRootNode()->addChildNode(results);

    }
#endif
}

//--------------------------------------------------------------------------------------
float PCIConsistentCrossShapeGrammar::computeGroup2GroupScore(
    const pair<SymmetricElement, SymmetricElement>& pair1, const pair<card32,card32>& idxA,
    const pair<SymmetricElement, SymmetricElement>& pair2, const pair<card32,card32>& idxB,
    const DMatrixF& vertexScore)
{
    using namespace sym;

    //#define CAST_PAIR_ELEMENT(elem, cls) (elem.group->getInstanceClass() == cls::getClass() ? dynamic_cast<cls*>(elem.group) : nullptr);
#define CAST_PAIR_ELEMENT(elem, cls) (dynamic_cast<cls*>(elem.group));

    std::vector<pair<SymmetricElement, SymmetricElement>> edges;
    edges.push_back(pair1);
    edges.push_back(pair2);

    std::vector<float> angles(2, FLT_MAX);
    for (unsigned e=0; e < 2; e++)
    {
        const pair<SymmetricElement, SymmetricElement>& edge = edges[e];

        // TODO maybe compare here geometry axis (lets say just simple PCA-axis comparison)
        // this is for the non-symmetrical case
        if (!edge.first.group || !edge.second.group) return 0;

        // Cast second given pair
        Reflection* refA = CAST_PAIR_ELEMENT(edge.first, Reflection);
        Reflection* refB = CAST_PAIR_ELEMENT(edge.second, Reflection);

        Rotation* rotA = CAST_PAIR_ELEMENT(edge.first, Rotation);
        Rotation* rotB = CAST_PAIR_ELEMENT(edge.second, Rotation);

        RotationH* rotHA = CAST_PAIR_ELEMENT(edge.first, RotationH);
        RotationH* rotHB = CAST_PAIR_ELEMENT(edge.second, RotationH);

        RotationV* rotVA = CAST_PAIR_ELEMENT(edge.first, RotationV);
        RotationV* rotVB = CAST_PAIR_ELEMENT(edge.second, RotationV);

        Dihedral* dihA = CAST_PAIR_ELEMENT(edge.first, Dihedral);
        Dihedral* dihB = CAST_PAIR_ELEMENT(edge.second, Dihedral);

        DihedralH* dihHA = CAST_PAIR_ELEMENT(edge.first, DihedralH);
        DihedralH* dihHB = CAST_PAIR_ELEMENT(edge.second, DihedralH);

        Lattice* latA = CAST_PAIR_ELEMENT(edge.first, Lattice);
        Lattice* latB = CAST_PAIR_ELEMENT(edge.second, Lattice);

        float& angle = angles[e];

        // very easy and trivial case: take rotation axis or reflection direction of one element and compare it to the same of another
        Vector3f structuralVector_1, structuralVector_2;
        if (refA)  structuralVector_1 = refA->getGenerator()->plane.getNormal();
        if (rotA)  structuralVector_1 = rotA->getGenerator()->getRotationAxis();
        if (rotHA) structuralVector_1 = rotHA->getGenerator()->getRotationAxis();
        if (rotVA) structuralVector_1 = rotVA->getGenerator()->getRotationAxis();
        if (dihA)  structuralVector_1 = dihA->getGenerator()->getRotationAxis();
        if (dihHA)  structuralVector_1 = dihHA->getGenerator()->getRotationAxis();
        if (latA)  structuralVector_1 = latA->getGenerator()->u();

        if (refB)  structuralVector_2 = refB->getGenerator()->plane.getNormal();
        if (rotB)  structuralVector_2 = rotB->getGenerator()->getRotationAxis();
        if (rotHB) structuralVector_2 = rotHB->getGenerator()->getRotationAxis();
        if (rotVB) structuralVector_2 = rotVB->getGenerator()->getRotationAxis();
        if (dihB)  structuralVector_2 = dihB->getGenerator()->getRotationAxis();
        if (dihHB)  structuralVector_2 = dihHB->getGenerator()->getRotationAxis();
        if (latB)  structuralVector_2 = latB->getGenerator()->u();

        angle = fabs(normalize(structuralVector_1) * normalize(structuralVector_2));

        // when comparing with dihedral group, we need to take care that a dihedral group has also additional rotation axes
        // TODO: need better check here, currently only compare to a 2fold dihedral group
        if (dihA && dihA->getNumberRotations() == 2)
        {
            for (int i=0; i < dihA->getNumRotations(); i++)
            {
                float an = normalize(dynamic_cast<Rotation*>(dihA->getRotation(i))->getGenerator()->getRotationAxis()) * normalize(structuralVector_2);
                if (fabs(an) > angle) angle = fabs(an);
            }
        }else if (dihB && dihB->getNumberRotations() == 2)
        {
            for (int i=0; i < dihB->getNumRotations(); i++)
            {
                float an = normalize(dynamic_cast<Rotation*>(dihB->getRotation(i))->getGenerator()->getRotationAxis()) * normalize(structuralVector_1);
                if (fabs(an) > angle) angle = fabs(an);
            }
        }
    }

    float score = exp(- pow((angles[0] - angles[1]) / m_Settings->angelGroupGroupCompareSigma ,2.f));

    return score;
}
