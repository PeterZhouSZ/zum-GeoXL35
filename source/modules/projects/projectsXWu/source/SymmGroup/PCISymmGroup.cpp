#include "StdAfx.h"

#include "PCISymmGroup.h"

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

#include "Util\ColorSchemer.hpp"

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
IMPLEMENT_CLASS(SymmGroupSettings, 27)
{
    BEGIN_CLASS_INIT(SymmGroupSettings);
    INIT_PROPERTY_TABLE();

    ADD_SEPARATOR("Settings");
    ADD_INT32_PROP(numCPUCores, 0);
    ADD_FLOAT32_PROP(adjMaxDistance, 1);
    SET_USER_EDIT(false);
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
SymmGroupSettings::SymmGroupSettings()
{
    numCPUCores = -1;
    adjMaxDistance = 0.01f;
    descriptorRadiusBBFactor = 0.1f;
    gidSelectedColor = makeVector4f(1.0f, 0.0f, 0.0f, 1.0f);
    gidDefaultColor = makeVector4f(0.9f, 0.9f, 0.7f, 1.0f);
    gidSymmetricColor = makeVector4f(0.0f, 0.6f, 0.0f, 1.0f);
    gidNonrelevantColor = makeVector4f(0.6f, 0.6f, 0.4f, 0.25f);

    proximityRelationFactor = 5.0f;


    runPreliminaryPlausibilityCheck = false;
    performTopologyCheckOnFoundGroup = true;

    detectorAngleTolerance = 0.99f; 
    minFeatureLineLength = 0.05; // 0.08;

    symMeshSubsampling = 0.003; // 0.007;
    spatialTolerance = 0.01; // 0.03;
    angleTolerance = 0.01; // 0.075;
    symGroupCombineAngleTolerance = 0.05f;
    symmetryCoveredPercentage = 0.8f;
    minConnectedComponent = 500;
    useICPWhileDetectingBaseElements = false;

    minNumRotations = 3;
    clusterFoundGroupsBeforeExtractGeometry = true;
    gridAngleTolerance = 0.05f;
    latticeMinGeneratorLength = 0.07f;
    reuseSegmentation = true;
    searchLattice = false;
    doVeryConservativeRegionGrowing = false;

    maxIcpIterations = 64;
    icpOutlierDistanceFactor = 3;
    icpMinPercentInliers = 0.4f;
    shapeSymmThreshold = 0.05f;
    distCompareSubpartThreshold = 0.075f;
    smallPartMinimalSizeFactor = 0.02f;

    symGroupCompareThreshold = 0.5f;
    rotGroupCompareSigma = 0.2f;
    refGroupCompareSigma = 0.5f;
    latGroupCompareSigma = 0.3f;
    dihGroupCompareSigma = 0.5f;
    angelGroupGroupCompareSigma = 0.15f;

    //embeddingDimensions = -1;
    embeddingCutoffEigenvalue = 10.0f;
}

// -------------------------------------------------------------------------------
int SymmGroupSettings::getNumAvailableThreads()
{
    int numthreads = QThread::idealThreadCount() + numCPUCores;
    if (numCPUCores > 0) 
        numthreads = numCPUCores;

    if (numthreads > QThread::idealThreadCount()) numthreads = QThread::idealThreadCount();
    if (numthreads <= 0) numthreads = 1;

    return numthreads;
}
//--------------------------------------------------------------------------------------
SymmGroupSettings::~SymmGroupSettings()
{
}


//--------------------------------------------------------------------------------------
IMPLEMENT_CLASS(SymmGroupClass, 3)
{
    BEGIN_CLASS_INIT(SymmGroupClass);
    INIT_PROPERTY_TABLE();

    ADD_OBJECT_LIST_PROP(symGroup, 2, sym::SymmetryGroupAttachment::getClass());

    ADD_STRING_PROP(loggedOutput, 3);
    ADD_OBJECT_PROP(stats, 3, Statistics::getClass(), true);
}
//--------------------------------------------------------------------------------------
SymmGroupClass::SymmGroupClass()
{
    stats = new Statistics;
}

//--------------------------------------------------------------------------------------
SymmGroupClass::~SymmGroupClass()
{
    if (stats) delete stats;
    for (unsigned i=0; i < m_symGroup.size(); i++) if (m_symGroup[i]) delete m_symGroup[i];
}

//--------------------------------------------------------------------------------------
void SymmGroupClass::write(OutputObjectStream *s) const
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
void SymmGroupClass::read(InputObjectStream *s)
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
IMPLEMENT_CLASS(PCISymmGroup, 0)
{
    BEGIN_CLASS_INIT(PCISymmGroup);
    INIT_PROPERTY_TABLE();

    ADD_SEPARATOR("Settings");

    ADD_OBJECT_PROP(m_Settings, 0, SymmGroupSettings::getClass(), true);

    ADD_VECTOR4F_PROP(symColorDetected, 0);
    ADD_VECTOR4F_PROP(symColorVirtual,0);
    ADD_VECTOR4F_PROP(symRotColorAxis,0);
    ADD_FLOAT32_PROP(symRotAxisThickness,0);

    ADD_INT32_PROP(showCluster, 0);
    ADD_BOOLEAN_PROP(doRenderSelected, 0);
    ADD_FLOAT32_PROP(selectedSymDistanceThreshold, 0);

    ADD_FLOAT32_PROP(featureLineSize, 0);
    ADD_BOOLEAN_PROP(showSymmetryGraph, 0);

    ADD_SEPARATOR_METHOD("Symmetries");
    ADD_NOARGS_METHOD_DESC(detectFeatureLines, "Try to find all feature lines within the selected shape.");
    ADD_NOARGS_METHOD_DESC(clusterFeatureLines, "Cluster all detected feature lines to identify identical one.");
    //ADD_NOARGS_METHOD_DESC(findLatticeGroups, "Find lattice structures within found symmetries");
    ADD_NOARGS_METHOD_DESC(findPreliminarySymmetryGroups, "Find groups within the set of line features");
    ADD_NOARGS_METHOD_DESC(extractFinalSymmetricGroups, "Extract final symmetric groups from the geometry with according geometry");
    ADD_NOARGS_METHOD_DESC(constructDisjointRegions, "Find disjoint regions and extract them as point clouds");
    ADD_NOARGS_METHOD_DESC(execute, "Automatic symmetry detection");

    ADD_SEPARATOR_METHOD("Debug");
    ADD_NOARGS_METHOD_DESC(showStatisticsAndLogs, "Show statistics and logs stored for the selected point cloud.");
    ADD_NOARGS_METHOD_DESC(debugRenderFeatureLine, "Show all featuer lines.");
    ADD_NOARGS_METHOD_DESC(debugRenderCluster, "Show all clusters.");
}

//--------------------------------------------------------------------------------------
PCISymmGroup::PCISymmGroup()
{
    m_Settings = new SymmGroupSettings();
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
    showSymmetryGraph = false;
    showGroupGraph = nullptr;
    extractSubgroupsWhileSelecting = true;
    showFoundGroups = false;

    featureLineSize = 2.0f;

    embCurrentlySelectedNode = -1;
    loadMatchForEmbedding = false;
    embDistanceResultSigma = 0.2f;
}

//--------------------------------------------------------------------------------------
PCISymmGroup::~PCISymmGroup()
{
    if (m_Settings) delete m_Settings;
}

//----------------------------------------------------------------------------------
#include "Util\FunctionContainer.hpp"
void PCISymmGroup::execute() 
{
    std::ostringstream ss; ss << "\n";
    Timer timer_symm_det; timer_symm_det.getDeltaValue();
    typedef void(PCISymmGroup::*VoidFuncPtr)(void);
    FunctionContainer<VoidFuncPtr> func_list;
    func_list.push_back(&PCISymmGroup::detectFeatureLines);
    func_list.push_back(&PCISymmGroup::clusterFeatureLines);
    //func_list.push_back(&PCISymmGroup::findLatticeGroups);
    func_list.push_back(&PCISymmGroup::findPreliminarySymmetryGroups);
    func_list.push_back(&PCISymmGroup::extractFinalSymmetricGroups);

    const size_t num_func = func_list.size();
    progressWindow->pushStep(true, "execute");
    for (size_t fi = 0; fi < num_func; ++fi) {
        Timer timer; timer.getDeltaValue();
        (this->*(func_list[fi]))();
        ss << "step " << fi << " time: " << convertTimeToString( timer.getDeltaValue() ) << "\n";
        progressWindow->progressf((float)fi/(float)(num_func-1));
    }
    progressWindow->popStep();
    debugOutput << ss.str() << "symmetry detection time: " << convertTimeToString( timer_symm_det.getDeltaValue() ) << "\n";
}

//----------------------------------------------------------------------------------
void PCISymmGroup::connectToSceneImpl(Scene * scene,OnSceneChangeCallback *onChange, QWidget *sceneEditorWidget, QWidget* toolWidget)
{
    dynamic_cast<NAMESPACE_VERSION::SceneEditorWidget*>(sceneEditorWidget)->on_action_Auto_Layout_triggered(true);
    sceneEditorW = dynamic_cast<SceneEditorWidget*>(sceneEditorWidget);

    this->scene = this->scene;

    // load scene settings
    if (scene->getAttachments()->getData(SymmGroupSettings::getDefaultName()))
    {
        m_Settings = dynamic_cast<SymmGroupSettings*>(scene->getAttachments()->getData(SymmGroupSettings::getDefaultName()));
    }else
    {
        m_Settings->setup(SymmGroupSettings::getDefaultName(), AttachedData::ADF_PERSISTENT);
        scene->getAttachments()->attachData(m_Settings);
    }
}

//--------------------------------------------------------------------------------------
void PCISymmGroup::attachSettingsToScene()
{
    if (getScene() && m_Settings)
    {
        getScene()->getAttachments()->attachData(m_Settings);
    }
}

//--------------------------------------------------------------------------------------
bool PCISymmGroup::drawGLSceneRendering()
{
    if (m_State == SELECT_GENERATOR) return false;
    if (showCluster >= 0) return false;
    if (!(selectedSymDistanceThreshold > 1 || selectedSymDistanceThreshold < 0)) return false;

    return true;
}

//--------------------------------------------------------------------------------------
void PCISymmGroup::glDrawToolImplementation(GLContext* glContext)
{
    SceneObject* objToRender = nullptr;

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

#if 0
//--------------------------------------------------------------------------------------
void PCISymmGroup::computeGraphs()
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

                    for (std::set<int32>::const_iterator jt=it->second.begin(); jt != it->second.end(); jt++)
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

//--------------------------------------------------------------------------------------
UnstructuredInCorePointCloud* PCISymmGroup::getGraphNode(SGObjectNode* node)
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

#include <detector/SymmetryGroupDetector.h>

//--------------------------------------------------------------------------------------

std::vector<sym::SymmetryGroupAttachment*> PCISymmGroup::findLatticeGroups(FeatureSet* fs, DVectorUObjectList* clusters, ProgressWindow* progress)
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

std::vector<sym::SymmetryGroupAttachment*> PCISymmGroup::findOtherGroups(FeatureSet* fs, DVectorUObjectList* clusters, ProgressWindow* progress)
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
        //	groupsResult.push_back(sym::SymmetryGroupAttachment::buildFromSymmetryGroupAttachment(groups[i]));
    }

#if 0
    // DEBUG show groups
    for (unsigned i=0; i < groupsResult.size() && showFoundGroups; i++)
    {
        sym::SymmetryGroupAttachment* group = groupsResult[i];

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

//--------------------------------------------------------------------------------------

std::vector<UnstructuredInCorePointCloud*> PCISymmGroup::extractOtherGroupElements(
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

    struct SortPredicate // order symmetry groups by their rank first
    {
        static bool sort(const sym::SymmetryGroupAttachment* a, const sym::SymmetryGroupAttachment* b)
        {
            if (a->group->rank_ == b->group->rank_)
                return a->symmetryPoints.size() < b->symmetryPoints.size();
            return a->group->rank_ < b->group->rank_;
        }
    };
    sort(groups.begin(), groups.end(), SortPredicate::sort);

    std::vector<sym::SymmetryGroupAttachment*> _groups = groups;
    groups.clear();

    float meddist = getCachedMedianPointDistance(shape);


    // extract symmetric geometry from all groups in parallel (remove very tiny groups)
    std::vector<UnstructuredInCorePointCloud*> resultShapes;
    omp_set_num_threads(m_Settings->getNumAvailableThreads());
#pragma omp parallel for shared(resultShapes, _groups, assigned) ordered schedule(dynamic)
    for (int j=0; j < _groups.size(); j++)
    {
        UnstructuredInCorePointCloud* pc = nullptr;

#pragma omp ordered
        {
            debugOutput << j << " - symmetric check of [" << _groups[j]->id << "] : " << _groups[j]->key << "\n";
        }

        if (_groups[j]->clearGeometrically(shape, tpg, fs, meddist, m_Settings->minConnectedComponent, &pc,
            true/*true*/, false, false, m_Settings->doVeryConservativeRegionGrowing, nullptr, false))
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

    // perform final test, where we remove groups which are spurious
    //_groups = symDetector.removeSpuriousGroups(getScene(), shape, groups);
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
pair<UnstructuredInCorePointCloud*, std::vector<UnstructuredInCorePointCloud*>> PCISymmGroup::detectAndExtractFinalSymmetries(
    const std::string& shapeName,
    UnstructuredInCoreTriangleMesh* mesh, 
    FeatureSet* fs, 
    DVectorUObjectList* cluster)
{
    pair<UnstructuredInCorePointCloud*, SymmGroupClass*> shapeccsg = findPreliminarySymmetryGroups(shapeName, mesh, fs, cluster);

    UnstructuredInCorePointCloud* shape = shapeccsg.first;
    SymmGroupClass* ccsg = shapeccsg.second;

    FeatureSet* _fs = dynamic_cast<FeatureSet*>(shape->getAttachments()->getData("FeatureLines"));
    DVectorUObjectList* _clusters = dynamic_cast<DVectorUObjectList*>(shape->getAttachments()->getData("LineClusters"));
    InCorePCTopologyGraph* tpg = dynamic_cast<TopologyAttachment*>(shape->getAttachments()->getData(TopologyAttachment::getDefaultName()))->getTopology();

    return extractFinalSymmetricGroups(shape, fs, cluster, ccsg, tpg);
}

//--------------------------------------------------------------------------------------
void PCISymmGroup::showStatisticsAndLogs()
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
    SymmGroupClass* ccsg = dynamic_cast<SymmGroupClass*>(shape->getAttachments()->getData(SymmGroupClass::getDefaultName()));

    debugOutput << "------------- LOGS --------------\n";
    debugOutput << ccsg->loggedOutput << "\n";

    debugOutput << "------------- STATS --------------\n";
    debugOutput << ccsg->stats->outputAll();	
}

//--------------------------------------------------------------------------------------
void PCISymmGroup::extractFinalSymmetricGroups()
{
    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    //if (selection.size() != 1) 
    //{
    //    error("Please select only one node for which symmetries were computed");
    //    return;
    //}
    for (unsigned si = 0; si < selection.size(); ++si) {
        SGObjectNode* ssnode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[si]));

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
        SymmGroupClass* ccsg = dynamic_cast<SymmGroupClass*>(shape->getAttachments()->getData(SymmGroupClass::getDefaultName()));
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


            //if (result.first)
            //{
            //    SGObjectNode* nonsym = new SGObjectNode(result.first);
            //    nonsym->setName("nonsym");
            //    sublist->addChildNode(nonsym);
            //}

            unsigned numsymm = result.second.size();
            for(unsigned i=0; i < numsymm; i++)
            {
                //Vector3f color = rainbowGradient3f((double)i / numsymm);
                //Vector3f color = ColorListUnit[i % NumColorListUnit];
                //Vector3f color = (i==0) ? ColorListUnit[0] : ColorListUnit[1];

                UnstructuredInCorePointCloud* pc = result.second.at(i);
                //sym::SymmetryGroupAttachment* att = dynamic_cast<sym::SymmetryGroupAttachment*>(
                //    pc->getAttachments()->getData("SymmetryGroup"));
                //Vector3f color = (dynamic_cast<sym::Reflection*>(att->group)) ?
                //    StandardColors<>::color_red : StandardColors<>::color_blue;
                //PointSet* ps = pc->getPointSet();
                //AAT colorAAT = ps->getAAT("color");
                //for (unsigned jj = 0; jj < ps->getNumEntries(); ++jj) {
                //    ps->set3f(jj, colorAAT, color);
                //}
                //att->color = color;

                SGObjectNode* sym = new SGObjectNode(pc);
                sym->setName(intToStr(i + (result.first ? 1 : 0)));
                sublist->addChildNode(sym);
            }
        }
        getScene()->getRootNode()->addChildNode(listnode);

        //if (result.first) addPointCloud(getScene(), result.first, "nonsymmetric");

        //for(unsigned i=0; i < result.second.size(); i++)
        //	addPointCloud(getScene(), result.second.at(i), std::string("element_") + intToStr(i));
    }
}

//--------------------------------------------------------------------------------------
pair<UnstructuredInCorePointCloud*, std::vector<UnstructuredInCorePointCloud*>>
    PCISymmGroup::extractFinalSymmetricGroups(
    UnstructuredInCorePointCloud* shape,
    FeatureSet* _fs,
    DVectorUObjectList* _clusters,
    SymmGroupClass* ccsg,
    InCorePCTopologyGraph* tpg
    )
{
    std::vector<sym::SymmetryGroupAttachment*>& _symGroups = ccsg->m_symGroup;

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
        InCorePCTopologyGraph* _tpg = cmd.computeEpsKNNTopology(nonsym->getPointSet(), nonsym);
        removeSmallPatches(nonsym, _tpg, nullptr, m_Settings->minConnectedComponent);
        delete _tpg;
    }

    if (nonsym->getNumPoints() < m_Settings->minConnectedComponent)
    {
        delete nonsym;
        nonsym = nullptr;
    }

    //struct ResultSortPredicate
    //{
    //    bool operator()(UnstructuredInCorePointCloud*& a, UnstructuredInCorePointCloud*& b) const
    //    {
    //        return a->getNumPoints() > b->getNumPoints();
    //    }
    //};
    //sort(results.begin(), results.end(), ResultSortPredicate());

    for (unsigned i=0; i < latticeGroup.size(); i++) delete latticeGroup[i];
    for (unsigned i=0; i < otherGroup.size(); i++) delete otherGroup[i];
    delete fs;
    delete clusters;

    return pair<UnstructuredInCorePointCloud*, std::vector<UnstructuredInCorePointCloud*>>(nonsym, results);
}

//--------------------------------------------------------------------------------------
void PCISymmGroup::findPreliminarySymmetryGroups()
{
    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    //if (selection.size() != 1) 
    //{
    //    error("Please select only one node for which symmetries should be detected");
    //    return;
    //}
    for (unsigned si = 0; si < selection.size(); ++si) {
        SGObjectNode* selnode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[si]));
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
}

//--------------------------------------------------------------------------------------
pair<UnstructuredInCorePointCloud*,SymmGroupClass*> PCISymmGroup::findPreliminarySymmetryGroups(const std::string& shapeName, UnstructuredInCoreTriangleMesh* selcloud, FeatureSet* _fs, DVectorUObjectList* _clusters)
{
    SymmGroupClass* ccsg = new SymmGroupClass();
    ccsg->setup(SymmGroupClass::getDefaultName(), AttachedData::ADF_PERSISTENT);

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
        tpg = cmd.computeEpsKNNTopology(shape->getPointSet(), shape);

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

    return pair<UnstructuredInCorePointCloud*, SymmGroupClass*>(shape, ccsg);
}

//--------------------------------------------------------------------------------------
void PCISymmGroup::constructDisjointRegions()
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
    SymmGroupClass* ccsg = dynamic_cast<SymmGroupClass*>(selpc->getAttachments()->getData(SymmGroupClass::getDefaultName()));
    if (!ccsg)
    {
        error("Selected point cloud must have a SymmGroupClass attached. Did you performed the symmetry detection before that?");
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

//--------------------------------------------------------------------------------------
bool PCISymmGroup::pickPoint(int32 x, int32 y, PointCloud *pc, mpcard &index)
{
    UnstructuredInCorePointCloud *upc = dynamic_cast<UnstructuredInCorePointCloud*>(pc);
    if (!upc) return false;
    PointSet *ps = upc->getPointSet();
    float32 dist = computeMinOnScreenDistancePoint(
        x,y,ps, 
        getScene()->getRootState()->staticState->viewFrustum,
        getScene()->getRootState()->staticState->camera,
        screenSizeX, screenSizeY, 
        index);
    if (dist <= 6.0f) return true;
    return false;
}

//--------------------------------------------------------------------------------------
void PCISymmGroup::mouseMoved(int32 x, int32 y)
{
    // in case if we are about to select candidates
    mpcard index;
    if (m_selectSymmetryPC && pickPoint(x,y,m_selectSymmetryPC,index))
    {
        AAT cidAAT = m_selectSymmetryPC->getAAT("cid");
        AAT colorAAT = m_selectSymmetryPC->getAAT("color");

        card32 gid = m_selectSymmetryPC->getPointSet()->get2i(index, cidAAT)[1];
        if (lastSelectedGID == gid) return;

        // work with this gid only if this one can still be selected
        if (find(m_selectSymmetryRemainingGIDs.begin(), m_selectSymmetryRemainingGIDs.end(), gid) == m_selectSymmetryRemainingGIDs.end()) return;

        // mark all point of the new gid and unmark all previous points
        for (unsigned i=0; i < m_selectSymmetryPC->getNumPoints(); i++)
        {
            Vector4f color = m_selectSymmetryPC->getPointSet()->get4f(i, colorAAT);

            if (m_selectSymmetryPC->getPointSet()->get2i(i, cidAAT)[1] == gid)
                m_selectSymmetryPC->getPointSet()->set4f(i, colorAAT, color * 2.0f);
            if (m_selectSymmetryPC->getPointSet()->get2i(i, cidAAT)[1] == lastSelectedGID)
                m_selectSymmetryPC->getPointSet()->set4f(i, colorAAT, color * 0.5f);
        }

        lastSelectedGID = gid;

        rebuildMaterials();
    }
}

//--------------------------------------------------------------------------------------
void PCISymmGroup::mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState)
{
    if (m_selectSymmetryPC && buttonsState.getLeft())
    {
        // add selected gid to the symmetric set
        if (modifiersState.getCtrl())
        {
            if (find(m_selectedGIDs.begin(), m_selectedGIDs.end(), lastSelectedGID) != m_selectedGIDs.end()) return;

            m_selectedGIDs.push_back(lastSelectedGID);
        }

        // remove selected gid to the symmetric set
        else if (modifiersState.getAlt())
        {
            std::vector<card32>::iterator it = find(m_selectedGIDs.begin(), m_selectedGIDs.end(), lastSelectedGID);
            if (it == m_selectedGIDs.end()) return;

            m_selectedGIDs.erase(it);
        }
    }
}

//--------------------------------------------------------------------------------------
void PCISymmGroup::keyDown(GeneralKey key)
{
    if (m_State == SELECT_GENERATOR && key.getKey() == keyReturn)
    {
        selectGeneratorCondition.wakeAll();
    }
}

//--------------------------------------------------------------------------------------
void PCISymmGroup::rebuildMaterials()
{
    for ( unsigned int n = 0; n < scene->getNumGLMaterials(); n++ )
    {
        GLShaderMaterial* mat = dynamic_cast<GLShaderMaterial*>( scene->getGLMaterial( n ) );
        if ( mat != nullptr ) mat->increaseVersion();
    }
}

//--------------------------------------------------------------------------------------
void PCISymmGroup::detectFeatureLines()
{
    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    //if (selection.size() != 1) 
    //{
    //    error("Please select only one cloud for testing");
    //    return;
    //}
    for (unsigned si = 0; si < selection.size(); ++si) {
        SGObjectNode* selnode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[si]));
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
}

//--------------------------------------------------------------------------------------
FeatureSet* PCISymmGroup::detectFeatureLines(UnstructuredInCoreTriangleMesh* selcloud)
{
    using namespace NAMESPACE_VERSION::sym;

    debugOutput << "detect feature lines ... "; debugOutput.flush();
    //X4_TIMER_START(lines);
    FeatureSet* fs = nullptr;
    {
        CreaseLineFeatureDetector detector;
        detector.setSpatialTolerance(m_Settings->spatialTolerance);
        detector.setAngleTolerance(m_Settings->detectorAngleTolerance);
        detector.setMinimalLength(m_Settings->minFeatureLineLength);
        fs = detector.detect(selcloud);
    }
    debugOutput << "done - " << fs->m_Features.size() << "\n";
    //X4_TIMER_STOP(lines);
    return fs;
}

//--------------------------------------------------------------------------------------
void PCISymmGroup::clusterFeatureLines()
{
    using namespace NAMESPACE_VERSION::sym;

    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    //if (selection.size() != 1) 
    //{
    //    error("Please select only one cloud for testing");
    //    return;
    //}
    for (unsigned si = 0; si < selection.size(); ++si) {
        SGObjectNode* selnode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[si]));
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
}

//--------------------------------------------------------------------------------------
DVectorUObjectList* PCISymmGroup::clusterFeatureLines(FeatureSet* fs)
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

//--------------------------------------------------------------------------------------
void PCISymmGroup::debugRenderFeatureLine()
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

    FeatureSet* fs = dynamic_cast<FeatureSet*>(selcloud->getAttachments()->getData("FeatureLines"));
    if (!fs) return;

    debugRenderer->beginRenderJob_OneFrame("feature_lines_", DR_FRAME++);
    for (unsigned i=0; i < fs->m_Features.size(); i++)
    {
        FeatureLine* line = dynamic_cast<FeatureLine*>(fs->m_Features[i]);
        debugRenderer->addLine(line->m_Position, line->getEndPosition(), makeVector3f(0.8,0,0), makeVector3f(0.8,0,0), featureLineSize);
        debugRenderer->addLine(line->getCenterPosition(), line->getCenterPosition() + line->m_Normal * 0.08f, makeVector3f(0.8,0,0), makeVector3f(0,0,1), featureLineSize);
    }
    debugRenderer->endRenderJob();
}

//--------------------------------------------------------------------------------------
void PCISymmGroup::debugRenderCluster()
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

    for (unsigned i=0; i < list->getNumLists(); i++)
    {
        debugRenderer->beginRenderJob_OneFrame("clusters_", DR_FRAME++);
        DVectorUObject* data = dynamic_cast<DVectorUObject*>(list->getList(i));

        srand(i);
        Vector3f color = makeVector3f(rnd01(), rnd01(), rnd01());

        for (unsigned j=0; j < data->getDim(); j++)
        {
            //debugOutput << (*data)[j] << "\n";
            FeatureLine* line = dynamic_cast<FeatureLine*>(fs->m_Features[(*data)[j]]);
            debugRenderer->addLine(line->m_Position, line->m_Position+line->m_Length*line->m_Direction,
                makeVector3f(1, 0, 0), makeVector3f(0, 0, 1), 3);
            //debugRenderer->addFastCylinder(line->m_Position, line->m_Direction, line->m_Length, featureLineSize, color);
        }
        debugRenderer->endRenderJob();
    }
    //debugRenderer->instanciateMesh(getScene(), "clusters", 5);
}

//--------------------------------------------------------------------------------------
float PCISymmGroup::computeGroup2GroupScore(
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
