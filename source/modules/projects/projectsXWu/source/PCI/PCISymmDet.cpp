#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "PCI/PCISymmDet.h"
#include "Symmetry/SymmDetHiPrec.h"
#include "Util/TrimeshStatic.h"
//---------------------------------------------------------------------------
#include "UnstructuredInCoreTriangleMesh.h"
#include "Timer.h"
#include "ProgressWindow.h"

#include <group/Lattice.h>
#include <group/Rotation.h>
#include <group/Reflection.h>
#include <group/Dihedral.h>
#include "detector/CreaseLineFeatureDetector.h"
#include "detector/SymmetryGroupDetector.h"
#include "GeometricFeature.h"
#include "DVectorObject.h"

#include "PCCTriangleMeshSampler.h"
#include "PCCComputeTopology.h"
#include "TopologyAttachment.h"
#include "TopologicalKNNIterator.h"
#include "GeometricTools.h"

#include "SGListNode.h"
#include "SGRelativeTimeAnimationNode.h"

#include "PropertyTableProperty.h"
#include "SeparatorClassProperty.h"
//---------------------------------------------------------------------------
IMPLEMENT_CLASS( PCISymmDet ,0)
{
    BEGIN_CLASS_INIT( PCISymmDet );
    INIT_PROPERTY_TABLE();

    ADD_SEPARATOR("Settings");
    //ADD_OBJECT_PROP(symmDetContCCSG, 0, SymmDetContCCSG::getClass(), true);

    ADD_SEPARATOR_METHOD("Precise Symmetries");
    ADD_NOARGS_METHOD( DetectHiPrecSymmtry );

    ADD_SEPARATOR_METHOD("Automatic Symmetries");
    //ADD_NOARGS_METHOD( DetectCCSGSymmtry );
}
//---------------------------------------------------------------------------

////--------------------------------------------------------------------------------------
//IMPLEMENT_CLASS(CCSGClassification, 1)
//{
//    BEGIN_CLASS_INIT(CCSGClassification);
//    INIT_PROPERTY_TABLE();
//
//    ADD_OBJECT_LIST_PROP(Group, 1, CCSGSymmetryGroup::getClass());
//}
////--------------------------------------------------------------------------------------
//CCSGClassification::~CCSGClassification()
//{
//    for (unsigned i=0; i < m_Group.size(); i++) delete m_Group[i];
//}
//
////--------------------------------------------------------------------------------------
//void CCSGClassification::write(OutputObjectStream *s) const
//{
//    AttachedData::write(s);
//
//    //shape std::map
//    card32 size = (card32)shapeMap.size();
//    s->write<card32>(size);
//    for (std::map<std::string, std::vector<int> >::const_iterator it = shapeMap.begin(); it != shapeMap.end(); it++)
//    {
//        s->writeString(it->first);
//        card32 subsize = (card32)it->second.size();
//        s->write<card32>(subsize);
//        for (std::vector<int>::const_iterator jt = it->second.begin(); jt != it->second.end(); jt ++)
//            s->write<int32>(*jt);
//    }
//
//    // symmetryT
//    size = (card32)symmetryT.size();
//    s->write<card32>(size);
//    for (std::map<unsigned, std::vector<Matrix4f> >::const_iterator it = symmetryT.begin(); it != symmetryT.end(); it++)
//    {
//        s->write<card32>(it->first);
//        card32 subsize = (card32)it->second.size();
//        s->write<card32>(subsize);
//        for (std::vector<Matrix4f>::const_iterator jt = it->second.begin(); jt != it->second.end(); jt ++)
//            s->writeBuffer(jt->data(), 16 * sizeof(float32));
//    }
//
//    // symmetryid
//    size = (card32)symmetryId.size();
//    s->write<card32>(size);
//    for (std::map<unsigned, std::vector<unsigned> >::const_iterator it = symmetryId.begin(); it != symmetryId.end(); it++)
//    {
//        s->write<card32>(it->first);
//        card32 subsize = (card32)it->second.size();
//        s->write<card32>(subsize);
//        for (std::vector<unsigned>::const_iterator jt = it->second.begin(); jt != it->second.end(); jt ++)
//            s->write<card32>(*jt);
//    }
//
//}
//
////--------------------------------------------------------------------------------------
//void CCSGClassification::read(InputObjectStream *s)
//{
//    AttachedData::read(s);
//
//    //shape std::map
//    card32 size = 0;
//    s->read<card32>(size);
//    for (unsigned i=0; i < size; i++)
//    {
//        std::string id;
//        s->readString(id);
//        card32 subsize = 0;
//        s->read<card32>(subsize);
//        for (unsigned j=0; j < subsize; j++)
//        {
//            int32 k = 0;
//            s->read<int32>(k);
//            shapeMap[id].push_back(k);
//        }
//    }
//
//    // symmetryT
//    s->read<card32>(size);
//    for (unsigned i=0; i < size; i++)
//    {
//        card32 id;
//        s->read<card32>(id);
//        card32 subsize = 0;
//        s->read<card32>(subsize);
//        for (unsigned j=0; j < subsize; j++)
//        {
//            Matrix4f T;
//            s->readBuffer(T.data(), 16 * sizeof(float32));
//            symmetryT[id].push_back(T);
//        }
//    }
//
//    // symmetryid
//    s->read<card32>(size);
//    for (unsigned i=0; i < size; i++)
//    {
//        card32 id;
//        s->read<card32>(id);
//        card32 subsize = 0;
//        s->read<card32>(subsize);
//        for (unsigned j=0; j < subsize; j++)
//        {
//            card32 k=0;
//            s->read<card32>(k);
//            symmetryId[id].push_back(k);
//        }
//    }
//
//
//}
//
//void PCISymmDet::DetectCCSGSymmtry(void)
//{
//    std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
//    if (selection.size() != 1) {
//        error("Please select only one cloud for testing");
//        return;
//    }
//    SGObjectNode* selnode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), selection[0]));
//    UnstructuredInCoreTriangleMesh* mesh = dynamic_cast<UnstructuredInCoreTriangleMesh*>(selnode->getSceneObject());
//    if (mesh == nullptr) {
//        error("Work only on tri meshes");
//        return;
//    }
//    //TrimeshStatic::Ptr smesh(new TrimeshStatic(mesh));
//    //UnstructuredInCoreTriangleMeshPtr pmesh(mesh);
//
//    FeatureSet* fset;
//    DVectorUObjectList* clusters;
//
//    // compute lines features
//    {
//        fset = detectFeatureLines(mesh);
//        fset->setup("FeatureLines", AttachedData::ADF_PERSISTENT);
//        mesh->getAttachments()->attachData(fset);
//    }
//
//    // cluster lines
//    {
//        clusters = clusterFeatureLines(dynamic_cast<FeatureSet*>(fset));
//        clusters->setup("LineClusters", AttachedData::ADF_PERSISTENT);
//        mesh->getAttachments()->attachData(clusters);
//    }
//
//    findPreliminarySymmetryGroups(selnode->getName(), mesh, fset, clusters);
//
//    SGListNode* root = dynamic_cast<SGListNode*>(getScene()->getRootNode());
//    SGObjectNode* swsnode = dynamic_cast<SGObjectNode*>(root->getChildNode(nullptr, root->getChildIndex(std::string("__sym_work_shape_") + selnode->getName())));
//    if (!swsnode)
//    {
//        error(std::string("No temporary processing node (") + std::string("__sym_work_shape_") + selnode->getName() + ") found. You have first to find preliminary groups");
//        return;
//    }
//    if (selnode->getParent(0))
//        selectSymmetryGroupName = selnode->getParent(0)->getName() + std::string("_") + selnode->getName() + "_groups";
//    else
//        selectSymmetryGroupName = "data_0_groups";
//
//    UnstructuredInCorePointCloud* shape = dynamic_cast<UnstructuredInCorePointCloud*>(swsnode->getSceneObject());
//
//    FeatureSet* _fs = dynamic_cast<FeatureSet*>(shape->getAttachments()->getData("FeatureLines"));
//    DVectorUObjectList* _clusters = dynamic_cast<DVectorUObjectList*>(shape->getAttachments()->getData("LineClusters"));
//    CCSGClassification* ccsg = dynamic_cast<CCSGClassification*>(shape->getAttachments()->getData(CCSGClassification::getDefaultName()));
//    InCorePCTopologyGraph* tpg = dynamic_cast<TopologyAttachment*>(shape->getAttachments()->getData(TopologyAttachment::getDefaultName()))->getTopology();
//
//    pair<UnstructuredInCorePointCloud*, std::vector<UnstructuredInCorePointCloud*>> result = extractFinalSymmetricGroups(shape, _fs, _clusters, ccsg, tpg);
//
//    SGRelativeTimeAnimationNode* listnode = new SGRelativeTimeAnimationNode;
//    listnode->setName(selectSymmetryGroupName);
//    {
//        SGListNode* sublist = new SGListNode;
//        sublist->setName("0");
//        listnode->addChildNode(sublist);
//
//
//        if (result.first)
//        {
//            SGObjectNode* nonsym = new SGObjectNode(result.first);
//            nonsym->setName("nonsym");
//            sublist->addChildNode(nonsym);
//        }
//
//        for(unsigned i=0; i < result.second.size(); i++)
//        {
//            SGObjectNode* sym = new SGObjectNode(result.second.at(i));
//            sym->setName(intToStr(i + (result.first ? 1 : 0)));
//            sublist->addChildNode(sym);
//        }
//    }
//    getScene()->getRootNode()->addChildNode(listnode);
//
//    //if (result.first) addPointCloud(getScene(), result.first, "nonsymmetric");
//
//    //for(unsigned i=0; i < result.second.size(); i++)
//    //	addPointCloud(getScene(), result.second.at(i), std::string("element_") + intToStr(i));
//}
//
//FeatureSet* PCISymmDet::detectFeatureLines(UnstructuredInCoreTriangleMesh* selcloud)
//{
//    using namespace NAMESPACE_VERSION::sym;
//
//    debugOutput << "detect feature lines ... "; debugOutput.flush();
//    X4_TIMER_START(lines);
//    FeatureSet* fs = nullptr;
//    {
//        if (symmDetContCCSG->bDrawFeatureLines) {
//            debugRenderer->clearDebugData("fslines");
//            debugRenderer->beginRenderJob("fslines");
//        }
//
//        CreaseLineFeatureDetector detector;
//        detector.setSpatialTolerance(symmDetContCCSG->spatialTolerance);
//        detector.setAngleTolerance(symmDetContCCSG->detectorAngleTolerance);
//        detector.setMinimalLength(symmDetContCCSG->minFeatureLineLength);
//        fs = detector.detect(&*selcloud);
//
//        if (symmDetContCCSG->bDrawFeatureLines) {
//            for (unsigned i=0; i < fs->m_Features.size(); i++) {
//                FeatureLine* line = dynamic_cast<FeatureLine*>(fs->m_Features[i]);
//                debugRenderer->addLine(
//                    line->m_Position, line->getEndPosition(),
//                    makeVector3f(0.8,0,0), makeVector3f(0.8,0,0),
//                    symmDetContCCSG->featureLineSize);
//                debugRenderer->addLine(
//                    line->getCenterPosition(), line->getCenterPosition() + line->m_Normal * 0.08f,
//                    makeVector3f(0.8,0,0), makeVector3f(0.8,0,0),
//                    symmDetContCCSG->featureLineSize);
//            }
//            debugRenderer->endRenderJob();
//        }
//    }
//    debugOutput << "done - " << fs->m_Features.size() << "\n";
//    X4_TIMER_STOP(lines);
//    return fs;
//}
//
//DVectorUObjectList* PCISymmDet::clusterFeatureLines(FeatureSet* fs)
//{
//    using namespace NAMESPACE_VERSION::sym;
//
//    debugOutput << "cluster feature lines ... "; debugOutput.flush();
//    X4_TIMER_START(lines);
//    CreaseLineFeatureDetector detector;
//    detector.setSpatialTolerance(symmDetContCCSG->spatialTolerance);
//    detector.setAngleTolerance(symmDetContCCSG->angleTolerance);
//    std::vector<std::vector<card32> > clusters = detector.clusterLines(fs, true);
//    debugOutput << "done\n";
//    X4_TIMER_STOP(lines);
//
//    card32 numClusters = 0;
//    DVectorUObjectList* list = new DVectorUObjectList();
//    for (unsigned i=0; i < clusters.size(); i++)
//    {
//        if (clusters[i].size() <= 3) continue;
//        numClusters++;
//        DVectorUObject* data = new DVectorUObject;
//        data->data.setDim(clusters[i].size());
//        for (unsigned j=0; j < clusters[i].size(); j++)
//            (*data)[j] = clusters[i][j];
//        list->addList(data);
//    }
//
//    debugOutput << "found " << numClusters << " clusters\n";
//    return list;
//
//}
//
//std::vector<CCSGSymmetryGroup*> PCISymmDet::findLatticeGroups(FeatureSet* fs, DVectorUObjectList* clusters, ProgressWindow* progress)
//{
//    debugOutput << "find lattice structures in line clusters\n";
//
//    std::vector<CCSGSymmetryGroup*> groupsResult;
//    {
//        sym::SymmetryGroupDetector symDetector;
//        symDetector.setAngleTolerance(symmDetContCCSG->angleTolerance);
//        symDetector.setSpatialTolerance(symmDetContCCSG->spatialTolerance);
//        symDetector.setMinNumRotations(symmDetContCCSG->minNumRotations);
//        symDetector.setSymmetryCoveredPercentage(symmDetContCCSG->symmetryCoveredPercentage);
//        symDetector.setLatticeMinGeneratorLength(symmDetContCCSG->latticeMinGeneratorLength);
//        symDetector.setGridAngleTolerance(symmDetContCCSG->gridAngleTolerance);
//
//        symDetector.setUseDebugOutput(true);
//
//        std::vector<sym::SymmetryGroupAttachment*> groups = symDetector.findLatticeGroups(fs, clusters, progress);
//        for (unsigned i=0; i < groups.size(); i++)
//            groupsResult.push_back(CCSGSymmetryGroup::buildFromSymmetryGroupAttachment(groups[i]));
//    }
//
//
//    debugOutput << "found " << groupsResult.size() << " lattice groups\n";
//
//    return groupsResult;
//}
//
////--------------------------------------------------------------------------------------
//std::vector<CCSGSymmetryGroup*> PCISymmDet::findOtherGroups(FeatureSet* fs, DVectorUObjectList* clusters, ProgressWindow* progress)
//{
//    std::vector<CCSGSymmetryGroup*> groupsResult;
//    {
//        sym::SymmetryGroupDetector symDetector;
//        symDetector.setAngleTolerance(symmDetContCCSG->angleTolerance);
//        symDetector.setSpatialTolerance(symmDetContCCSG->spatialTolerance);
//        symDetector.setMinNumRotations(symmDetContCCSG->minNumRotations);
//        symDetector.setSymmetryCoveredPercentage(symmDetContCCSG->symmetryCoveredPercentage);
//        symDetector.setUseDebugOutput(true);
//
//        std::vector<sym::SymmetryGroupAttachment*> groups =  symDetector.findOnePointGroups(fs, clusters, progress, symmDetContCCSG->bShowCluster);
//        for (unsigned i=0; i < groups.size(); i++)
//            groupsResult.push_back(CCSGSymmetryGroup::buildFromSymmetryGroupAttachment(groups[i]));
//    }
//
//#if 0
//    // DEBUG show groups
//    for (unsigned i=0; i < groupsResult.size() && showFoundGroups; i++)
//    {
//        CCSGSymmetryGroup* group = groupsResult[i];
//
//        sym::Reflection* ref = dynamic_cast<sym::Reflection*>(group->group);
//        sym::Rotation* rot = dynamic_cast<sym::Rotation*>(group->group);
//
//        debugRenderer->beginRenderJob_OneFrame(std::string("group_"), i);
//
//        if (ref)
//        {
//            BoundingBox3f bbox;
//            sym::ReflectionGenerator* gen = dynamic_cast<sym::ReflectionGenerator*>(ref->getGenerator());
//            for (unsigned si=0; si < group->lines.size(); si++)
//            {
//                sym::FeatureLine* line1 = dynamic_cast<sym::FeatureLine*>(fs->m_Features[group->lines[si]]);
//                debugRenderer->addLine(line1->getPosition(), line1->getEndPosition(),
//                    gen->getParameterCoordinate(line1->getPosition())[0] < 0 ? makeVector3f(1,0,1) : makeVector3f(0,1,1),
//                    gen->getParameterCoordinate(line1->getEndPosition())[0] < 0 ? makeVector3f(1,0,1) : makeVector3f(0,1,1),
//                    3.0f);
//                bbox.addPoint(line1->getPosition());
//                bbox.addPoint(line1->getEndPosition());
//            }
//            debugRenderer->addPlane(gen->getPlane(), bbox.getMaxSideLength(), makeVector3f(1,1,0));
//        }
//
//        if (rot)
//        {
//            sym::RotationGenerator* gen = dynamic_cast<sym::RotationGenerator*>(rot->getGenerator());
//            for (unsigned si=0; si < group->lines.size(); si++)
//            {
//                sym::FeatureLine* line3 = dynamic_cast<sym::FeatureLine*>(fs->m_Features[group->lines[si]]);
//                debugRenderer->addLine(line3->m_Position, line3->m_Position + line3->m_Length * line3->m_Direction, makeVector3f(1,0,1), makeVector3f(1,0,1), 3.0f);
//            }
//
//            debugRenderer->addLine(gen->getRotationAxisCenter(), gen->getRotationAxisCenter() + gen->getRotationAxis(), makeVector3f(1,0,0), makeVector3f(1,0,0), 3.05f);
//
//            sym::FeatureLine* line1 = dynamic_cast<sym::FeatureLine*>(fs->m_Features[group->lines[0]]);
//            (dynamic_cast<sym::RotationGenerator*>(rot->getGenerator()))->getRigidMotion()->draw(line1->getCenterPosition(), makeVector3f(1,0,0), 20);
//
//            //gen->rigidMotion->draw(line1->getCenterPosition(), makeVector3f(1,0,0), 20);
//        }
//
//        debugOutput << "[" << i << "] - " << " : "   << group->key << "\n";
//
//        debugRenderer->endRenderJob();
//    }
//#endif
//
//    return groupsResult;
//}
//
//pair<UnstructuredInCorePointCloud*,CCSGClassification*>PCISymmDet::findPreliminarySymmetryGroups(
//    const std::string& shapeName, UnstructuredInCoreTriangleMesh* selcloud,
//    FeatureSet* _fs, DVectorUObjectList* _clusters)
//{
//    CCSGClassification* ccsg = new CCSGClassification();
//    ccsg->setup(CCSGClassification::getDefaultName(), AttachedData::ADF_PERSISTENT);
//
//    srand(42);
//    srandom(42);
//
//    FeatureSet* fs = (FeatureSet*)_fs->copy();
//    DVectorUObjectList* clusters = (DVectorUObjectList*)_clusters->copy();
//
//    ProgressWindow* progress = getProgressWindow();
//
//    std::vector<CCSGSymmetryGroup*> groups;
//
//    // lattice groups
//    if (symmDetContCCSG->searchLattice)
//    {
//        debugOutput << "search lattice groups\n";
//        const std::vector<CCSGSymmetryGroup*> _groups = findLatticeGroups(fs, clusters, progress);
//        for (unsigned i=0; i < _groups.size(); i++)groups.push_back(_groups[i]);
//    }
//
//    // rotation, reflection groups
//    {
//        debugOutput << "look for other group types\n";
//        const std::vector<CCSGSymmetryGroup*> _groups = findOtherGroups(fs, clusters, progress);
//        for (unsigned i=0; i < _groups.size(); i++) groups.push_back(_groups[i]);
//    }
//
//    for (unsigned i=0; i < groups.size(); i++)
//    {
//        groups[i]->featureSet = fs;
//        groups[i]->group->setSymCoverage(symmDetContCCSG->symmetryCoveredPercentage);
//        groups[i]->group->setSpatialTolerance(symmDetContCCSG->spatialTolerance);
//        groups[i]->group->setAngleTolerance(symmDetContCCSG->angleTolerance);
//        groups[i]->id = i;
//        groups[i]->update();
//    }
//
//
//    debugOutput << "upsample shape for later element extraction and group validation\n";
//    UnstructuredInCorePointCloud* shape = nullptr;
//    InCorePCTopologyGraph* tpg = nullptr;
//    {
//        UnstructuredInCoreTriangleMesh* _shape = (UnstructuredInCoreTriangleMesh*)selcloud->copy();
//        _shape->calculateVertexNormals();
//        shape = PCCTriangleMeshSampler::sampleMeshPoisson(_shape, symmDetContCCSG->symMeshSubsampling, 0);
//        delete _shape;
//        checkAttribute(shape, "flags", 1, VAD::DATA_FORMAT_INT32);
//        shape->clearAttachments();
//
//        fs->setup("FeatureLines", AttachedData::ADF_PERSISTENT);
//        clusters->setup("LineClusters", AttachedData::ADF_PERSISTENT);
//        shape->getAttachments()->attachData(fs);
//        shape->getAttachments()->attachData(clusters);
//        shape->getAttachments()->attachData(ccsg);
//
//        PCCComputeTopology cmd;
//        cmd.setup(PCCComputeTopology::TOPTYPE_EPS, symmDetContCCSG->symMeshSubsampling * 2.5f);//symmDetContCCSG->spatialTolerance);
//        tpg = cmd.computeEpsKNNTopology(shape->getPointSet(), shape);
//
//        // HACK: reuse segmentation if provided
//        if (symmDetContCCSG->reuseSegmentation)
//        {
//            const PointSet* edges = tpg->getEdges();
//            AAT vindex = edges->getAAT("vindex");
//            AAT cidAAT = NULL_AAT;
//
//            InCorePCTopologyGraph* tpgnew = new InCorePCTopologyGraph;
//            tpgnew->clearAndSetup((VertexDescriptor*)tpg->getDescr()->copy());
//
//            if (shape->providesAttribute("cid"))
//                cidAAT = shape->getAAT("cid");
//
//            for (unsigned i=0; i < tpg->getNumEdges(); i++)
//            {
//                Vector2i vi = edges->get2i(i, vindex);
//                if (cidAAT == NULL_AAT || shape->getPointSet()->get2i(vi[0], cidAAT)[1] == shape->getPointSet()->get2i(vi[1], cidAAT)[1])
//                    tpgnew->addEdge(vi[0],vi[1]);
//            }
//            delete tpg;
//            tpg = tpgnew;
//        }
//
//        getScene()->getRootNode()->removeChildNode(std::string("__sym_work_shape_") + shapeName);
//        addPointCloud(scene, shape, std::string("__sym_work_shape_") + shapeName);
//
//        TopologyAttachment* tatt= new TopologyAttachment;
//        tatt->setup(TopologyAttachment::getDefaultName(), AttachedData::ADF_PERSISTENT);
//        tatt->setTopology(tpg);
//        tatt->setVisible(false);
//        shape->getAttachments()->attachData(tatt);
//
//        SGListNode* root = dynamic_cast<SGListNode*>(getScene()->getRootNode());
//        SGObjectNode* node = dynamic_cast<SGObjectNode*>(root->getChildNode(nullptr, root->getChildIndex(std::string("__sym_work_shape_") + shapeName)));
//        node->setVisible(false);
//    }
//
//
//    for (std::vector<CCSGSymmetryGroup*>::iterator it = groups.begin(); it != groups.end(); )
//    {
//        if (*it /*&& (*it)->group->getNumTransformations() == 6*/) it++;
//        else it = groups.erase(it);
//    }
//
//
//    for (unsigned i=0; i < groups.size(); i++)
//    {
//        debugOutput << groups[i]->key << "\n";
//        ccsg->addGroup(groups[i]);
//    }
//    debugOutput << "done, found " << groups.size() << "\n";
//
//    return pair<UnstructuredInCorePointCloud*, CCSGClassification*>(shape, ccsg);
//}
//
//std::vector<UnstructuredInCorePointCloud*> PCISymmDet::extractLatticeGroupElements(
//	std::vector<CCSGSymmetryGroup*>& groups, 
//	UnstructuredInCorePointCloud* shape, 
//	InCorePCTopologyGraph* tpg, 
//	FeatureSet* fs, 
//	DVectorUObjectList* clusters, 
//	std::vector<char>& assigned)
//{
//	ProgressWindow* progress = getProgressWindow();
//
//	std::vector<UnstructuredInCorePointCloud*> resultShapes;
//
//	HierarchicalKNNIterator hIt(shape, 32, nullptr);
//	hIt.setMaxDistanceToSeekPoint(symmDetContCCSG->spatialTolerance);
//
//	std::vector<int32> pointGroupMap(shape->getNumPoints(), -1);
//
//	AAT flagsAAT = shape->getAAT("flags");
//	AAT posAAT = shape->getAAT("position");
//
//	for (unsigned gc=0; gc < groups.size(); gc++)
//	{
//		CCSGSymmetryGroup*& group = groups[gc];
//		group->group->setSymCoverage(symmDetContCCSG->symmetryCoveredPercentage);
//		group->group->setSpatialTolerance(symmDetContCCSG->spatialTolerance);
//		group->group->setAngleTolerance(symmDetContCCSG->angleTolerance);
//
//		debugOutput << "\t[" << gc << "] - check group [" << group->id << "] " << group->key << "\n";	
//
//		// before we start detection of base elements, we need to check if the lines are already covered
//		// a coverage means that there was already region growing performed where the points of those lines
//		// were already assigned to another group. in case if the assignment happened for the same type of group
//		// then we don't have to extract elements again and can ignore these group of lines
//		bool alreadyCovered = false;
//		for (unsigned i=0; i < group->lines.size() && !alreadyCovered; i++)
//		{
//			sym::FeatureLine* line = dynamic_cast<sym::FeatureLine*>(fs->m_Features[group->lines[i]]);
//			if (!line) continue;
//			
//			// TODO: take care when taking center point. in case of triangulated shape, center point might not have a correct vertex in the near
//			hIt.setSeekPointAndReset(line->getCenterPosition());
//			while (!hIt.atEnd() && !alreadyCovered)
//			{
//				alreadyCovered = pointGroupMap[hIt.getCurrentPointIndex()] >= 0;
//				hIt.next();
//			}
//		}
//		if (alreadyCovered)
//		{
//			debugOutput << "\t[" << gc << "] - no need, already covered before\n";
//			delete group;
//			group = nullptr;
//			continue;
//		}
//
//		// detect generator and the rest
//		pair<std::vector<mpcard>, std::vector<mpcard>> result = group->group->detectBaseElements(shape, tpg, fs, group->lines, symmDetContCCSG->useICPWhileDetectingBaseElements);				
//		if (result.first.size() < symmDetContCCSG->minConnectedComponent) 
//		{
//			delete group;
//			group = nullptr;
//			continue;
//		}
//
//		// put found group points on a side for later check
//		std::vector<mpcard>& generatorPoints = group->symmetryPoints;
//		{
//			generatorPoints = result.first;
//			sort(generatorPoints.begin(), generatorPoints.end());
//			generatorPoints.resize(unique(generatorPoints.begin(), generatorPoints.end()) - generatorPoints.begin());
//		}
//
//		// HACK: removes the possibility to detect group within lattice elements
//		//result.second.insert(result.second.end(), generatorPoints.begin(), generatorPoints.end()); 
//		{
//			sort(result.second.begin(), result.second.end());
//			result.second.resize(unique(result.second.begin(), result.second.end()) - result.second.begin());
//		}
//
//		// mark all points found within the grid with the group id
//		for (unsigned i=0; i < result.second.size(); i++)
//		{
//			pointGroupMap[result.second[i]] = group->id;
//			assigned[result.second[i]]++;
//		}
//		/*for (unsigned i=0; i < generatorPoints.size(); i++)
//		{
//			pointGroupMap[generatorPoints[i]] = group->id;
//			assigned[generatorPoints[i]]++;
//		}*/
//
//		UnstructuredInCorePointCloud* redundantGeometry = new UnstructuredInCorePointCloud;
//		redundantGeometry->setPointSet(shape->getPointSet()->subset(result.second));
//		redundantGeometry->clearAttachments();
//
//		HierarchicalKNNIterator cIt(redundantGeometry, 32, nullptr);
//		cIt.setMaxDistanceToSeekPoint(symmDetContCCSG->spatialTolerance);
//
//		// mark all points within the symmetric region with invalid flag, so that they are not further processed
//		{
//			std::vector<char> marked(shape->getNumPoints(), 0);
//			for (unsigned i=0; i < result.second.size(); i++)
//			{
//				if (marked[result.second[i]]) continue;
//				PointCloudIterator* titer = tpg->createIterator(shape->getPointSet());
//				TopologicalKNNIterator_Fast* tIt = new TopologicalKNNIterator_Fast(shape, titer);
//				tIt->setStartPointAndReset(result.second[i]);
//				while(!tIt->atEnd())
//				{
//					marked[tIt->getPointSetPointNumber()] = 1;
//					tIt->next();
//				}
//				delete tIt;
//				delete titer;
//			}
//			for (unsigned i=0; i < shape->getNumPoints(); i++)
//			{
//				if (!marked[i]) continue;
//				cIt.setSeekPointAndReset(shape->getPointSet()->get3f(i,posAAT));
//				if (cIt.atEnd()) continue;
//				shape->getPointSet()->set1i(i, flagsAAT, PF_FLAG_INVALID);
//			}
//		}
//
//		for (unsigned i=0; i < generatorPoints.size(); i++)
//			shape->getPointSet()->set1i(generatorPoints[i], flagsAAT, 0);
//
//		// remove all feature lines, which are covered by the points in the symmetry (not by the generator)
//		for (unsigned cl=0; cl < clusters->getNumLists(); cl++)
//		{
//			//DVectorUObject* elems = dynamic_cast<DVectorUObject*>(clusters->getList(cl));
//
//			std::vector<unsigned> elems = dynamic_cast<DVectorUObject*>(clusters->getList(cl))->toStd();
//
//			for (std::vector<unsigned>::iterator it = elems.begin(); it != elems.end(); )
//			{
//				sym::FeatureLine* line = dynamic_cast<sym::FeatureLine*>(fs->m_Features[*it]);
//				if (!line)
//				{
//					it++;
//					continue;
//				}
//
//				// TODO: take care when taking center point. in case of triangulated shape, center point might not have a correct vertex in the near
//				cIt.setSeekPointAndReset(line->getCenterPosition());
//				if (cIt.atEnd())
//				{
//					it++;
//					continue;
//				}
//
//				delete line; 
//				fs->m_Features[*it] = nullptr;
//				it = elems.erase(it);
//			}
//			dynamic_cast<DVectorUObject*>(clusters->getList(cl))->fromStd(elems);
//
//		}
//		addPointCloud(scene, redundantGeometry, std::string("red_") + intToStr(group->id));
//		//delete redundantGeometry;
//
//	} // end for each lattice group
//
//	// extract all base elements
//	for (unsigned j=0; j < groups.size(); j++)
//	{
//		if (!groups[j]) continue;
//
//		// perform some clean-up on the found data (remove small unconnected components)
//		UnstructuredInCorePointCloud* pc = nullptr;
//		if (groups[j]->clearGeometrically(shape, symmDetContCCSG->minConnectedComponent, &pc, false))
//		{
//			delete groups[j];
//			groups[j] = nullptr;
//			continue;
//		}
//
//		CCSGSymmetryGroup* group = (CCSGSymmetryGroup*)groups[j]->copy();
//		group->setup("SymmetryGroup", AttachedData::ADF_PERSISTENT);
//		pc->getAttachments()->attachData(group);
//		resultShapes.push_back(pc);
//		debugOutput << "\tfound new grid [" << group->id <<"] " << group->key << "\n";
//	}		
//	debugOutput << "done\n";
//
//	return resultShapes;
//}
//
//std::vector<UnstructuredInCorePointCloud*> PCISymmDet::extractOtherGroupElements(
//	std::vector<CCSGSymmetryGroup*>& groups,
//	UnstructuredInCorePointCloud* shape, 
//	InCorePCTopologyGraph* tpg,
//	FeatureSet* fs, 
//	DVectorUObjectList* clusters, 
//	std::vector<char>& assigned)
//{
//	ProgressWindow* progress = getProgressWindow();
//
//	srand(42);
//	srandom(42);
//
// 	debugOutput << "analyze and consolidate groups\n";
//
//	{
//		sym::SymmetryGroupDetector symDetector;
//		symDetector.setAngleTolerance(symmDetContCCSG->angleTolerance);
//		symDetector.setSpatialTolerance(symmDetContCCSG->spatialTolerance);
//		symDetector.setMinNumRotations(symmDetContCCSG->minNumRotations);
//		symDetector.setSymmetryCoveredPercentage(symmDetContCCSG->symmetryCoveredPercentage);
//		symDetector.setLatticeMinGeneratorLength(symmDetContCCSG->latticeMinGeneratorLength);
//		symDetector.setGridAngleTolerance(symmDetContCCSG->gridAngleTolerance);
//		
//		symDetector._rotGroupCompareSigma = symmDetContCCSG->rotGroupCompareSigma;
//		symDetector._refGroupCompareSigma = symmDetContCCSG->refGroupCompareSigma;
//
//		symDetector._symGroupCompareThreshold = symmDetContCCSG->symGroupCompareThreshold;
//		symDetector._useICPWhileDetectingBaseElements = symmDetContCCSG->useICPWhileDetectingBaseElements;
//		symDetector._performTopologyCheckOnFoundGroup = symmDetContCCSG->performTopologyCheckOnFoundGroup;
//		symDetector._minConnectedComponent = symmDetContCCSG->minConnectedComponent;
//
//		symDetector.setUseDebugOutput(true);
//
//		std::vector<sym::SymmetryGroupAttachment*> _groups;
//		for (unsigned i=0; i < groups.size(); i++)
//		{
//			_groups.push_back(CCSGSymmetryGroup::buildSymmetryGroupAttachment(groups[i]));
//			delete groups[i];
//		}
//
//		symDetector.consolidateOnePointGroups(_groups, shape, tpg, fs, progress);
//		groups.clear();
//		for (unsigned i=0; i < _groups.size(); i++)
//		{
//			if (!_groups[i]) continue;
//			groups.push_back(CCSGSymmetryGroup::buildFromSymmetryGroupAttachment(_groups[i]));
//		}
//	}
//
//	debugOutput << "combine to complex groups and extract geometry\n";
//
//	// DEBUG: output preliminary identified groups
//	std::vector<CCSGSymmetryGroup*> resultGroups = groups;
//
//	//std::vector<CCSGSymmetryGroup*> resultGroups = combineFoundGroups(shape, groups);
//	groups.clear();
//
//	for (std::vector<CCSGSymmetryGroup*>::iterator it = resultGroups.begin(); it != resultGroups.end(); )
//	{
//		if (*it == nullptr) it = resultGroups.erase(it);
//		else it++;
//	}
//
//	struct SortPredicate
//	{
//		static bool sort(const CCSGSymmetryGroup* a, const CCSGSymmetryGroup* b)  { return a->symmetryPoints.size() > b->symmetryPoints.size(); }
//	};
//	sort(resultGroups.begin(), resultGroups.end(), SortPredicate::sort);
//
//	// extract all base elements
//	std::vector<UnstructuredInCorePointCloud*> resultShapes;
//	for (unsigned j=0; j < resultGroups.size(); j++)
//	{
//		UnstructuredInCorePointCloud* pc = nullptr;
//
//
//		//debugOutput << "final check of [" << resultGroups[j]->id << "]\n";
//		if (resultGroups[j]->clearGeometrically(shape, symmDetContCCSG->minConnectedComponent, &pc, true))
//		{
//			//debugOutput << "\tcleared geometrically\n";
//			delete resultGroups[j];
//			continue;
//		}
//		if (pc->getPointSet()->getBoundingBox().getMaxSideLength() < shape->getPointSet()->getBoundingBox().getMaxSideLength() * 0.1f)
//		{
//
//			//debugOutput << "\tsmall bounding box\n";
//			delete pc;
//			delete resultGroups[j];
//			continue;
//		}
//
//		HierarchicalKNNIterator hIt(pc, 32, nullptr);
//		hIt.setMaxDistanceToSeekPoint(symmDetContCCSG->spatialTolerance);
//		for (std::vector<card32>::iterator it = resultGroups[j]->allLinesCovered.begin(); it != resultGroups[j]->allLinesCovered.end(); )
//		{
//			int hits = 0;
//			sym::FeatureLine* line = dynamic_cast<sym::FeatureLine*>(fs->m_Features[*it]);
//			if (line)
//			{
//				hIt.setSeekPointAndReset(line->getCenterPosition());
//				if (!hIt.atEnd()) hits++;
//				hIt.setSeekPointAndReset(line->getEndPosition());
//				if (!hIt.atEnd()) hits++;
//				hIt.setSeekPointAndReset(line->getPosition());
//				if (!hIt.atEnd()) hits++;
//			}
//			if (hits != 3) it = resultGroups[j]->allLinesCovered.erase(it);
//			else it++;
//		}
//		for (std::vector<card32>::iterator it = resultGroups[j]->lines.begin(); it != resultGroups[j]->lines.end(); )
//		{
//			int hits = 0;
//			sym::FeatureLine* line = dynamic_cast<sym::FeatureLine*>(fs->m_Features[*it]);
//			if (line)
//			{
//				hIt.setSeekPointAndReset(line->getCenterPosition());
//				if (!hIt.atEnd()) hits++;
//				hIt.setSeekPointAndReset(line->getEndPosition());
//				if (!hIt.atEnd()) hits++;
//				hIt.setSeekPointAndReset(line->getPosition());
//				if (!hIt.atEnd()) hits++;
//			}
//			if (hits != 3) it = resultGroups[j]->lines.erase(it);
//			else it++;
//		}
//
//		// HACK - I think this might help to reduce clutter, we just argue that when combining symmetries line features
//		// do not survive, then the combination is just crap
//		/*if ((float)resultGroups[j]->lines.size() / (float)resultGroups[j]->group->getNumTransformations() < symmDetContCCSG->symmetryCoveredPercentage)
//		{
//			debugOutput << "remove group [" << resultGroups[j]->id << "] due to insufficient coverage of " << resultGroups[j]->lines.size() << " lines (" << resultGroups[j]->allLinesCovered.size() << ")\n";
//			delete pc;
//			delete resultGroups[j];
//			continue;
//		}*/
//
//		pc->clearAttachments();
//		resultGroups[j]->update();
//
//		debugOutput << "\tfound new symmetric element in group [" << resultGroups[j]->id <<"]" << resultGroups[j]->key << "\n";
//		for (unsigned k=0; k < resultGroups[j]->symmetryPoints.size(); k++)
//			assigned[resultGroups[j]->symmetryPoints[k]] ++;
//
//
//		CCSGSymmetryGroup* resgroup = (CCSGSymmetryGroup*)resultGroups[j]->copy();
//		resgroup->setup("SymmetryGroup", AttachedData::ADF_PERSISTENT);
//
//		pc->getAttachments()->attachData(resgroup);
//		resultShapes.push_back(pc);
//		groups.push_back(resultGroups[j]);
//	}		
//
//	debugOutput << "done\n";
//
//
//	return std::vector<UnstructuredInCorePointCloud*>(resultShapes);
//}
//
//pair<UnstructuredInCorePointCloud*, std::vector<UnstructuredInCorePointCloud*>>
//    PCISymmDet::extractFinalSymmetricGroups(
//    UnstructuredInCorePointCloud* shape,
//    FeatureSet* _fs,
//    DVectorUObjectList* _clusters,
//    CCSGClassification* ccsg,
//    InCorePCTopologyGraph* tpg
//    )
//{
//    const std::vector<CCSGSymmetryGroup*>& _groups = ccsg->m_Group;
//
//    // split groups into lattice and normal groups, because due to historical reasons we have two methods to extract main elements from there
//    // we create copies of each object with which we further work with
//    std::vector<CCSGSymmetryGroup*> latticeGroup;
//    std::vector<CCSGSymmetryGroup*> otherGroup;
//    for (unsigned i=0; i < ccsg->m_Group.size(); i++) 
//    {
//        if (dynamic_cast<sym::Lattice*>(ccsg->m_Group[i]->group)) latticeGroup.push_back((CCSGSymmetryGroup*)ccsg->m_Group[i]->copy());
//        else otherGroup.push_back((CCSGSymmetryGroup*)ccsg->m_Group[i]->copy());
//    }
//    FeatureSet* fs = (FeatureSet*)_fs->copy();
//    DVectorUObjectList* clusters = (DVectorUObjectList*)_clusters->copy();
//    std::vector<char> assigned(shape->getNumPoints(), 0);
//
//    checkAttribute(shape, "flags", 1, VAD::DATA_FORMAT_INT32);
//    AAT flagsAAT = shape->getAAT("flags");
//    for (unsigned i=0; i < shape->getNumPoints(); i++)
//        shape->getPointSet()->set1i(i, flagsAAT, 0);
//
//    std::vector<UnstructuredInCorePointCloud*> results = extractLatticeGroupElements(latticeGroup, shape, tpg, fs, clusters, assigned);
//    std::vector<UnstructuredInCorePointCloud*> resultsR = extractOtherGroupElements(otherGroup, shape, tpg, fs, clusters, assigned);
//    results.insert(results.end(), resultsR.begin(), resultsR.end());
//
//    // compute the non-symmetric part
//    std::vector<mpcard> nonsymmetric;
//    for (unsigned i=0; i < shape->getNumPoints(); i++)
//        if (!assigned[i]) nonsymmetric.push_back(i);
//    UnstructuredInCorePointCloud* nonsym = shape->createCopyWithNewPointSet(shape->getPointSet()->subset(nonsymmetric));
//
//
//    // cleanup the non-symmetric part
//    {
//        nonsym->clearAttachments();
//        PCCComputeTopology cmd;
//        cmd.setup(PCCComputeTopology::TOPTYPE_EPS, symmDetContCCSG->spatialTolerance);
//        InCorePCTopologyGraph* _tpg = cmd.computeEpsKNNTopology(nonsym->getPointSet(), nonsym);
//        removeSmallPatches(nonsym, &_tpg, symmDetContCCSG->minConnectedComponent);
//        delete _tpg;
//    }
//
//    if (nonsym->getNumPoints() < symmDetContCCSG->minConnectedComponent)
//    {
//        delete nonsym;
//        nonsym = nullptr;
//    }
//
//
//    for (unsigned i=0; i < latticeGroup.size(); i++) delete latticeGroup[i];
//    for (unsigned i=0; i < otherGroup.size(); i++) delete otherGroup[i];
//    delete fs;
//    delete clusters;
//
//    return pair<UnstructuredInCorePointCloud*, std::vector<UnstructuredInCorePointCloud*>>(nonsym, results);
//}

PCISymmDet::PCISymmDet(void) : symmDetCont (new SymmDetCont)//, symmDetContCCSG(new SymmDetContCCSG)
{
}

PCISymmDet::~PCISymmDet(void)
{
}

void PCISymmDet::DetectHiPrecSymmtry(void)
{
    std::vector<PointCloud*> pcs = getAllPointCloudsFromScene();
    if (pcs.size() == 0)
        return;
    UnstructuredInCoreTriangleMesh* mesh = dynamic_cast<UnstructuredInCoreTriangleMesh*>(pcs[0]);
    if (mesh == nullptr) {
        return;
    }
    TrimeshStatic::Ptr smesh(new TrimeshStatic(mesh));

    SymmDetHiPrec::Ptr symmDetHiPrec (new SymmDetHiPrec);
    symmDetCont->Clear();
    symmDetHiPrec->SetContext(symmDetCont);
    symmDetHiPrec->DetectSymmtry(smesh);

    //debugRenderer->clearDebugData();
    int sframe = 0;
    sframe = symmDetHiPrec->ShowFeatureLines(sframe);
    sframe = symmDetHiPrec->ShowCoPlanarity(sframe);
    sframe = symmDetHiPrec->ShowReflections(sframe, true);
    sframe = symmDetHiPrec->ShowRotations(sframe, true);
    sframe = symmDetHiPrec->ShowTranslations(sframe, true);
    sframe = symmDetHiPrec->ShowFeatureLinesCluster(sframe);
}
