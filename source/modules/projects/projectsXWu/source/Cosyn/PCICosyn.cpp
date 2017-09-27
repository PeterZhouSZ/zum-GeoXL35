#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "PCICosyn.h"
#include "Util/NoUse.h"
#include "SingleScene.h"
#include "ObjectCollection.h"
#include "GraphicalModelProxy.h"
#include "Util/SceneAndPointCloudTools.h"
#include "Util\ColorSchemer.hpp"
#include "GridLabelAttachment.h"

//---------------------------------------------------------------------------
#include "CopyObjectProperties.h"
#include "Timer.h"
#include "ProgressWindow.h"
#include "PropertyTableProperty.h"
#include "SeparatorClassProperty.h"

#include "SceneEditorWidget.h"
#include "SceneGraphTools.h"
#include "SGListNode.h"
#include "SGRelativeTimeAnimationNode.h"
#include "MHSelectionIteratorAdaptor.h"

#include "MultiScaleInCorePointCloud.h"
#include "InCorePCTopologyGraph.h"
#include "PointCloudTopologyGraph.h"
#include "TopologyAttachment.h"
#include "LineFeature.h"
#include "LBase.h"
#include "LBaseSet.h"
#include "VertexArray.h"

//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

IMPLEMENT_CLASS( PCICosyn ,0)
{
    BEGIN_CLASS_INIT( PCICosyn );
    INIT_PROPERTY_TABLE();
    //ADD_STRING_PROP( data_source, 0 );
    ADD_CARD32_PROP( num_infer_trans, 0 );
    ADD_FLOAT32_PROP( cell_size, 0 );
    ADD_CARD32_PROP( verbose, 0 );

    ADD_NOARGS_METHOD( GenerateRandomRangeScans );
    ADD_NOARGS_METHOD( StitchScans );
    ADD_NOARGS_METHOD( InferPatches );

    ADD_SEPARATOR_METHOD("Debug");
    ADD_NOARGS_METHOD( ExtractCurrentResult );
    ADD_NOARGS_METHOD( VisualizeResult );
}

PCICosyn::PCICosyn(void)
{
    data_folder = "data";
    data_source = "pc1";
    musc_folder = "multiScale";
    feature_folder = "msFeatures";
    range_scan_folder = "scans";
    range_scan_prefix = "rrss_";
    result_folder = "result";
    result_prefix = "combined_pc_";
    range_scan_num = 10;
    num_infer_trans = 10;
    cell_size = 0.9f;
    verbose = 0;
}

PCICosyn::~PCICosyn(void)
{
}

void PCICosyn::assign(const Object* obj, COPY_CONTEXT *context)
{
    PCInteractionTool::assign(obj,context);
    const PCICosyn * o = dynamic_cast<const PCICosyn*>(obj);
    pAssert( o != nullptr );

    copyObjectProperties(obj,this);
}

void PCICosyn::GenerateRandomRangeScans(void)
{
    SGListNode *sourceLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), NU::PrefixRootName(data_folder)));
    if (!sourceLN) {
        sourceLN = new SGRelativeTimeAnimationNode();
        sourceLN->setName(data_folder);
        addSceneGraphNode(getScene(), "root", sourceLN);
    }
    PointCloud *sourcePC = getPointCloud(getScene(), NU::PrefixRootName(data_folder)+"/"+data_source);
    if (!sourcePC) {
        error("PCICosyn::GenerateRandomRangeScans() - range scan source not not found.");
        return;
    }
    SGListNode *scansLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), NU::PrefixRootName(range_scan_folder)));
    if (!scansLN) {
        scansLN = new SGRelativeTimeAnimationNode();
        scansLN->setName(range_scan_folder);
        addSceneGraphNode(getScene(), "root", scansLN);
    }

    std::string tmpnode = "tmp";
    UnstructuredInCorePointCloud *tmpPC = new UnstructuredInCorePointCloud();
    tmpPC->clearAndSetup(sourcePC->getDescr(), sourcePC->getNumPoints());
    addOrReplacePointCloud(getScene(), tmpnode, tmpPC);
    PointSet *tmpPS = tmpPC->getPointSet();
    AAT posAAT = tmpPS->getAAT("position");
    AAT normAAT = tmpPS->getAAT("normal");
    BasicPointCloudIterator *inIt = dynamic_cast<BasicPointCloudIterator*>(sourcePC->createIterator(SOIT::CAP_BASIC_PC));
    BasicPointCloudIterator *outIt = dynamic_cast<BasicPointCloudIterator*>(tmpPC->createIterator(SOIT::CAP_BASIC_PC | SOIT::CAP_MODIFYABLE_PC));
    ModifyablePointCloudIterator *outModIt = dynamic_cast<ModifyablePointCloudIterator*>(outIt);

    sourceLN->setVisible(false);
    for (unsigned ii = 0; ii < range_scan_num; ++ii) {
        float a0 = 360.f * float(rand()) / RAND_MAX;
        float a1 = 180.f * (float(rand()) / RAND_MAX - 0.5f);
        float a2 = 360.f * float(rand()) / RAND_MAX;
        float x = cos(a0 * M_PI / 180.f);
        float y = sin(a0 * M_PI / 180.f);
        float z = sin(a1 * M_PI / 180.f);
        Vector3f ax = normalize(makeVector3f(x, y, z));
        Matrix4f trans = makeRotVector4f(ax, a2);

        //sourcePC->setTransformation(trans);
        inIt->reset();
        outIt->reset();
        while (!inIt->atEnd()) {
            Vector3f p = inIt->get3f(posAAT);
            Vector3f p_t = transformVector3f(trans, p);
            outModIt->set3f(posAAT, p_t);
            outModIt->set3f(normAAT, inIt->get3f(normAAT));
            inIt->next();
            outIt->next();
        }

        std::ostringstream ss;
        ss << NU::PrefixRootName(range_scan_folder) << "/" << range_scan_prefix << ii;
        dynamic_cast<NAMESPACE_VERSION::SceneEditorWidget*>(sceneEditorWidget)->repaintGLArea();
        dynamic_cast<NAMESPACE_VERSION::SceneEditorWidget*>(sceneEditorWidget)->rangeScan(ss.str());

        Matrix4f trans_tr = trans.transpose();
        PointCloud *scannedPC = getPointCloud(getScene(), ss.str());
        BasicPointCloudIterator *scanIt = dynamic_cast<BasicPointCloudIterator*>(scannedPC->createIterator(SOIT::CAP_BASIC_PC | SOIT::CAP_MODIFYABLE_PC));
        ModifyablePointCloudIterator *scanModIt = dynamic_cast<ModifyablePointCloudIterator*>(scanIt);
        while (!scanIt->atEnd()) {
            Vector3f p = scanIt->get3f(posAAT);
            Vector3f p_t = transformVector3f(trans_tr, p);
            scanModIt->set3f(posAAT, p_t);
            scanIt->next();
        }
    }
    sourceLN->setVisible(true);
    for (unsigned ii = 0; ii < range_scan_num; ++ii) {
        std::ostringstream ss;
        ss << NU::PrefixRootName(range_scan_folder) << "/" << range_scan_prefix << ii;
        SceneGraphNode* scannedCloud = getSceneGraphNode(getScene(), ss.str());
        scannedCloud->setVisible(true);
    }

    delete inIt;
    delete outIt;
    getScene()->getRootNode()->removeChildNode(tmpnode);
}

void PCICosyn::StitchScans(void)
{
    SGListNode *scansLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), NU::PrefixRootName(range_scan_folder)));
    if (!scansLN) {
        error("PCICosyn::StitchScans() - range scans not not found.");
        return;
    }
    const mpcard numscan = scansLN->getNumChildNodes(nullptr);
    std::deque<UnstructuredInCorePointCloud*> upcvec;
    for (mpcard si = 0; si < numscan; ++si) {
        SGObjectNode *objN = dynamic_cast<SGObjectNode*>(scansLN->getChildNode(nullptr, si));
        if (!objN) continue;
        SceneObject *so = objN->getSceneObject();
        if (!so) continue;
        UnstructuredInCorePointCloud *pc = dynamic_cast<UnstructuredInCorePointCloud*>(so);
        upcvec.push_back(pc);
    }   
    ObjectCollection::Ptr objcoll(new ObjectCollection);
    typedef ObjectCollection::GridType GridType;
    GridType::Ptr grid(new GridType);
    objcoll->AddUPC(upcvec);
    objcoll->BuildGrid(grid);
    grid->DrawWithDR(makeVector3f(0.f, 1.f, 0.f));
}

void PCICosyn::InferPatches(void)
{
    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "# inference on patches \n";
    debugOutput << "########################################\n";
    debugOutput << "\n";

    //X4_TIMER_START(PCICosyn_InferPatches);

    SGListNode *featureLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), NU::PrefixRootName(feature_folder)));
    if (!featureLN) {
        error("feature folder not found (no list node)");
        return;
    }
    SGListNode *muscPCLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), NU::PrefixRootName(musc_folder)));
    if (!muscPCLN) {
        error("multi-scale data folder not found (no list node)");
        return;
    }
    const mpcard num_node = featureLN->getNumChildNodes(nullptr);
    if (num_node != muscPCLN->getNumChildNodes(nullptr)) {
        error("feature & data mis-match");
        return;
    }

    const size_t node_index = 0;
    SGObjectNode *featureObjN = dynamic_cast<SGObjectNode*>(featureLN->getChildNode(nullptr, node_index));
    if (!featureObjN) return;
    SceneObject *featureSObj = featureObjN->getSceneObject();
    if (!featureSObj) return;
    MultiScaleInCorePointCloud *featureMSPC = dynamic_cast<MultiScaleInCorePointCloud*>(featureSObj);
    SGObjectNode *muscObjN = dynamic_cast<SGObjectNode*>(muscPCLN->getChildNode(nullptr, node_index));
    if (!muscObjN) return;
    SceneObject *muscSObj = muscObjN->getSceneObject();
    if (!muscSObj) return;
    MultiScaleInCorePointCloud *muscMSPC = dynamic_cast<MultiScaleInCorePointCloud*>(muscSObj);
    if (!featureMSPC || !muscMSPC) {
        error("PCICosyn::InferPatches - data/feature missing.");
        return;
    }

    SGListNode * patchLN = NU::GetOrCreateSGListNode(getScene(), NU::PrefixRootName(range_scan_folder), true);
    patchLN->setVisible(false);
    PointSet *muscPS = muscMSPC->getLevel(0); // always work on the 1st level
    SingleScene::Ptr single_scene(new SingleScene(cell_size, (verbose > 0) ? true : false));
    single_scene_vec_.push_back(single_scene);
    single_scene->SetScanData(muscPS);

    std::deque<Matrix4f> transformations;
    std::deque<LBaseSetTranSet*> symmetries;
    for (unsigned level = 0; level < featureMSPC->getNumLevels(); ++level) {
        LBaseSet* lbs = dynamic_cast<LBaseSet*>(featureMSPC->getAttachedData(level, LBaseSet::getDefaultName()));
        const std::vector<LBaseSetTranSet*>& symm_vec = lbs->symmtranset;
        for (LBaseSetTranSet* symm : symm_vec) {
            symmetries.push_back(symm);
        }
        const std::vector<Matrix4f> trans_vec = lbs->getTransformationVector();
        for (Matrix4f T : trans_vec) {
            transformations.push_back(T);
        }
    }
    unsigned num_trans = std::min<unsigned>(transformations.size(), num_infer_trans);

    for (unsigned ti = 0; ti < num_trans; ++ti) {
        UnstructuredInCorePointCloud* copypc = new UnstructuredInCorePointCloud;
        std::ostringstream ss;
        ss << range_scan_prefix << ti;

        const std::vector<card32>& supp_list = symmetries[ti]->supp_list;
        if (supp_list.empty() || true) { // use whole geometry if support list is not present
            PointSet* queryps = (PointSet*)muscPS->copy();
            copypc->setPointSet( queryps );
            NU::TransformPointCloud(copypc, symmetries[ti]->T);
        } else {
            PointSet* queryps = muscPS->subset(supp_list);
            copypc->setPointSet( queryps );
            NU::TransformPointCloud(copypc, symmetries[ti]->T);
        }

        VertexArray points(copypc);
        const Vector3f color = ColorSchemer::GetColor(ti);
        const unsigned num_points = points.getNumElements();
        for (unsigned jj = 0; jj < num_points; ++jj) {
            points.setColor3f(jj, color);
        }

        SGObjectNode * objNode = new SGObjectNode;
        objNode->setName(ss.str());
        objNode->clearAndSetup(copypc);
        patchLN->addChildNode(objNode);

        single_scene->AddCandidate(copypc, ss.str());
    }
    debugOutput << str( boost::format("generated %1% candidates.\n")
        % num_trans
        );
    single_scene->BuildGrid();

    GraphModelPtr gr_model = single_scene->BuildGraphicalModel();
    //GraphicalModelProxy::Inference(gr_model);

    //X4_TIMER_STOP(PCICosyn_InferPatches);
}

void PCICosyn::ExtractCurrentResult(void)
{
    for (unsigned si = 0; si < single_scene_vec_.size(); ++si) {
        SingleScene::Ptr single_scene = single_scene_vec_[si];
        UnstructuredInCorePointCloud* result_upc = single_scene->ExtractCurrentPC();
        SGListNode * listnode = NU::GetOrCreateSGListNode(getScene(), NU::PrefixRootName(result_folder), false);
        listnode->setVisible(true);
        SGObjectNode * objNode = new SGObjectNode;
        objNode->setName(str( boost::format("%1%%2%") % result_prefix % (single_scene_vec_.size()-1) ));
        objNode->clearAndSetup(result_upc);
        listnode->addChildNode(objNode);

        GridLabelAttachment* latt= new GridLabelAttachment;
        latt->setup(GridLabelAttachment::getDefaultName(), AttachedData::ADF_PERSISTENT);
        SpaceType& gr_space = single_scene->gr_model_->space();
        const size_t num_var = gr_space.numberOfVariables();
        latt->labels.resize(num_var);
        latt->cell_cen.resize(num_var);
        latt->cell_size = single_scene->curr_grid_->cell_size;
        for (size_t variable = 0; variable < num_var; ++variable) {
            const LabelType label = gr_space(variable);
            const Vector3f ccen = single_scene->curr_grid_->GetCellCenter(variable);
            latt->labels[variable] = label;
            latt->cell_cen[variable] = ccen;
        }
        result_upc->getAttachments()->attachData(latt);
    }
}

void PCICosyn::VisualizeResult(void)
{
    if (!single_scene_vec_.empty()) {
        for (SingleScene::Ptr single_scene : single_scene_vec_) {
            single_scene->VisualizeGrid();
        }
        return;
    }

    SGListNode *resultLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), NU::PrefixRootName(result_folder)));
    if (!resultLN)
        return;
    mpcard num_node = resultLN->getNumChildNodes(nullptr);
    for (mpcard ni = 0; ni < num_node; ++ni) {
        SGObjectNode *objN = dynamic_cast<SGObjectNode*>(resultLN->getChildNode(nullptr, ni));
        if (!objN) continue;
        SceneObject *sceneObj = objN->getSceneObject();
        if (!sceneObj) continue;
        UnstructuredInCorePointCloud* result_upc = dynamic_cast<UnstructuredInCorePointCloud*>(sceneObj);
        GridLabelAttachment* latt = dynamic_cast<GridLabelAttachment*>(
            result_upc->getAttachments()->getData(GridLabelAttachment::getDefaultName()));
        if (!latt) continue;
        latt->VisualizeGrid();
    }

    SGListNode *scanLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), NU::PrefixRootName(range_scan_folder)));
    if (!scanLN)
        return;
    debugRenderer->beginRenderJob_OneFrame("candidate_patches_", DebugRenderer::DR_FRAME++);
    num_node = scanLN->getNumChildNodes(nullptr);
    for (mpcard ni = 0; ni < num_node; ++ni) {
        SGObjectNode *objN = dynamic_cast<SGObjectNode*>(scanLN->getChildNode(nullptr, ni));
        if (!objN) continue;
        SceneObject *sceneObj = objN->getSceneObject();
        if (!sceneObj) continue;
        UnstructuredInCorePointCloud* scan_pc = dynamic_cast<UnstructuredInCorePointCloud*>(sceneObj);

        AAT posAAT = scan_pc->getAAT( "position" );
        VertexArray points(scan_pc);
        const unsigned num_points = points.getNumElements();
        for (unsigned jj = 0; jj < num_points; ++jj) {
            const Vector3f& pos = points.getPosition3f(jj);
            debugRenderer->addPoint(pos, ColorSchemer::GetColor(ni));
        }
    }
    debugRenderer->endRenderJob();
}
