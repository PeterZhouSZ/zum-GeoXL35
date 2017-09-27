#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "PCI/PCISymmDefield.h"
#include "SToolBox/SymmSampler.h"
#include "SToolBox/SymmLineSampler.h"
#include "SToolBox/DummySampler.h"
#include "SPSolver/HandleConstraints.h"
#include "Util/TrimeshUnorderedGrid.h"
#include "SPSolver/SymmSpaceBasisSolver.h"
#include "SPSolver/SymmSpaceFoldSolver.h"
#include "Util\numerical\CudaAdaptor.h"
//---------------------------------------------------------------------------
#include "SGRelativeTimeAnimationNode.h"
#include "PCCComputeTopology.h"
#include "TopologyAttachment.h"
#include "TopologicalKNNIterator.h"
#include "basics\DeformationTools.h"

#include "SymmGroup/CCSGSymmetryGroup.h"
#include "SymmetryGroupAttachment.h"
#include "SceneEditorWidget.h"
#include "SeparatorClassProperty.h"
#include "BinaryObjectStreams.h"
#include "FileDialogs.h"

#include "basics\ARSettings.h"
#include "basics\ARConstraints.h"
#include "sequenceMerging\operations\AREvaluateDeformation.h"

#include "PCCTriangleMeshSampler.h"
#include "SimpleAttachment.h"

#include <QMessageBox>

#include <iomanip>

namespace {
    int DR_FRAME = 0;
}

//---------------------------------------------------------------------------
IMPLEMENT_CLASS( PCISymmDefield ,0)
{
    BEGIN_CLASS_INIT( PCISymmDefield );

    ADD_SEPARATOR("Sampling");
    ADD_FLOAT32_PROP(sampleEps, 0);
    ADD_FLOAT32_PROP(gridResolution, 0);
    ADD_BOOLEAN_PROP(sampleOnLines, 0);
    ADD_FLOAT32_PROP(topRadiusMultiplier, 0);
    ADD_SEPARATOR("Symmetry");
    ADD_BOOLEAN_PROP(useNullSpace, 0);
    ADD_BOOLEAN_PROP(useSymmetry, 0);
    ADD_FLOAT32_PROP(symmetryWeight, 0);
    ADD_SEPARATOR("Topology");
    ADD_BOOLEAN_PROP(useSubdivision, 0);
    ADD_FLOAT32_PROP(interpoMultiplier, 0);
    ADD_BOOLEAN_PROP(seperateGroup, 0);
    ADD_BOOLEAN_PROP(useInertia, 0);
    ADD_SEPARATOR("Interactive");
    ADD_BOOLEAN_PROP(bRealtime, 0);
    ADD_CARD32_PROP(numIterations, 0);
    ADD_BOOLEAN_PROP(bShowGraph, 0);
    ADD_SEPARATOR("Constraints");
    ADD_FLOAT32_PROP(handleWeight, 0);
    ADD_FLOAT32_PROP(regularizerWeight, 0);
    ADD_BOOLEAN_PROP(useFixedRigion, 0);
    ADD_BOOLEAN_PROP(useColinearity, 0);
    ADD_FLOAT32_PROP(colinearityWeight, 0);
    ADD_BOOLEAN_PROP(useCoplanarity, 0);
    ADD_FLOAT32_PROP(coplanarityWeight, 0);
    ADD_CARD32_PROP(minPlaneSize, 0);
    ADD_SEPARATOR("Misc.");
    ADD_CARD32_PROP(startDRFrame, 0);

    ADD_SEPARATOR_METHOD("Symmetric Defield");
    ADD_NOARGS_METHOD(OrganizeFiles);
    ADD_NOARGS_METHOD(Initialize);
    ADD_NOARGS_METHOD(Deform);
    ADD_NOARGS_METHOD(Reset);
    ADD_NOARGS_METHOD(Rebuild);

    ADD_SEPARATOR_METHOD("Symmetric Defield");
    ADD_NOARGS_METHOD(ShowSampleGraph);
}
//---------------------------------------------------------------------------


PCISymmDefield::PCISymmDefield(void) :
    symmWorkNameBase("root/__sym_work_shape_"),
    symmGroupNameBase("root/__sym_groups_"),
    inputSymmName("root/__sym_sample"),
    outputSymmName("root/__sym_deformed"),
    resultName("root/result"),
    dataFolder("root/data"), // only used for extracting names of each part
    partFolder("root/part"), // for producing result mesh

    showMesh_(true),
    showSample_(true),
    showGizmo_(true),
    uiStep_(0),

    uiMode_(Handle3DGizmoHandler::CONTROL),
    controlMode_(Handle3DGizmo::INACTIVE),

    bIntialized_(false),
    bRealtime(true),
    bShowGraph(false),
    iteration_(0),
    numIterations(5),

    optimize_(true),
    solver_(nullptr),

    sampleEps(0.22), // 0.8;
    gridResolution(0.0001), // very fine resolution can also cause ambiguity problem, which makes local symmetry impossible
    sampleOnLines(false),
    topRadiusMultiplier(1),
    minClique(8),
    useSubdivision(true),
    interpoMultiplier(0.8),
    basePoissonResolution(0.003),

    seperateGroup(true),
    useInertia(false),

    useNullSpace(true),
    useSymmetry(false),

    useFixedRigion(false),
    useColinearity(false),
    useCoplanarity(false),
    regularizerWeight(1),
    symmetryWeight(10),
    handleWeight(100),
    colinearityWeight(5),
    coplanarityWeight(5),
    minPlaneSize(12),
    startDRFrame(1)
{
    sampleEpsReset_ = sampleEps;
    minCliqueReset_ = minClique;
    gridResolutionReset_ = gridResolution;
    topRadiusMultiplierReset_ = topRadiusMultiplier;
    sampleOnLinesReset_ = sampleOnLines;
    seperateGroupReset_ = seperateGroup;

    handleConstraints_.reset( new HandleConstraints(handleWeight) );
    symmetryConstraints_.reset( new SymmetryConstraints() );
    slideConstraints_.reset( new SlideConstraints() );

    handle3DList_ = boost::make_shared<Handle3DList>();
    handle3DListWidget_ = boost::make_shared<Handle3DListWidget>(handle3DList_);
    handle3DListWidget_->move(1200, 0);
    handle3DListWidget_->hide();
    handle3DSymList_ = boost::make_shared<Handle3DList>();
    handle3DSymListWidget_ = boost::make_shared<Handle3DListWidget>(handle3DSymList_);
    handle3DSymListWidget_->move(800, 0);
    handle3DSymListWidget_->hide();
    gizmoHandlerMap_[Handle3DGizmoHandler::CONTROL] = boost::make_shared<Handle3DGizmoHandlerControl>(this);
    gizmoHandlerMap_[Handle3DGizmoHandler::SELECT] = boost::make_shared<Handle3DGizmoHandlerSelect>(this);
    gizmoHandlerMap_[Handle3DGizmoHandler::SYMCON] = boost::make_shared<Handle3DGizmoHandlerSymCon>(this);
    gizmoHandlerMap_[Handle3DGizmoHandler::SYMSEL] = boost::make_shared<Handle3DGizmoHandlerSymSel>(this);

#ifdef USE_CUDA
    CudaAdaptor::Initialize();
#endif
}

PCISymmDefield::~PCISymmDefield(void)
{
#ifdef USE_CUDA
    CudaAdaptor::ShutDown();
#endif
}

//----------------------------------------------------------------------
UICPC* PCISymmDefield::RetrieveUICPC(
    std::string const& node, UICPC* reference)
{
    UICPC* uicpc = dynamic_cast<UICPC*>(getPointCloud(scene, node));

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if (!uicpc)
    {
        if (!reference) { return 0; }

        uicpc = dynamic_cast<UICPC*>(reference->copy());

        addPointCloud(scene, uicpc, node);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if (uicpc && uicpc->getAAT("position") == NULL_AAT)
    {
        debugOutput << "PCISymmDefield::RetrieveUICPC() - "
            "\"position\" attribute missing.\n";
        uicpc = 0;
    }

    //// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //if (uicpc && uicpc->getAAT("normal"  ) == NULL_AAT)
    //{
    //    debugOutput << "PCISymmDefield::RetrieveUICPC() - "
    //        "\"normal\" attribute missing.\n";
    //    uicpc = 0;
    //}

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    return uicpc;
}

void PCISymmDefield::OrganizeFiles(void)
{
    SGListNode* dataList = dynamic_cast<SGListNode*>(
        getSceneGraphNodeByName(scene, dataFolder));
    if (!dataList)
    {
        error("data list node not found, but created now. just drag that pc into it");
        dataList = new SGListNode();
        dataList->setName("data");
        scene->getRootNode()->addChildNode(dataList);
        return;
    }
    if (0 == dataList->getNumChildNodes(0))
    {
        error("no data pc found.");
        return;
    }
    SGListNode* partList = dynamic_cast<SGListNode*>(
        getSceneGraphNodeByName(scene, partFolder));
    if (!partList)
    {
        partList = dataList;
        partFolder = dataFolder;
    }
    if (0 == partList->getNumChildNodes(0))
    {
        error("no part pc found.");
        return;
    }
    if (dataList->getNumChildNodes(0) != partList->getNumChildNodes(0))
    {
        error("data & part number mis-match.");
        return;
    }

    const unsigned numParts = dataList->getNumChildNodes(0);
    for (unsigned ni = 0; ni < numParts; ++ni) {
        SGObjectNode* partNode = dynamic_cast<SGObjectNode*>(dataList->getChildNode(0, ni));
        UICTM* partNodeMesh = dynamic_cast<UICTM*>(partNode->getSceneObject());
        if (!partNodeMesh)
        {
            std::stringstream ss;
            ss << "PCISymmDefield::OrganizeFiles - part " << ni << " is not a UICTM";
            error(ss.str());
            return;
        }
        const std::string& partName = partNode->getName();
        const std::string& symmWorkName = symmWorkNameBase + partName;
        const std::string& groupFolder = symmGroupNameBase + partName;

        SceneGraphNode* workNode = getSceneGraphNodeByName(scene, symmWorkName);
        if (!workNode)
        {
            workNode = getSceneGraphNodeByName(scene, symmWorkNameBase + "_tmp_" + partName);
            if (!workNode) {
                error("Symmetry work points not found.");
                return;
            }
            workNode->setName("__sym_work_shape_" + partName);
        }

        SGRelativeTimeAnimationNode* groupList = dynamic_cast<SGRelativeTimeAnimationNode*>(
            getSceneGraphNodeByName(scene, groupFolder));
        if (!groupList)
        {
            //groupList = dynamic_cast<SGRelativeTimeAnimationNode*>(
            //    getSceneGraphNodeByName(scene, "root/root_" + partName + "_groups"));
            //if (!groupList) {
            //    groupList = dynamic_cast<SGRelativeTimeAnimationNode*>(
            //        getSceneGraphNodeByName(scene, "root/data_" + partName + "_groups"));
            //    if (!groupList) {
            groupList = new SGRelativeTimeAnimationNode;
            getScene()->getRootNode()->addChildNode(groupList);
            //    }
            //}
            groupList->setName("__sym_groups_" + partName);
        }
        groupList->setVisible(false);

        // move all the subgroups to the group root folder
        {
            SGRelativeTimeAnimationNode* newList = new SGRelativeTimeAnimationNode;
            for (mpcard gi = 0; gi < groupList->getNumChildNodes(0); ++gi)
            {
                SGObjectNode* topNode = dynamic_cast<SGObjectNode*>(groupList->getChildNode(0, gi));
                if (topNode)
                {
                    UICPC* symNodePC = dynamic_cast<UICPC*>(topNode->getSceneObject());
                    if (!symNodePC) continue;
                    AttachedData* att = symNodePC->getAttachments()->getData("SymmetryGroup");
                    if (!att) continue;

                    std::stringstream ss;
                    ss << "tmp_new_" << ni * 100 + gi;
                    topNode->setName(ss.str());
                    newList->addChildNode(topNode);
                }
                SGListNode* topList = dynamic_cast<SGListNode*>(groupList->getChildNode(0, gi));
            }
            groupList->clear();
            for (unsigned gi = 0; gi < newList->getNumChildNodes(0); ++gi)
            {
                SceneGraphNode* symNode = newList->getChildNode(0, gi);
                std::stringstream ss;
                ss << ni * 100 + gi;
                symNode->setName(ss.str());
                groupList->addChildNode(symNode);
            }
            delete newList;
        }
    }
}

int PCISymmDefield::InitializeSymmGroup()
{
    UICPC* outPoisson = new UnstructuredInCorePointCloud;
    outPoisson->clearAndSetup(0, true, false);
    checkAttribute( outPoisson, "group", 2, VAD::DATA_FORMAT_INT32 );
    outPoisson->setMaterialIndex(4);

    struct SymmetryGroupWithRank
    {
        SGObjectNode* node;
        sym::SymmetryGroupAttachment* att;
        int rank;
        unsigned num;
        bool operator<(const struct SymmetryGroupWithRank& rhs) const {
            //if (this->rank == rhs.rank)
            //    return this->num > rhs.num; // points from smaller group can be covered by larger one later
            //return this->rank > rhs.rank; // higher rank, higher priority
            if (this->num == rhs.num)
                return this->rank > rhs.rank; // points from smaller group can be covered by larger one later
            return this->num > rhs.num; // higher rank, higher priority
        }
    };

    unsigned num_symm_groups = 0;
    SGListNode* dataList = dynamic_cast<SGListNode*>(
        getSceneGraphNodeByName(scene, dataFolder));
    if (!dataList)
    {
        error("data list node not found, but created now. just drag that pc into it");
        dataList = new SGListNode();
        dataList->setName("data");
        scene->getRootNode()->addChildNode(dataList);
        return -1;
    }
    UICTM* firstNodeMesh = dynamic_cast<UICTM*>(
        dynamic_cast<SGObjectNode*>(dataList->getChildNode(0, 0))->getSceneObject());
    const unsigned numParts = dataList->getNumChildNodes(0);
    for (unsigned ni = 0; ni < numParts; ++ni) {
        SGObjectNode* partNode = dynamic_cast<SGObjectNode*>(dataList->getChildNode(0, ni));
        const std::string& partName = partNode->getName();
        const std::string& symmWorkName = symmWorkNameBase + partName;

        UICPC* workPoints = dynamic_cast<UICPC*>(getPointCloud(scene, symmWorkName));
        workPoints->setMaterialIndex(2);
        checkAttribute( workPoints, "color", 3, VAD::DATA_FORMAT_FLOAT32 );
        checkAttribute( workPoints, "group", 2, VAD::DATA_FORMAT_INT32 );
        PointSet& workPointsPS = *workPoints->getPointSet();
        const AAT& symGroupAAT = workPointsPS.getAAT("group");
        const AAT& colorAAT = workPointsPS.getAAT("color");
        //{
        //    //const int groupID = ni * 100;
        //    const int groupID = SymmSampler::UNTOUCHED;
        //    const Vector3f group_color = SymmSampler::MapGroupColor(groupID);
        //    for (unsigned pi = 0; pi < workPointsPS.getNumEntries(); ++pi) {
        //        workPointsPS.set2i(pi, symGroupAAT, groupID);
        //        workPointsPS.set3f(pi, colorAAT, group_color);
        //    }
        //}
        DefieldSymmVec* symmVecAtt = new DefieldSymmVec;
        std::deque<DefieldSymm*>& symmVec = symmVecAtt->symmvec_;

        SGRelativeTimeAnimationNode* groupList = dynamic_cast<SGRelativeTimeAnimationNode*>(
            getSceneGraphNodeByName(scene, symmGroupNameBase + partName));
        if (!groupList) {
            std::stringstream ss;
            ss << "PCISymmDefield::InitializeSymmGroup - group folder " << ni << " not found";
            error(ss.str());
            return -1;
        }

        std::deque<SymmetryGroupWithRank> SymmetryGroupWithRank2Sort;
        std::deque<std::string> oldNames;
        const size_t num_group =  groupList->getNumChildNodes(0);
        for (mpcard gi = 0; gi < num_group; ++gi)
        {
            SGObjectNode* symNode = dynamic_cast<SGObjectNode*>(groupList->getChildNode(0, gi));
            if (!symNode)
            {
                error("Sym not found");
                return -1;
            }
            UICPC* symNodePC = dynamic_cast<UICPC*>(symNode->getSceneObject());
            if (!symNodePC)
            {
                error("Pc not found");
                return -1;
            }
            oldNames.push_back(symNode->getName());
            if ("nonsym" == symNode->getName()) continue;
            symNodePC->setMaterialIndex(firstNodeMesh->getMaterialIndex());

            AttachedData* att = symNodePC->getAttachments()->getData("SymmetryGroup");
            sym::SymmetryGroupAttachment* symAtt = dynamic_cast<sym::SymmetryGroupAttachment*>(att);
            if (!symAtt)
            {
                CCSGSymmetryGroup* ccsgroup = dynamic_cast<CCSGSymmetryGroup*>(att);
                if (!ccsgroup) {
                    //error("Group not found");
                    continue; // delete incomplete groups automatically
                }
                symAtt = ccsgroup->buildSymmetryGroupAttachment(ccsgroup);
                symNodePC->getAttachments()->deleteData("SymmetryGroup");
                symAtt->setup("SymmetryGroup", AttachedData::ADF_PERSISTENT);
                symNodePC->getAttachments()->attachData(symAtt);
            }

            //{
            //    PointSet& symNodePS = *symNodePC->getPointSet();
            //    const AAT& colorAAT = symNodePS.getAAT("color");
            //    const Vector3f symmPointColor = makeVector3f(255, 189, 13);
            //    for (unsigned pi = 0; pi < symNodePS.getNumEntries(); ++pi) { // preprocessing
            //        workPointsPS.set3f(pi, colorAAT, symmPointColor);
            //    }
            //}

            SymmetryGroupWithRank sgwr;
            sgwr.node = symNode;
            sgwr.att = symAtt;
            sgwr.rank = symAtt->group->rank_;
            sgwr.num = symAtt->getNumsymmetryPointss();
            SymmetryGroupWithRank2Sort.push_back(sgwr);
        }
        sort(SymmetryGroupWithRank2Sort.begin(), SymmetryGroupWithRank2Sort.end());
        {
            SGRelativeTimeAnimationNode* newList = new SGRelativeTimeAnimationNode;
            for (mpcard gi = 0; gi < SymmetryGroupWithRank2Sort.size(); ++gi)
            {
                const SymmetryGroupWithRank& sgwr = SymmetryGroupWithRank2Sort[gi];
                sym::SymmetryGroupAttachment* symAtt = sgwr.att;
                sym::SymmetryGroup* symmGroup = symAtt->group;

                DefieldSymm* defSymm;
                if (0 > DefieldSymm::CreateFromSG(symmGroup, &defSymm)) { // now the symmetry groups are ordered by rank
                    //warning("symmetry group not created");
                    continue; // do not add problematic group
                };
                symmVec.push_back(defSymm);

                symAtt->color = defSymm->color_;
                std::stringstream ss;
                ss << "tmp_new_" << ni * 100 + gi;
                sgwr.node->setName(ss.str());
                newList->addChildNode(sgwr.node);

                const Vector2i groupID = makeVector2i(ni, gi);
                const Vector3f group_color = SymmSampler::MapGroupColor(groupID);
                const std::vector<mpcard>& symmetryPoints = symAtt->symmetryPoints;
                for (unsigned si = 0; si < symmetryPoints.size(); ++si) { // preprocessing
                    const mpcard& index = symmetryPoints[si];
                    workPointsPS.set2i(index, symGroupAAT, groupID);
                    workPointsPS.set3f(index, colorAAT, group_color);
                }
            }
            groupList->clear();
            for (unsigned gi = 0; gi < newList->getNumChildNodes(0); ++gi)
            {
                SceneGraphNode* symNode = newList->getChildNode(0, gi);
                std::stringstream ss;
                ss << ni * 100 + gi;
                symNode->setName(ss.str());
                groupList->addChildNode(symNode);
                ++num_symm_groups;
            }
            delete newList;
        }

        symmVecAtt->setup(DefieldSymmVec::getDefaultName(), AttachedData::ADF_PERSISTENT);
        workPoints->getAttachments()->attachData(symmVecAtt);
    }
    debugOutput << "number of symmetry groups: " << num_symm_groups << "\n\n";

    LineSymmBlockVec* lineBlockAtt = new LineSymmBlockVec;
    PointSymmBlockVec* pointBlockAtt = new PointSymmBlockVec;
    SymmSampleGraph* symSamGraphAtt = new SymmSampleGraph;

    lineBlockAtt->setup(LineSymmBlockVec::getDefaultName(), AttachedData::ADF_PERSISTENT);
    outPoisson->getAttachments()->attachData(lineBlockAtt);
    pointBlockAtt->setup(PointSymmBlockVec::getDefaultName(), AttachedData::ADF_PERSISTENT);
    outPoisson->getAttachments()->attachData(pointBlockAtt);
    symSamGraphAtt->setup(SymmSampleGraph::getDefaultName(), AttachedData::ADF_PERSISTENT);
    outPoisson->getAttachments()->attachData(symSamGraphAtt);
    addOrReplacePointCloud(scene, inputSymmName, outPoisson);

    return 0;
}

void BuildSymmSampleTpg(
    Scene* scene,
    const std::string& inputSymmName,
    const float& sampleth,
    const bool& seperateGroup
    )
{
    UICPC* samplePC = dynamic_cast<UICPC*>(
        getPointCloud(scene, inputSymmName));
    if (!samplePC)
    {
        error("Symmetry sampling points not found.");
        return;
    }
    const PointSet& samplePS = *samplePC->getPointSet();
    const AAT& samPosAAT = samplePS.getAAT("position", 3, VAD::DATA_FORMAT_FLOAT32);
    const AAT& symGroupAAT = samplePS.getAAT("group");

    TopologyAttachment* tatt= new TopologyAttachment;
    tatt->setup(TopologyAttachment::getDefaultName(), AttachedData::ADF_PERSISTENT);
    tatt->setVisible(true);

    // construct topology for symmetric samples
    PCCComputeTopology cmd;
    cmd.setup(PCCComputeTopology::TOPTYPE_EPS, sampleth);
    InCorePCTopologyGraph* tpgSymmSamples = cmd.computeEpsKNNTopology(samplePC->getPointSet(), samplePC);

    //if (false)
    if (seperateGroup)
    {
        // remove edges that connecting different clique (symmetry group)
        InCorePCTopologyGraph* tpgSymClique = new InCorePCTopologyGraph();
        VertexDescriptor vd;
        vd.pushAttrib( VAD("vindex", 2, VAD::DATA_FORMAT_INT32) );
        tpgSymClique->clearAndSetup(&vd);
        PointSet* tpgEdges = tpgSymmSamples->getEdges();
        const AAT& vindexAAT = tpgEdges->getAAT("vindex");
        for (mpcard ei = 0; ei < tpgEdges->getNumEntries(); ++ei) {
            Vector2i vind = tpgEdges->get2i(ei, vindexAAT);
            const int32& sym_v0 = SymmSampler::GroupCode(samplePS.get2i(vind[0], symGroupAAT));
            const int32& sym_v1 = SymmSampler::GroupCode(samplePS.get2i(vind[1], symGroupAAT));
            if (sym_v0 != sym_v1
                && SymmSampler::FILL_IN != sym_v0 && SymmSampler::FILL_IN != sym_v1
                ) continue;
            tpgSymClique->addEdge(vind[0], vind[1]);
        }
        tatt->setTopology(tpgSymClique);

        if (tpgSymClique->getNumVertices() != tpgSymmSamples->getNumVertices()) {
            std::stringstream ss;
            ss << "BuildSymmSampleTpg - topology reduced after seperating groups: "
                << tpgSymClique->getNumVertices() << " - " << tpgSymmSamples->getNumVertices();
            warning(ss.str());
        }

        delete tpgSymmSamples;
    } else {
        tatt->setTopology(tpgSymmSamples);
    }
    samplePC->getAttachments()->attachData(tatt);
}

void PCISymmDefield::Initialize(void)
{
    if (bIntialized_) return;

    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "############## Initialize ##############\n";
    debugOutput << "########################################\n";
    debugOutput << "\n";

    {
        SGListNode* dataList = dynamic_cast<SGListNode*>(
            getSceneGraphNodeByName(scene, dataFolder));
        if (!dataList)
        {
            error("data list node not found, but created now. just drag that pc into it");
            dataList = new SGListNode();
            dataList->setName("data");
            scene->getRootNode()->addChildNode(dataList);
            return;
        }
        if (0 == dataList->getNumChildNodes(0))
        {
            error("no data pc found.");
            return;
        }
        SGListNode* partList = dynamic_cast<SGListNode*>(
            getSceneGraphNodeByName(scene, partFolder));
        if (!partList)
        {
            partList = dataList;
            partFolder = dataFolder;
        }
        if (0 == partList->getNumChildNodes(0))
        {
            error("no part pc found.");
            return;
        }
        if (dataList->getNumChildNodes(0) != partList->getNumChildNodes(0))
        {
            error("data & part number mis-match.");
            return;
        }
        dataList->setVisible(false);
        partList->setVisible(false);
    }

    // symmetric sampling
    UICPC* samplePC = RetrieveUICPC(inputSymmName);
    if (!samplePC
        || abs(sampleEps - sampleEpsReset_) > std::numeric_limits<float>::epsilon()
        || minCliqueReset_ != minClique
        || abs(gridResolution - gridResolutionReset_) > std::numeric_limits<float>::epsilon()
        || abs(topRadiusMultiplier - topRadiusMultiplierReset_) > std::numeric_limits<float>::epsilon()
        || sampleOnLines != sampleOnLinesReset_
        || seperateGroupReset_ != seperateGroup
        )
    {
        if (0 != InitializeSymmGroup())
        {
            warning("file structure changed, better to save now");
            return;
        }

        UnorderedGridUniq pointgrid(gridResolution);

        if (sampleOnLines) SymmLineSampler::DoSample(
            scene, dataFolder, symmWorkNameBase, inputSymmName, symmGroupNameBase, &pointgrid, sampleEps);

        SymmSampler::DoSample(scene, dataFolder, symmWorkNameBase, inputSymmName, symmGroupNameBase, &pointgrid, sampleEps);

        //DummySampler::DoSample(scene, symmWorkName, inputSymmName, &pointgrid, sampleEps);

        //BuildSymmSampleTpg(scene, inputSymmName, topRadiusMultiplier * sampleEps + 1e-6, seperateGroup);

        samplePC = RetrieveUICPC(inputSymmName);
        if (!samplePC)
        {
            debugOutput << "PCISymmDefield::Initialize() - "
                "Null pointer, symmetric sampling failed.\n";
            return;
        }

        sampleEpsReset_ = sampleEps;
        minCliqueReset_ = minClique;
        gridResolutionReset_ = gridResolution;
        topRadiusMultiplierReset_ = topRadiusMultiplier;
        sampleOnLinesReset_ = sampleOnLines;
        seperateGroupReset_ = seperateGroup;
    }

    // make a copy of deformed PC
    UICPC* deformPC = RetrieveUICPC(outputSymmName, samplePC);
    if (!deformPC)
    {
        warning("PCISymmDefield::Initialize() - "
            "Output symmetric sampling node not found or created or attributes missing.");
    }
    deformPC->clearAttachments();
    {
        // build symmetry -> sample mapping
        symPointMap.clear();
        const PointSet& deformPS = *deformPC->getPointSet();
        const AAT& symGroupAAT = deformPS.getAAT("group");
        const unsigned& num_sample = deformPS.getNumEntries();
        for (unsigned si = 0; si < num_sample; ++si) {
            const Vector2i& groupCode = deformPS.get2i(si, symGroupAAT);
            key_type key = {groupCode[0], groupCode[1]};
            symPointMap[key].push_back(si);
        }
    }

    // generate result mesh from input data
    UICTM* resultTM = dynamic_cast<UICTM*>(RetrieveUICPC(resultName));
    if (resultTM) deletePointCloud(scene, resultName);
    {
        SGListNode* partList = dynamic_cast<SGListNode*>(
            getSceneGraphNodeByName(scene, partFolder));
        UICTM* firstNodeMesh = dynamic_cast<UICTM*>(
            dynamic_cast<SGObjectNode*>(partList->getChildNode(0, 0))->getSceneObject());
        resultTM = new UICTM;
        resultTM->clearAndSetup(firstNodeMesh->getDescr(), 0);
        checkAttribute( resultTM, "color", 3, VAD::DATA_FORMAT_FLOAT32 );
        checkAttribute( resultTM, "group", 2, VAD::DATA_FORMAT_INT32 );
        const AAT& posAAT = resultTM->getAAT("position");
        const unsigned numParts = partList->getNumChildNodes(0);
        for (unsigned ni = 0; ni < numParts; ++ni) {
            SGObjectNode* partNode = dynamic_cast<SGObjectNode*>(partList->getChildNode(0, ni));
            UICTM* partNodeMesh = dynamic_cast<UICTM*>(partNode->getSceneObject());
            const std::string& partName = partNode->getName();
            const std::string& symmWorkName = symmWorkNameBase + partName;

            UICPC* workPoints = dynamic_cast<UICPC*>(getPointCloud(scene, symmWorkName));
            PointSet& workPointsPS = *workPoints->getPointSet();
            const AAT& symGroupAAT = workPointsPS.getAAT("group");
            const AAT& colorAAT = workPointsPS.getAAT("color");
            const float poisson_resolution = getCachedMedianPointDistance(workPoints);
            workPoints->deleteDelOnWriteAttachments();

            PointSetANNQuery knn(workPoints->getPointSet(), 1);
            float distSqr;
            UICTM* meshCopy = dynamic_cast<UICTM*>(partNodeMesh->copy());
            checkAttribute( meshCopy, "color", 3, VAD::DATA_FORMAT_FLOAT32 );
            checkAttribute( meshCopy, "group", 2, VAD::DATA_FORMAT_INT32 );
            {
                PointSet& meshCopyPS = *meshCopy->getPointSet();
                const AAT& symGroupAAT = meshCopyPS.getAAT("group");
                for (unsigned vi = 0; vi < meshCopyPS.getNumEntries(); ++vi) {
                    const Vector3f& pos = meshCopyPS.get3f(vi, posAAT);
                    const int32& index = knn.getNearestPointIndexAndSqrDistance(pos, distSqr);
                    const Vector2i& groupID = workPointsPS.get2i(index, symGroupAAT);
                    meshCopyPS.set2i(vi, symGroupAAT, groupID);
                    meshCopyPS.set3f(vi, colorAAT, SymmSampler::MapGroupColor(groupID));
                }
            }
            resultTM->joinOtherMesh(meshCopy);
            delete meshCopy;
        }
        resultTM->setMaterialIndex(firstNodeMesh->getMaterialIndex());
        addOrReplacePointCloud(scene, resultName, resultTM);
        resultTM->clearAttachments();
    }

    //////////////////////////////////////////////////////////////////////////
    // initialize solver, and start building all constraints
    DebugRenderer::DR_FRAME = startDRFrame; // starting debug rendering frame

    solver_.reset( new LapSymmSpaceSolver() );

    //if (seperateGroup) useInertia = true; // prevent singular regularizer
    if (0 != solver_->Initialize(
        samplePC, deformPC, resultTM,
        useNullSpace,
        regularizerWeight, useInertia, topRadiusMultiplier * sampleEps + 1e-6,
        interpoMultiplier * sampleEps + 1e-6, seperateGroup
        )) return;

    if (useNullSpace || useSymmetry) symmetryConstraints_->BuildSymmetryConstraints(samplePC, solver_, useFixedRigion, symmetryWeight);

    if (useColinearity) slideConstraints_->BuildColinearity(samplePC);
    if (useColinearity) slideConstraints_->AddColinearity(solver_, colinearityWeight);

    if (useCoplanarity) slideConstraints_->BuildCoplanarity(samplePC, minPlaneSize);

    handle3DList_->setPC(deformPC);
    handle3DSymList_->setPC(deformPC);
    uiStep_ = 0; showGizmo_ = true;
    //////////////////////////////////////////////////////////////////////////

    {
        SGObjectNode* node = getSceneGraphObject(scene, inputSymmName);
        if (node) node->setVisible(false);
    }

    debugOutput << "Initialization done."
        << "\n  number of samples: " << samplePC->getNumPoints() << ";"
        << "\n  number of mesh vertices: " << resultTM->getNumPoints() << ";"
        << "\n  number of mesh triangles: " << resultTM->getNumTriangles() << "\n";

    bIntialized_ = true;
}

void PCISymmDefield::Deform(void)
{
    const bool debug_output = false;

    Initialize();
    if (!bIntialized_) return;
    DR_FRAME = DebugRenderer::DR_FRAME;

    UICPC* samplePC  = RetrieveUICPC(inputSymmName);
    if (!samplePC)
    {
        warning("PCIConstrainedICP::Deform() - "
            "Input node not found or attributes missing.");
        return;
    }
    const PointSet& samplePS = *samplePC->getPointSet();

    UICPC* deformPC = RetrieveUICPC(outputSymmName);
    UICTM* resultTM = dynamic_cast<UICTM*>(RetrieveUICPC(resultName));
    if (!deformPC || !resultTM)
    { 
        warning("PCIConstrainedICP::Deform() - "
            "Result node not found or created or attributes missing.");
        return;
    }
    PointSet& deformPS = *deformPC->getPointSet();
    AAT deformPosAAT = deformPS.getAAT("position", 3, VAD::DATA_FORMAT_FLOAT32);
    PointSet& resultPS = *resultTM->getPointSet();
    AAT resultPosAAT = resultPS.getAAT("position", 3, VAD::DATA_FORMAT_FLOAT32);

    Timer timer_def; timer_def.getDeltaValue();
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    for (card32 ii = 0; ii < numIterations; ++ii)
    {
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        if (optimize_ && debug_output)
        {
            debugOutput << "\n";
            debugOutput << "########################################\n";
            debugOutput << "# Deformation step: " << iteration_ << "\n";
            debugOutput << "########################################\n";

            ++iteration_;
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // preparing for all constraints
        solver_->beginConstraints();

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // handle constraints, for adding all positional constraints
        {
            handleConstraints_->BuildConstraints(solver_, handle3DList_);
            handleConstraints_->BuildConstraints(solver_, handle3DSymList_);
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // coplanarity constraints    
        if (useCoplanarity)
        {
            slideConstraints_->AddCoplanarity(solver_, coplanarityWeight);
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        if (!optimize_) { break; } //for debugging

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        solver_->solve();

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        //if (1 < numIterations)
        {
            solver_->Update(1 < numIterations);
            //solver_->Interpolate(resultTM);
            slideConstraints_->Update(solver_->getCurrentPositions()); // update current positions for estimating plane normal
            handle3DList_->Update(solver_->getCurrentPositions()); //  re-estimate current frame
            handle3DSymList_->Update(solver_->getCurrentPositions()); //  re-estimate current frame
        }

        DebugRenderer::DR_FRAME = DR_FRAME;
    }
    const float solve_time = (float)timer_def.getDeltaValue();
    const float solve_frate = 1000 / (solve_time);

    if (debug_output) debugOutput << "\n";

    // update deformed samples
    {
        const unsigned& num_sample = deformPS.getNumEntries();
        for (unsigned si = 0; si < num_sample; ++si) {
            deformPS.set3f(si, deformPosAAT, solver_->GetPosition(si));
        }
        deformPC->deleteDelOnWriteAttachments();
    }
    const float update_time = (float)timer_def.getDeltaValue();

    // interpolate result mesh
    solver_->Interpolate(resultTM);
    const float interpo_time = (float)timer_def.getDeltaValue();

    //{ // copy topology, just for visulization
    //    const InCorePCTopologyGraph* sample_tpy = getTopology(samplePC);
    //    InCorePCTopologyGraph* deform_tpg = sample_tpy->copy();
    //    TopologyAttachment* tatt= new TopologyAttachment;
    //    tatt->setup(TopologyAttachment::getDefaultName(), AttachedData::ADF_PERSISTENT);
    //    tatt->setTopology(deform_tpg);
    //    tatt->setVisible(true);
    //    UICPC* deformPC = RetrieveUICPC(outputSymmName);
    //    deformPC->getAttachments()->attachData(tatt);
    //}

    updateSceneView();

    if (debug_output) debugOutput << "\n";

    const float total_time = solve_time + update_time + interpo_time;
    const float render_time = std::max<float>(rtTime_ - total_time, 0.f);
    const float total_frate = 1000 / (rtTime_);

    std::ostringstream ss;
    ss.setf(std::ios::fixed, std::ios::floatfield);
    ss
        << "FR: " << std::setprecision(1) << total_frate << " <" << convertTimeToString( (card32)rtTime_ ) << ">; "
        << "timing: " << convertTimeToString( (card32)total_time ) << " <" << convertTimeToString( (card32)solve_time )
        //<< ", " << convertTimeToString( (card32)update_time )
        << ", " << convertTimeToString( (card32)interpo_time ) << ", " << convertTimeToString( (card32)render_time ) << ">"
        << "\n";
    debugOutput << ss.str();

    if (bShowGraph) ShowSampleGraph();
}

//----------------------------------------------------------------------
void PCISymmDefield::Reset()
{
    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "################ Reset #################\n";
    debugOutput << "########################################\n";
    debugOutput << "\n";

    iteration_ = 1;

    handle3DList_->clear();
    handle3DSymList_->clear();
    handle3DListWidget_->clear();
    handle3DSymListWidget_->clear();

    handle3DList_->setPC(nullptr);
    handle3DSymList_->setPC(nullptr);
    deletePointCloud(scene, outputSymmName);
    deletePointCloud(scene, resultName);

    bIntialized_ = false;

    Initialize();

    updateSceneView();
}

//----------------------------------------------------------------------
void PCISymmDefield::Rebuild(void)
{
    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "############### Rebuild ################\n";
    debugOutput << "########################################\n";
    debugOutput << "\n";

    iteration_ = 1;

    bIntialized_ = false;

    Initialize();

    if (bRealtime) Deform();

    updateSceneView();
}
