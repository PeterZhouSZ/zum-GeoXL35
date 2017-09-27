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
#include "Util\DampingICP.h"
#include "Util\DampingICL.h"
//---------------------------------------------------------------------------
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
#include "FastSphereQuerry.h"
#include "LineFeature.h"
#include "LBase.h"
#include "LBaseSet.h"
#include "PCCComputeTopology.h"
#include "TopologyRangeSearch.h"
#include "PCCBoundaryDetector.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

void cleanTransformations(std::vector<LBaseSetTranSet*>& symmetries, LBaseSet* lbs,
                          const float& bb_diag_length, const float& similarityEpsilon = 5e-2)
{
    const size_t num_mat = symmetries.size();
    std::deque< LBaseSetTranSet* > symm_unique;
    std::vector< bool > bmark(num_mat, false);
    for (size_t m1 = 0; m1 < num_mat; ++m1) {
        if (bmark[m1]) continue;
        const Matrix4f& T1 = symmetries[m1]->T;
        symm_unique.push_back(symmetries[m1]);
        bmark[m1] = true;
        for (size_t m2 = m1+1; m2 < num_mat; ++m2) {
            if (bmark[m2]) continue;
            const Matrix4f& T2 = symmetries[m2]->T;
            if (
                !NU::IsZeroMatrix(T1 - T2, similarityEpsilon, bb_diag_length, false)
                //&& !(NU::IsZeroMatrix((T1*T2)-IDENTITY4F, similarityEpsilon, bb_diag_length))
                ) continue;
            bmark[m2] = true;
            delete symmetries[m2]; // delete redandent transformations
        }
    }
    symmetries.clear();
    std::copy(symm_unique.begin(), symm_unique.end(), std::back_inserter(symmetries));

    std::vector<Matrix4f> transformations;
    transformations.reserve(symmetries.size());
    for (int l = 0; l < symmetries.size(); ++l) {
        transformations.push_back(symmetries[l]->T);
    }
    lbs->setTransformationVector(transformations);
}

void PCIMSFeatureAlign::execute() 
{
    typedef void(PCIMSFeatureAlign::*VoidFuncPtr)(void);
    FunctionContainer<VoidFuncPtr> func_list;
    //func_list.push_back(&PCIMSFeatureAlign::convert2MSPC);
    func_list.push_back(&PCIMSFeatureAlign::CoarseSegmentation);
    func_list.push_back(&PCIMSFeatureAlign::detectFeatures);
    //func_list.push_back(&PCIMSFeatureAlign::extractSupports);
    func_list.push_back(&PCIMSFeatureAlign::extractCrossPoints);
    func_list.push_back(&PCIMSFeatureAlign::computeDescriptors);
    func_list.push_back(&PCIMSFeatureAlign::computeCrossPointTpg);
    //func_list.push_back(&PCIMSFeatureAlign::alignLBase);
    //func_list.push_back(&PCIMSFeatureAlign::geometricValidation);

    const size_t num_func = func_list.size();
    progressWindow->pushStep(true, "execute");
    for (size_t fi = 0; fi < num_func; ++fi) {
        (this->*(func_list[fi]))();
        progressWindow->progressf((float)fi/(float)(num_func-1));
    }
    progressWindow->popStep();
}

void PCIMSFeatureAlign::convert2MSPC()
{
    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "# converting to multi-scale PC \n";
    debugOutput << "########################################\n";
    debugOutput << "\n";

    SGListNode *dataLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), dataFolder));
    if (!dataLN) {
        error("source folder not found (no list node)");
        return;
    }
    SGListNode *muscPCLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), muscFolder));
    if (!muscPCLN) {
        error("target folder not found (no list node)");
        return;
    }
    progressWindow->pushStep(true, "converting pc");
    const mpcard num_node = dataLN->getNumChildNodes(nullptr);
    for (mpcard ni = 0; ni < num_node; ++ni) {
        progressWindow->progressf((float)ni/(float)(num_node));
        SGObjectNode *dataObjN = dynamic_cast<SGObjectNode*>(dataLN->getChildNode(nullptr, ni));
        if (!dataObjN) continue;
        SceneObject *dataSO = dataObjN->getSceneObject();
        if (!dataSO) continue;

        PointCloud *pc = dynamic_cast<PointCloud*>(dataSO);
        MultiScaleInCorePointCloud *resultPC = multiScalePCCreator->convert(pc);
        if (!resultPC) continue;

        const bool est_top = false; // store huge topology for saving time in icp debug
        if (est_top) {
            PointSet *muscPS = resultPC->getLevel(0); // always work on the 1st level for icp alignment
            UnstructuredInCorePointCloud *muscPC = new UnstructuredInCorePointCloud();
            PointSet* tmpPS = (PointSet*)muscPS->copy();
            muscPC->setPointSet(tmpPS);
            const MultiScalePCParams* params = resultPC->getMultiScaleParameters();
            const float& base_grid_spacing = params->baseGridSpacing;
            const float top_radius = base_grid_spacing * settings_->outlier_dist_factor; // sqrt(2.f);
            PCCComputeTopology cmd;
            cmd.setup(PCCComputeTopology::TOPTYPE_EPS, top_radius);
            InCorePCTopologyGraph* tpg = cmd.computeEpsTopology(tmpPS, muscPC);
            TopologyAttachment* tatt= new TopologyAttachment;
            tatt->setup(TopologyAttachment::getDefaultName(), AttachedData::ADF_PERSISTENT);
            tatt->setTopology(tpg);
            tatt->setVisible(true);
            resultPC->attachData(tatt, 0);
            delete muscPC;
        }

        SGObjectNode *node = new SGObjectNode();
        node->clearAndSetup(resultPC);
        node->setName(dataObjN->getName());
        muscPCLN->addChildNode(node);
    }
    progressWindow->popStep();
}

void PCIMSFeatureAlign::CoarseSegmentation(void)
{
    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "# coarse segmentation \n";
    debugOutput << "########################################\n";
    debugOutput << "\n";
    //X4_TIMER_START(coarse_segmentation);

    SGListNode *muscPCLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), muscFolder));
    if (!muscPCLN) {
        error("source folder not found (no list node)");
        return;
    }
    progressWindow->pushStep(true, "coarse segmentation");
    const mpcard num_node = muscPCLN->getNumChildNodes(nullptr);
    for (mpcard ni = 0; ni < num_node; ++ni) {
        MultiScaleInCorePointCloud *muscMSPC = getMSPCFromListNode(muscPCLN, ni);
        if (!muscMSPC) {
            error("PCIMSFeatureAlign::detectFeatures - input missing.");
            return;
        }

        for (unsigned level = 0; level < muscMSPC->getNumLevels(); ++level) {
            PointSet *sourcePS = muscMSPC->getLevel(level);
            const size_t num_th = sourcePS->getNumEntries() * 0.005f;

            NormalClusterExtractor normal_cluster(sourcePS);
            //debugRenderer->beginRenderJob_OneFrame("normal_cluster_", DR_FRAME++);
            //normal_cluster.DrawWithDR();
            //debugRenderer->endRenderJob();
            if (normal_cluster.size() == 0) continue;

            const MultiScalePCParams* params = muscMSPC->getMultiScaleParameters();
            const float& top_spacing = params->baseGridSpacing * 6;
            debugOutput << str( boost::format(
                "clustering planes on the first normal cluster, with topological spacing: %1%.\n"
                )
                % top_spacing
                );
            PlaneClusterExtractor* plane_cluster = new PlaneClusterExtractor(sourcePS, normal_cluster[0], top_spacing, num_th);
            plane_cluster->setup(PlaneClusterExtractor::getDefaultName(), AttachedData::ADF_PERSISTENT);
            muscMSPC->attachData(plane_cluster, level);

            //debugRenderer->beginRenderJob_OneFrame("plane_cluster_", DR_FRAME++);
            //plane_cluster->DrawWithDR(sourcePS);
            //debugRenderer->endRenderJob();
        }
        progressWindow->progressf((float)ni/(float)(num_node-1));
    }
    progressWindow->popStep();

    //X4_TIMER_STOP(coarse_segmentation);
}

void PCIMSFeatureAlign::detectFeatures()
{
    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "# feature detection \n";
    debugOutput << "########################################\n";
    debugOutput << "\n";

    SGListNode *muscPCLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), muscFolder));
    if (!muscPCLN) {
        error("source folder not found (no list node)");
        return;
    }
    SGListNode *featureLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), featFolder));
    if (!featureLN) {
        error("target folder not found (no list node)");
        return;
    }
    progressWindow->pushStep(true, "detecting features");
    const mpcard num_node = muscPCLN->getNumChildNodes(nullptr);
    for (mpcard ni = 0; ni < num_node; ++ni) {
        MultiScaleInCorePointCloud *muscMSPC = getMSPCFromListNode(muscPCLN, ni);
        if (!muscMSPC) {
            error("PCIMSFeatureAlign::detectFeatures - input missing.");
            return;
        }
        SGObjectNode *objN = dynamic_cast<SGObjectNode*>(muscPCLN->getChildNode(nullptr, ni));

        PointCloud* featurePC = featureDetector->detectFeatures(muscMSPC);
        if (!featurePC) {
            error("PCIMSFeatureAlign::detectFeatures - input missing.");
            return;
        }
        SGObjectNode *node = new SGObjectNode();
        node->clearAndSetup( featurePC );
        node->setName(objN->getName());
        featureLN->addChildNode(node);
        progressWindow->progressf((float)ni/(float)(num_node-1));
    }
    progressWindow->popStep();
}

void PCIMSFeatureAlign::extractSupports()
{
    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "# extract supporting points \n";
    debugOutput << "########################################\n";
    debugOutput << "\n";

    SGListNode *muscPCLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), muscFolder));
    if (!muscPCLN) {
        error("source folder not found (no list node)");
        return;
    }
    SGListNode *featureLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), featFolder));
    if (!featureLN) {
        error("feature folder not found (no list node)");
        return;
    }
    const mpcard num_node = featureLN->getNumChildNodes(nullptr);
    for (mpcard ni = 0; ni < num_node; ++ni) {
        MultiScaleInCorePointCloud *muscMSPC = getMSPCFromListNode(muscPCLN, ni);
        if (!muscMSPC) {
            error("PCIMSFeatureAlign::extractSupports - input missing.");
            return;
        }
        MultiScaleInCorePointCloud *featureMSPC = getMSPCFromListNode(featureLN, ni);
        if (!featureMSPC) {
            error("PCIMSFeatureAlign::extractSupports - feature missing.");
            return;
        }

        PointSet *muscPS = muscMSPC->getLevel(0); // always work on the 1st level
        const float diag_length = muscPS->getBoundingBox().getDiagonalLength();
        const unsigned sl = std::min(minLevel, featureMSPC->getNumLevels()-1);
        const unsigned el = std::min(maxLevel, featureMSPC->getNumLevels());
        const unsigned numlevel = el - sl;
        for (unsigned level = sl; level < el; ++level) {
            PointSet *featurePS = featureMSPC->getLevel(level);
            C13DescriptorAttachment* att = dynamic_cast<C13DescriptorAttachment*>(
                muscMSPC->getAttachedData(level, C13DescriptorAttachment::getDefaultName()));
            mpcard numFeatures = featurePS->getNumEntries();
            const float radius = settings_->support_ratio * diag_length;

            UnstructuredInCorePointCloud *muscPC = new UnstructuredInCorePointCloud();
            muscPC->setPointSet((PointSet*)muscPS->copy());
            FastSphereQuerry *query = new FastSphereQuerry(muscPC);

            AAT positionAAT = featurePS->getAAT("position", 3, VAD::DATA_FORMAT_FLOAT32);
            att->featureSupports.resize(numFeatures);
            for (mpcard f = 0; f < numFeatures; ++f) {
                std::vector< card32 >& supp_inds = att->featureSupports[f];

                const Vector3f& pos = featurePS->get3f(f, positionAAT);
                mpcard numAllPoints;
                mpcard *allIndices;
                query->querry(pos, radius, &allIndices, numAllPoints);
                supp_inds.resize(numAllPoints);
                for (size_t i = 0; i < numAllPoints; ++i) {
                    supp_inds[i] = allIndices[i];
                }
            }

            delete muscPC;
            delete query;

            debugOutput << str( boost::format("On level %1%: extracted supports for %2% feature points.\n")
                % level % numFeatures
                );
        }
    }
}

void PCIMSFeatureAlign::extractCrossPoints()
{
    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "# extract cross points \n";
    debugOutput << "########################################\n";
    debugOutput << "\n";

    SGListNode *muscPCLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), muscFolder));
    if (!muscPCLN) {
        error("source folder not found (no list node)");
        return;
    }
    SGListNode *featureLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), featFolder));
    if (!featureLN) {
        error("feature folder not found (no list node)");
        return;
    }
    SGListNode *crossPtLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), crosFolder));
    if (!crossPtLN) {
        error("feature folder not found (no list node)");
        return;
    }

    const mpcard num_node = featureLN->getNumChildNodes(nullptr);
    for (mpcard ni = 0; ni < num_node; ++ni) {
        MultiScaleInCorePointCloud *muscMSPC = getMSPCFromListNode(muscPCLN, ni);
        if (!muscMSPC) {
            error("PCIMSFeatureAlign::extractCrossPoints - input missing.");
            return;
        }
        MultiScaleInCorePointCloud *featureMSPC = getMSPCFromListNode(featureLN, ni);
        if (!featureMSPC) {
            error("PCIMSFeatureAlign::extractCrossPoints - feature missing.");
            return;
        }
        SGObjectNode *featureObjN = dynamic_cast<SGObjectNode*>(featureLN->getChildNode(nullptr, ni));

        VertexDescriptor vd;
        vd.pushAttrib( VAD("position", 3, VAD::DATA_FORMAT_FLOAT32) );
        vd.pushAttrib( VAD("scale",    1, VAD::DATA_FORMAT_FLOAT32) );
        vd.pushAttrib( VAD("level",    1, VAD::DATA_FORMAT_CARD8) );
        MultiScaleInCorePointCloud *targetMSPC = new MultiScaleInCorePointCloud();
        targetMSPC->clearAndSetup(&vd);
        std::deque<PointSet*> psvec;

        for (unsigned level = 0; level < featureMSPC->getNumLevels(); ++level) {
            PointSet *featurePS = featureMSPC->getLevel(level);
            LBaseSet* lbs = dynamic_cast<LBaseSet*>(featureMSPC->getAttachedData(level, LBaseSet::getDefaultName()));
            std::vector<LBase*> lbasevec = lbs->getBaseVector();
            C13DescriptorAttachment* att = dynamic_cast<C13DescriptorAttachment*>(
                muscMSPC->getAttachedData(level, C13DescriptorAttachment::getDefaultName()));

            UnstructuredInCorePointCloud *tmpPC = new UnstructuredInCorePointCloud;
            tmpPC->clearAndSetup(&vd, lbasevec.size());
            PointSet *tmpPS = tmpPC->getPointSet();
            AAT posAAT = tmpPC->getAAT("position");
            AAT scaleAAT = tmpPC->getAAT("scale");
            AAT levelAAT = tmpPC->getAAT("level");

            for (unsigned li = 0; li < lbasevec.size(); ++li) {
                tmpPS->set3f(li, posAAT, lbasevec[li]->getBasePoint());
                tmpPS->set1f(li, scaleAAT, att->scale);
                tmpPS->set1ub(li, levelAAT, att->level);
            }
            psvec.push_back(tmpPS);
            debugOutput << "On level " << level << ": " << tmpPS->getNumEntries() << " intersections\n";
        }
        MultiScalePCParams *params = (MultiScalePCParams*)featureMSPC->getMultiScaleParameters()->copy();
        targetMSPC->build(params, psvec);
        delete params;

        SGObjectNode *node = new SGObjectNode();
        node->clearAndSetup(targetMSPC);
        node->setName(featureObjN->getName());
        crossPtLN->addChildNode(node);
    }
}

void PCIMSFeatureAlign::computeDescriptors()
{
    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "# compute descriptors for cross points \n";
    debugOutput << "########################################\n";
    debugOutput << "\n";

    SGListNode *muscPCLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), muscFolder));
    if (!muscPCLN) {
        error("source folder not found (no list node)");
        return;
    }
    SGListNode *featureLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), crosFolder));
    if (!featureLN) { // note: only on cross points, which might not be in the feature point set
        error("target folder not found (no list node)");
        return;
    }
    progressWindow->pushStep(true, "computing descriptiors");
    const mpcard num_node = muscPCLN->getNumChildNodes(nullptr);
    for (mpcard ni=0; ni<num_node; ni++) {
        MultiScaleInCorePointCloud *muscMSPC = getMSPCFromListNode(muscPCLN, ni);
        if (!muscMSPC) {
            error("PCIMSFeatureAlign::computeDescriptors - input missing.");
            return;
        }
        MultiScaleInCorePointCloud *featureMSPC = getMSPCFromListNode(featureLN, ni);
        if (!featureMSPC) {
            error("PCIMSFeatureAlign::computeDescriptors - feature missing.");
            return;
        }

        featureDescriptor->computeDescriptors(muscMSPC, featureMSPC);
        progressWindow->progressf((float)ni/(float)(num_node-1));
    }
    progressWindow->popStep();
}

InCorePCTopologyGraph* PCIMSFeatureAlign::buildKNN(PointSet* ups, const card32& nnnum)
{
    AAT positionAAT = ups->getAAT("position");
    AAT descriptorsAAT = ups->getAAT("descriptors");
    const mpcard numPts = ups->getNumEntries();
    card32 dim = ups->getDescr()->getAttribute("descriptors").numComponents;
    std::vector<float> descs;
    descs.resize(numPts * dim);
    std::vector<float*> descs_p;
    descs_p.resize(numPts);
    for (mpcard i = 0; i < numPts; ++i) {
        //Vector3f position = ups->get3f(i, positionAAT);
        card8 *ptr = (card8*)ups->getDataPointer(i);
        ptr += descriptorsAAT.getOffset();
        float32 *fptr = (float32*)ptr;
        for (mpcard j = 0; j < dim; ++j) {
            descs[i*dim+j] = fptr[j];
        }
        descs_p[i] = &descs[i*dim];
    }

    //debugRenderer->beginRenderJob_OneFrame("feature_match_knn_", DR_FRAME++);
    InCorePCTopologyGraph *tpg = new InCorePCTopologyGraph();
    VertexDescriptor vd;
    vd.pushAttrib(VAD("vindex", 2, VAD::DATA_FORMAT_INT32));
    vd.pushAttrib(VAD("color", 3, VAD::DATA_FORMAT_FLOAT32));
    tpg->clearAndSetup(&vd);
    AnnSearch anns(0.01f);
    anns.buildSearchStructure(numPts, dim, &descs_p[0]);
    std::vector<float> nndist;
    nndist.resize(nnnum);
    std::vector<int> nnindx;
    nnindx.resize(nnnum);
    for (mpcard i = 0; i < numPts; ++i) {
        Vector3f position = ups->get3f(i, positionAAT);
        anns.getkNearestNeighbors(descs_p[i], nnnum, &nndist[0], &nnindx[0]);
        for (mpcard j = 0; j < nnnum; ++j) {
            if (nnindx[j] == -1) continue;
            Vector3f target = ups->get3f(nnindx[j], positionAAT);
            //debugRenderer->addLine(
            //    position,
            //    target,
            //    makeVector3f(0.5f, 0, 0), makeVector3f(0.5f, 0, 0),
            //    3.f
            //    );
            tpg->addEdge(i, nnindx[j]);
        }
    }
    //debugRenderer->endRenderJob();

    return tpg;
}

void PCIMSFeatureAlign::computeCrossPointTpg()
{
    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "# compute topology for cross points \n";
    debugOutput << "########################################\n";
    debugOutput << "\n";

    bool debugshow = false;
    SGListNode *featureLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), crosFolder));
    if (!featureLN) {
        error("feature folder not found (no list node)");
        return;
    }
    const mpcard num_node = featureLN->getNumChildNodes(nullptr);
    for (mpcard ni = 0; ni < num_node; ++ni) {
        MultiScaleInCorePointCloud *featureMSPC = getMSPCFromListNode(featureLN, ni);
        if (!featureMSPC) {
            error("PCIMSFeatureAlign::computeCrossPointTpg - feature missing.");
            return;
        }

        for (unsigned level = 0; level < featureMSPC->getNumLevels(); ++level) {
            PointSet *featurePS = featureMSPC->getLevel(level);
            if (featurePS->getNumEntries() < 16) // FIX later
                continue;
            card32 nnum = std::min<unsigned>(featurePS->getNumEntries()>>2, settings_->knnTopology);
            InCorePCTopologyGraph* tpg = buildKNN(featurePS, nnum);
            TopologyAttachment* tatt= new TopologyAttachment;
            tatt->setup(TopologyAttachment::getDefaultName(), AttachedData::ADF_PERSISTENT);
            tatt->setTopology(tpg);
            tatt->setVisible(true);
            featureMSPC->attachData(tatt, level);

            debugOutput << str( boost::format("On level %1%: computed cross point topology with %2% edges.\n")
                % level % tpg->getNumEdges()
                );

            if (!debugshow)
                continue;

            //TopologyAttachment *topAtt = dynamic_cast<TopologyAttachment*>(featureMSPC->getAttachedData(level, TopologyAttachment::getDefaultName()));
            //if (!topAtt) continue;
            //InCorePCTopologyGraph* tpg = topAtt->getTopology();

            debugRenderer->beginRenderJob_OneFrame("feature_match_validate_", DR_FRAME++);
            AAT featurePosAAT = featurePS->getAAT("position");
            PointSet* edges = tpg->getEdges();
            AAT VIND = edges->getAAT("vindex");
            for (mpcard ei = 0; ei < tpg->getNumEdges(); ++ei)
            {
                Vector2i edgeInd = edges->get2i(ei, VIND);
                const float i0 = edgeInd[0], i1 = edgeInd[1];
                Vector3f pos0 = featurePS->get3f(i0, featurePosAAT);
                Vector3f pos1 = featurePS->get3f(i1, featurePosAAT);
                debugRenderer->addLine(
                    pos0, pos1,
                    makeVector3f(0.5f, 0, 0), makeVector3f(0.5f, 0, 0),
                    3.f);
            }
            debugRenderer->endRenderJob();

            for (unsigned pi = 0; pi < featurePS->getNumEntries(); ++pi) {
                debugRenderer->beginRenderJob_OneFrame("feature_match_validate_", DR_FRAME++);
                Vector3f pos0 = featurePS->get3f(pi, featurePosAAT);
                const unsigned numNV = tpg->getNumAdjacentVertices(pi);
                for (unsigned ni = 0; ni < numNV; ++ni) {
                    const unsigned k = tpg->getAdjacentVertex(pi, ni);
                    Vector3f pos1 = featurePS->get3f(k, featurePosAAT);
                    debugRenderer->addLine(
                        pos0, pos1,
                        makeVector3f(0.5f, 0, 0), makeVector3f(0.5f, 0, 0),
                        3.f);
                }
                debugRenderer->endRenderJob();
            }
        }
    }
}

float refineAndGrow(DampingICL* icl, LBase* base1, LBase* base2,
                    Matrix4f& trans, LineFeatureSet* lfs, FastSphereQuerry* query, float searchRadius,
                    std::vector<unsigned>& resultIndices)
{
    trans = base1->computeTransformation(base2);
    const unsigned base1Index1 = base1->getIndex1();
    const unsigned base1Index2 = base1->getIndex2();

    // get initial base cluster
    std::vector<LineFeature*> subset;
    const int base1ClusterIndex1 = lfs->m_ClusterID[base1Index1];
    const int base1ClusterIndex2 = lfs->m_ClusterID[base1Index2];

    // start with the entire section of the same cluster id
    for (int index = 0; index < lfs->m_LineFeatures.size(); index++){
        if (lfs->m_ClusterID[index] == base1ClusterIndex1 || lfs->m_ClusterID[index] == base1ClusterIndex2)
            subset.push_back(lfs->m_LineFeatures[index]);
    }

    // initial alignment
    float score = 0;
    bool success = icl->align(subset, trans, score);
    if (!success)
        return 0;

    float last_score = 0;
    Matrix4f curr_trans = trans;
    std::vector<unsigned> visited(lfs->m_LineFeatures.size(), 0);
    unsigned currentVisitedID = 0;
    for (unsigned growStep = 0; /*growStep < 4*/; ++growStep) { // grow until reaching maximal
        ++currentVisitedID; // prevent re-visit while stack traversal in the same step
        std::deque<unsigned> stack;

        stack.push_back(base1Index1);
        stack.push_back(base1Index2);

        visited[base1Index1] = currentVisitedID;
        visited[base1Index2] = currentVisitedID;

        std::vector<LineFeature*> growedSubset;
        std::vector<unsigned> growedIndices;
        while (!stack.empty()) {
            unsigned currentIndex = stack.back();
            stack.pop_back();

            LineFeature * lf = lfs->m_LineFeatures[currentIndex];
            growedSubset.push_back(lf);
            growedIndices.push_back(currentIndex);

            Vector3f cPos = lf->m_Position;
            mpcard numIndices;
            mpcard* indices;
            query->querry(cPos, searchRadius, &indices, numIndices);
            for (int nnii = 0; nnii < numIndices; ++nnii) {
                unsigned nIndex = indices[nnii];
                LineFeature* nlf = lfs->m_LineFeatures[nIndex];
                if (visited[nIndex] != currentVisitedID) {
                    visited[nIndex] = currentVisitedID;
                    if (icl->findCorrespondingLine(nlf, curr_trans))
                        stack.push_back(nIndex);
                }	
            }
        }

        if (growedIndices.size() <= resultIndices.size())
            break;

        // refine alignment
        bool success = icl->align(growedSubset, curr_trans, score);
        if (!success)
            return 0;

        if (score < last_score)
            break;

        // only update after a positive grow
        trans = curr_trans;
        last_score = score;
        resultIndices.resize(growedIndices.size());
        std::copy(growedIndices.begin(), growedIndices.end(), resultIndices.begin());
    }
    return score;
}

void ICLValidation(std::vector<LBaseSetTranSet*>& symmetries,
                   LBaseSet* lbs, const unsigned& bi, const unsigned& bj,
                   LineFeatureSet* lfs, DampingICL* icl, FastSphereQuerry* query,
                   const float& searchRadius, const float& similarityEpsilon, const float& bb_diag_length,
                   const float& minScore, const float& angleThreshold, const bool& show_icl
                   )
{
    //if (symmetries.size() > 20) return; // DEBUG: only test and draw this instance
    //if (symmetries.size() != 6) return; // DEBUG: only test and draw this instance

    const std::vector<LBase*>& lbasevec = lbs->getBaseVector();
    const std::vector<float>& angles = lbs->getAngleVector();
    LBase* currentBase = lbasevec[bi];
    LBase* otherBase = lbasevec[bj];

    // check deviation in angles
    if (fabs(angles[bi] - angles[bj]) > angleThreshold)
        return;
    if (norm(currentBase->getBasePoint()-otherBase->getBasePoint()) < 0.05f*bb_diag_length)
        return;

    Matrix4f trans;
    std::vector<unsigned> growedSubSet;
    growedSubSet.reserve(lfs->getNumFeatures());
    const float score1 = refineAndGrow(icl, currentBase, otherBase,
        trans, lfs, query, searchRadius, growedSubSet);
    const float supp_perc = (float)growedSubSet.size() / (float)lfs->getNumFeatures();
    bool valid1 = (!NU::IsZeroMatrix(trans-IDENTITY4F), similarityEpsilon, bb_diag_length)
        && (score1 > minScore)
        //&& (supp_perc > 0.01)
        ;
    if (valid1) {
        LBaseSetTranSet* s1 = new LBaseSetTranSet;
        s1->T = trans;
        s1->bseed.first = bi;
        s1->bseed.second = bj;
        s1->subset = growedSubSet;
        s1->score  = score1;
        symmetries.push_back(s1);

        //std::set< card32 > supp_set;
        //for (unsigned ii : growedSubSet) {
        //    const std::vector< card32 >& suppInds = att->featureSupports[ii];
        //    for (card32 jj : suppInds) {
        //        supp_set.insert(jj);
        //    }
        //}
        //std::copy(supp_set.begin(), supp_set.end(), std::back_inserter(s1->supp_list));

        if (show_icl)
        {
            debugRenderer->beginRenderJob_OneFrame("icl_validation_", DR_FRAME++);
            debugRenderer->setSmoothLine(true);
            Vector3f sourceColor = makeVector3f(1.f, 0.f, 0.f);
            Vector3f destColor = makeVector3f(0.f, 0.f, 1.f);
            Vector3f diffColor = makeVector3f(0.f, 1.f, 0.f);
            Vector3f subsetColor1 = makeVector3f(0.8f, 0.8f, 0.f);
            Vector3f subsetColor2 = makeVector3f(0.f, 0.8f, 0.8f);
            LBase *lb1 = currentBase;
            LBase *lb2 = otherBase;
            Vector3f bp1 = lb1->getBasePoint();
            Vector3f bp2 = lb2->getBasePoint();
            debugRenderer->addPoint(bp1,
                sourceColor);
            lb1->drawWithDebugRenderer(sourceColor, 0.01, 1);
            debugRenderer->addPoint(bp2,
                destColor);
            lb2->drawWithDebugRenderer(destColor, 0.01, 1);
            debugRenderer->addLine(
                bp1, bp2,
                sourceColor, destColor, 10.0f);
            debugRenderer->addLine(
                transformVector3f(trans, bp1), bp2,
                diffColor, diffColor, 10.0f);

            for (card32 ii : growedSubSet) {
                const Vector3f& pos = lfs->m_LineFeatures[ii]->m_Position;
                const Vector3f pos_t = transformVector3f(trans, pos);
                debugRenderer->addPoint(pos,
                    subsetColor1);
                debugRenderer->addPoint(pos_t,
                    subsetColor2);
                //debugRenderer->addLine(
                //    pos, pos_t,
                //    subsetColor1, subsetColor2, 10.0f);
            }
            debugRenderer->endRenderJob();
        }

        debugOutput << str( boost::format("  #%1% - %2%: aligned with %3% supports [%4%%%]; score: %5%.\n\n")
            % bi % bj % growedSubSet.size() % (supp_perc * 100) % score1
            );
    } else {
        if (show_icl)
        {
            debugRenderer->beginRenderJob_OneFrame("icl_validation_", DR_FRAME++);
            debugRenderer->setSmoothLine(true);
            Vector3f sourceColor = makeVector3f(1.f, 0.f, 0.f);
            Vector3f destColor = makeVector3f(0.f, 0.f, 1.f);
            Vector3f diffColor = makeVector3f(0.f, 1.f, 0.f);
            Vector3f supportColor = makeVector3f(0.8f, 0.f, 0.8f);
            LBase *lb1 = currentBase;
            LBase *lb2 = otherBase;
            Vector3f bp1 = lb1->getBasePoint();
            Vector3f bp2 = lb2->getBasePoint();
            debugRenderer->addPoint(bp1,
                sourceColor);
            lb1->drawWithDebugRenderer(sourceColor, 0.01, 1);
            debugRenderer->addPoint(bp2,
                destColor);
            lb2->drawWithDebugRenderer(destColor, 0.01, 1);
            debugRenderer->addLine(
                bp1, bp2,
                sourceColor, destColor, 10.0f);
            debugRenderer->addLine(
                transformVector3f(trans, bp1), bp2,
                diffColor, diffColor, 10.0f);

            for (card32 ii : growedSubSet) {
                const Vector3f& pos = lfs->m_LineFeatures[ii]->m_Position;
                const Vector3f pos_t = transformVector3f(trans, pos);
                debugRenderer->addPoint(pos,
                    supportColor);
                debugRenderer->addPoint(pos_t,
                    supportColor);
                //debugRenderer->addLine(
                //    pos, pos_t,
                //    subsetColor1, subsetColor2, 10.0f);
            }
            debugRenderer->endRenderJob();
        }

        debugOutput << str( boost::format("  #%1% - %2%: bad transformation with %3% supports [%4%%%]; score: %5%.\n\n")
            % bi % bj % growedSubSet.size() % (supp_perc * 100) % score1
            );
    }
}

void PCIMSFeatureAlign::alignLBase()
{
    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "# aligning L-bases (icl) \n";
    debugOutput << "########################################\n";
    debugOutput << "\n";

    //X4_TIMER_START(align_lbase);

    SGListNode *muscPCLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), muscFolder));
    if (!muscPCLN) {
        error("source folder not found (no list node)");
        return;
    }
    SGListNode *featureLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), featFolder));
    if (!featureLN) {
        error("feature folder not found (no list node)");
        return;
    }
    SGListNode *crossPtLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), crosFolder));
    if (!crossPtLN) {
        error("feature folder not found (no list node)");
        return;
    }
    const mpcard num_node = featureLN->getNumChildNodes(nullptr);
    for (mpcard ni = 0; ni < num_node; ++ni) {
        MultiScaleInCorePointCloud *muscMSPC = getMSPCFromListNode(muscPCLN, ni);
        if (!muscMSPC) {
            error("PCIMSFeatureAlign::showFeatureLineClusters - input missing.");
            return;
        }
        MultiScaleInCorePointCloud *featureMSPC = getMSPCFromListNode(featureLN, ni);
        if (!featureMSPC) {
            error("PCIMSFeatureAlign::showFeatureLineClusters - feature missing.");
            return;
        }
        MultiScaleInCorePointCloud *crossPtMSPC = getMSPCFromListNode(crossPtLN, ni);
        if (!crossPtMSPC) {
            error("PCIMSFeatureAlign::showFeatureLineClusters - feature missing.");
            return;
        }

        progressWindow->pushStep(true, "aligning LBase ...");
        const unsigned sl = std::min(minLevel, featureMSPC->getNumLevels()-1);
        const unsigned el = std::min(maxLevel, featureMSPC->getNumLevels());
        const unsigned numlevel = el - sl;
        for (unsigned level = sl; level < el; ++level) {
            TopologyAttachment *topAtt = dynamic_cast<TopologyAttachment*>(crossPtMSPC->getAttachedData(level, TopologyAttachment::getDefaultName()));
            if (!topAtt) {
                debugOutput << str( boost::format("No topology for level %1%\n")
                    % level
                    );
                continue;
            }
            InCorePCTopologyGraph* tpg = topAtt->getTopology();

            PointSet *featurePS = featureMSPC->getLevel(level);
            const float bb_diag_length = featurePS->getBoundingBox().getDiagonalLength();

            LineFeatureSet* lfs = dynamic_cast<LineFeatureSet*>(featureMSPC->getAttachedData(level, LineFeatureSet::getDefaultName()));
            LBaseSet* lbs = dynamic_cast<LBaseSet*>(featureMSPC->getAttachedData(level, LBaseSet::getDefaultName()));
            const std::vector<LBase*>& lbasevec = lbs->getBaseVector();
            lbs->computeAngles();
            std::vector<LBaseSetTranSet*>& symmetries = lbs->symmtranset;
            const unsigned nBases = lbasevec.size();

            C13DescriptorAttachment* att = dynamic_cast<C13DescriptorAttachment*>(
                muscMSPC->getAttachedData(level, C13DescriptorAttachment::getDefaultName()));
            const float sradius = att->scale;

            //const float minScore = 50.f; // HC
            const float minScore = settings_->icl_min_score;
            const float angleThreshold = 10.0f; // HC
            const float similarityEpsilon = 1e-2;
            const float distSigma = settings_->icl_dist_ratio * bb_diag_length;

            debugOutput << str( boost::format(
                "icl minimal score: %1%.\n"
                "distance sigma: %2%.\n"
                "angle threshold: %3%.\n\n"
                )
                % minScore % distSigma % angleThreshold
                );

            FastSphereQuerry* query = new FastSphereQuerry(lfs->getFeaturesAsPointCloud());
            DampingICL* icl = new DampingICL;
            icl->setLineDistanceSigma(distSigma);
            icl->setMaxNumIterations(10);
            //icl->setVisualization(true);
            icl->begin(lfs);
            int nSelected =0;

            progressWindow->pushStep(true, "Single LBase feature ...");
            const unsigned enum_thresh = 30; // perform complete pair-wise construction for small number
            for (unsigned bi = 0; bi < nBases; ++bi) {

                if (nBases < enum_thresh) {
                    for (unsigned bj = 0; bj < nBases; ++bj) {
                        if(bi == bj) continue;

                        ICLValidation(symmetries,
                            lbs, bi, bj,
                            lfs, icl, query,
                            sradius, similarityEpsilon, bb_diag_length,
                            minScore, angleThreshold, false
                            );
                    }
                } else {
                    const unsigned numNV = tpg->getNumAdjacentVertices(bi);
                    for (unsigned ni = 0; ni < numNV; ++ni) {
                        const unsigned bj = tpg->getAdjacentVertex(bi, ni);

                        ICLValidation(symmetries,
                            lbs, bi, bj,
                            lfs, icl, query,
                            sradius, similarityEpsilon, bb_diag_length,
                            minScore, angleThreshold, false
                            );
                    }
                }
                //break;
                progressWindow->progressf((float)bi/(float)(nBases-1));
            }
            progressWindow->popStep();

            icl->end();
            delete icl;
            delete query;

            // keep only one representative for each transformation
            cleanTransformations(symmetries, lbs, bb_diag_length);
            sort(symmetries.begin(), symmetries.end(), LBaseSetTranSet::com_func);
            //for (unsigned ii = 0; ii < symmetries.size(); ++ii) {
            //    debugOutput << symmetries[ii]->subset.size() << " - " << symmetries[ii]->supp_list.size() << "\n";
            //}

            progressWindow->progressf((float)(level-sl)/(float)(numlevel-1));

            debugOutput << str( boost::format("On level %1%: %2% transformations after aligning L-bases.\n\n")
                % level % symmetries.size()
                );
        }
        progressWindow->popStep();
    }

    //X4_TIMER_STOP(align_lbase);
}

bool ICPExpansionValidation(
    std::vector<LBaseSetTranSet*>& symmetries,
    const std::vector<LBase*>& lbasevec, const unsigned& ti, PointSet* muscPS,
    DampingICP* icp, InCorePCTopologyGraph* tpg, TopNeighborhoodSphereSearch* query_top, HierarchicalKNNIterator* hIt,
    const float& init_radius, const float& top_radius, const unsigned& num_expansion, const float& min_residual,
    const bool& show_icp
    )
{
    AAT positionAAT = muscPS->getAAT("position", 3, VAD::DATA_FORMAT_FLOAT32);
    Matrix4f& trans = symmetries[ti]->T;
    Matrix4f curr_trans = trans;
    const Vector3f src_base_pos = lbasevec[symmetries[ti]->bseed.first]->getBasePoint();
    hIt->setSeekPointAndReset(src_base_pos);
    const unsigned src_base_index = hIt->getCurrentPointIndex();

    // for debug drawing points within one top radius
    std::set< unsigned > top_stack;
    if (show_icp)
    {
        query_top->setup(tpg, muscPS, src_base_index, src_base_pos, top_radius, positionAAT);
        while (!query_top->atEnd()) {
            const unsigned curr_index = query_top->getCurrentIndex();
            top_stack.insert(curr_index);
            query_top->next();
        }
    }

    // temp storage for support list
    std::vector< unsigned > supp_list;
    supp_list.reserve(muscPS->getNumEntries());
    std::deque<unsigned> inlier_indices;

    // recalculate query radius and fill in the list
    float curr_query_radius = init_radius, last_radius = init_radius;
    debugOutput << str( boost::format("\n  #%1%: query radius %2%.\n")
        % ti % curr_query_radius
        );
    query_top->setup(tpg, muscPS, src_base_index, src_base_pos, curr_query_radius, positionAAT);
    while (!query_top->atEnd()) {
        const unsigned curr_index = query_top->getCurrentIndex();
        supp_list.push_back(curr_index);
        query_top->next();
    }

    // perform icp
    PointSet* queryps = muscPS->subset(supp_list);
    float residual = icp->calculateResidualAfterICP(queryps, curr_trans, inlier_indices);
    delete queryps;

    bool valid1 = (!NU::IsZeroMatrix(curr_trans-IDENTITY4F))
        && (residual > min_residual)
        ;
    if (valid1) { // failed on first try
        if (show_icp)
        {
            debugRenderer->beginRenderJob_OneFrame("icp_expansion_", DebugRenderer::DR_FRAME++);
            debugRenderer->setSmoothLine(true);
            Vector3f sourceColor = makeVector3f(1.f, 0.f, 0.f);
            Vector3f destColor = makeVector3f(0.f, 0.f, 1.f);
            Vector3f diffColor = makeVector3f(0.f, 1.f, 0.f);
            Vector3f supportColor = makeVector3f(0.8f, 0.f, 0.8f);
            LBase *lb1 = lbasevec[symmetries[ti]->bseed.first];
            LBase *lb2 = lbasevec[symmetries[ti]->bseed.second];
            Vector3f bp1 = lb1->getBasePoint();
            Vector3f bp2 = lb2->getBasePoint();

            debugRenderer->addPoint(bp1,
                sourceColor);
            lb1->drawWithDebugRenderer(sourceColor, 0.01, 1);
            debugRenderer->addPoint(bp2,
                destColor);
            lb2->drawWithDebugRenderer(destColor, 0.01, 1);
            debugRenderer->addLine(
                bp1, bp2,
                sourceColor, destColor, 10.0f);
            debugRenderer->addLine(
                transformVector3f(curr_trans, bp1), bp2,
                diffColor, diffColor, 10.0f);

            for (card32 ii : supp_list) {
                const Vector3f& pos = muscPS->get3f(ii, positionAAT);
                const Vector3f pos_t = transformVector3f(curr_trans, pos);
                if (top_stack.find(ii) == top_stack.end()) {
                    debugRenderer->addPoint(pos,
                        supportColor);
                } else {
                    debugRenderer->addPoint(pos,
                        destColor);
                }
                debugRenderer->addPoint(pos_t,
                    supportColor);
                //debugRenderer->addLine(
                //    pos, pos_t,
                //    sourceColor, destColor, 10.0f);
            }
            debugRenderer->endRenderJob();
        }

        delete symmetries[ti];
        debugOutput << str( boost::format("  #%1%: bad transformation with residual %2%.\n")
            % ti % residual
            );
        return false;
    } else { // will do expansion to determine the supporting points
        trans = curr_trans;

        float curr_residual = residual;
        int exp_direct = 1; // expanding or shrinking
        unsigned radius_multiplier = 2;
        const unsigned damper_multiplier = 2;
        std::vector< unsigned >& supp_list_all = symmetries[ti]->supp_list;
        unsigned nex = 0;
        while (supp_list.size() < muscPS->getNumEntries() && nex++ < num_expansion && radius_multiplier > 1) {
            supp_list_all.resize(supp_list.size());
            std::copy(supp_list.begin(), supp_list.end(), supp_list_all.begin());
            supp_list.clear();

            // recalculate query radius and fill in the list
            curr_query_radius = last_radius + init_radius * radius_multiplier / 2; // power expansion
            //curr_query_radius = init_radius + top_radius * radius_multiplier; // topological power expansion
            debugOutput << str( boost::format("\n  #%1%: query radius %2%, with multiplayer %3%.\n")
                % ti % curr_query_radius % radius_multiplier
                );
            query_top->setup(tpg, muscPS, src_base_index, src_base_pos, curr_query_radius, positionAAT);
            while (!query_top->atEnd()) {
                const unsigned curr_index = query_top->getCurrentIndex();
                supp_list.push_back(curr_index);
                query_top->next();
            }

            // perform icp
            PointSet* queryps = muscPS->subset(supp_list);
            curr_residual = icp->calculateResidualAfterICP(queryps, curr_trans, inlier_indices);
            delete queryps;

            if (curr_residual > min_residual) { // reached the maximum radius
                //radius_multiplier /= damper_multiplier;
                //exp_direct = -1;
                break; // disable shrinking
            } else {
                (exp_direct > 0) ? radius_multiplier *= damper_multiplier : radius_multiplier /= damper_multiplier; // once shrinking, always decrease
                trans = curr_trans;
                residual = curr_residual;
                last_radius = curr_query_radius;
            }
        }

        if (show_icp)
        {
            debugRenderer->beginRenderJob_OneFrame("icp_expansion_", DebugRenderer::DR_FRAME++);
            debugRenderer->setSmoothLine(true);
            Vector3f sourceColor = makeVector3f(1.f, 0.f, 0.f);
            Vector3f destColor = makeVector3f(0.f, 0.f, 1.f);
            Vector3f diffColor = makeVector3f(0.f, 1.f, 0.f);
            Vector3f subsetColor1 = makeVector3f(0.8f, 0.8f, 0.f);
            Vector3f subsetColor2 = makeVector3f(0.f, 0.8f, 0.8f);
            LBase *lb1 = lbasevec[symmetries[ti]->bseed.first];
            LBase *lb2 = lbasevec[symmetries[ti]->bseed.second];
            Vector3f bp1 = lb1->getBasePoint();
            Vector3f bp2 = lb2->getBasePoint();

            debugRenderer->addPoint(bp1,
                sourceColor);
            lb1->drawWithDebugRenderer(sourceColor, 0.01, 1);
            debugRenderer->addPoint(bp2,
                destColor);
            lb2->drawWithDebugRenderer(destColor, 0.01, 1);
            debugRenderer->addLine(
                bp1, bp2,
                sourceColor, destColor, 10.0f);
            debugRenderer->addLine(
                transformVector3f(curr_trans, bp1), bp2,
                diffColor, diffColor, 10.0f);

            for (card32 ii : supp_list) {
                const Vector3f& pos = muscPS->get3f(ii, positionAAT);
                const Vector3f pos_t = transformVector3f(curr_trans, pos);
                if (top_stack.find(ii) == top_stack.end()) {
                    debugRenderer->addPoint(pos,
                        subsetColor1);
                } else {
                    debugRenderer->addPoint(pos,
                        destColor);
                }
                debugRenderer->addPoint(pos_t,
                    subsetColor2);
                //debugRenderer->addLine(
                //    pos, pos_t,
                //    subsetColor1, subsetColor2, 10.0f);
            }
            debugRenderer->endRenderJob();
        }

        debugOutput << str( boost::format("  #%1%: validated transformation with residual %2%.\n")
            % ti % residual
            );
        return true;
    }
}

bool ICPFullValidation(
    std::vector<LBaseSetTranSet*>& symmetries,
    const std::vector<LBase*>& lbasevec, const unsigned& ti, PointSet* muscPS,
    DampingICP* icp, InCorePCTopologyGraph* tpg, TopNeighborhoodSphereSearch* query_top, HierarchicalKNNIterator* hIt,
    float init_radius, float top_radius, const unsigned& num_expansion, const float& min_residual, const bool& show_icp
    )
{
    AAT positionAAT = muscPS->getAAT("position", 3, VAD::DATA_FORMAT_FLOAT32);
    Matrix4f& trans = symmetries[ti]->T;
    Matrix4f curr_trans = trans;
    const Vector3f src_base_pos = lbasevec[symmetries[ti]->bseed.first]->getBasePoint();
    hIt->setSeekPointAndReset(src_base_pos);
    const unsigned src_base_index = hIt->getCurrentPointIndex();

    // for debug drawing points within one top radius
    std::set< unsigned > top_stack;
    if (show_icp)
    {
        query_top->setup(tpg, muscPS, src_base_index, src_base_pos, top_radius, positionAAT);
        while (!query_top->atEnd()) {
            const unsigned curr_index = query_top->getCurrentIndex();
            top_stack.insert(curr_index);
            query_top->next();
        }
    }

    // temp storage for support list
    std::vector< unsigned > supp_list;
    supp_list.reserve(muscPS->getNumEntries());
    std::deque<unsigned> inlier_indices;

    // recalculate query radius and fill in the list
    float curr_query_radius = init_radius;
    debugOutput << str( boost::format("\n  #%1%: query radius %2%.\n")
        % ti % curr_query_radius
        );
    query_top->setup(tpg, muscPS, src_base_index, src_base_pos, curr_query_radius, positionAAT);
    while (!query_top->atEnd()) {
        const unsigned curr_index = query_top->getCurrentIndex();
        supp_list.push_back(curr_index);
        query_top->next();
    }

    // perform icp
    PointSet* queryps = muscPS->subset(supp_list);
    float residual = icp->calculateResidualAfterICP(queryps, curr_trans, inlier_indices);
    delete queryps;

    bool valid1 = (!NU::IsZeroMatrix(curr_trans-IDENTITY4F))
        && (residual > min_residual)
        ;
    if (valid1) { // failed on first try
        if (show_icp)
        {
            debugRenderer->beginRenderJob_OneFrame("icp_expansion_", DebugRenderer::DR_FRAME++);
            debugRenderer->setSmoothLine(true);
            Vector3f sourceColor = makeVector3f(1.f, 0.f, 0.f);
            Vector3f destColor = makeVector3f(0.f, 0.f, 1.f);
            Vector3f diffColor = makeVector3f(0.f, 1.f, 0.f);
            Vector3f supportColor = makeVector3f(0.8f, 0.f, 0.8f);
            LBase *lb1 = lbasevec[symmetries[ti]->bseed.first];
            LBase *lb2 = lbasevec[symmetries[ti]->bseed.second];
            Vector3f bp1 = lb1->getBasePoint();
            Vector3f bp2 = lb2->getBasePoint();

            debugRenderer->addPoint(bp1,
                sourceColor);
            lb1->drawWithDebugRenderer(sourceColor, 0.01, 1);
            debugRenderer->addPoint(bp2,
                destColor);
            lb2->drawWithDebugRenderer(destColor, 0.01, 1);
            debugRenderer->addLine(
                bp1, bp2,
                sourceColor, destColor, 10.0f);
            debugRenderer->addLine(
                transformVector3f(curr_trans, bp1), bp2,
                diffColor, diffColor, 10.0f);

            for (card32 ii : supp_list) {
                const Vector3f& pos = muscPS->get3f(ii, positionAAT);
                const Vector3f pos_t = transformVector3f(curr_trans, pos);
                if (top_stack.find(ii) == top_stack.end()) {
                    debugRenderer->addPoint(pos,
                        supportColor);
                } else {
                    debugRenderer->addPoint(pos,
                        destColor);
                }
                debugRenderer->addPoint(pos_t,
                    supportColor);
                //debugRenderer->addLine(
                //    pos, pos_t,
                //    sourceColor, destColor, 10.0f);
            }
            debugRenderer->endRenderJob();
        }

        delete symmetries[ti];
        debugOutput << str( boost::format("  #%1%: bad transformation with residual %2%.\n")
            % ti % residual
            );
        return false;
    } else { // perform icp on the full point set
        trans = curr_trans;

        PointSet* queryps = (PointSet*)muscPS->copy();
        float residual = icp->calculateResidualAfterICP(queryps, curr_trans, inlier_indices, true);
        delete queryps;

        trans = curr_trans;

        if (show_icp)
        {
            debugRenderer->beginRenderJob_OneFrame("icp_expansion_", DebugRenderer::DR_FRAME++);
            debugRenderer->setSmoothLine(true);
            Vector3f sourceColor = makeVector3f(1.f, 0.f, 0.f);
            Vector3f destColor = makeVector3f(0.f, 0.f, 1.f);
            Vector3f diffColor = makeVector3f(0.f, 1.f, 0.f);
            Vector3f subsetColor1 = makeVector3f(0.8f, 0.8f, 0.f);
            Vector3f subsetColor2 = makeVector3f(0.f, 0.8f, 0.8f);
            LBase *lb1 = lbasevec[symmetries[ti]->bseed.first];
            LBase *lb2 = lbasevec[symmetries[ti]->bseed.second];
            Vector3f bp1 = lb1->getBasePoint();
            Vector3f bp2 = lb2->getBasePoint();

            debugRenderer->addPoint(bp1,
                sourceColor);
            lb1->drawWithDebugRenderer(sourceColor, 0.01, 1);
            debugRenderer->addPoint(bp2,
                destColor);
            lb2->drawWithDebugRenderer(destColor, 0.01, 1);
            debugRenderer->addLine(
                bp1, bp2,
                sourceColor, destColor, 10.0f);
            debugRenderer->addLine(
                transformVector3f(curr_trans, bp1), bp2,
                diffColor, diffColor, 10.0f);

            for (card32 ii : supp_list) {
                const Vector3f& pos = muscPS->get3f(ii, positionAAT);
                const Vector3f pos_t = transformVector3f(curr_trans, pos);
                if (top_stack.find(ii) == top_stack.end()) {
                    debugRenderer->addPoint(pos,
                        subsetColor1);
                } else {
                    debugRenderer->addPoint(pos,
                        destColor);
                }
                debugRenderer->addPoint(pos_t,
                    subsetColor2);
                //debugRenderer->addLine(
                //    pos, pos_t,
                //    subsetColor1, subsetColor2, 10.0f);
            }
            debugRenderer->endRenderJob();
        }

        debugOutput << str( boost::format("  #%1%: validated transformation with residual %2%.\n")
            % ti % residual
            );
        return true;
    }
}

void PCIMSFeatureAlign::geometricValidation()
{
    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "# geometric validation (icp) \n";
    debugOutput << "########################################\n";
    debugOutput << "\n";

    //X4_TIMER_START(geometric_validation);

    if (settings_->show_icl_icp) DebugRenderer::DR_FRAME = 0;

    SGListNode *featureLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), featFolder));
    if (!featureLN) {
        error("feature folder not found (no list node)");
        return;
    }
    SGListNode *muscPCLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), muscFolder));
    if (!muscPCLN) {
        error("musc folder not found (no list node)");
        return;
    }
    const mpcard num_node = featureLN->getNumChildNodes(nullptr);
    if (num_node != muscPCLN->getNumChildNodes(nullptr)) {
        error("feature & musc mis-match");
        return;
    }

    for (mpcard ni = 0; ni < num_node; ++ni) {
        MultiScaleInCorePointCloud *featureMSPC = getMSPCFromListNode(featureLN, ni);
        MultiScaleInCorePointCloud *muscMSPC = getMSPCFromListNode(muscPCLN, ni);
        if (!featureMSPC || !muscMSPC) {
            error("PCIMSFeatureAlign::geometricValidation - feature missing.");
            return;
        }
        const MultiScalePCParams* params = muscMSPC->getMultiScaleParameters();
        const float& base_grid_spacing = params->baseGridSpacing;

        PointSet *muscPS = muscMSPC->getLevel(0); // always work on the 1st level for icp alignment
        UnstructuredInCorePointCloud *muscPC = new UnstructuredInCorePointCloud();
        PointSet* tmpPS = (PointSet*)muscPS->copy();
        muscPC->setPointSet(tmpPS);

        const float bb_diag_length = muscPS->getBoundingBox().getDiagonalLength();
        const float median_dist = getMedianPointDistance(muscPC);
        const float init_radius = bb_diag_length * settings_->icp_init_radius_radio;
        const unsigned num_expansion = 4;
        const float min_residual = std::max<float>(
            base_grid_spacing * 0.5f, median_dist * 0.5f);
        const float min_residual_iter = min_residual * 10.f;
        const float top_radius = base_grid_spacing * settings_->outlier_dist_factor; // sqrt(2.f);

        DampingICP* icp = new DampingICP(muscPC);
        icp->setupParameters(
            20,                                 // number of iterations
            median_dist,                        // median point distance
            settings_->outlier_dist_factor,     // outlier distance factor
            0.8f,                               // minimum inlier percentage
            bb_diag_length *0.05f,              // allowed move distance
            min_residual,                       // maximal residual tolerance
            min_residual * 0.1f,                // convergence difference threshold
            settings_->show_icl_icp             // show_icl_icp
            );

        TopologyAttachment *topAtt = dynamic_cast<TopologyAttachment*>(muscMSPC->getAttachedData(0, TopologyAttachment::getDefaultName()));
        InCorePCTopologyGraph* tpg = nullptr;
        if (!topAtt) {
            debugOutput << "PCIMSFeatureAlign::geometricValidation - topology missing, recomputing ...\n";
            PCCComputeTopology cmd;
            cmd.setup(PCCComputeTopology::TOPTYPE_EPS, top_radius);
            tpg = cmd.computeEpsTopology(tmpPS, muscPC);
        } else {
            tpg = topAtt->getTopology();
        }

        TopNeighborhoodSphereSearch* query_top = new TopNeighborhoodSphereSearch;
        HierarchicalKNNIterator* hIt = new HierarchicalKNNIterator(muscPC,32,nullptr);
        debugOutput << str( boost::format(
            "icp residual threshold: %1%.\n"
            "bbox diagnal distance: %7%.\n"
            "base grid spacing: %2%.\n"
            "median point distance: %8%.\n"
            "top radius: %3%, with outlier distance factor: %6%.\n"
            "initial radius: %4%, maximum %5% times expansion.\n"
            )
            % min_residual % base_grid_spacing % top_radius % init_radius % num_expansion % settings_->outlier_dist_factor
            % bb_diag_length % median_dist
            );

        const unsigned sl = std::min(minLevel, featureMSPC->getNumLevels()-1);
        const unsigned el = std::min(maxLevel, featureMSPC->getNumLevels());
        const unsigned numlevel = el - sl;
        for (unsigned level = sl; level < el; ++level) {
            LBaseSet* lbs = dynamic_cast<LBaseSet*>(featureMSPC->getAttachedData(level, LBaseSet::getDefaultName()));
            if (nullptr == lbs) {
                error("PCIMSFeatureAlign::geometricValidation - LBaseSet attachment missing.");
                return;
            }
            const std::vector<LBase*>& lbasevec = lbs->getBaseVector();
            std::vector<LBaseSetTranSet*>& symmetries = lbs->symmtranset;
            C13DescriptorAttachment* att = dynamic_cast<C13DescriptorAttachment*>(
                muscMSPC->getAttachedData(level, C13DescriptorAttachment::getDefaultName()));

            progressWindow->pushStep(true, "geometric validation ...");
            std::deque<LBaseSetTranSet*> symm_valid;
            const unsigned num_trans = symmetries.size();
            for (unsigned ti = 0; ti < num_trans; ++ti) {
                // NOTE: the 1st level may not contain the specified number of transformations
                //if (ti > 20 || ti < 0 || level != 0) { // DEBUG: only test and draw this instance
                //    delete symmetries[ti];
                //    continue;
                //}
                //if (ti != 0 || level != 0) { // DEBUG: only test and draw this instance
                //    delete symmetries[ti];
                //    continue;
                //}
                if (settings_->use_region_growing)
                {
                    if (ICPExpansionValidation(
                        symmetries,
                        lbasevec, ti, muscPS,
                        icp, tpg, query_top, hIt,
                        init_radius, top_radius, num_expansion, min_residual_iter, settings_->show_icl_icp
                        )) {
                            symm_valid.push_back(symmetries[ti]);
                            //} else {
                            //    delete symmetries[ti]; // already done in validation
                    }
                } else {
                    if (ICPFullValidation(
                        symmetries,
                        lbasevec, ti, muscPS,
                        icp, tpg, query_top, hIt,
                        init_radius, top_radius, num_expansion, min_residual_iter, settings_->show_icl_icp
                        )) {
                            symm_valid.push_back(symmetries[ti]);
                            //} else {
                            //    delete symmetries[ti]; // already done in validation
                    }
                }

                progressWindow->progressf((float)(ti)/(float)(num_trans-1));
            }
            symmetries.clear();
            std::copy(symm_valid.begin(), symm_valid.end(), std::back_inserter(symmetries));

            // keep only one representative for each transformation
            cleanTransformations(symmetries, lbs, bb_diag_length);
            sort(symmetries.begin(), symmetries.end(), LBaseSetTranSet::com_func);
            //for (unsigned ii = 0; ii < symmetries.size(); ++ii) {
            //    debugOutput << symmetries[ii]->subset.size() << " - " << symmetries[ii]->supp_list.size() << "\n";
            //}

            debugOutput << str( boost::format("On level %1%: %2% transformations after geometric validation.\n")
                % level % symmetries.size()
                );
            progressWindow->popStep();
        }

        if (!topAtt) {
            delete tpg;
            // better to delete the attchment now, but how?
        }
        delete query_top;
        delete hIt;
        delete muscPC;
        delete icp;
    }

    //X4_TIMER_STOP(geometric_validation);
}

void PCIMSFeatureAlign::CleanTransformations(void)
{
    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "# Cleaning transformations \n";
    debugOutput << "########################################\n";
    debugOutput << "\n";

    //X4_TIMER_START(clean_transformation);

    SGListNode *featureLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), featFolder));
    if (!featureLN) {
        error("feature folder not found (no list node)");
        return;
    }
    SGListNode *muscPCLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), muscFolder));
    if (!muscPCLN) {
        error("musc folder not found (no list node)");
        return;
    }
    const mpcard num_node = featureLN->getNumChildNodes(nullptr);
    if (num_node != muscPCLN->getNumChildNodes(nullptr)) {
        error("feature & musc mis-match");
        return;
    }

    for (mpcard ni = 0; ni < num_node; ++ni) {
        MultiScaleInCorePointCloud *featureMSPC = getMSPCFromListNode(featureLN, ni);
        MultiScaleInCorePointCloud *muscMSPC = getMSPCFromListNode(muscPCLN, ni);
        if (!featureMSPC || !muscMSPC) {
            error("PCIMSFeatureAlign::geometricValidation - feature missing.");
            return;
        }

        PointSet *muscPS = muscMSPC->getLevel(0); // always work on the 1st level for icp alignment
        const float bb_diag_length = muscPS->getBoundingBox().getDiagonalLength();

        const unsigned sl = std::min(minLevel, featureMSPC->getNumLevels()-1);
        const unsigned el = std::min(maxLevel, featureMSPC->getNumLevels());
        const unsigned numlevel = el - sl;
        for (unsigned level = sl; level < el; ++level) {
            LBaseSet* lbs = dynamic_cast<LBaseSet*>(featureMSPC->getAttachedData(level, LBaseSet::getDefaultName()));
            const std::vector<LBase*>& lbasevec = lbs->getBaseVector();
            std::vector<LBaseSetTranSet*>& symmetries = lbs->symmtranset;
            debugOutput << str( boost::format("On level %1%: %2% transformations before reduction.\n")
                % level % symmetries.size()
                );
            cleanTransformations(symmetries, lbs, bb_diag_length);
            debugOutput << str( boost::format("On level %1%: %2% transformations after reduction.\n")
                % level % symmetries.size()
                );
        }
    }

    //X4_TIMER_STOP(clean_transformation);
}
