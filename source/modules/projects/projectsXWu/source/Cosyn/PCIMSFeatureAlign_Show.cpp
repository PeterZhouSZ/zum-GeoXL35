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
#include "SGRelativeTimeAnimationNode.h"
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
    int DR_FRAME = 0;
}

void PCIMSFeatureAlign::showPlanes(void)
{
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
            error("PCIMSFeatureAlign::showPlanes - input missing.");
            return;
        }

        for (unsigned level = 0; level < muscMSPC->getNumLevels(); ++level) {
            PointSet *sourcePS = muscMSPC->getLevel(level);
            PlaneClusterExtractor* plane_cluster = dynamic_cast<PlaneClusterExtractor*>(
                muscMSPC->getAttachedData(level, PlaneClusterExtractor::getDefaultName()));
            if (!plane_cluster) {
                error("PCIMSFeatureAlign::showPlanes - attachment PlaneClusterExtractor missing.");
                return;
            }
            debugRenderer->beginRenderJob_OneFrame("plane_cluster_", DR_FRAME++);
            plane_cluster->DrawWithDR(sourcePS);
            debugRenderer->endRenderJob();
        }
        progressWindow->progressf((float)ni/(float)(num_node-1));
    }
    progressWindow->popStep();
}

void PCIMSFeatureAlign::showLBases()
{
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
            error("PCIMSFeatureAlign::extractCrossPoints - input missing.");
            return;
        }
        MultiScaleInCorePointCloud *featureMSPC = getMSPCFromListNode(featureLN, ni);
        if (!featureMSPC) {
            error("PCIMSFeatureAlign::extractCrossPoints - feature missing.");
            return;
        }
        SGObjectNode *featureObjN = dynamic_cast<SGObjectNode*>(featureLN->getChildNode(nullptr, ni));

        for (unsigned level = 0; level < featureMSPC->getNumLevels(); ++level) {
            PointSet *featurePS = featureMSPC->getLevel(level);
            LBaseSet* lbs = dynamic_cast<LBaseSet*>(featureMSPC->getAttachedData(level, LBaseSet::getDefaultName()));
            if (nullptr == lbs) {
                error("PCIMSFeatureAlign::showLBases - LBaseSet attachment missing.");
                return;
            }
            std::vector<LBase*> lbasevec = lbs->getBaseVector();
            C13DescriptorAttachment* att = dynamic_cast<C13DescriptorAttachment*>(
                muscMSPC->getAttachedData(level, C13DescriptorAttachment::getDefaultName()));

            debugRenderer->beginRenderJob_OneFrame("LBases_", DR_FRAME++);
            for (LBase* lbase : lbasevec) {
                const Vector3f color = makeVector3f(0, 1, 1);
                lbase->drawWithDebugRenderer(color, 0.01, 1);
            }
            debugRenderer->endRenderJob();
            for (LBase* lbase : lbasevec) {
                const Vector3f color = makeVector3f(0, 1, 1);
                debugRenderer->beginRenderJob_OneFrame("LBases_", DR_FRAME++);
                lbase->drawWithDebugRenderer(color, 0.01, 1);
                debugRenderer->endRenderJob();
            }
        }
    }
}

void PCIMSFeatureAlign::showBoundaryLines()
{
    SGListNode *target = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), featFolder));
    if (!target) {
        error("target folder not found (no list node)");
        return;
    }
    const mpcard num_node = target->getNumChildNodes(nullptr);
    for (mpcard ni = 0; ni < num_node; ++ni) {
        SGObjectNode *objN = dynamic_cast<SGObjectNode*>(target->getChildNode(nullptr, ni));
        if (!objN) continue;
        SceneObject *so = objN->getSceneObject();
        if (!so) continue;

        UnstructuredInCoreTriangleMesh *mesh = dynamic_cast<UnstructuredInCoreTriangleMesh*>(so);
        UnstructuredInCorePointCloud   *upc  = dynamic_cast<UnstructuredInCorePointCloud*>(so);

        debugRenderer->beginRenderJob_OneFrame("boundary_", DR_FRAME++);
        PointSet *ps = upc->getPointSet();
        AAT positionAAT = ps->getAAT("position");
        AAT boundaryAAT = ps->getAAT("boundary");
        AAT boundaryTangentsAAT = ps->getAAT("boundaryTangent");
        const mpcard numPts = ps->getNumEntries();
        unsigned sbp = 0;
        for (mpcard i = 0; i < numPts; ++i) {
            float boundary = ps->get1f(i, boundaryAAT);
            if (boundary < .8f) continue;
            ++sbp;
            Vector3f position = ps->get3f(i, positionAAT);
            Vector3f tangent = ps->get3f(i, boundaryTangentsAAT);
            debugRenderer->addLine(
                position,
                //position + tangent,
                position + 0.1f*tangent,
                makeVector3f(0.5f, 0, 0), makeVector3f(0, 0, 0.5f),
                1.f
                //boundary*5
                );
        }
        debugOutput << sbp << "\n";
        debugRenderer->endRenderJob();
    }
}

void PCIMSFeatureAlign::showFeatureLineClusters()
{
    SGListNode *featureLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), featFolder));
    if (!featureLN) {
        error("feature folder not found (no list node)");
        return;
    }
    const mpcard num_node = featureLN->getNumChildNodes(nullptr);
    for (mpcard ni = 0; ni < num_node; ++ni) {
        MultiScaleInCorePointCloud *featureMSPC = getMSPCFromListNode(featureLN, ni);
        if (!featureMSPC) {
            error("PCIMSFeatureAlign::showFeatureLineClusters - feature missing.");
            return;
        }

        for (unsigned level = 0; level < featureMSPC->getNumLevels(); ++level) {
            std::map<unsigned, Vector3f> colorMap;
            debugRenderer->beginRenderJob_OneFrame("line_feature_cluster_", DR_FRAME++);
            PointSet *featurePS = featureMSPC->getLevel(level);
            LineFeatureSet* lfs = dynamic_cast<LineFeatureSet*>(featureMSPC->getAttachedData(level, LineFeatureSet::getDefaultName()));
            for (unsigned li = 0; li < lfs->getNumFeatures(); ++li) {
                unsigned cid = lfs->m_ClusterID[li];
                if (colorMap.find(cid) == colorMap.end()) colorMap[cid] = makeVector3f(rnd01(), rnd01(), rnd01());
                LineFeature * lf = lfs->m_LineFeatures[li];
                debugRenderer->addLine(
                    lf->m_Position-lf->m_LineDirection, lf->m_Position+lf->m_LineDirection,
                    colorMap[cid], colorMap[cid], 3.0f);
            }
            debugRenderer->endRenderJob();
            debugOutput << "On level " << level << ": " << colorMap.size() << " line clusters\n";
        }
    }
}

void PCIMSFeatureAlign::showLBaseAlignment()
{
    SGListNode *featureLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), featFolder));
    if (!featureLN) {
        error("feature folder not found (no list node)");
        return;
    }
    SGListNode *muscPCLN = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), muscFolder));
    if (!muscPCLN) {
        error("feature folder not found (no list node)");
        return;
    }
    const mpcard num_node = featureLN->getNumChildNodes(nullptr);
    if (num_node != muscPCLN->getNumChildNodes(nullptr)) {
        error("feature & data mis-match");
        return;
    }
    for (mpcard ni = 0; ni < num_node; ++ni) {
        MultiScaleInCorePointCloud *featureMSPC = getMSPCFromListNode(featureLN, ni);
        MultiScaleInCorePointCloud *muscMSPC = getMSPCFromListNode(muscPCLN, ni);
        if (!featureMSPC || !muscMSPC) {
            error("PCIMSFeatureAlign::showLBaseAlignment - feature missing.");
            return;
        }

        PointSet *muscPS = muscMSPC->getLevel(0); // always work on the 1st level
        const float diag_length = muscPS->getBoundingBox().getDiagonalLength();
        AAT posAAT = muscPS->getAAT("position", 3, VAD::DATA_FORMAT_FLOAT32);
        for (unsigned level = 0; level < featureMSPC->getNumLevels(); ++level) {
            PointSet *featurePS = featureMSPC->getLevel(level);
            LineFeatureSet* lfs = dynamic_cast<LineFeatureSet*>(featureMSPC->getAttachedData(level, LineFeatureSet::getDefaultName()));
            LBaseSet* lbs = dynamic_cast<LBaseSet*>(featureMSPC->getAttachedData(level, LBaseSet::getDefaultName()));
            std::vector<LBase*> lbasevec = lbs->getBaseVector();
            const std::vector<LBaseSetTranSet*>& symmetries = lbs->symmtranset;

            for (unsigned si = 0; si < symmetries.size(); ++si) {
                debugRenderer->beginRenderJob_OneFrame("lbase_alignment_", DR_FRAME++);
                debugRenderer->setSmoothLine(true);
                Vector3f sourceColor = makeVector3f(1.f, 0.f, 0.f);
                Vector3f destColor = makeVector3f(0.f, 0.f, 1.f);
                Vector3f diffColor = makeVector3f(0.f, 1.f, 0.f);
                Vector3f subsetColor1 = makeVector3f(0.8f, 0.8f, 0.f);
                Vector3f subsetColor2 = makeVector3f(0.f, 0.8f, 0.8f);
                Vector3f supportColor = makeVector3f(0.8f, 0.f, 0.8f);
                const Matrix4f& trans = symmetries[si]->T;
                LBase *lb1 = lbasevec[symmetries[si]->bseed.first];
                LBase *lb2 = lbasevec[symmetries[si]->bseed.second];
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

                const std::vector<unsigned>& lsset = symmetries[si]->subset;
                for (unsigned li = 0; li < lsset.size(); ++li) {
                    LineFeature * lf = lfs->m_LineFeatures[lsset[li]];
                    Vector3f lp = lf->m_Position;
                    Vector3f ld = lf->m_LineDirection * 0.1f / diag_length;
                    Vector3f lp1 = lp-0.5f*ld;
                    Vector3f lp2 = lp+0.5f*ld;
                    debugRenderer->addPoint(bp1,
                        subsetColor1);
                    debugRenderer->addLine(
                        lp, transformVector3f(trans, lp),
                        subsetColor1, subsetColor2, 1.0f);
                    debugRenderer->addLine(
                        lp1, lp2,
                        subsetColor1, subsetColor1, 5.0f);
                    Vector3f lp1_t = transformVector3f(trans, lp1);
                    Vector3f lp2_t = transformVector3f(trans, lp2);
                    debugRenderer->addLine(
                        lp1_t, lp2_t,
                        subsetColor2, subsetColor2, 5.0f);
                }

                const std::vector<card32>& supp_list = symmetries[si]->supp_list;
                if (supp_list.empty()) {
                    const unsigned num_points = muscPS->getNumEntries();
                    for (unsigned ii = 0; ii < num_points; ++ii) {
                        const Vector3f& pos = muscPS->get3f(ii, posAAT);
                        //debugRenderer->addPoint(pos,
                        //    subsetColor1);
                        debugRenderer->addPoint(transformVector3f(trans, pos),
                            subsetColor2);
                    }
                } else {
                    for (card32 ii : supp_list) {
                        const Vector3f& pos = muscPS->get3f(ii, posAAT);
                        debugRenderer->addPoint(pos,
                            subsetColor1);
                        debugRenderer->addPoint(transformVector3f(trans, pos),
                            subsetColor2);
                    }
                }
                debugRenderer->endRenderJob();
            }
        }
    }
}
