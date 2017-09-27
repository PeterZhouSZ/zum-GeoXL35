#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "SToolBox/SymmLineSampler.h"
#include "SToolBox/SymmSampler.h"
#include "SToolBox/DefieldSymm.h"
#include "SToolBox/DefieldSymmRotation.h"
//---------------------------------------------------------------------------
#include "SGRelativeTimeAnimationNode.h"
#include "PCCTriangleMeshSampler.h"
#include "SymmGroup/CCSGSymmetryGroup.h"
#include "SymmetryGroupAttachment.h"
#include "detector/FeatureLine.h"

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

#include "SGlistNode.h"
#include "SGRelativeTimeAnimationNode.h"

#include "SimpleAttachment.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}
std::deque<Vector3f> generateRegularSpacePoints(const Vector3f& a, const Vector3f& b, const float& delta)
{
    std::deque<Vector3f> ret;
    Vector3f const ab = b - a;
    const float length = ab.getNorm();
    const unsigned segments = (unsigned)ceil(length / delta);
    if (2 > segments) return ret;
    Vector3f direction(ab);
    direction.normalize();
    float32 const offset = ab.getNorm() / segments;
    for (unsigned ii = 0; ii < segments; ++ii) {
        ret.push_back(a + direction * offset *  ii);
    }
    ret.push_back(b);
    return ret;
}

void SymmLineSampler::DoSample(
    Scene* scene,
    const std::string& dataFolder,
    const std::string& symmWorkNameBase,
    const std::string& inputSymmName,
    const std::string& symmGroupNameBase,
    UnorderedGridUniq* p_pointGrid,
    const float& sampleth
    )
{
    UnorderedGridUniq& pointGrid = *p_pointGrid;
    const float grid_cell_size =  pointGrid.cell_size();

    SGListNode* dataList = dynamic_cast<SGListNode*>(
        getSceneGraphNodeByName(scene, dataFolder));
    if (!dataList)
    {
        error("data list node not found");
        return;
    }

    UICPC* outPoisson = dynamic_cast<UICPC*>(
        getPointCloud(scene, inputSymmName));
    if (!outPoisson)
    {
        error("Symmetry sampling points not found.");
        return;
    }
    PointSet& outPoissonPS = *outPoisson->getPointSet();
    const AAT& outPoissonAAT = outPoissonPS.getAAT("position");
    const AAT& symGroupAAT = outPoissonPS.getAAT("group");
    const AAT& colorAAT = outPoissonPS.getAAT("color");

    LineSymmBlockVec* lineBlockAtt = dynamic_cast<LineSymmBlockVec*>(
        outPoisson->getAttachments()->getData(LineSymmBlockVec::getDefaultName()));
    if (!lineBlockAtt) {
        error("Line symmetry block not found.");
        return;
    }
    std::vector<LineSymmBlock*>& lineBlockVec = lineBlockAtt->blockvec_;

    const unsigned numParts = dataList->getNumChildNodes(0);
    for (unsigned ni = 0; ni < numParts; ++ni) {
        {
            SGObjectNode* partNode = dynamic_cast<SGObjectNode*>(dataList->getChildNode(0, ni));
            const std::string& partName = partNode->getName();
            const std::string& symmWorkName = symmWorkNameBase + partName;
            const std::string& groupFolder = symmGroupNameBase + partName;

            UICPC* workPoints = dynamic_cast<UICPC*>(getPointCloud(scene, symmWorkName));
            if (!workPoints)
            {
                error("Symmetry work points not found.");
                return;
            }

            UICTM* inputMesh = dynamic_cast<UICTM*>(partNode->getSceneObject());
            if (!inputMesh)
            {
                error("input mesh not found");
                return;
            }
            FeatureSet* featureSet = dynamic_cast<FeatureSet*>(inputMesh->getAttachments()->getData("FeatureLines"));
            if (!featureSet)
            {
                error("Feature set not found");
                return;
            }
            const std::vector<GeometricFeature*>& featureVec = featureSet->m_Features;
            size_t num_feature = featureVec.size();

            DefieldSymmVec* defSymmVecAtt = dynamic_cast<DefieldSymmVec*>(
                workPoints->getAttachments()->getData(DefieldSymmVec::getDefaultName()));
            if (!defSymmVecAtt) {
                error("Defield symmetry vector not found.");
                return;
            }
            std::deque<DefieldSymm*>& symmVec = defSymmVecAtt->symmvec_;

            SymmSampleGraph* symSamGraph = dynamic_cast<SymmSampleGraph*>(
                outPoisson->getAttachments()->getData(SymmSampleGraph::getDefaultName()));
            if (!symSamGraph) {
                error("SymmSpaceSolver::Initialize() - SymmSampleGraph missing.");
                return;
            }

            SGRelativeTimeAnimationNode* groupList = dynamic_cast<SGRelativeTimeAnimationNode*>(
                getSceneGraphNodeByName(scene, groupFolder));
            if (!groupList) {
                std::stringstream ss;
                ss << "SymmSampler::DoSample - groug folder " << ni << " not found";
                error(ss.str());
                return;
            }

            const float poisson_resolution = getCachedMedianPointDistance(workPoints);
            const float grid_cell_size_sqr =  pointGrid.cell_size() * pointGrid.cell_size();
            const float radiusSam = sampleth; // for removing neighbor points
            const float radiusDet = poisson_resolution * 1.5; // 2-times of signal rate, for retrieve symmetry supports

            const size_t num_group =  groupList->getNumChildNodes(0);
            for (size_t groupIndex = 0; groupIndex < num_group; ++groupIndex)
            {
                SGObjectNode* symNode = dynamic_cast<SGObjectNode*>(groupList->getChildNode(0, groupIndex));
                if (!symNode)
                {
                    error("Sym not found");
                    return;
                }
                UICPC* symNodePC = dynamic_cast<UICPC*>(symNode->getSceneObject());
                if (!symNodePC)
                {
                    error("Pc not found");
                    return;
                }

                AttachedData* att = symNodePC->getAttachments()->getData("SymmetryGroup");
                sym::SymmetryGroupAttachment* groupAtt = dynamic_cast<sym::SymmetryGroupAttachment*>(att);
                if (!groupAtt)
                {
                    CCSGSymmetryGroup* ccsgroup = dynamic_cast<CCSGSymmetryGroup*>(att);
                    if (!ccsgroup) {
                        error("Group not found");
                        return;
                    }
                    groupAtt = ccsgroup->buildSymmetryGroupAttachment(ccsgroup);
                }
                DefieldSymm* defSymm = symmVec[groupIndex];
                const unsigned num_trans = defSymm->GetNumTransformation();
                const Vector2i groupID = makeVector2i(ni, (int32)groupIndex);

                // update poisson sampling resolution from each symmetry group
                const float radiusSam = sampleth; // for removing neighbor points
                const float radiusDet = poisson_resolution * 1.5; // 2-times of signal rate, for retrieve symmetry supports

                const std::vector<card32>& lines = groupAtt->lines;
                enum LineVisitedMark { UNTOUCHED, VISITED, ELEMENT, IN_ORBIT };
                std::vector<LineVisitedMark> lineVisited(lines.size(), UNTOUCHED);
                for (unsigned fi = 0; fi < lines.size(); ++fi) {
                    if (UNTOUCHED != lineVisited[fi]) continue;
                    std::deque<unsigned> lineIndice;
                    //lineIndice.push_back(fi);
                    sym::FeatureLine* line_i = dynamic_cast<sym::FeatureLine*>(featureVec[lines[fi]]);
                    if (!line_i)
                    {
                        error("Feature line missing");
                        return;
                    }
                    if (norm(line_i->getPosition() - line_i->getEndPosition()) < sampleth)
                    {
                        lineVisited[fi] = VISITED;
                        continue; // do not add short lines
                    }

                    // generate orbit slots for each representing element
                    std::deque<Vector3f> elemPoints = generateRegularSpacePoints(line_i->getPosition(), line_i->getEndPosition(), sampleth);
                    const size_t num_elem = elemPoints.size();
                    if (0 == num_elem) { // prevent sampling small feature lines
                        lineVisited[fi] = VISITED;
                        continue;
                    }

                    // feature line matching
                    std::deque<unsigned> transIndice;
                    for (unsigned tt = 0; tt < num_trans; ++tt) {
                        Matrix4f trans = defSymm->GetTransformation(tt);
                        for (unsigned fj = fi; fj < lines.size(); ++fj) {
                            if (UNTOUCHED != lineVisited[fj]) continue;
                            sym::FeatureLine* line_j = dynamic_cast<sym::FeatureLine*>(featureVec[lines[fj]]);
                            if (!line_j)
                            {
                                error("Feature line missing");
                                return;
                            }
                            if (!line_i->isLineEqual(*line_j, trans, grid_cell_size)) continue;
                            if ((norm(line_i->getPosition() - line_j->getPosition()) < grid_cell_size ||
                                norm(line_i->getEndPosition() - line_j->getEndPosition()) < grid_cell_size ||
                                norm(line_i->getPosition() - line_j->getEndPosition()) < grid_cell_size ||
                                norm(line_j->getPosition() - line_i->getEndPosition()) < grid_cell_size)
                                &&
                                ! line_i->isLineEqual(*line_j, IDENTITY4F, grid_cell_size)
                                ) continue; // remove close line pair
                            lineIndice.push_back(fj);
                            transIndice.push_back(tt);
                            break; // ensure only single match for each transformation
                        }
                    }
                    if (lineIndice.size() > num_trans || transIndice.size() > num_trans) {
                        error("multiple matches for a single feature line by transformation");
                        return;
                    }
                    if (lineIndice.size() < num_trans || transIndice.size() < num_trans) { // possible for self-reflection
                        //error("incomplete matches for a single feature line by transformation");
                        continue; // only export complete line orbit
                    }
                    for (const unsigned li : lineIndice) {
                        lineVisited[li] = IN_ORBIT; // confirmed all other elements in this orbit
                    }
                    lineVisited[fi] = ELEMENT; // candidate for representing element of this orbit

                    // build storage
                    LineSymmBlock* lineBlock = new LineSymmBlock;
                    lineBlock->elem_ = fi;
                    std::copy(lineIndice.begin(), lineIndice.end(), back_inserter(lineBlock->pindx_));
                    const size_t num_lines = lineIndice.size();
                    lineBlock->knots_.resize(num_lines);

                    // generate the full orbit by transforming each point on element line
                    enum TraversalMark { UNTOUCHED, PROCESSING, COVERED };
                    std::deque<TraversalMark> elemChecked(num_elem, UNTOUCHED);
                    for (unsigned epi = 0; epi < num_elem; ++epi) {
                        if (UNTOUCHED != elemChecked[epi]) continue;
                        const Vector3f& currElem = elemPoints[epi];
                        std::deque<Vector3f> pointCandid(num_trans);
                        std::deque<unsigned> pointIndice(num_trans);
                        unsigned curr_tt = 0;
                        for (unsigned tt = 0; tt < num_trans; ++tt) {
                            Matrix4f trans = defSymm->GetTransformation(tt);
                            pointCandid[tt] = transformVector3f(trans, currElem);
                            //debugRenderer->beginRenderJob_OneFrame("debug_", DR_FRAME++);
                            //debugRenderer->addPoint(pointCandid[tt], makeVector3f(1, 0, 0));
                            //debugRenderer->addLine(
                            //    pointCandid[tt], currElem,
                            //    makeVector3f(0, 1, 0),
                            //    makeVector3f(0, 0, 1),
                            //    1);
                            //debugRenderer->endRenderJob();
                            Vector3i idx;
                            if (pointGrid.AddPoint(pointCandid[tt], &idx))
                            {
                                const size_t& numEntries = outPoissonPS.getNumEntries();
                                outPoissonPS.changeHeight(numEntries + 1);
                                outPoissonPS.set3f(numEntries, outPoissonAAT, pointCandid[tt]);
                                outPoissonPS.set2i(numEntries, symGroupAAT, groupID);
                                outPoissonPS.set3f(numEntries, colorAAT, SymmSampler::MapGroupColor(groupID));
                            } else {}
                            const unsigned odx = pointGrid.GetOrdx(idx);
                            pointIndice[tt] = odx;
                            if (curr_tt >= num_lines) continue;
                            if (transIndice[curr_tt] != tt) continue;
                            lineBlock->knots_[curr_tt++].push_back(odx);
                        }
                        //debugRenderer->beginRenderJob_OneFrame("debug_", DR_FRAME++);
                        //for (unsigned tt = 0; tt < num_trans - 1; ++tt) {
                        //    debugRenderer->addLine(
                        //        pointCandid[tt], pointCandid[tt + 1],
                        //        makeVector3f(1, 0, 0),
                        //        makeVector3f(0, 0, 1),
                        //        1);
                        //}
                        //debugRenderer->endRenderJob();
                        { // add description of this orbit
                            defSymm->AddOrbitIndice(pointIndice, pointCandid);
                        }
                        { // add symmetric sampling graph
                            SymmSampleGraph::graph_t orbitGraph;
                            const unsigned numtrans = defSymm->GetNumTransformation();
                            for (unsigned tt = 0; tt < numtrans; ++tt)
                            {
                                const Matrix4f& trans = defSymm->GetTransformation(tt);
                                const Matrix4f& trans_r = trans /** rootRot*/;
                                add_edge(pointIndice[0], pointIndice[tt],
                                    SymmSampleGraph::edge_property_t(groupID, trans, trans_r),
                                    orbitGraph);
                            }
                            symSamGraph->push_back(orbitGraph);
                            //SymmSampleGraph::DrawWithDR(orbitGraph, outPoissonPS, DR_FRAME, true);
                        }
                        { // do not repeatedly adding points on the same orbit
                            for (unsigned rei = 0; rei < num_elem; ++rei) {
                                if (UNTOUCHED != elemChecked[rei]) continue;
                                const Vector3f& restElem = elemPoints[rei];
                                for (unsigned tt = 0; tt < num_trans; ++tt) {
                                    const Vector3f& currCandi = pointCandid[tt];
                                    if (grid_cell_size > norm(restElem - currCandi)) { elemChecked[rei] = COVERED; break; }
                                }
                            }
                        }

                        // check index and transformation
                        {
                            const Vector3f& pos_p = pointCandid[0]; // by candidate position
                            for (unsigned ei = 0; ei < pointCandid.size(); ++ei) {
                                const Matrix4f& trans = defSymm->GetTransformation(ei);
                                const Vector3f& pos_p_t = transformVector3f(trans, pos_p);
                                const Vector3f& pos_ei = pointCandid[ei];
                                if (norm(pos_p_t - pos_ei) > grid_cell_size)
                                {
                                    debugRenderer->beginRenderJob_OneFrame("debug_forward_trans_", DR_FRAME++);
                                    debugRenderer->addLine(
                                        pos_p, pos_p_t,
                                        makeVector3f(1, 0, 0),
                                        makeVector3f(0, 0, 1),
                                        2);
                                    debugRenderer->addLine(
                                        pos_ei, pos_p_t,
                                        makeVector3f(1, 1, 0),
                                        makeVector3f(1, 1, 0),
                                        2);
                                    debugRenderer->addPoint(
                                        pos_ei, makeVector3f(1, 1, 0)
                                        );
                                    debugRenderer->endRenderJob();

                                    error( str( boost::format("SymmSampler::DoSample - representation check failed, forward (%1%, %2%)")
                                        % SymmSampler::GroupCode(groupID) % ei
                                        ));
                                    return;
                                }

                                const Matrix4f& trans_b = defSymm->GetTransformation(-(int)ei);
                                const Vector3f& pos_p_b = transformVector3f(trans_b, pos_p_t);
                                if (norm(pos_p - pos_p_b) > grid_cell_size)
                                {
                                    debugRenderer->beginRenderJob_OneFrame("debug_backward_trans_", DR_FRAME++);
                                    debugRenderer->addPoint(pos_p, makeVector3f(1, 0, 0));
                                    debugRenderer->addPoint(pos_p_t, makeVector3f(0, 1, 0));
                                    debugRenderer->addPoint(pos_p_b, makeVector3f(0, 0, 1));
                                    debugRenderer->addLine(
                                        pos_p, pos_p_t,
                                        makeVector3f(1, 0, 0),
                                        makeVector3f(0, 0, 1),
                                        2);
                                    debugRenderer->addLine(
                                        pos_p_b, pos_p,
                                        makeVector3f(1, 1, 0),
                                        makeVector3f(1, 1, 0),
                                        2);
                                    debugRenderer->addPoint(
                                        pos_p_b, makeVector3f(1, 1, 0)
                                        );
                                    debugRenderer->endRenderJob();

                                    error( str( boost::format("SymmSampler::DoSample - representation check failed, backward (%1%, %2%)")
                                        % SymmSampler::GroupCode(groupID) % -(int)ei
                                        ));
                                    debugOutput << defSymm->GetDescription() << "\n";
                                    return;
                                }
                            }
                        }
                    }
                    lineBlockVec.push_back(lineBlock);

                    //for (unsigned tt = 0; tt < num_lines; ++tt) {
                    //    for (unsigned epi = 0; epi < num_elem; ++epi) {
                    //        debugOutput << lineBlock->knots_[tt][epi] << " ";
                    //    }
                    //    debugOutput << "\n";
                    //}

                    //debugRenderer->beginRenderJob_OneFrame("line_symm_block_", DR_FRAME++);
                    //for (unsigned epi = 0; epi < num_elem; ++epi) {
                    //    for (unsigned tt = 0; tt < num_lines - 1; ++tt) {
                    //        Vector3f pos0 = outPoissonPS.get3f(lineBlock->knots_[tt][epi], outPoissonAAT);
                    //        Vector3f pos1 = outPoissonPS.get3f(lineBlock->knots_[tt+1][epi], outPoissonAAT);
                    //        debugRenderer->addLine(
                    //            pos0, pos1,
                    //            makeVector3f(1, 0, 0),
                    //            makeVector3f(0, 0, 1),
                    //            1);
                    //    }
                    //}
                    //for (unsigned tt = 0; tt < num_lines; ++tt) {
                    //    for (unsigned epi = 0; epi < num_elem - 1; ++epi) {
                    //        Vector3f pos0 = outPoissonPS.get3f(lineBlock->knots_[tt][epi], outPoissonAAT);
                    //        Vector3f pos1 = outPoissonPS.get3f(lineBlock->knots_[tt][epi+1], outPoissonAAT);
                    //        debugRenderer->addLine(
                    //            pos0, pos1,
                    //            makeVector3f(1, 1, 0),
                    //            makeVector3f(0, 1, 1),
                    //            2);
                    //    }
                    //}
                    //debugRenderer->endRenderJob();

                    //debugRenderer->beginRenderJob_OneFrame("line_symm_block_", DR_FRAME++);
                    //{
                    //    sym::FeatureLine* oline = dynamic_cast<sym::FeatureLine*>(featureVec[lines[lineBlock->elem_]]);
                    //    debugRenderer->addLine(
                    //        oline->getPosition(), oline->getEndPosition(),
                    //        makeVector3f(1, 1, 0),
                    //        makeVector3f(0, 1, 1),
                    //        1);
                    //}
                    //for (unsigned li = 0; li < lineBlock->pindx_.size(); ++li) {
                    //    {
                    //        sym::FeatureLine* oline = dynamic_cast<sym::FeatureLine*>(featureVec[lines[lineBlock->pindx_[li]]]);
                    //        debugRenderer->addLine(
                    //            oline->getPosition(), oline->getCenterPosition(),
                    //            makeVector3f(1, 1, 0),
                    //            makeVector3f(0, 1, 1),
                    //            1);
                    //    }
                    //}
                    //debugRenderer->endRenderJob();
                }
            }
        }
    }
}

SymmLineSampler::SymmLineSampler()
{
}
