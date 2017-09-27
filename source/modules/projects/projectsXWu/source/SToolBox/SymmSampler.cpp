#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "SToolBox/SymmSampler.h"
#include "SToolBox/DefieldSymm.h"
#include "SToolBox/DefieldSymmRotation.h"
#include "SToolBox/SamplerSymmBlock.h"
#include "Util/TrimeshUnorderedGrid.h"
#include "Util/ColorSchemer.hpp"
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
#include "GeometricTools.h"

#include "SGlistNode.h"
#include "SGRelativeTimeAnimationNode.h"

#include "PropertyTableProperty.h"
#include "SeparatorClassProperty.h"

#include "SimpleAttachment.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

// success: 0, failed: -1
int GenerateOrbitHdr(
    const DefieldSymm* defSymm,
    const Vector3f& pos, const float& grid_cell_size_sqr, const float& sample_dist_sqr,
    FastSphereQuerryPtr queryDet, const float& radiusDet,
    std::deque<Vector3f>* p_pointCandid,
    const bool& debug_output = false
    ) 
{
    std::deque<Vector3f>& pointCandid = *p_pointCandid;

    const unsigned numtrans = defSymm->GetNumTransformation();
    for (unsigned tt = 0; tt < numtrans; ++tt)
    {
        const Matrix4f& trans = defSymm->GetTransformation(tt);
        const Vector3f& pos_trans = transformVector3f(trans, pos);

        mpcard numAllPoints;
        mpcard *allIndices;
        queryDet->querry(pos_trans, radiusDet, &allIndices, numAllPoints);
        if (0 == numAllPoints) {
            if (debug_output)
            {
                error("GenerateOrbitHdr - symmetry point not supported");
                debugRenderer->beginRenderJob_OneFrame("debug_", DR_FRAME++);
                debugRenderer->addPoint(
                    pos,
                    makeVector3f(1, 0, 0)
                    );
                for (unsigned ii = 0; ii < tt; ++ii) {
                    debugRenderer->addLine(
                        pos, pointCandid[ii],
                        makeVector3f(0, 1, 0),
                        makeVector3f(1, 0, 1),
                        1);
                    debugRenderer->addPoint(
                        pointCandid[ii],
                        makeVector3f(0, 1, 0)
                        );
                }
                debugRenderer->addLine(
                    pos, pos_trans,
                    makeVector3f(0, 1, 1),
                    makeVector3f(0, 0, 1),
                    1);
                debugRenderer->addPoint(
                    pos_trans,
                    makeVector3f(1, 1, 0)
                    );
                debugRenderer->endRenderJob();
            }
            break;
        }

        pointCandid.push_back(pos_trans);
    }
    if (pointCandid.size() != numtrans) return -1;

    float dist_sqr = grid_cell_size_sqr;
    //float dist_sqr = sample_dist_sqr;
    //{ // special case for rotational symmetry class
    //    const DefieldSymmRotation* sym_rot = dynamic_cast<const DefieldSymmRotation*>(defSymm);
    //    if (nullptr != sym_rot) {
    //        const float tdist = normQuad(pointCandid[0] - pointCandid[1]);
    //        if (tdist < sample_dist_sqr) dist_sqr = grid_cell_size_sqr;
    //    }
    //}

    // prevent producing small orbit
    unsigned ti;
    for (ti = 0; ti < numtrans; ++ti) {
        int j = ti - 1;
        while (j >= 0) {
            const float tdist = normQuad(pointCandid[ti] - pointCandid[j]);
            if (tdist < dist_sqr && tdist > 1e-10) { // two points are close, but not the same
                if (debug_output)
                {
                    std::ostringstream ss;
                    ss << "GenerateOrbitHdr - symmetry points closer than grid resolution: "
                        << sqrt(tdist) << " [" << sqrt(dist_sqr) << "]";
                    error(ss.str());

                    debugRenderer->beginRenderJob_OneFrame("debug_", DR_FRAME++);
                    debugRenderer->addPoint(
                        pos,
                        makeVector3f(1, 0, 0)
                        );
                    for (unsigned ii = 0; ii < ti; ++ii) {
                        debugRenderer->addLine(
                            pos, pointCandid[ii],
                            makeVector3f(0, 1, 0),
                            makeVector3f(1, 0, 1),
                            1);
                        debugRenderer->addPoint(
                            pointCandid[ii],
                            makeVector3f(0, 1, 0)
                            );
                    }
                    debugRenderer->addLine(
                        pos, pointCandid[ti],
                        makeVector3f(0, 1, 1),
                        makeVector3f(0, 0, 1),
                        1);
                    debugRenderer->addLine(
                        pointCandid[j], pointCandid[ti],
                        makeVector3f(1, 1, 0),
                        makeVector3f(1, 1, 0),
                        1);
                    debugRenderer->addPoint(
                        pointCandid[ti],
                        makeVector3f(1, 1, 0)
                        );
                    debugRenderer->endRenderJob();
                }
                break;
            }
            --j;
        }
        if (j >= 0) break;
    }
    if (ti != numtrans) return -1;

    return 0;
}

// failed: -1, otherwise number of samples added without overlapping
int ConstructOrbitHdr(
    DefieldSymm* defSymm, const Vector2i& groupIndex, PointSet* p_outPoissonPS,
    const std::deque<Vector3f>& pointCandid, std::deque<unsigned>* p_pointIndice,
    UnorderedGridUniq* p_pointGrid,
    const bool& debug_output = false
    )
{
    std::deque<unsigned>& pointIndice = *p_pointIndice;
    UnorderedGridUniq& pointGrid = *p_pointGrid;

    const unsigned numtrans = pointCandid.size();
    if (defSymm->GetNumTransformation() != numtrans) {
        debugOutput << "ConstructOrbitHdr - orbit construction error" << "\n";
        return -1;
    }

    PointSet& outPoissonPS = *p_outPoissonPS;
    const AAT& outPoissonAAT = outPoissonPS.getAAT("position");
    const AAT& symGroupAAT = outPoissonPS.getAAT("group");
    const AAT& colorAAT = outPoissonPS.getAAT("color");

    int num_sample = 0;
    for (unsigned tt = 0; tt < numtrans; ++tt)
    {
        Vector3i idx; Vector3f posUpdated;
        if (pointGrid.AddPoint(pointCandid[tt], &idx, &posUpdated)) {
            const size_t& numEntries = outPoissonPS.getNumEntries();
            outPoissonPS.changeHeight(numEntries + 1);
            outPoissonPS.set3f(numEntries, outPoissonAAT, pointCandid[tt]);
            outPoissonPS.set2i(numEntries, symGroupAAT, groupIndex);
            outPoissonPS.set3f(numEntries, colorAAT, SymmSampler::MapGroupColor(groupIndex));
            ++num_sample;
        }
        else { // update position by taking average
            const unsigned odx = pointGrid.GetOrdx(idx);
            outPoissonPS.set3f(odx, outPoissonAAT, posUpdated);
        }
        const unsigned odx = pointGrid.GetOrdx(idx);
        pointIndice.push_back(odx);
    }

    // check index and transformation
    {
        const Vector3f& pos_p = pointCandid[0]; // by candidate position
        for (unsigned ei = 0; ei < pointCandid.size(); ++ei) {
            const Matrix4f& trans = defSymm->GetTransformation(ei);
            const Vector3f& pos_p_t = transformVector3f(trans, pos_p);
            const Vector3f& pos_ei = pointCandid[ei];
            if (normQuad(pos_p_t - pos_ei) > 1e-8)
            {
                //if (debug_output)
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
                        % SymmSampler::GroupCode(groupIndex) % ei
                        ));
                }
                return -1;
            }

            const Matrix4f& trans_b = defSymm->GetTransformation(-(int)ei);
            const Vector3f& pos_p_b = transformVector3f(trans_b, pos_p_t);
            if (normQuad(pos_p - pos_p_b) > 1e-8)
            {
                //if (debug_output)
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
                        % SymmSampler::GroupCode(groupIndex) % -(int)ei
                        ));
                    debugOutput << defSymm->GetDescription() << "\n";
                }
                return -1;
            }
        }
    }

    { // add description of this orbit
        defSymm->AddOrbitIndice(pointIndice, pointCandid);
    }

    return num_sample;
}

//void buildStorage(
//    const std::deque<Vector3f>& pointCandid, const std::deque<unsigned>& pointIndice,
//    const unsigned& groupIndex,
//    std::vector<PointSymmBlock*>* pointBlockVec
//    )
//{
//    PointSymmBlock* pointBlock = new PointSymmBlock;
//    pointBlock->elem_ = pointIndice[0];
//    for (unsigned ii = 0; ii < pointIndice.size(); ++ii)
//    {
//        pointBlock->pindx_.push_back(pointIndice[ii]);
//    }
//    std::deque<PointSymmBlock::PointSymmIndex>& psindexvec = pointBlock->knots_;
//    for (unsigned ei = 0; ei < pointIndice.size(); ++ei) {
//        PointSymmBlock::PointSymmIndex psindex;
//        psindex.symm_ = groupIndex;
//        psindex.ordx_ = ei;
//        psindexvec.push_back(psindex);
//    }
//    (*pointBlockVec).push_back(pointBlock);
//}

// failed: -1, otherwise number of samples added without overlapping
int GenerateConstructOrbit(
    const Vector3f& basePos,
    const float& grid_cell_size_sqr, const Vector2i& GROUP_ID,
    DefieldSymm* defSymm, std::deque<int>* p_markGroup,
    FastSphereQuerryPtr queryDet, const float& radiusDet,
    FastSphereQuerryPtr querySam, const float& radiusSam,
    std::deque<Vector3f>* p_pointCandid, std::deque<unsigned>* p_pointIndice,
    PointSet* p_outPoissonPS, UnorderedGridUniq* p_pointGrid,
    const bool& debug_output = false
    )
{
    std::deque<Vector3f>& pointCandid = *p_pointCandid;
    std::deque<unsigned>& pointIndice = *p_pointIndice; // corresponding to sampled points, or porint grid
    std::deque<int>& markGroup = *p_markGroup;
    UnorderedGridUniq& pointGrid = *p_pointGrid;

    PointSet& outPoissonPS = *p_outPoissonPS;
    const AAT& outPoissonAAT = outPoissonPS.getAAT("position");
    const AAT& symGroupAAT = outPoissonPS.getAAT("group");
    const AAT& colorAAT = outPoissonPS.getAAT("color");

    int res;
    // generate orbit points
    res = GenerateOrbitHdr(
        defSymm,
        basePos, grid_cell_size_sqr, radiusSam * radiusSam,
        queryDet, radiusDet,
        &pointCandid,
        debug_output
        );
    if (0 > res) return res;

    // construct symmetry orbits
    res = ConstructOrbitHdr(defSymm, GROUP_ID, p_outPoissonPS, pointCandid, &pointIndice, &pointGrid, debug_output);
    if (0 > res) return res;

    //// mark processed points, within sampling radius
    //for (unsigned ci = 0; ci < pointCandid.size(); ++ci)
    //{
    //    const Vector3f& pos_ci = outPoissonPS.get3f(pointIndice[ci], outPoissonAAT);
    //    if (normQuad(pos_ci - pointCandid[ci]) > 1e-6) // depends on grid resolution
    //    {
    //        //if (debug_output)
    //        {
    //            debugRenderer->beginRenderJob_OneFrame("debug_orbit_check_", DR_FRAME++);
    //            debugRenderer->addLine(
    //                pos_ci, pointCandid[ci],
    //                makeVector3f(0, 1, 0),
    //                makeVector3f(0, 0, 1),
    //                2);
    //            debugRenderer->addPoint(
    //                pos_ci, makeVector3f(1, 0, 0)
    //                );
    //            for (unsigned ii = 0; ii < ci; ++ii) {
    //                debugRenderer->addPoint(
    //                    pointCandid[ii], makeVector3f(1, 1, 0)
    //                    );
    //            }
    //            debugRenderer->endRenderJob();
    //            std::ostringstream ss;
    //            ss << "SymmSampler::DoSample - point position not matching index, with error: "
    //                << norm(pos_ci - pointCandid[ci]);
    //            error(ss.str());
    //        }
    //        //return -1; // small error is possible, due to grid resolution and taking average of overlapping points
    //    }

    //    mpcard numAllPoints;
    //    mpcard *allIndices;
    //    querySam->querry(pointCandid[ci], radiusSam, &allIndices, numAllPoints);
    //    for (size_t qi = 0; qi < numAllPoints; ++qi) {
    //        const mpcard& symIndex = symPCInfo.symmetryPoints[allIndices[qi]];
    //        markGroup[allIndices[qi]] = GROUP_ID;
    //    }
    //}

    return res;
}

struct SymPCInfo
{
    UICPC* symNodePC;
    UICPC* workShape;
    DefieldSymm* defSymm;
    //std::deque<int> markGroup;
    std::vector<mpcard> symmetryPoints;
    FastSphereQuerryPtr querySamDet;
    Vector2i groupID;
};

int GenerateConstructOrbitRecursive(
    const Vector3f& basePos, const unsigned& groupStart,
    std::deque<SymPCInfo>* p_symPCInfoVec,
    std::deque<int>* p_markGroup,
    const float& grid_cell_size_sqr,
    const float& radiusDet, const float& radiusSam,
    PointSet* p_outPoissonPS, UnorderedGridUniq* p_pointGrid,
    SymmSampleGraph::graph_t* p_orbitGraph,
    unsigned* rootId, const Matrix4f& rootTrans,
    const bool& debug_output = false
    )
{
    std::deque<SymPCInfo>& symPCInfoVec = *p_symPCInfoVec;
    std::deque<int>& markGroup = *p_markGroup;
    SymmSampleGraph::graph_t& orbitGraph = *p_orbitGraph;

    int num_sample = 0;

    size_t num_group =  symPCInfoVec.size();
    for (size_t groupIndex = groupStart; groupIndex < num_group; ++groupIndex)
    {
        // for recursively checking, base point may not supported
        SymPCInfo& symPCInfo = symPCInfoVec[groupIndex];
        {
            mpcard numAllPoints; mpcard *allIndices;
            symPCInfo.querySamDet->querry(basePos, radiusDet, &allIndices, numAllPoints);
            if (0 == numAllPoints) continue;
            const mpcard& symIndex = symPCInfo.symmetryPoints[allIndices[0]];
            if (SymmSampler::GroupCode(symPCInfo.groupID) <= markGroup[symIndex]) continue; // already sampled from finer level group
        }

        std::deque<Vector3f> pointCandid;
        std::deque<unsigned> pointIndice; // corresponding to sampled points, or porint grid
        int res = GenerateConstructOrbit(
            basePos,
            grid_cell_size_sqr, symPCInfo.groupID,
            symPCInfo.defSymm, &markGroup,
            symPCInfo.querySamDet, radiusDet,
            symPCInfo.querySamDet, radiusSam,
            &pointCandid, &pointIndice,
            p_outPoissonPS, p_pointGrid,
            debug_output
            );
        if (0 > res) {
            //if (debug_output)
            {
                //for (unsigned ii = 0; ii < groupStart+1; ++ii) debugOutput << ".";
                //debugOutput << "failed to generate the orbit, from group " << symPCInfo.groupID << "\n";
            }
            continue;
        } else {
            //if (debug_output)
            {
                //for (unsigned ii = 0; ii < groupStart+1; ++ii) debugOutput << ".";
                //debugOutput << "generate an orbit of " << pointCandid.size() << " points, from group " << symPCInfo.groupID << "\n";
            }
            num_sample += pointCandid.size(); // count total number of samples
        }

        // mark processed points, within sampling radius
        for (unsigned ci = 0; ci < pointCandid.size(); ++ci)
        {
            mpcard numAllPoints; mpcard *allIndices;
            symPCInfo.querySamDet->querry(pointCandid[ci], radiusSam, &allIndices, numAllPoints);
            for (size_t qi = 0; qi < numAllPoints; ++qi) {
                const mpcard& symIndex = symPCInfo.symmetryPoints[allIndices[qi]];
                markGroup[symIndex] = SymmSampler::GroupCode(symPCInfo.groupID);
            }
        }

        // update orbit graph
        const unsigned numtrans = symPCInfo.defSymm->GetNumTransformation();
        std::deque<Matrix4f> rootTransVec;
        for (unsigned tt = 0; tt < numtrans; ++tt)
        {
            const Matrix4f& trans = symPCInfo.defSymm->GetTransformation(tt);
            const Matrix4f& trans_r = trans * rootTrans;
            add_edge(pointIndice[0], pointIndice[tt],
                SymmSampleGraph::edge_property_t(symPCInfo.groupID, trans, trans_r),
                orbitGraph);
            orbitGraph[pointIndice[tt]].groupId = symPCInfo.groupID; // group assignment for target, help dis-ambiguity of overlapping vertex
            rootTransVec.push_back(trans_r);
        }
        *rootId = pointIndice[0]; // set to the first point for constructing this orbit, will assign to graph *after* sampling

        //SymmSampleGraph::DrawWithDR(orbitGraph, *p_outPoissonPS, DR_FRAME, false);
        //debugOutput << "draw orbit from group " << symPCInfo.groupID << ", started from " << groupStart << "\n";
        //{
        //    debugRenderer->beginRenderJob_OneFrame("debug_", DR_FRAME++);
        //    debugRenderer->addPoint(
        //        basePos,
        //        makeVector3f(1, 0, 0)
        //        );
        //    for (unsigned ii = 0; ii < pointCandid.size(); ++ii) {
        //        debugRenderer->addPoint(
        //            pointCandid[ii],
        //            makeVector3f(1, 1, 0)
        //            );
        //        debugRenderer->addLine(
        //            basePos, pointCandid[ii],
        //            makeVector3f(0, 1, 0),
        //            makeVector3f(0, 0, 1),
        //            1);
        //    }
        //    debugRenderer->endRenderJob();
        //}

        // recursively sampling for next group
        SymmSampleGraph::graph_t orbitGraphNext;
        unsigned rootIdNext;
        int num_sample_next = 0;
        {
            for (unsigned tt = 0; tt < numtrans; ++tt)
            {
                const Vector3f& pos = pointCandid[tt];
                num_sample_next += GenerateConstructOrbitRecursive(
                    pos, groupStart + 1,
                    p_symPCInfoVec,
                    &markGroup,
                    grid_cell_size_sqr,
                    radiusDet, radiusSam,
                    p_outPoissonPS, p_pointGrid,
                    &orbitGraphNext,
                    &rootIdNext, rootTransVec[tt],
                    true
                    );
            }
            boost::graph_traits<SymmSampleGraph::graph_t>::edge_iterator eit, eit_end;
            for (boost::tie(eit, eit_end) = edges(orbitGraphNext); eit != eit_end; ++eit) {
                add_edge(boost::source(*eit, orbitGraphNext), boost::target(*eit, orbitGraphNext),
                    orbitGraphNext[*eit],
                    orbitGraph);
                orbitGraph[boost::target(*eit, orbitGraphNext)].groupId = orbitGraphNext[boost::target(*eit, orbitGraphNext)].groupId; // group assignment for target
            }

            //if (0 < num_sample_next)
            //{
            //    SymmSampleGraph::DrawWithDR(orbitGraph, *p_outPoissonPS, DR_FRAME, false);
            //    debugOutput << "draw orbit from group " << symPCInfo.groupID << ", recursive back\n";
            //}
        }
        //if (0 == num_sample_next) // update symmetric partition assignment at finest level (this level)
        //{
        //    UICPC* workPoints = symPCInfo.workShape;
        //    PointSet& workPointsPS = *workPoints->getPointSet();
        //    const AAT& symGroupAAT = workPointsPS.getAAT("group");
        //    const AAT& colorAAT = workPointsPS.getAAT("color");
        //    const Vector3f& groupColor = SymmSampler::MapGroupColor(symPCInfo.groupID);
        //    FastSphereQuerryPtr queryDet( new FastSphereQuerry(workPoints));
        //    for (unsigned tt = 0; tt < numtrans; ++tt)
        //    {
        //        const Vector3f& pos = pointCandid[tt];
        //        mpcard numAllPoints; mpcard *allIndices;
        //        queryDet->querry(pos, radiusSam, &allIndices, numAllPoints); // 2-times of signal rate
        //        for (mpcard qi = 0; qi < numAllPoints; ++qi) {
        //            const size_t& index = allIndices[qi];
        //            workPointsPS.set2i(index, symGroupAAT, symPCInfo.groupID);
        //            workPointsPS.set3f(index, colorAAT, groupColor);
        //        }
        //    }
        //}
        if (0 == num_sample_next) // update sample point group assignment at finest level (this level)
        {
            PointSet& outPoissonPS = *p_outPoissonPS;
            const AAT& symGroupAAT = outPoissonPS.getAAT("group");
            const AAT& colorAAT = outPoissonPS.getAAT("color");

            for (unsigned tt = 0; tt < numtrans; ++tt)
            {
                const unsigned& index = pointIndice[tt];
                outPoissonPS.set2i(index, symGroupAAT, symPCInfo.groupID);
                outPoissonPS.set3f(index, colorAAT, SymmSampler::MapGroupColor(symPCInfo.groupID));
            }
        }

        //// mark invalid for all previous overlapping groups
        //{
        //    PointSet& outPoissonPS = *p_outPoissonPS;
        //    const AAT& outPoissonAAT = outPoissonPS.getAAT("position");
        //    typedef boost::property_map<SymmSampleGraph::graph_t, boost::vertex_index_t>::type IndexMap;
        //    IndexMap indMap = boost::get(boost::vertex_index, orbitGraphNext);
        //    boost::graph_traits<SymmSampleGraph::graph_t>::vertex_iterator vit, vit_end;
        //    for (boost::tie(vit, vit_end) = vertices(orbitGraphNext); vit != vit_end; ++vit)
        //    {
        //        const Vector3f& pos = outPoissonPS.get3f(indMap[*vit], outPoissonAAT);
        //        mpcard numAllPoints;
        //        mpcard *allIndices;
        //        symPCInfo.querySamDet->querry(pos, radiusSam, &allIndices, numAllPoints);
        //        for (size_t qi = 0; qi < numAllPoints; ++qi) {
        //            const mpcard& symIndex = symPCInfo.symmetryPoints[allIndices[qi]];
        //            markGroup[symIndex] = symPCInfo.groupID;
        //        }
        //    }
        //}
    }

    return num_sample;
}

void SymmSampler::DoSample(
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

    SymmSampleGraph* symSamGraphAtt = dynamic_cast<SymmSampleGraph*>(
        outPoisson->getAttachments()->getData(SymmSampleGraph::getDefaultName()));
    if (!symSamGraphAtt) {
        error("SymmSpaceSolver::Initialize() - SymmSampleGraph missing.");
        return;
    }

    const unsigned numParts = dataList->getNumChildNodes(0);
    for (unsigned partIndex = 0; partIndex < numParts; ++partIndex) {
        SGObjectNode* partNode = dynamic_cast<SGObjectNode*>(dataList->getChildNode(0, partIndex));
        const std::string& partName = partNode->getName();
        const std::string& symmWorkName = symmWorkNameBase + partName;
        const std::string& groupFolder = symmGroupNameBase + partName;

        UICPC* workPoints = dynamic_cast<UICPC*>(getPointCloud(scene, symmWorkName));
        if (!workPoints)
        {
            error("Symmetry work points not found.");
            return;
        }

        DefieldSymmVec* defSymmVecAtt = dynamic_cast<DefieldSymmVec*>(
            workPoints->getAttachments()->getData(DefieldSymmVec::getDefaultName()));
        if (!defSymmVecAtt) {
            error("Defield symmetry vector not found.");
            return;
        }
        std::deque<DefieldSymm*>& symmVec = defSymmVecAtt->symmvec_;

        SGRelativeTimeAnimationNode* groupList = dynamic_cast<SGRelativeTimeAnimationNode*>(
            getSceneGraphNodeByName(scene, groupFolder));
        if (!groupList) {
            std::stringstream ss;
            ss << "SymmSampler::DoSample - groug folder " << partIndex << " not found";
            error(ss.str());
            return;
        }

        const float poisson_resolution = getCachedMedianPointDistance(workPoints);
        const float grid_cell_size_sqr =  pointGrid.cell_size() * pointGrid.cell_size();
        const float radiusSam = sampleth; // for removing neighbor points
        const float radiusDet = poisson_resolution * 1.5; // 2-times of signal rate, for retrieve symmetry supports

        std::deque<SymPCInfo> symPCInfoVec;
        std::deque<int> markGroup(workPoints->getNumPoints(), SymmSampler::UNTOUCHED); // global mark for this group
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
                error("Group not found");
                return;
            }
            DefieldSymm* defSymm = symmVec[groupIndex];
            const std::vector<mpcard>& symmetryPoints = groupAtt->symmetryPoints;

            SymPCInfo symPCInfo;
            symPCInfo.symNodePC = symNodePC;
            symPCInfo.workShape = workPoints;
            symPCInfo.defSymm = defSymm;
            symPCInfo.symmetryPoints = symmetryPoints;
            symPCInfo.querySamDet.reset( new FastSphereQuerry(symNodePC) );
            symPCInfo.groupID = makeVector2i(partIndex, (int32)groupIndex);

            // this should use a copy of inititial sample PC
            ////debugRenderer->beginRenderJob_OneFrame("debug_", DR_FRAME++);
            //for (unsigned kk = 0; kk < outPoissonPS.getNumEntries(); ++kk) {
            //    const int& groupID = outPoissonPS.get1i(kk, symGroupAAT);
            //    if (partIndex != SymmSampler::PartIndex(groupID)) continue;
            //    const Vector3f& pos = outPoissonPS.get3f(kk, outPoissonAAT);
            //    { //invalidate all existing samples' neighbors, i.e. from feature line sampling
            //        mpcard numAllPoints; mpcard *allIndices;
            //        symPCInfo.querySamDet->querry(pos, radiusSam, &allIndices, numAllPoints);
            //        for (size_t qi = 0; qi < numAllPoints; ++qi) {
            //            symPCInfo.markGroup[allIndices[qi]] = SymmSampler::PRE_ALLOCATED;
            //        }
            //        //{
            //        //    debugRenderer->addPoint(
            //        //        pos,
            //        //        makeVector3f(1, 0, 0)
            //        //        );
            //        //    for (unsigned ii = 0; ii < numAllPoints; ++ii) {
            //        //        debugRenderer->addPoint(
            //        //            symNodePC->getPointSet()->get3f(allIndices[ii], symNodePC->getPointSet()->getAAT("position")),
            //        //            makeVector3f(0, 1, 0)
            //        //            );
            //        //    }
            //        //}
            //    }
            //}
            ////debugRenderer->endRenderJob();

            symPCInfoVec.push_back(symPCInfo);
        }

        // do sampling for each group seperately, check possible conflict with global flag
        for (size_t groupIndex = 0; groupIndex < num_group; ++groupIndex)
        {
            SymPCInfo& symPCInfo = symPCInfoVec[groupIndex];
            const PointSet& symNodePS = *symPCInfo.symNodePC->getPointSet();
            const AAT& symNodePosAAT = symNodePS.getAAT("position");

            const unsigned num_candidate = symPCInfo.symNodePC->getNumPoints();
            for (unsigned ci = 0; ci < num_candidate; ++ci)
            {
                const mpcard& symIndex = symPCInfo.symmetryPoints[ci];
                if (SymmSampler::UNTOUCHED != markGroup[symIndex]) continue;

                //debugOutput << "generated a seed point\n";
                Vector3f basePos = symNodePS.get3f(ci, symNodePosAAT);
                SymmSampleGraph::graph_t orbitGraph;
                unsigned rootId;
                int num_sample = GenerateConstructOrbitRecursive(
                    basePos, groupIndex,
                    &symPCInfoVec,
                    &markGroup,
                    grid_cell_size_sqr,
                    radiusDet, radiusSam,
                    &outPoissonPS, p_pointGrid,
                    &orbitGraph,
                    &rootId, IDENTITY4F,
                    false
                    );

                // check if has successfully sampled from current seed
                if (0 == num_sample) continue;
                orbitGraph[boost::graph_bundle].rootId = rootId;
                symSamGraphAtt->push_back(orbitGraph);

                //SymmSampleGraph::DrawWithDR(orbitGraph, outPoissonPS, DR_FRAME, true);
                //{
                //    debugRenderer->beginRenderJob_OneFrame("point_remove_", DR_FRAME++);
                //    for (unsigned ci = 0; ci < outPoissonPS.getNumEntries(); ++ci) {
                //        debugRenderer->addPoint(
                //            outPoissonPS.get3f(ci, outPoissonAAT),
                //            makeVector3f(1, 0, 0)
                //            );
                //    }
                //    for (unsigned ci = 0; ci < num_candidate; ++ci) {
                //        const mpcard& symIndex = symPCInfo.symmetryPoints[ci];
                //        if (symPCInfo.groupID != markGroup[symIndex]) continue;
                //        debugRenderer->addPoint(
                //            symNodePS.get3f(ci, symNodePosAAT),
                //            makeVector3f(0, 1, 0)
                //            );
                //    }
                //    debugRenderer->endRenderJob();
                //}
            }
        }

        //debugRenderer->beginRenderJob_OneFrame("grid_cell_check_", 1);
        //if (pointGrid.CheckDrawAdjacency())
        //{
        //    pointGrid.DrawWithDR();
        //    warning("SymmSampler::DoSample - adjacent grid cell found, better to increase grid resolution");
        //}
        //debugRenderer->endRenderJob();
    }
}

SymmSampler::SymmSampler()
{
    //GenerateGroupHandlerMap["Rotation"] = &GenerateOrbitHdr;
    //GenerateGroupHandlerMap["Reflection"] = &GenerateOrbitHdr;
    //GenerateGroupHandlerMap["Dihedral"] = &GenerateOrbitHdr;
    //GenerateGroupHandlerMap["DihedralH"] = &GenerateOrbitHdr;

    //ConstructGroupHandlerMap["Rotation"] = &ConstructOrbitHdr;
    //ConstructGroupHandlerMap["Reflection"] = &ConstructOrbitHdr;
    //ConstructGroupHandlerMap["Dihedral"] = &ConstructOrbitHdr;
    //ConstructGroupHandlerMap["DihedralH"] = &ConstructOrbitHdr;
}

int32 SymmSampler::GroupCode(const Vector2i& groupId)
{
    return groupId[0] * 100 + groupId[1];
}

Vector3f SymmSampler::MapGroupColor(const int32& groupId)
{
    if (0 <= groupId)
    {
        float sat = 0.8f, val = 0.8f, hue = 360 * (float)(groupId % 17) / 17;
        float r, g, b;
        ColorSchemer::HSVtoRGB(&r, &g, &b, hue, sat, val);
        return makeVector3f(r, g, b);
        //srandom(groupId % 17);
        //return makeVector3f(rnd01(),rnd01(),rnd01());
    }

    if (UNTOUCHED == groupId) return makeVector3f(0, 1, 1);
    if (FEATURE_LINE == groupId) return makeVector3f(1, 0, 1);
    if (FILL_IN == groupId) return makeVector3f(0, 0, 1);

    error("SymmSampler::MapGroupColor - not recognize");
    return NULL_VECTOR3F;
}

Vector3f SymmSampler::MapGroupColor(const Vector2i& groupId)
{
    return SymmSampler::MapGroupColor(SymmSampler::GroupCode(groupId));
}

int SymmSampler::PartIndex(const int groupID)
{
    return (int)(groupID / (int)SymmSampler::PART_SEP);
}

int SymmSampler::GroupSN(const int groupID)
{
    return (int)(groupID % (int)SymmSampler::PART_SEP);
}
