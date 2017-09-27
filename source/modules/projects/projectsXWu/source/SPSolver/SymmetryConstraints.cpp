//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "SymmetryConstraints.h"
#include "SToolBox/DefieldSymm.h"
#include "SToolBox/DefieldSymmRotation.h"
#include "SToolBox/DefieldSymmDihedral.h"
#include "SToolBox/SamplerSymmBlock.h"
//----------------------------------------------------------------------
//----------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

SymmetryConstraints::SymmetryConstraints()
{
}

//int SymmetryConstraints::BuildSymmetryConstraints(
//    SceneObject* samobj,
//    SymmSpaceSolver::Ptr receiver,
//    const bool& bAddFixedRigion,
//    const float& symmetryWeight
//    )
//{
//    const bool debug_output = false;
//
//    UICPC* samplePC = dynamic_cast<UICPC*>(samobj);
//    if (!samplePC) {
//        error("SymmetryConstraints::BuildSymmetryConstraints() - Type mismatch.");
//        return -1;
//    }
//
//    DefieldSymmVec* symmVecAtt = dynamic_cast<DefieldSymmVec*>(
//        samplePC->getAttachments()->getData(DefieldSymmVec::getDefaultName()));
//    if (!symmVecAtt) {
//        error("SymmetryConstraints::BuildSymmetryConstraints() - DefieldSymmVec missing.");
//        return -1;
//    }
//    const std::deque<DefieldSymm*>& symmVec = symmVecAtt->symmvec_;
//
//    for (unsigned si = 0; si < symmVec.size(); ++si) {
//        std::set<unsigned> fixedRigion;
//        if (bAddFixedRigion)
//        {
//            if (debug_output) debugRenderer->beginRenderJob_OneFrame("symmetry_constraint_fixed_", DebugRenderer::DR_FRAME++);
//            DefieldSymmRotation* symm_rot = dynamic_cast<DefieldSymmRotation*>(symmVec[si]);
//            DefieldSymmDihedral* symm_dih = dynamic_cast<DefieldSymmDihedral*>(symmVec[si]);
//            if (symm_rot && !symm_dih) {
//                //symm_rot->Sort(samplePC->getPointSet()); // already sorted at solver initialization
//                for (unsigned vi = 0; vi < symm_rot->fixedRigion.size(); ++vi) {
//                    receiver->addFixedConstraint(symm_rot->fixedRigion[vi], symmetryWeight * 10);
//                }
//
//                std::copy(symm_rot->fixedRigion.begin(), symm_rot->fixedRigion.end(), std::inserter(fixedRigion, fixedRigion.end()));
//            }
//            if (debug_output) debugRenderer->endRenderJob();
//        }
//
//        if (debug_output) debugRenderer->beginRenderJob_OneFrame("symmetry_constraint_", DebugRenderer::DR_FRAME++);
//        const std::deque<OrbitPairTransformation> orbitPairs = symmVec[si]->GetOrbitPairTransformations();
//        for (unsigned orx = 0; orx < orbitPairs.size(); ++orx) {
//            const OrbitPairTransformation& orbit = orbitPairs[orx];
//            const std::deque<unsigned>& source = orbit.source;
//            const std::deque<unsigned>& target = orbit.target;
//            const Matrix4f& transformation = orbit.transformation;
//            const unsigned num_v = source.size();
//            if (num_v != target.size()) {
//                error("SymmetryConstraints::BuildSymmetryConstraints - orbit pair inconsistence");
//            }
//
//            bool bInFix = false; // no additional symmetry constraints for fixed region
//            for (unsigned vi = 0; vi < num_v; ++vi) {
//                if (fixedRigion.find(source[vi]) != fixedRigion.end()) { bInFix = true; break;}
//            }
//            if (bInFix) continue;
//
//            for (unsigned vi = 0; vi < num_v; ++vi) {
//                receiver->addSymmetryConstraint(
//                    source[vi], target[vi],
//                    transformation,
//                    symmetryWeight
//                    );
//            }
//        }
//        if (debug_output) debugRenderer->endRenderJob();
//    }
//
//    return 0;
//}

int SymmetryConstraints::BuildSymmetryConstraints(
    SceneObject* samobj,
    SymmSpaceSolver::Ptr receiver,
    const bool& bAddFixedRigion,
    const float& symmetryWeight
    )
{
    const bool debug_output = false;

    UICPC* samplePC = dynamic_cast<UICPC*>(samobj);
    if (!samplePC) {
        error("SymmetryConstraints::BuildSymmetryConstraints() - Type mismatch.");
        return -1;
    }

    SymmSampleGraph* symSamGraph = dynamic_cast<SymmSampleGraph*>(
        samplePC->getAttachments()->getData(SymmSampleGraph::getDefaultName()));
    if (!symSamGraph) {
        error("SymmSpaceSolver::Initialize() - SymmSampleGraph missing.");
        return -1;
    }

    typedef boost::property_map<SymmSampleGraph::graph_t, boost::vertex_index_t>::type IndexMap;
    const std::deque< SymmSampleGraph::graph_t >& orbitGraphVec = symSamGraph->graphVec;
    const unsigned& num_orbit = orbitGraphVec.size();
    for (unsigned oi = 0; oi < num_orbit; ++oi) {
        if (debug_output) debugRenderer->beginRenderJob_OneFrame("symmetry_constraint_", DebugRenderer::DR_FRAME++);
        unsigned num_entry = 0;
        std::set<unsigned> vertSet;
        const SymmSampleGraph::graph_t& orbitGraph = orbitGraphVec[oi];
        IndexMap indMap = boost::get(boost::vertex_index, orbitGraph);
        const unsigned rootId = orbitGraph[boost::graph_bundle].rootId;
        boost::graph_traits<SymmSampleGraph::graph_t>::edge_iterator eit, eit_end;
        for (boost::tie(eit, eit_end) = edges(orbitGraph); eit != eit_end; ++eit)
        {
            vertSet.insert(indMap[boost::source(*eit, orbitGraph)]);
            vertSet.insert(indMap[boost::target(*eit, orbitGraph)]);
            const unsigned& index = indMap[boost::target(*eit, orbitGraph)];
            if (index == rootId) continue;
            if (orbitGraph[*eit].groupId != orbitGraph[index].groupId) continue; // trick to add only one const for each overlapping vertex
            const Matrix4f& transformation = orbitGraph[*eit].rootTrans;
            receiver->addSymmetryConstraint(
                rootId, index,
                transformation,
                symmetryWeight
                );
            if (debug_output) debugOutput << rootId << " -> " << index << "\n";
            ++num_entry;
        }
        if (debug_output) debugOutput << num_entry << " constraints added on orbit " << oi << "\n";
        if (debug_output) debugRenderer->endRenderJob();
        //if (vertSet.size() - 1 != num_entry) {
        //    std::ostringstream ss;
        //    ss << "SymmetryConstraints::BuildSymmetryConstraints - ";
        //    ss << "[" << oi << "]: " << vertSet.size() << " vertices, " << num_entry << " constraints";
        //    error(ss.str());
        //    SymmSampleGraph::DrawWithDR(orbitGraph, *samplePC->getPointSet(), DR_FRAME, true);
        //}
    }

    return 0;
}
