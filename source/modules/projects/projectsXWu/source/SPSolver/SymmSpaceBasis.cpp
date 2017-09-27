//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "SPSolver/SymmSpaceBasis.h"
#include "SToolBox/DefieldSymm.h"
//----------------------------------------------------------------------
//----------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

//int SymmSpaceBasis::BuildBasis(
//    SceneObject* samobj,
//    std::deque<SparseMatrixD>& basisVec
//    )
//{
//    UICPC* samPC = dynamic_cast<UICPC*>(samobj);
//    if (!samPC) {
//        error("SymmSpaceBasis::BuildBasis() - Type mismatch.");
//        return -1;
//    }
//
//    DefieldSymmVec* symmVecAtt = dynamic_cast<DefieldSymmVec*>(
//        samPC->getAttachments()->getData(DefieldSymmVec::getDefaultName()));
//    if (!symmVecAtt) {
//        error("SymmSpaceBasis::BuildBasis() - DefieldSymmVec missing.");
//        return -1;
//    }
//    const std::vector<DefieldSymm*>& symmVec = symmVecAtt->symmvec_;
//
//    std::vector<Vector3f> canonical_basis(3);
//    canonical_basis[0] = XAXIS_VECTOR3F;
//    canonical_basis[1] = YAXIS_VECTOR3F;
//    canonical_basis[2] = ZAXIS_VECTOR3F;
//
//    for (size_t gi = 0; gi < symmVec.size(); ++gi)
//    {
//        DefieldSymm* defSymm = symmVec[gi];
//        const size_t& num_trans = defSymm->GetNumTransformation();
//        const DefieldSymm::OrbitVerticeMap& overt = defSymm->GetOrbitVerticeMap();
//        if (num_trans != overt.size()) {
//            error("SymmSpaceBasis::BuildBasis - orbit size mis-match");
//            return -1;
//        }
//
//        const unsigned& num_orbit = overt.begin()->second.size();
//        SparseMatrixD basis(num_orbit * 3); // for x, y, z; each row is a basis for one dimension
//        for (DefieldSymm::OrbitVerticeMap::const_iterator itMap = overt.begin(); itMap != overt.end(); ++itMap) {
//            const std::deque< card32 >& list = itMap->second;
//            if (num_orbit != list.size()) {
//                error("SymmSpaceBasis::BuildBasis: orbit vertice mis-match");
//                return -1;
//            }
//            const Matrix4f& trans = defSymm->GetTransformation(itMap->first);
//            for (unsigned rr = 0; rr < num_orbit; ++rr) {
//                const unsigned& index = list[rr];
//                for (unsigned bi = 0; bi < canonical_basis.size(); ++bi) {
//                    const Vector3f& axis = transformVector3f(trans, canonical_basis[bi]);
//                    for (unsigned ai = 0; ai < axis.size(); ++ai) {
//                        basis[3 * rr + ai].addEntryBinary(3 * index + bi, axis[ai]); // always 3x3 block
//                    }
//                }
//            }
//        }
//
//        basisVec.push_back(basis);
//
//        debugRenderer->beginRenderJob_OneFrame("debug_", DR_FRAME++);
//        const PointSet* samPS = samPC->getPointSet();
//        const AAT& posAAT = samPS->getAAT("position");
//        const unsigned& numElements = samPC->getNumPoints();
//        for (unsigned pi = 0; pi < numElements; ++pi) {
//            const Vector3f& pos = samPS->get3f(pi, posAAT);
//            SparseVectorD var, var_1;
//            const Vector3f& pos_1 = pos + defSymm->GetNormalDirection(pos);
//            for (unsigned d = 0; d < 3; ++d) {
//                var.addEntryBinary(3 * pi + d, pos_1[d]);
//                var_1.addEntryBinary(3 * pi + d, pos[d]);
//            }
//            debugRenderer->addPoint(
//                pos, makeVector3f(1, 1, 0)
//                );
//            debugRenderer->addPoint(
//                pos_1, makeVector3f(1, 1, 0)
//                );
//            for (unsigned rr = 0; rr < num_orbit; ++rr) {
//                Vector3f pos_t, pos_1_t;
//                for (unsigned rd = 0; rd < 3; ++rd) {
//                    const SparseVectorD& row = basis[3 * rr +rd];
//                    debugOutput << row.getNumSparseEntries() << ": ";
//                    SparseVectorD::EIteratorConst it = row.begin();
//                    for (; it != row.end(); ++it) {
//                        debugOutput << it->index << ", ";
//                    }
//                    debugOutput << "\n";
//                    it = row.begin(); 
//                    for (unsigned d = 0; d < 3; ++d) {
//                        const unsigned vid = 3 * pi + d;
//                        while (it != row.end()) { if (it->index == vid) break; ++it; }
//                        if (it == row.end()) break;
//                        pos_t[d] = it->value * var[vid];
//                        pos_1_t[d] = it->value * var_1[vid];
//                        ++it;
//                    }
//                    if (it == row.end()) break;
//                    debugRenderer->addLine(
//                        pos_t, pos_1_t,
//                        makeVector3f(1, 0, 0),
//                        makeVector3f(0, 0, 1),
//                        2);
//                }
//            }
//        }
//        debugRenderer->endRenderJob();
//    }
//
//    return 0;
//}
