//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "SlideConstraints.h"
#include "SToolBox/SamplerSymmBlock.h"
#include "Util\numerical\EigenAdaptor.h"
//----------------------------------------------------------------------
#include "InCorePCTopologyGraph.h"
#include "basics\DeformationTools.h"
//----------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

ColinearityConstraint::ColinearityConstraint(const std::deque<card32>& indice) : indice_(indice)
{
}

void ColinearityConstraint::AddColinearity(
    SymmSpaceSolver::Ptr receiver,
    const float& colinearityWeight
    )
{
    const unsigned& num_knots = indice_.size();
    for (unsigned k0 = 0, k1 = 1, k2 = 2; k2 < num_knots; ++k0, ++k1, ++k2) {
        receiver->addColinearity(
            indice_[k0], indice_[k1],
            indice_[k1], indice_[k2],
            colinearityWeight
            );
    }
    if (num_knots < 4) return;
    receiver->addColinearity( // add end line segments
        indice_[0], indice_[1],
        indice_[num_knots - 2], indice_[num_knots - 1],
        colinearityWeight
        );
}

CoplanarityConstraint::CoplanarityConstraint(std::set<unsigned>& indice)
{
    std::copy(indice.begin(), indice.end(), std::back_inserter(indice_));
}

void CoplanarityConstraint::AddCoplanarity(
    const std::vector<Vector3f>& deformed,
    SymmSpaceSolver::Ptr receiver,
    const float& coplanarityWeight
    )
{
    const unsigned& num_knots = indice_.size();
    Vector3f normal; // for shared edge, there must be conflictions
    {
        //Vector3f mean = NULL_VECTOR3F;
        //for (unsigned ii = 0; ii < num_knots; ++ii) {
        //    const unsigned& index = indice_[ii];
        //    const Vector3f& pos = deformed[index];
        //    mean = mean + pos;
        //}
        //mean = mean / (float)num_knots;

        //SparseMatrixD cov(num_knots); // build covariance matrix
        //for (unsigned ii = 0; ii < num_knots; ++ii) {
        //    const unsigned& index = indice_[ii];
        //    const Vector3f& pos = deformed[index];
        //    for (unsigned d = 0; d < 3; ++d) {
        //        cov[ii].addEntryBinary(d, pos[d] - mean[d]);
        //    }
        //}
        //std::vector<double> U, S, V_T;
        //Clapack::ClapackAdaptor::dgesvd(cov, &U, &S, &V_T, 3, false); // corresponding to the smallest eigen vector
        //for (unsigned d = 0; d < 3; ++d) {
        //    normal[d] = V_T[d * 3 + 2];
        //}
        //normal = normalize(normal);

        Eigen::MatrixXf COV(3, num_knots);
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for (unsigned ii = 0; ii < num_knots; ++ii) {
            const Eigen::Vector3f posI = EigenAdaptor::ToEigen(deformed[indice_[ii]]);
            COV.col(ii) = posI;
            centroid += posI;
        }
        centroid /= (float)num_knots;

        for (unsigned ii = 0; ii < num_knots; ++ii) {
            COV.col(ii) -= centroid;
        }

        Eigen::JacobiSVD<Eigen::MatrixXf> svd(COV, Eigen::ComputeThinU);
        Eigen::Matrix3f U = svd.matrixU();

        normal = EigenAdaptor::FromEigen(static_cast<Eigen::Vector3f>(U.col(2)));
        normal.normalize();
    }

    for (unsigned k0 = 0, k1 = 1; k1 < num_knots; ++k0, ++k1) {
        receiver->addCoplanarity(
            indice_[k0], indice_[k1],
            normal,
            coplanarityWeight
            );
    }
    if (num_knots < 3) return;
    receiver->addCoplanarity(
        indice_[num_knots-1], indice_[0],
        normal,
        coplanarityWeight
        );
}

SlideConstraints::SlideConstraints()
{}

int SlideConstraints::BuildColinearity(
    SceneObject* samobj
    )
{
    colinearities_.clear();

    UICPC* samPC = dynamic_cast<UICPC*>(samobj);
    if (!samPC) {
        error("SlideConstraints::BuildColinearity() - sample missing.");
        return -1;
    }

    LineSymmBlockVec* lineBlockAtt = dynamic_cast<LineSymmBlockVec*>(
        samPC->getAttachments()->getData(LineSymmBlockVec::getDefaultName()));
    if (!lineBlockAtt) {
        error("SlideConstraints::BuildColinearity() - LineSymmBlockVec missing.");
        return -1;
    }
    std::vector<LineSymmBlock*>& lineBlockVec = lineBlockAtt->blockvec_;

    for (unsigned bi = 0; bi < lineBlockVec.size(); ++bi) {
        const card32& elem = lineBlockVec[bi]->elem_;
        const std::deque<card32>& basisvec = lineBlockVec[bi]->pindx_;
        const std::deque< std::deque<card32> >& knotvec = lineBlockVec[bi]->knots_;

        if (basisvec.size() != knotvec.size()) {
            error("SlideConstraints::BuildColinearity - orbit index mis-matching");
            return -1;
        }

        const unsigned num_knot = knotvec.size();
        for (unsigned oei = 0; oei < num_knot; ++oei) {
            const std::deque<card32>& indice = knotvec[oei];
            if (0 == indice.size()) {
                error("SlideConstraints::BuildColinearity - empty knot set");
                continue;
            }
            colinearities_.push_back(ColinearityConstraint(indice));
        }
    }

    debugOutput << "Added " << colinearities_.size() << " colinearity constraints.\n";

    return 0;
}

void SlideConstraints::AddColinearity(
    SymmSpaceSolver::Ptr receiver,
    const float& colinearityWeight
    )
{
    const bool debug_output = false;
    if (debug_output) {
        const std::string debug_id = "colinearity_constraint_";
        debugRenderer->clearRenderJob_AllFrames(debug_id);
        debugRenderer->beginRenderJob_OneFrame(debug_id, DebugRenderer::DR_FRAME++);
    }
    for (std::deque<ColinearityConstraint>::iterator it = colinearities_.begin(); it != colinearities_.end(); ++it) {
        it->AddColinearity(receiver, colinearityWeight);
    }
    if (debug_output) debugRenderer->endRenderJob();
}

int SlideConstraints::BuildCoplanarity(
    SceneObject* samobj,
    const unsigned& min_size,
    const float& tri_cos_th,
    const float& nor_cos_th
    )
{
    coplanarities_.clear();

    UICPC* samPC = dynamic_cast<UICPC*>(samobj);
    if (!samPC) {
        error("SlideConstraints::BuildCoplanarity() - sample missing.");
        return -1;
    }
    const PointSet& samPS = *samPC->getPointSet();
    const AAT& samPosAAT = samPS.getAAT("position", 3, VAD::DATA_FORMAT_FLOAT32);
    const unsigned numElements = samPC->getNumPoints();

    const InCorePCTopologyGraph* tpy = getTopology((PointCloud*)samobj);
    if (!tpy) {
        error("SlideConstraints::BuildCoplanarity() - topology missing.");
        return -1;
    }
    if (tpy->getNumVertices() != numElements) {
        error("SlideConstraints::BuildCoplanarity() - topology mis-match.");
        return -1;
    }

    const bool debug_output = false;

    unsigned num_plane = 0;
    std::vector<unsigned> markVec(numElements, 0);
    for (unsigned pi = 0; pi < numElements; ++pi) {
        if (0 < markVec[pi]) continue;
        const unsigned num_nv_pi = tpy->getNumAdjacentVertices(pi);
        const Vector3f& pos_pi = samPS.get3f(pi, samPosAAT);

        for (unsigned ni = 0; ni < num_nv_pi; ++ni) {
            const unsigned pi_ni = tpy->getAdjacentVertex(pi, ni);
            for (unsigned nj = ni+1; nj < num_nv_pi; ++nj) {
                const unsigned pi_nj = tpy->getAdjacentVertex(pi, nj);
                if (0 < markVec[pi_ni] && 0 < markVec[pi_nj]) continue; // it's possible for two planes to share edge(pi_ni, pi_nj)
                if (!tpy->areVerticesAdjacent(pi_ni, pi_nj)) continue;

                // construct a starting candidate from this triple set
                const Vector3f& pos_pi_ni = samPS.get3f(pi_ni, samPosAAT);
                const Vector3f& pos_pi_nj = samPS.get3f(pi_nj, samPosAAT);
                const Vector3f vec_pi_ni = normalize(pos_pi_ni - pos_pi);
                const Vector3f vec_pi_nj = normalize(pos_pi_nj - pos_pi);
                //const Vector3f vec_ni_nj = normalize(pos_pi_nj - pos_pi_ni);
                if (tri_cos_th < abs(vec_pi_ni * vec_pi_nj)) continue; // this triangle is too sharp
                const Vector3f currNormal = normalize(vec_pi_ni.crossProduct(vec_pi_nj));

                //markVec[pi] = markVec[pi_ni] = markVec[pi_nj] = ++num_plane; // possible overwrite for pi_ni and pi_nj
                std::set<unsigned> inSet; // indice of current expansion
                inSet.insert(pi); inSet.insert(pi_ni); inSet.insert(pi_nj);
                std::vector<unsigned> markFringe(numElements, 0);
                markFringe[pi] = 1; markFringe[pi_ni] = 1; markFringe[pi_nj] = 1;

                // construct fringe set for region growing
                typedef std::pair<unsigned, unsigned> fringePair;
                std::deque< fringePair > fringeSet;
                for (std::set<unsigned>::iterator it = inSet.begin(); it !=  inSet.end(); ++it) {
                    const unsigned num_nv_it = tpy->getNumAdjacentVertices(*it);
                    for (unsigned k = 0; k < num_nv_it; ++k) {
                        const unsigned it_k = tpy->getAdjacentVertex(*it, k);
                        if (inSet.find(it_k) != inSet.end()) continue;
                        if (0 < markFringe[it_k]) continue;
                        fringeSet.push_back(std::make_pair(it_k, *it));
                        markFringe[it_k] = 1;
                    }
                }

                typedef std::pair<Vector3f, Vector3f> edgePair;
                std::deque< edgePair > edgeSet; // for debug only
                if (debug_output)
                {
                    edgeSet.push_back(std::make_pair(pos_pi, pos_pi_ni));
                    edgeSet.push_back(std::make_pair(pos_pi, pos_pi_nj));
                    edgeSet.push_back(std::make_pair(pos_pi_ni, pos_pi_nj));
                }

                while (!fringeSet.empty()) {
                    fringePair fringe = fringeSet.front();
                    fringeSet.pop_front();
                    const unsigned& f_base = fringe.first;
                    const unsigned& f_to = fringe.second;
                    const Vector3f& pos_f_base = samPS.get3f(f_base, samPosAAT);
                    const Vector3f& pos_f_to = samPS.get3f(f_to, samPosAAT);
                    const Vector3f vec_f_base = normalize(pos_f_to - pos_f_base);

                    if (debug_output)
                    {
                        debugRenderer->beginRenderJob_OneFrame("debug_", DR_FRAME++);
                        for (std::deque< edgePair >::iterator it = edgeSet.begin(); it != edgeSet.end(); ++it) {
                            debugRenderer->addLine(
                                it->first, it->second,
                                makeVector3f(1, 0, 0), makeVector3f(1, 0, 0),
                                2);
                        }
                        for (std::set<unsigned>::iterator it = inSet.begin(); it!= inSet.end(); ++it) {
                            const Vector3f& pos = samPS.get3f(*it, samPosAAT);
                            debugRenderer->addPoint(pos, makeVector3f(1, 0, 0));
                        }
                        debugRenderer->addLine(
                            pos_f_base, pos_f_to,
                            makeVector3f(1, 0, 1), makeVector3f(1, 0, 1),
                            2);
                        debugRenderer->addPoint(pos_f_base, makeVector3f(1, 0, 1));
                    }

                    bool bCandi = false;
                    std::deque< unsigned > tempFringe;
                    const unsigned num_nv_f_base = tpy->getNumAdjacentVertices(f_base);
                    for (unsigned k = 0; k < num_nv_f_base; ++k) {
                        const unsigned f_base_k = tpy->getAdjacentVertex(f_base, k);
                        if (f_to == f_base_k) continue;
                        if (!tpy->areVerticesAdjacent(f_base_k, f_to)) continue;
                        const Vector3f& pos_f_base_k = samPS.get3f(f_base_k, samPosAAT);

                        if (inSet.find(f_base_k) != inSet.end()) { // a possible candidate
                            if (bCandi) continue; // already added base point of this fringe

                            const Vector3f vec_f_base_k = normalize(pos_f_base_k - pos_f_base);
                            if (tri_cos_th < abs(vec_f_base_k * vec_f_base)) continue; // this triangle is too sharp
                            const Vector3f normal_f = normalize(vec_f_base_k.crossProduct(vec_f_base));
                            const float norm_diff = normal_f * currNormal;
                            if (nor_cos_th < abs(1 - abs(norm_diff))) continue; // normal differece too large

                            bCandi = true;

                            if (debug_output)
                            {
                                debugRenderer->addLine(
                                    pos_f_base, pos_f_base_k,
                                    makeVector3f(0, 1, 0), makeVector3f(0, 1, 0),
                                    2);
                            }
                        } else { // otherwise put into fringe set, and check for expansion
                            if (0 < markFringe[f_base_k]) continue;
                            tempFringe.push_back(f_base_k);

                            if (debug_output)
                            {
                                debugRenderer->addLine(
                                    pos_f_base, pos_f_base_k,
                                    makeVector3f(0, 1, 1), makeVector3f(0, 1, 1),
                                    2);
                            }
                        }
                        if (bCandi) {
                            edgeSet.push_back(std::make_pair(pos_f_base, pos_f_base_k));
                            edgeSet.push_back(std::make_pair(pos_f_base, pos_f_to));
                            inSet.insert(f_base);
                            for (unsigned ii = 0; ii < tempFringe.size(); ++ii) {
                                fringeSet.push_back(std::make_pair(tempFringe[ii], f_base));
                                markFringe[tempFringe[ii]] = 1;
                            }
                        }
                    }

                    if (debug_output)
                    {
                        debugRenderer->endRenderJob();
                    }
                }

                if (min_size <= inSet.size()) {
                    ++num_plane;
                    for (std::set<unsigned>::iterator it = inSet.begin(); it!= inSet.end(); ++it) {
                        markVec[*it] = num_plane;
                    }
                    coplanarities_.push_back(CoplanarityConstraint(inSet));
                    //if (debug_output)
                    //{
                    //debugRenderer->beginRenderJob_OneFrame("debug_", DR_FRAME++);
                    //for (std::set<unsigned>::iterator it = inSet.begin(); it!= inSet.end(); ++it) {
                    //    const Vector3f& pos = samPS.get3f(*it, samPosAAT);
                    //    debugRenderer->addPoint(pos, makeVector3f(1, 1, 0));
                    //}
                    //debugRenderer->endRenderJob();
                    //}
                    if (debug_output)
                    {
                        for (std::vector<unsigned>::iterator it = markVec.begin(); it != markVec.end(); ++it) {
                            debugOutput << *it << " ";
                        }
                        debugOutput << "\n";
                    }
                }
            }
        }
    }

    debugOutput << "Added " << coplanarities_.size() << " coplanarity constraints.\n";

    return 0;
}

void SlideConstraints::AddCoplanarity(
    SymmSpaceSolver::Ptr receiver,
    const float& coplanarityWeight
    )
{
    const bool debug_output = false;
    if (debug_output) {
        const std::string debug_id = "coplanarity_constraint_";
        debugRenderer->clearRenderJob_AllFrames(debug_id);
        debugRenderer->beginRenderJob_OneFrame(debug_id, DebugRenderer::DR_FRAME++);
    }
    for (std::deque<CoplanarityConstraint>::iterator it = coplanarities_.begin();
        it != coplanarities_.end(); ++it)
    {
        it->AddCoplanarity(deformedPositions, receiver, coplanarityWeight);
    }
    if (debug_output) debugRenderer->endRenderJob();
}

void SlideConstraints::Update(const std::vector<Vector3f>& deformed)
{
    deformedPositions.assign(deformed.begin(), deformed.end());
}
