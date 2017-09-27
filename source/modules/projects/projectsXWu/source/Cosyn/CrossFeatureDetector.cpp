//---------------------------------------------------------------------------
#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "CrossFeatureDetector.h"
//---------------------------------------------------------------------------
#include "SeparatorClassProperty.h"
#include "CopyObjectProperties.h"
#include "MultiScaleInCorePointCloud.h"
#include "FastSphereQuerry.h"
#include "ProgressWindow.h"
#include "LineFeature.h"
#include "LBase.h"
#include "LBaseSet.h"
#include "Random.h"
#include "DebugRenderer.h"
#include "PCA.h"
#include "TopologyRangeSearch.h"
#include "PCCComputeTopology.h"
#include "PointCloudTools.h"
#include "Monomials2ndOrder.h"
//---------------------------------------------------------------------------
//
//namespace {
//    int DR_FRAME = 0;
//}
//
//using namespace X4;
//
//IMPLEMENT_X4_CLASS( CrossFeatureDetector , 0 )
//{
//    BEGIN_CLASS_INIT( CrossFeatureDetector );
//    ADD_SEPARATOR("");
//    ADD_FLOAT32_PROP(curvThreshold, 0);
//    ADD_FLOAT32_PROP(cluster_ratio, 0);
//    ADD_FLOAT32_PROP(intersect_ratio, 0);
//    ADD_FLOAT32_PROP(intersect_ring_ratio, 0);
//    ADD_BOOLEAN_PROP(use1stPlane, 0);
//    ADD_SEPARATOR("");
//    ADD_FLOAT32_PROP(curvRatio, 0);
//    ADD_FLOAT32_PROP(msAngleTollerance, 0);
//    ADD_CARD32_PROP(msMaxNIter, 0);
//    ADD_FLOAT32_PROP(intersectionAngle, 0);
//}
//
//CrossFeatureDetector::CrossFeatureDetector()
//{
//    curvThreshold = 0.8f;
//    curvRatio = 2;
//    msAngleTollerance = 20.f;
//    msMaxNIter = 20;
//    intersectionAngle = 70.f;
//    use1stPlane = false;
//    cluster_ratio = 10;
//    intersect_ratio = 10;
//    intersect_ring_ratio = 0.6f;
//}
//
//#define radToDeg (180.0f / M_PI)
//PointSet* applyMSFilter(PointSet* ps, const float& sigmaSpace, const float& sigmaAlpha, const int& maxMSIterations, const bool& debugdraw)
//{
//    if (!ps) {
//        warning("applyMSFilter - passed nullptr pointer as argument PointSet");
//        return nullptr;
//    }
//    if ((sigmaSpace < 0.0f) || (sigmaAlpha < 0.0f)) {
//        warning("applyMSFilter - at least one non meaningful parameter < 0");
//        return nullptr;
//    }
//
//    AAT oldPosAAT = ps->getDescr()->getToken("position");
//    AAT oldTanVAAT = ps->getDescr()->getToken("tangent_v");
//
//    PointSet* result = dynamic_cast<PointSet*>(ps->copy());
//    AAT newPosAAt	= result->getDescr()->getToken("position");
//    AAT newTanVAAt	= result->getDescr()->getToken("tangent_v");
//
//    const float inv_2_SigmaSpace_2 = 1.0f / (2 * sigmaSpace * sigmaSpace);
//    const float inv_2_SigmaAngle_2 = 1.0f / (2 * sigmaAlpha * sigmaAlpha);
//    const float radiusSpace = 3 * sigmaSpace;
//
//    std::set<int> markForDeletion;
//    progressWindow->pushStep(true, "MS Filtering");
//    for (int nMSIter = 0; nMSIter < maxMSIterations; ++nMSIter) {
//
//        if (debugdraw) {
//            debugRenderer->beginRenderJob_OneFrame("MS_Filtering_", DR_FRAME++);
//        }
//
//        int nPoints = result->getNumEntries();
//        AttachedIndexedOctree* octree = new AttachedIndexedOctree();
//        octree->setup("SimpleOctree", AttachedData::ADF_QUERY_DELETE_ON_UPDATE,  nullptr);
//        octree->octreesetup(ps, 32);
//        FastSphereQuerry querry(octree, ps); // query structure fix for each iteration
//        for(int ptii = 0; ptii < nPoints; ++ptii) {
//            const Vector3f currentPosition = result->get3f(ptii, newPosAAt);
//            const Vector3f currentTangent = normalize(result->get3f(ptii, newTanVAAt));
//
//            Vector3f msSpace = NULL_VECTOR3F;
//            //Vector3f msDirec = NULL_VECTOR3F;
//            float32 weightSum = 0.0f;
//            PCA<float,3> pca;
//
//            mpcard numIndices;
//            mpcard* indices;
//            querry.querry(currentPosition, radiusSpace, &indices, numIndices);
//            std::vector<float32> weightvec(numIndices);
//            //debugOutput << "\n" << numIndices << "\n";
//            for (int nnii = 0; nnii < numIndices; ++nnii) {
//                // self skip
//                if(ptii == indices[nnii]) continue;
//                // space
//                const Vector3f neighborPosition = ps->get3f(indices[nnii], newPosAAt);
//                const float32 distanceSpaceSqr = normQuad(currentPosition - neighborPosition);
//                // range
//                const Vector3f neighborTangent = normalize(ps->get3f(indices[nnii], newTanVAAt));
//                const float32 dotPCurvedness = std::min<float>(fabs(currentTangent * neighborTangent), 1.0f);
//                const float32 angle = acos(dotPCurvedness) * radToDeg;
//                //truncation
//                if (angle > 2 * sigmaAlpha) continue;
//                const float32 weightCurvedness = exp(- sqr(angle) * inv_2_SigmaAngle_2);
//                const float32 weightSpace = exp(- distanceSpaceSqr * inv_2_SigmaSpace_2);
//                const float32 weight = weightSpace * weightCurvedness;
//
//                msSpace += neighborPosition * weight;
//                //msDirec += neighborTangent * weight;
//                pca.addPoint(neighborPosition, weight);
//                weightSum += weight;
//                weightvec[nnii] = weight;
//                //debugOutput << distanceSpaceSqr << "\t" << dotPCurvedness << "\t"
//                //    << angle << "\t" << weightSpace << "\t" << weightCurvedness << "\t" << weight << "\n";
//            }
//            //debugOutput << weightSum << "\n\n";
//
//            if (weightSum < 1e-6) {
//                // division by zero and no valid neighbor
//                markForDeletion.insert(ptii);
//                {
//                    if (debugdraw /*&& nMSIter == 1*/) {
//                        //debugRenderer->beginRenderJob_OneFrame("MS_Filtering_", DR_FRAME++);
//                        debugRenderer->addPoint(
//                            currentPosition, makeVector3f(1, 0, 0)
//                            );
//                        debugRenderer->addLine(
//                            currentPosition, currentPosition+.1f*currentTangent,
//                            makeVector3f(1, 0, 0), makeVector3f(1, 0, 0),
//                            2);
//
//                        //debugOutput << numIndices << "\n";
//                        //float wtsum = 0.f;
//                        //for (int nnii = 0; nnii < numIndices; ++nnii) {
//                        //    if(ptii == indices[nnii]) continue;
//                        //    const Vector3f neighborPosition = result->get3f(indices[nnii], newPosAAt);
//                        //    const Vector3f neighborTangent = normalize(result->get3f(indices[nnii], newTanVAAt));
//                        //    debugRenderer->addLine(
//                        //        neighborPosition, neighborPosition+0.1f*neighborTangent,
//                        //        makeVector3f(0, 1, 0), makeVector3f(0, 1, 0),
//                        //        1);
//
//                        //    const float32 distanceSpaceSqr = normQuad(currentPosition - neighborPosition);
//                        //    const float32 dotPCurvedness = std::min(fabs(currentTangent * neighborTangent), 1.0f);
//                        //    const float32 angle = acos(dotPCurvedness) * radToDeg;
//                        //    if (angle > 2 * sigmaAlpha) continue;
//                        //    const float32 weightCurvedness = exp(- sqr(angle) * inv_2_SigmaAngle_2);
//                        //    const float32 weightSpace = exp(- distanceSpaceSqr * inv_2_SigmaSpace_2);
//                        //    const float32 weight = weightSpace * weightCurvedness;
//                        //    wtsum += weight;
//                        //    debugOutput << distanceSpaceSqr << "\t" << dotPCurvedness << "\t"
//                        //        << angle << "\t" << weightSpace << "\t" << weightCurvedness << "\t" << weight << "\n";
//
//                        //    //debugRenderer->addLine(
//                        //    //    neighborPosition, neighborPosition+weightvec[nnii]*neighborTangent,
//                        //    //    makeVector3f(0, 1, 0), makeVector3f(0, 1, 0),
//                        //    //    1);
//                        //}
//                        //debugRenderer->endRenderJob();
//                        //debugOutput << wtsum << "\t" << weightSum << "\n\n";
//                    }
//                }
//                continue;
//            }
//
//            const Vector3f diff = (IDENTITY3F - outerProduct(currentTangent, currentTangent)) *
//                ((msSpace / weightSum) - currentPosition);
//            result->set3f(ptii, newPosAAt, currentPosition+diff);
//            //result->set3f(ptii, newTanVAAt, (msDirec+currentTangent) / weightSum);
//            Vector3f newtan;
//            {
//                Vector3f eigenValues,centroid;
//                Matrix3f eigenVectors;
//                pca.analyze(eigenValues,eigenVectors,centroid);
//                newtan = normalize(eigenVectors[0]);
//                // only update tangent in the last step, to prevent drifting
//                if (nMSIter == maxMSIterations-1) {
//                    result->set3f(ptii, newTanVAAt, newtan);
//                }
//
//                if (debugdraw) { // local frame at the old position
//                    debugRenderer->addLine(
//                        currentPosition, currentPosition+.1f*currentTangent,
//                        makeVector3f(1, 1, 0), makeVector3f(1, 1, 0),
//                        2);
//                    debugRenderer->addLine(
//                        currentPosition, currentPosition+.1f*eigenVectors[0],
//                        makeVector3f(1, 0, 1), makeVector3f(1, 0, 1),
//                        2);
//                    //debugRenderer->addLine(
//                    //    currentPosition, currentPosition+.1f*eigenVectors[1],
//                    //    makeVector3f(0, 1, 1), makeVector3f(0, 1, 1),
//                    //    2);
//                    debugRenderer->addLine(
//                        currentPosition, currentPosition+.1f*eigenVectors[2],
//                        makeVector3f(0, 1, 1), makeVector3f(0, 1, 1),
//                        2);
//                }
//            }
//
//            //if (ptii == 0)
//            //{
//            //    debugRenderer->beginRenderJob_OneFrame("MS_Filtering_", DR_FRAME++);
//            if (debugdraw) { // new shifted position
//                debugRenderer->addPoint(
//                    currentPosition+diff, makeVector3f(0, 1, 0)
//                    );
//                debugRenderer->addLine(
//                    currentPosition+diff, currentPosition+diff+.1f*newtan,
//                    makeVector3f(0, 1, 0), makeVector3f(0, 1, 0),
//                    2);
//            }
//            //    debugRenderer->addLine(
//            //        currentPosition, currentPosition+diff,
//            //        makeVector3f(1, 0, 0), makeVector3f(0, 0, 1),
//            //        5);
//            //    debugRenderer->addLine(
//            //        currentPosition, currentPosition+.1f*newtan,
//            //        makeVector3f(1, 0, 0), makeVector3f(1, 0, 0),
//            //        2);
//            //    debugRenderer->endRenderJob();
//            //    for (int nnii = 0; nnii < numIndices; ++nnii) {
//            //        const Vector3f neighborPosition = result->get3f(indices[nnii], newPosAAt);
//            //        const Vector3f neighborTangent = normalize(result->get3f(indices[nnii], newTanVAAt));
//            //        debugRenderer->addLine(
//            //            neighborPosition, neighborPosition+weightvec[nnii]*neighborTangent,
//            //            makeVector3f(0, 1, 0), makeVector3f(0, 1, 0),
//            //            1);
//            //    }
//            //    debugRenderer->endRenderJob();
//            //}
//        }
//
//        /*remove invalid candidates*/
//        //if (nMSIter == 0)
//        {
//            if (result->getNumEntries() == markForDeletion.size()) break;
//            //std::vector<int> markvec(markForDeletion.begin(), markForDeletion.end());
//            std::set<int> fullmark, resmark;
//            for (int ii = 0; ii < nPoints; ++ii) {
//                fullmark.insert(ii);
//            }
//            std::set_difference(fullmark.begin(), fullmark.end(),
//                markForDeletion.begin(), markForDeletion.end(),
//                std::inserter(resmark, resmark.end()));
//            //PointSet* tmpps = new PointSet();
//            //tmpps->clearAndSetup(1, resmark.size(), result->getDescr());
//            std::set<int>::iterator it = resmark.begin();
//            for (unsigned ii = 0; it != resmark.end(); ++ii, ++it) {
//                result->setVertex(ii, result->getDataPointer(*it));
//
//                if (debugdraw) { // result
//                    const Vector3f cpos = result->get3f(ii, newPosAAt);
//                    const Vector3f ctan = normalize(result->get3f(ii, newTanVAAt));
//                    debugRenderer->addPoint(
//                        cpos, makeVector3f(0, 0, 1)
//                        );
//                    debugRenderer->addLine(
//                        cpos, cpos-.1f*ctan, // reversed direction
//                        makeVector3f(0, 0, 1), makeVector3f(0, 0, 1),
//                        2);
//                }
//            }
//            result->changeHeight(resmark.size());
//            markForDeletion.clear();
//        }
//
//        if (debugdraw) {
//            debugRenderer->endRenderJob();
//        }
//
//        delete octree;
//        progressWindow->progress((float)nMSIter/(float)maxMSIterations);
//    }
//    progressWindow->popStep();
//
//    return result;
//}
//
//LineFeatureSet* clusterLineFeature(PointSet** ps_ref, const float& searchRadius, const float& sigmaAlpha)
//{
//    LineFeatureSet* lfs = new LineFeatureSet;
//    lfs->setup(LineFeatureSet::getDefaultName(), AttachedData::ADF_PERSISTENT, nullptr);
//
//    PointSet* ps = *ps_ref;
//    AAT PosAAT = ps->getAAT("position");
//    AAT NormAAT = ps->getAAT("normal");
//    AAT TanVAAT = ps->getAAT("tangent_v");
//
//    //debugRenderer->beginRenderJob_OneFrame("line_feature", DR_FRAME++);
//    for (int index = 0; index < ps->getNumEntries(); ++index) {
//        LineFeature* lf = new LineFeature;
//        lf->m_Position = ps->get3f(index, PosAAT);
//        lf->m_LineDirection = normalize(ps->get3f(index, TanVAAT));
//        lf->m_Normal = normalize(ps->get3f(index, NormAAT));
//        lf->m_RotationDirection = NULL_VECTOR3F;
//        lf->m_Curvature = 0.0f;
//        lf->m_Curvature2 = 0.0f;
//        lf->m_Sigma = 1.0f;
//
//        lfs->m_LineFeatures.push_back(lf);
//
//        Vector3f temp = lf->m_LineDirection;
//        //debugRenderer->addLine(lf->m_Position, lf->m_Position+temp,
//        //    makeVector3f(fabs(lf->m_LineDirection[0]),fabs(lf->m_LineDirection[1]),fabs(lf->m_LineDirection[2])),
//        //    makeVector3f(fabs(lf->m_LineDirection[0]),fabs(lf->m_LineDirection[1]),fabs(lf->m_LineDirection[2])),
//        //    3.0f);
//    }
//    //debugRenderer->endRenderJob();
//
//    ///////////////////////////////////////////////////////////////////////
//
//    const bool debug_draw = false;
//    size_t numLF = lfs->m_LineFeatures.size();
//    std::vector<unsigned> indices;
//    for (size_t i = 0; i < numLF; ++i)
//        indices.push_back(i);
//
//    UnstructuredInCorePointCloud * upc = lfs->getFeaturesAsPointCloud();
//    AAT lfPosAAT = upc->getAAT("position");
//    FastSphereQuerry querry(upc);
//    lfs->m_ClusterID = std::vector<card32>(lfs->m_LineFeatures.size(), 0);
//    int currentClusterID = 0;
//
//    const float angleSimilarity = fabs(cos(sigmaAlpha * M_PI / 180.0f));
//    for (size_t cii = 0; cii < numLF; ++cii) {
//        unsigned currentIndex = indices[cii];
//        if( lfs->m_ClusterID[currentIndex] != 0 )
//            continue;
//
//        if (debug_draw) {
//            debugRenderer->beginRenderJob_OneFrame("line_feature_cluster_", DR_FRAME++);
//        }
//        Vector3f debugColor = makeVector3f(rnd01(), rnd01(), rnd01());
//
//        // start new cluster
//        currentClusterID++;
//        lfs->m_ClusterID[currentIndex] = currentClusterID;
//        Vector3f rep_nor = lfs->m_LineFeatures[currentIndex]->m_LineDirection;
//        int num_clu = 1;
//
//        std::list<unsigned> lf_stack;
//        lf_stack.push_back(currentIndex);
//
//        while (!lf_stack.empty()) {
//            unsigned ci = lf_stack.front();
//            lf_stack.pop_front();
//
//            const Vector3f& cPos = lfs->m_LineFeatures[ci]->m_Position;
//            mpcard numIndices;
//            mpcard* indices;
//            querry.querry(cPos, searchRadius, &indices, numIndices);
//            for (int nnii = 0; nnii < numIndices; ++nnii) {
//                unsigned nIndex = indices[nnii];
//                if(cii == nIndex) continue;
//                Vector3f nPos = upc->getPointSet()->get3f(indices[nnii], lfPosAAT);
//
//                LineFeature* tmpLF = lfs->m_LineFeatures[nIndex];
//                const Vector3f& n_nor = tmpLF->m_LineDirection;
//                if( lfs->m_ClusterID[nIndex] == 0 
//                    && (fabs(rep_nor * n_nor) > angleSimilarity))
//                {
//                    lf_stack.push_back(nIndex);
//                    lfs->m_ClusterID[nIndex] = currentClusterID;
//
//                    { // re-estimate representing direction
//                        num_clu++;
//                        rep_nor /= num_clu;
//                        rep_nor *= (num_clu - 1);
//                        rep_nor += (n_nor / num_clu);
//                    }
//
//                    if (debug_draw) {
//                        debugRenderer->addLine(
//                            tmpLF->m_Position-n_nor, tmpLF->m_Position+n_nor,
//                            debugColor, debugColor, 3.0f);
//                    }
//                }
//            }
//        }
//        if (debug_draw) {
//            debugRenderer->endRenderJob();
//        }
//    }
//
//    int smallClusters = std::min<float>(100, (float)numLF * 0.01); // remove small clusters (often short sections) really helps pruning false intersections
//    const int nFeaturesBefore = lfs->getNumFeatures();
//    std::vector<unsigned> updated_indices = lfs->removeSmallClusters(smallClusters, ps_ref);
//    const int nFeaturesAfter = lfs->getNumFeatures();
//    debugOutput << "remaining feature number: " << nFeaturesAfter << " [" << nFeaturesBefore << "] with threshold " << smallClusters << "\n";
//
//    return lfs;
//}
//
//LBaseSet* detectIntersection(PointSet* ps, LineFeatureSet* lfs, const float& searchRadius, const float& intersect_ring_ratio, const float& intersectionAngle)
//{
//    std::vector<LBase*> resultBases;
//    UnstructuredInCorePointCloud * upc = lfs->getFeaturesAsPointCloud();
//    AAT posAAT = upc->getAAT("position");
//    FastSphereQuerry querry(upc);
//
//    const bool debug_draw = false;
//    const float searchRadiusX2 = searchRadius * intersect_ring_ratio;
//    std::set<unsigned> baseMap;
//    const float intersectionTollerance = fabs(cos(intersectionAngle * M_PI / 180.0f));
//    for (int cii = 0; cii < lfs->m_LineFeatures.size(); ++cii) {
//        const unsigned& currentClusterID = lfs->m_ClusterID[cii];
//        LineFeature* cf = lfs->m_LineFeatures[cii];
//        Vector3f cPos = cf->m_Position;
//
//        if (debug_draw) {
//            debugRenderer->beginRenderJob_OneFrame("intersections_", DR_FRAME++);
//            debugRenderer->addLine(cPos, cPos+cf->m_LineDirection,
//                makeVector3f(1, 0, 0), makeVector3f(1, 0, 0),
//                3.f);
//        }
//        mpcard numIndices;
//        mpcard* indices;
//        std::set< unsigned > false_set; // ring neighborhood: directional estimation within a very small radius can be very unstable
//        {
//            querry.querry(cPos, searchRadiusX2, &indices, numIndices);
//            for (int nnii = 0; nnii < numIndices; ++nnii) {
//                false_set.insert(indices[nnii]);
//            }
//        }
//        querry.querry(cPos, searchRadius, &indices, numIndices);
//        for (int nnii = 0; nnii < numIndices; ++nnii) {
//            const unsigned& nIndex = indices[nnii];
//            if (cii == nIndex) continue; // same line
//            if (false_set.find(nIndex) != false_set.end()) continue; // within inner part of the ring
//
//            const unsigned& nClusterID = lfs->m_ClusterID[nIndex];
//            if (currentClusterID == nClusterID) continue; // same cluster
//            const unsigned& baseIndex = currentClusterID * (unsigned)lfs->m_ClusterID.size() + nClusterID;
//
//            LineFeature * nf = lfs->m_LineFeatures[nIndex];
//            Vector3f nPos = upc->getPointSet()->get3f(indices[nnii], posAAT);
//
//            if (( fabs(cf->m_LineDirection*nf->m_LineDirection) < intersectionTollerance)
//                && (baseMap.find(baseIndex) == baseMap.end())) 
//            {
//                LBase * base = new LBase;
//                base->setup(cf,nf,cii,nIndex,1);
//                resultBases.push_back(base);
//                baseMap.insert(baseIndex);
//
//                if (debug_draw) {
//                    debugRenderer->addLine(nPos, nPos+nf->m_LineDirection,
//                        makeVector3f(0, 1, 0), makeVector3f(0, 1, 0),
//                        3.f);
//                    //debugRenderer->addSphere(
//                    //    base->getBasePoint(), 0.2f,
//                    //    makeVector3f(1.f, 0.f, 0.f));
//                }
//            }
//            else {
//                if (debug_draw) {
//                    debugRenderer->addLine(nPos, nPos+nf->m_LineDirection,
//                        makeVector3f(0, 0, 1), makeVector3f(0, 0, 1),
//                        3.f);
//                }
//            }
//        }
//        if (debug_draw) {
//            debugRenderer->endRenderJob();
//        }
//    }
//
//    const int nBases = (int)resultBases.size();
//    LBaseSet* lbs = new LBaseSet;
//    lbs->setup( LBaseSet::getDefaultName(), AttachedData::ADF_PERSISTENT, nullptr);
//    lbs->setBaseVector(resultBases);
//
//    //debugRenderer->beginRenderJob_OneFrame("intersections_", DR_FRAME++);
//    //for (int index = 0; index < resultBases.size(); ++index) {
//    //    debugRenderer->addSphere(
//    //        resultBases[index]->getBasePoint(), 0.2f,
//    //        makeVector3f(1.f, 0.f, 0.f));
//    //}
//    //debugRenderer->endRenderJob();
//
//    return lbs;
//}
//
//void computeCurv2forPC(PointCloud* pc, float base_sigma, float top_ring_factor, bool use_top_expansion, bool debugdraw)
//{
//    //X4_TIMER_START(compute_curvature);
//
//    checkAttribute( pc, "curv2", 2, VAD::DATA_FORMAT_FLOAT32 );
//    checkAttribute( pc, "tangent_u", 3, VAD::DATA_FORMAT_FLOAT32 );
//    checkAttribute( pc, "tangent_v", 3, VAD::DATA_FORMAT_FLOAT32 );
//
//    AAT posAAT = pc->getAAT( "position" );
//    AAT normalAAT = pc->getAAT( "normal" );
//    AAT curv2AAT = pc->getAAT( "curv2" );
//    AAT tangent_uAAT = pc->getAAT( "tangent_u" );
//    AAT tangent_vAAT = pc->getAAT( "tangent_v" );
//
//    SceneObjectIterator *iter = pc->createIterator( SOIT::CAP_BASIC_PC | SOIT::CAP_MODIFYABLE_PC );
//    pAssert( iter != nullptr );
//    BasicPointCloudIterator *bIt = dynamic_cast<BasicPointCloudIterator*>(iter);
//    ModifyablePointCloudIterator *mIt = dynamic_cast<ModifyablePointCloudIterator*>(iter);
//    IndexedPointCloudIterator *indIt = dynamic_cast<IndexedPointCloudIterator*>(iter);
//    const PointSet *ps = bIt->lockBlockRead();
//    const float top_radius = base_sigma * 2.1f; // sqrt(2.f);
//    const float query_radius = top_radius * top_ring_factor;
//    debugOutput << "computing curvature: base sigma: " << base_sigma << ", top radius "
//        << top_radius << ", query radius " << query_radius << ".\n";
//
//    InCorePCTopologyGraph* tpg = nullptr;
//    TopNeighborhoodSphereSearch* query_top = nullptr;
//    FastSphereQuerry* query = nullptr;
//    if (use_top_expansion) {
//        PCCComputeTopology cmd;
//        cmd.setup(PCCComputeTopology::TOPTYPE_EPS, top_radius);
//        tpg = cmd.computeEpsTopology(ps, pc);
//        query_top = new TopNeighborhoodSphereSearch;
//    } else {
//        query = new FastSphereQuerry(pc);
//    }
//
//    card32 index = 0;
//    progressWindow->pushStep(true, "curv2");
//    std::vector<Vector3f> posBuffer;
//    while( !bIt->atEnd() )
//    {
//        Vector3f pos = bIt->get3f( posAAT );
//        mpcard numPoints;
//        mpcard *pts;
//        std::vector<mpcard> nnvec;
//        if (use_top_expansion) {
//            //unsigned depnn = 3;
//            //nnvec = ExpandKdepNN(query, pos, query_radius, ps, depnn);
//            //nnvec = ExpandKdepNN(tpg, index, depnn);
//            query_top->setup(tpg, ps, (card32)indIt->getCurrentPointIndex(), pos, query_radius, posAAT);
//            nnvec.clear();
//            while (!query_top->atEnd()) {
//                nnvec.push_back(query_top->getCurrentIndex());
//                query_top->next();
//            }
//            numPoints = nnvec.size();
//            pts = &nnvec[0];
//        } else {
//            query->querry(pos, query_radius, &pts, numPoints);
//        }
//        if (numPoints == 0) {
//            bIt->next();
//            index++;
//            continue;
//        }
//
//        Vector3f pcaNormal;
//        //if (usePCANormal) {
//        PCA<float,3> pca;
//        for (card32 i=0; i<numPoints; i++) {
//            Vector3f sample_pos = ps->get3f(pts[i], posAAT);
//            float dist_2 = normQuad(sample_pos - pos);
//            float w = exp(-dist_2/(2*base_sigma*base_sigma));
//            pca.addPoint(sample_pos, w);
//        }
//        Vector3f eigenValues,centroid;
//        Matrix3f eigenVectors;
//        {
//            pca.analyze(eigenValues,eigenVectors,centroid);
//            if( eigenVectors[2] * bIt->get3f(normalAAT) < 0.0f )
//                eigenVectors *= -1.0f;
//            pcaNormal = normalize(eigenVectors[2]);
//            mIt->set3f(normalAAT, pcaNormal);
//        }
//        //} else {
//        //   pcaNormal = bIt->get3f(normalAAT);
//        //}
//        Matrix3f tangentSystem = calcTangentSystem( pcaNormal );
//        Matrix3f invTangentSystem = tangentSystem.transpose();
//
//        if( posBuffer.size() < numPoints )
//        {
//            posBuffer.resize(numPoints);
//        }
//
//        // transform to a canonical frame
//        for( card32 i=0;i<numPoints;i++ )
//        {
//            Vector3f tmp = ps->get3f(pts[i], posAAT) - pos;
//            //posBuffer[i] = invTangentSystem * tmp / (sqrt(2.0f)*baseSigma);
//            posBuffer[i] = invTangentSystem * tmp;
//        }
//
//        // compute second fundamental form
//        StaticVector<float32, 6> coeff;
//        Monomials2ndOrder::computeCoeffs( &posBuffer[0], (card32)numPoints, coeff, 1.0f );
//
//        float b_u = coeff[1];
//        float b_v = coeff[2];
//        float A_uu = coeff[3];
//        float A_uv = coeff[4];
//        float A_vv = coeff[5];
//
//        // eigen values corresponds to principal curvature
//        Matrix2f A = makeMatrix2f(A_uu, A_uv,
//            A_uv, A_vv);
//        Vector2f lambda;
//        Matrix2f eVects;
//        A.computeEigenStructure(lambda, eVects);
//
//        float32 lambda1 = lambda[0];
//        float32 lambda2 = lambda[1];
//
//        Vector2f v1 = eVects[0];
//        Vector2f v2 = eVects[1];
//
//        if (fabs(lambda1) < fabs(lambda2)) {
//            std::swap(lambda1, lambda2);
//            std::swap(v1, v2);
//        }
//
//        // make right handed coordinate system to preserve sign of curvature
//        if (determinant(v1, v2) < 0) {
//            v2 = -v2;
//        }
//
//        // transform back to tangential frame
//        Vector3f newTangentU = tangentSystem[0] * v1[0] + tangentSystem[1] * v1[1];
//        Vector3f newTangentV = tangentSystem[0] * v2[0] + tangentSystem[1] * v2[1];
//
//        //// take eigen std::vector for pencil neighborhood shape
//        //if (eigenValues[0] > 2.5f*eigenValues[1] && eigenValues[1] < 2.f*eigenValues[2]) {
//        //    newTangentV = eigenVectors[0];
//        //    lambda1 = eigenValues[0]*5000.f; // empirical value
//        //}
//
//        mIt->set2f(curv2AAT, makeVector2f(lambda1, lambda2));
//        mIt->set3f(tangent_uAAT, newTangentU);
//        mIt->set3f(tangent_vAAT, newTangentV);
//        index++;
//        bIt->next();
//
//        if (debugdraw
//            && abs(lambda1) > 0.1f
//            && abs(lambda1) / abs(lambda2) > 2
//            ) {
//                debugRenderer->beginRenderJob_OneFrame("curv2_query_", DR_FRAME++);
//                debugRenderer->addPoint(pos, makeVector3f(1, 0, 0));
//                for (card32 i=0; i<numPoints; i++) {
//                    Vector3f sample_pos = ps->get3f(pts[i], posAAT);
//                    debugRenderer->addPoint(sample_pos, makeVector3f(0, 1, 0));
//                }
//                debugRenderer->addLine(
//                    centroid, centroid+5000.f*eigenVectors[0]*eigenValues[0],
//                    makeVector3f(1, 0, 0), makeVector3f(1, 0, 0),
//                    2);
//                debugRenderer->addLine(
//                    centroid, centroid+5000.f*eigenVectors[1]*eigenValues[1],
//                    makeVector3f(0, 1, 0), makeVector3f(0, 1, 0),
//                    2);
//                debugRenderer->addLine(
//                    centroid, centroid+5000.f*eigenVectors[2]*eigenValues[2],
//                    makeVector3f(0, 0, 1), makeVector3f(0, 0, 1),
//                    2);
//                //debugOutput << eigenValues[0] << "\t" << eigenValues[1] << "\t";
//                debugRenderer->addLine(
//                    pos, pos+.5f*newTangentU*lambda1,
//                    makeVector3f(1, 1, 0), makeVector3f(1, 1, 0),
//                    2);
//                debugRenderer->addLine(
//                    pos, pos+.5f*newTangentV*lambda2,
//                    makeVector3f(0, 1, 1), makeVector3f(0, 1, 1),
//                    2);
//                //debugOutput << lambda1 << "\t" << lambda2 << "\n";
//                debugRenderer->endRenderJob();
//        }
//
//        if( index % 1000 == 0) {
//            // TODO (daniel): Canceling the process from the progressWindow doesn't work with OpenMP...
//            progressWindow->progress((float)index/(float)pc->getNumPoints()*100.0f);
//        }
//    }
//    progressWindow->popStep();
//
//    bIt->unlock();
//    delete iter;
//    delete tpg;
//    delete query_top;
//    delete query;
//    //X4_TIMER_STOP(compute_curvature);
//}
//
//UnstructuredInCorePointCloud *CrossFeatureDetector::detectFeatures(
//    float32 baseSpacing, float32 resampleSpacing,
//    UnstructuredInCorePointCloud *source)
//{
//    VertexDescriptor vd;
//    vd.pushAttrib( mVAD("position", 3, VAD::DATA_FORMAT_FLOAT32) );
//    vd.pushAttrib( mVAD("normal", 3, VAD::DATA_FORMAT_FLOAT32) );
//    vd.pushAttrib( mVAD("curv2", 2, VAD::DATA_FORMAT_FLOAT32) );
//    vd.pushAttrib( mVAD("tangent_v", 3, VAD::DATA_FORMAT_FLOAT32) );
//    vd.pushAttrib( mVAD("index",    1, VAD::DATA_FORMAT_CARD32) );
//
//    // compute curvature channel
//    computeCurv2forPC(source, baseSpacing, 4, false, false);
//    const PointSet *currIncorePS = source->getPointSet();
//    AAT positionAAT = currIncorePS->getAAT("position", 3, VAD::DATA_FORMAT_FLOAT32);
//    AAT normalAAT = currIncorePS->getAAT("normal", 3, VAD::DATA_FORMAT_FLOAT32);
//    AAT curv2AAT = currIncorePS->getAAT("curv2", 2, VAD::DATA_FORMAT_FLOAT32);
//    AAT tangentAAT = currIncorePS->getAAT("tangent_v", 3, VAD::DATA_FORMAT_FLOAT32);
//    PointSet* singleLevelPS = new PointSet();
//    singleLevelPS->clearAndSetup(1, currIncorePS->getNumEntries(), &vd);
//    AAT indexAAT = singleLevelPS->getAAT("index");
//
//    mpcard n = currIncorePS->getNumEntries();
//    mpcard actualNumberOfEntries = 0;
//    for (mpcard i=0; i<n; i++) {
//        Vector2f curv2;
//        curv2[0] = currIncorePS->get2f(i, curv2AAT)[0];
//        curv2[1] = currIncorePS->get2f(i, curv2AAT)[1];
//        if (abs(curv2[0]) < curvThreshold
//            || abs(curv2[0]) / abs(curv2[1]) < curvRatio
//            ) continue;
//
//        singleLevelPS->set3f(actualNumberOfEntries, positionAAT, currIncorePS->get3f(i, positionAAT));
//        singleLevelPS->set3f(actualNumberOfEntries, normalAAT, currIncorePS->get3f(i, normalAAT));
//        singleLevelPS->set2f(actualNumberOfEntries, curv2AAT, currIncorePS->get2f(i, curv2AAT));
//        singleLevelPS->set3f(actualNumberOfEntries, tangentAAT, currIncorePS->get3f(i, tangentAAT));
//        singleLevelPS->set1u(actualNumberOfEntries, indexAAT, i);
//        ++actualNumberOfEntries;
//    }
//    debugOutput << "selected " << actualNumberOfEntries << " out of " << n << " features\n";
//    singleLevelPS->changeHeight(actualNumberOfEntries);
//    BoundingBox3f bb_ps = singleLevelPS->getBoundingBox();
//
//    //// add boundary points
//    //{
//    //    PCCBoundaryDetector cmd;
//    //    UnstructuredInCorePointCloud* bPC = new UnstructuredInCorePointCloud;
//    //    bPC->clearAndSetup(&vd, currIncorePS->getNumEntries());
//    //    PointSet *bPS = (PointSet*)currIncorePS->copy();
//    //    bPC->setPointSet(bPS);
//    //    cmd.detectBoundaries(bPC);
//    //    bPS = bPC->getPointSet();
//    //    AAT bPosAAT = bPS->getAAT("position");
//    //    AAT bNormAAT = bPS->getAAT("normal");
//    //    AAT bBoundaryAAT = bPS->getAAT("boundary");
//    //    AAT bBoundaryTangentsAAT = bPS->getAAT("boundaryTangent");
//    //    char* buffer = new char[currIncorePS->getDescr()->getSize()];
//    //    for (mpcard i = 0; i < bPC->getNumPoints(); ++i) {
//    //        float boundary = bPS->get1f(i, bBoundaryAAT);
//    //        if (boundary < .8f) continue;
//    //        singleLevelPS->addPoint(buffer);
//    //        singleLevelPS->set3f(actualNumberOfEntries, resultPositionAAT, currIncorePS->get3f(i, bPosAAT));
//    //        singleLevelPS->set3f(actualNumberOfEntries, resultNormalAAT, currIncorePS->get3f(i, bNormAAT));
//    //        singleLevelPS->set3f(actualNumberOfEntries, resultTangentVAAT, currIncorePS->get3f(i, bBoundaryTangentsAAT));
//    //        singleLevelPS->set1u(actualNumberOfEntries, resultIndexAAT, actualNumberOfEntries);
//    //        ++actualNumberOfEntries;
//    //    }
//    //    delete bPC;
//    //    delete buffer;
//    //}
//
//    // MS Filtering
//    {
//        PointSet* tmpPS = applyMSFilter(singleLevelPS, resampleSpacing, msAngleTollerance, msMaxNIter, false);
//        delete singleLevelPS;
//        singleLevelPS = tmpPS;
//        debugOutput << tmpPS->getNumEntries() << " points after MS filtering\n";
//    }
//
//    // Cluster line features
//    LineFeatureSet* lfs;
//    LBaseSet* lbs;
//    {
//        const float cluster_radius = resampleSpacing * cluster_ratio;
//        const float intersect_radius = resampleSpacing * intersect_ratio;
//        debugOutput << "cluster radius: " << cluster_radius << "; intersection radius: " << intersect_radius << "\n";
//        lfs = clusterLineFeature(&singleLevelPS, cluster_radius, msAngleTollerance);
//        lbs = detectIntersection(singleLevelPS, lfs, intersect_radius, intersect_ring_ratio, intersectionAngle);
//    }
//
//    // extract crossing points
//    {
//        std::vector<LBase*> lbasevec = lbs->getBaseVector();
//
//        UnstructuredInCorePointCloud *tmpPC = new UnstructuredInCorePointCloud;
//        tmpPC->clearAndSetup(&vd, lbasevec.size());
//        PointSet *tmpPS = tmpPC->getPointSet();
//        AAT posAAT = tmpPC->getAAT("position");
//
//        for (unsigned li = 0; li < lbasevec.size(); ++li) {
//            tmpPS->set3f(li, posAAT, lbasevec[li]->getBasePoint());
//        }
//        delete singleLevelPS;
//        singleLevelPS = tmpPS;
//    }
//
//    UnstructuredInCorePointCloud *singleLevelPC = new UnstructuredInCorePointCloud;
//    singleLevelPC->clearAndSetup(&vd, singleLevelPS->getNumEntries());
//    singleLevelPC->setPointSet(singleLevelPS);
//
//    return singleLevelPC;
//}
