//---------------------------------------------------------------------------
#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "C13MultiScaleFeatureDetector.h"
#include "C13DescriptorAttachment.h"
#include "Util/PlaneCluster.h"
//---------------------------------------------------------------------------
#include "SeparatorClassProperty.h"
#include "CopyObjectProperties.h"
#include "MultiScaleInCorePointCloud.h"
#include "FastSphereQuerry.h"
#include "ProgressWindow.h"
#include "LineFeature.h"
#include "LBase.h"
#include "LBaseSet.h"
#include "PCCBoundaryDetector.h"
#include "Random.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

static std::string describeMultiScaleMode(card32 i) {
    if (i==0) return "MSMODE_MIN_LEVEL_ONLY";
    if (i==1) return "MSMODE_MIN_TO_MAX_SEPARATE";
    return "undefined";
}


IMPLEMENT_ABSTRACT_CLASS( C13MultiScaleFeatureDetector , 0 )
{
    BEGIN_CLASS_INIT( C13MultiScaleFeatureDetector );
    ADD_CARD32_PROP_UPDATE_DESCR(multiScaleMode, 0, nullptr, describeMultiScaleMode, 2, nullptr);
    ADD_CARD32_PROP(minLevel, 0);
    ADD_CARD32_PROP(maxLevel, 0);
    ADD_BOOLEAN_PROP(debugdraw, 0);
}

IMPLEMENT_CLASS( C13MSFeatDetectCrossPoint , 0 )
{
    BEGIN_CLASS_INIT( C13MSFeatDetectCrossPoint );
    ADD_SEPARATOR("");
    ADD_FLOAT32_PROP(curvThreshold, 0);
    ADD_FLOAT32_PROP(cluster_ratio, 0);
    ADD_FLOAT32_PROP(intersect_ratio, 0);
    ADD_FLOAT32_PROP(intersect_ring_ratio, 0);
    ADD_BOOLEAN_PROP(use1stPlane, 0);
    ADD_SEPARATOR("");
    ADD_FLOAT32_PROP(curvRatio, 0);
    ADD_FLOAT32_PROP(msAngleTollerance, 0);
    ADD_CARD32_PROP(msMaxNIter, 0);
    ADD_FLOAT32_PROP(intersectionAngle, 0);
}

C13MultiScaleFeatureDetector::C13MultiScaleFeatureDetector() 
{
    multiScaleMode = 1;
    minLevel = 1;
    maxLevel = 1000000;
    debugdraw = false;
}

void C13MultiScaleFeatureDetector::assign( const Object* obj, COPY_CONTEXT *context ) 
{
    const C13MultiScaleFeatureDetector *other = dynamic_cast<const C13MultiScaleFeatureDetector*>(obj);
    if (other) {
        copyObjectProperties( obj, this );
    } else {
        pAssert(false);
    }
}

C13MSFeatDetectCrossPoint::C13MSFeatDetectCrossPoint()
{
    curvThreshold = 0.1f;
    curvRatio = 2;
    msAngleTollerance = 20.f;
    msMaxNIter = 20;
    intersectionAngle = 70.f;
    use1stPlane = false;
    cluster_ratio = 10;
    intersect_ratio = 10;
    intersect_ring_ratio = 0.6f;
}

#define radToDeg (180.0f / M_PI)
PointSet* applyMSFilter(PointSet* ps, const float& sigmaSpace, const float& sigmaAlpha, const int& maxMSIterations, const bool& debugdraw)
{
    if (!ps) {
        warning("applyMSFilter - passed nullptr pointer as argument PointSet");
        return nullptr;
    }
    if ((sigmaSpace < 0.0f) || (sigmaAlpha < 0.0f)) {
        warning("applyMSFilter - at least one non meaningful parameter < 0");
        return nullptr;
    }

    AAT oldPosAAT = ps->getDescr()->getToken("position");
    AAT oldTanVAAT = ps->getDescr()->getToken("tangent_v");

    PointSet* result = dynamic_cast<PointSet*>(ps->copy());
    AAT newPosAAt	= result->getDescr()->getToken("position");
    AAT newTanVAAt	= result->getDescr()->getToken("tangent_v");

    const float inv_2_SigmaSpace_2 = 1.0f / (2 * sigmaSpace * sigmaSpace);
    const float inv_2_SigmaAngle_2 = 1.0f / (2 * sigmaAlpha * sigmaAlpha);
    const float radiusSpace = 3 * sigmaSpace;

    std::set<int> markForDeletion;
    progressWindow->pushStep(true, "MS Filtering");
    for (int nMSIter = 0; nMSIter < maxMSIterations; ++nMSIter) {

        if (debugdraw) {
            debugRenderer->beginRenderJob_OneFrame("MS_Filtering_", DR_FRAME++);
        }

        int nPoints = result->getNumEntries();
        AttachedIndexedOctree* octree = new AttachedIndexedOctree();
        octree->setup("SimpleOctree", AttachedData::ADF_QUERY_DELETE_ON_UPDATE,  nullptr);
        octree->octreesetup(ps, 32);
        FastSphereQuerry querry(octree, ps); // query structure fix for each iteration
        for(int ptii = 0; ptii < nPoints; ++ptii) {
            const Vector3f currentPosition = result->get3f(ptii, newPosAAt);
            const Vector3f currentTangent = normalize(result->get3f(ptii, newTanVAAt));

            Vector3f msSpace = NULL_VECTOR3F;
            //Vector3f msDirec = NULL_VECTOR3F;
            float32 weightSum = 0.0f;
            PCA<float,3> pca;

            mpcard numIndices;
            mpcard* indices;
            querry.querry(currentPosition, radiusSpace, &indices, numIndices);
            std::vector<float32> weightvec(numIndices);
            //debugOutput << "\n" << numIndices << "\n";
            for (int nnii = 0; nnii < numIndices; ++nnii) {
                // self skip
                if(ptii == indices[nnii]) continue;
                // space
                const Vector3f neighborPosition = ps->get3f(indices[nnii], newPosAAt);
                const float32 distanceSpaceSqr = normQuad(currentPosition - neighborPosition);
                // range
                const Vector3f neighborTangent = normalize(ps->get3f(indices[nnii], newTanVAAt));
                const float32 dotPCurvedness = std::min(fabs(currentTangent * neighborTangent), 1.0f);
                const float32 angle = acos(dotPCurvedness) * radToDeg;
                //truncation
                if (angle > 2 * sigmaAlpha) continue;
                const float32 weightCurvedness = exp(- sqr(angle) * inv_2_SigmaAngle_2);
                const float32 weightSpace = exp(- distanceSpaceSqr * inv_2_SigmaSpace_2);
                const float32 weight = weightSpace * weightCurvedness;

                msSpace += neighborPosition * weight;
                //msDirec += neighborTangent * weight;
                pca.addPoint(neighborPosition, weight);
                weightSum += weight;
                weightvec[nnii] = weight;
                //debugOutput << distanceSpaceSqr << "\t" << dotPCurvedness << "\t"
                //    << angle << "\t" << weightSpace << "\t" << weightCurvedness << "\t" << weight << "\n";
            }
            //debugOutput << weightSum << "\n\n";

            if (weightSum < 1e-6) {
                // division by zero and no valid neighbor
                markForDeletion.insert(ptii);
                {
                    if (debugdraw /*&& nMSIter == 1*/) {
                        //debugRenderer->beginRenderJob_OneFrame("MS_Filtering_", DR_FRAME++);
                        debugRenderer->addPoint(
                            currentPosition, makeVector3f(1, 0, 0)
                            );
                        debugRenderer->addLine(
                            currentPosition, currentPosition+.1f*currentTangent,
                            makeVector3f(1, 0, 0), makeVector3f(1, 0, 0),
                            2);

                        //debugOutput << numIndices << "\n";
                        //float wtsum = 0.f;
                        //for (int nnii = 0; nnii < numIndices; ++nnii) {
                        //    if(ptii == indices[nnii]) continue;
                        //    const Vector3f neighborPosition = result->get3f(indices[nnii], newPosAAt);
                        //    const Vector3f neighborTangent = normalize(result->get3f(indices[nnii], newTanVAAt));
                        //    debugRenderer->addLine(
                        //        neighborPosition, neighborPosition+0.1f*neighborTangent,
                        //        makeVector3f(0, 1, 0), makeVector3f(0, 1, 0),
                        //        1);

                        //    const float32 distanceSpaceSqr = normQuad(currentPosition - neighborPosition);
                        //    const float32 dotPCurvedness = std::min(fabs(currentTangent * neighborTangent), 1.0f);
                        //    const float32 angle = acos(dotPCurvedness) * radToDeg;
                        //    if (angle > 2 * sigmaAlpha) continue;
                        //    const float32 weightCurvedness = exp(- sqr(angle) * inv_2_SigmaAngle_2);
                        //    const float32 weightSpace = exp(- distanceSpaceSqr * inv_2_SigmaSpace_2);
                        //    const float32 weight = weightSpace * weightCurvedness;
                        //    wtsum += weight;
                        //    debugOutput << distanceSpaceSqr << "\t" << dotPCurvedness << "\t"
                        //        << angle << "\t" << weightSpace << "\t" << weightCurvedness << "\t" << weight << "\n";

                        //    //debugRenderer->addLine(
                        //    //    neighborPosition, neighborPosition+weightvec[nnii]*neighborTangent,
                        //    //    makeVector3f(0, 1, 0), makeVector3f(0, 1, 0),
                        //    //    1);
                        //}
                        //debugRenderer->endRenderJob();
                        //debugOutput << wtsum << "\t" << weightSum << "\n\n";
                    }
                }
                continue;
            }

            const Vector3f diff = (IDENTITY3F - outerProduct(currentTangent, currentTangent)) *
                ((msSpace / weightSum) - currentPosition);
            result->set3f(ptii, newPosAAt, currentPosition+diff);
            //result->set3f(ptii, newTanVAAt, (msDirec+currentTangent) / weightSum);
            Vector3f newtan;
            {
                Vector3f eigenValues,centroid;
                Matrix3f eigenVectors;
                pca.analyze(eigenValues,eigenVectors,centroid);
                newtan = normalize(eigenVectors[0]);
                // only update tangent in the last step, to prevent drifting
                if (nMSIter == maxMSIterations-1) {
                    result->set3f(ptii, newTanVAAt, newtan);
                }

                if (debugdraw) { // local frame at the old position
                    debugRenderer->addLine(
                        currentPosition, currentPosition+.1f*currentTangent,
                        makeVector3f(1, 1, 0), makeVector3f(1, 1, 0),
                        2);
                    debugRenderer->addLine(
                        currentPosition, currentPosition+.1f*eigenVectors[0],
                        makeVector3f(1, 0, 1), makeVector3f(1, 0, 1),
                        2);
                    //debugRenderer->addLine(
                    //    currentPosition, currentPosition+.1f*eigenVectors[1],
                    //    makeVector3f(0, 1, 1), makeVector3f(0, 1, 1),
                    //    2);
                    debugRenderer->addLine(
                        currentPosition, currentPosition+.1f*eigenVectors[2],
                        makeVector3f(0, 1, 1), makeVector3f(0, 1, 1),
                        2);
                }
            }

            //if (ptii == 0)
            //{
            //    debugRenderer->beginRenderJob_OneFrame("MS_Filtering_", DR_FRAME++);
            if (debugdraw) { // new shifted position
                debugRenderer->addPoint(
                    currentPosition+diff, makeVector3f(0, 1, 0)
                    );
                debugRenderer->addLine(
                    currentPosition+diff, currentPosition+diff+.1f*newtan,
                    makeVector3f(0, 1, 0), makeVector3f(0, 1, 0),
                    2);
            }
            //    debugRenderer->addLine(
            //        currentPosition, currentPosition+diff,
            //        makeVector3f(1, 0, 0), makeVector3f(0, 0, 1),
            //        5);
            //    debugRenderer->addLine(
            //        currentPosition, currentPosition+.1f*newtan,
            //        makeVector3f(1, 0, 0), makeVector3f(1, 0, 0),
            //        2);
            //    debugRenderer->endRenderJob();
            //    for (int nnii = 0; nnii < numIndices; ++nnii) {
            //        const Vector3f neighborPosition = result->get3f(indices[nnii], newPosAAt);
            //        const Vector3f neighborTangent = normalize(result->get3f(indices[nnii], newTanVAAt));
            //        debugRenderer->addLine(
            //            neighborPosition, neighborPosition+weightvec[nnii]*neighborTangent,
            //            makeVector3f(0, 1, 0), makeVector3f(0, 1, 0),
            //            1);
            //    }
            //    debugRenderer->endRenderJob();
            //}
        }

        /*remove invalid candidates*/
        //if (nMSIter == 0)
        {
            if (result->getNumEntries() == markForDeletion.size()) break;
            //std::vector<int> markvec(markForDeletion.begin(), markForDeletion.end());
            std::set<int> fullmark, resmark;
            for (int ii = 0; ii < nPoints; ++ii) {
                fullmark.insert(ii);
            }
            std::set_difference(fullmark.begin(), fullmark.end(),
                markForDeletion.begin(), markForDeletion.end(),
                std::inserter(resmark, resmark.end()));
            //PointSet* tmpps = new PointSet();
            //tmpps->clearAndSetup(1, resmark.size(), result->getDescr());
            std::set<int>::iterator it = resmark.begin();
            for (unsigned ii = 0; it != resmark.end(); ++ii, ++it) {
                result->setVertex(ii, result->getDataPointer(*it));

                if (debugdraw) { // result
                    const Vector3f cpos = result->get3f(ii, newPosAAt);
                    const Vector3f ctan = normalize(result->get3f(ii, newTanVAAt));
                    debugRenderer->addPoint(
                        cpos, makeVector3f(0, 0, 1)
                        );
                    debugRenderer->addLine(
                        cpos, cpos-.1f*ctan, // reversed direction
                        makeVector3f(0, 0, 1), makeVector3f(0, 0, 1),
                        2);
                }
            }
            result->changeHeight(resmark.size());
            markForDeletion.clear();
        }

        if (debugdraw) {
            debugRenderer->endRenderJob();
        }

        delete octree;
        progressWindow->progress((float)nMSIter/(float)maxMSIterations);
    }
    progressWindow->popStep();

    return result;
}

LineFeatureSet* clusterLineFeature(PointSet** ps_ref, const float& searchRadius, const float& sigmaAlpha)
{
    LineFeatureSet* lfs = new LineFeatureSet;
    lfs->setup(LineFeatureSet::getDefaultName(), AttachedData::ADF_PERSISTENT, nullptr);

    PointSet* ps = *ps_ref;
    AAT PosAAT = ps->getAAT("position");
    AAT NormAAT = ps->getAAT("normal");
    AAT TanVAAT = ps->getAAT("tangent_v");

    //debugRenderer->beginRenderJob_OneFrame("line_feature", DR_FRAME++);
    for (int index = 0; index < ps->getNumEntries(); ++index) {
        LineFeature* lf = new LineFeature;
        lf->m_Position = ps->get3f(index, PosAAT);
        lf->m_LineDirection = normalize(ps->get3f(index, TanVAAT));
        lf->m_Normal = normalize(ps->get3f(index, NormAAT));
        lf->m_RotationDirection = NULL_VECTOR3F;
        lf->m_Curvature = 0.0f;
        lf->m_Curvature2 = 0.0f;
        lf->m_Sigma = 1.0f;

        lfs->m_LineFeatures.push_back(lf);

        Vector3f temp = lf->m_LineDirection;
        //debugRenderer->addLine(lf->m_Position, lf->m_Position+temp,
        //    makeVector3f(fabs(lf->m_LineDirection[0]),fabs(lf->m_LineDirection[1]),fabs(lf->m_LineDirection[2])),
        //    makeVector3f(fabs(lf->m_LineDirection[0]),fabs(lf->m_LineDirection[1]),fabs(lf->m_LineDirection[2])),
        //    3.0f);
    }
    //debugRenderer->endRenderJob();

    ///////////////////////////////////////////////////////////////////////

    const bool debug_draw = false;
    size_t numLF = lfs->m_LineFeatures.size();
    std::vector<unsigned> indices;
    for (size_t i = 0; i < numLF; ++i)
        indices.push_back(i);

    UnstructuredInCorePointCloud * upc = lfs->getFeaturesAsPointCloud();
    AAT lfPosAAT = upc->getAAT("position");
    FastSphereQuerry querry(upc);
    lfs->m_ClusterID = std::vector<card32>(lfs->m_LineFeatures.size(), 0);
    int currentClusterID = 0;

    const float angleSimilarity = fabs(cos(sigmaAlpha * M_PI / 180.0f));
    for (size_t cii = 0; cii < numLF; ++cii) {
        unsigned currentIndex = indices[cii];
        if( lfs->m_ClusterID[currentIndex] != 0 )
            continue;

        if (debug_draw) {
            debugRenderer->beginRenderJob_OneFrame("line_feature_cluster_", DR_FRAME++);
        }
        Vector3f debugColor = makeVector3f(rnd01(), rnd01(), rnd01());

        // start new cluster
        currentClusterID++;
        lfs->m_ClusterID[currentIndex] = currentClusterID;
        Vector3f rep_nor = lfs->m_LineFeatures[currentIndex]->m_LineDirection;
        int num_clu = 1;

        std::list<unsigned> lf_stack;
        lf_stack.push_back(currentIndex);

        while (!lf_stack.empty()) {
            unsigned ci = lf_stack.front();
            lf_stack.pop_front();

            const Vector3f& cPos = lfs->m_LineFeatures[ci]->m_Position;
            mpcard numIndices;
            mpcard* indices;
            querry.querry(cPos, searchRadius, &indices, numIndices);
            for (int nnii = 0; nnii < numIndices; ++nnii) {
                unsigned nIndex = indices[nnii];
                if(cii == nIndex) continue;
                Vector3f nPos = upc->getPointSet()->get3f(indices[nnii], lfPosAAT);

                LineFeature* tmpLF = lfs->m_LineFeatures[nIndex];
                const Vector3f& n_nor = tmpLF->m_LineDirection;
                if( lfs->m_ClusterID[nIndex] == 0 
                    && (fabs(rep_nor * n_nor) > angleSimilarity))
                {
                    lf_stack.push_back(nIndex);
                    lfs->m_ClusterID[nIndex] = currentClusterID;

                    { // re-estimate representing direction
                        num_clu++;
                        rep_nor /= num_clu;
                        rep_nor *= (num_clu - 1);
                        rep_nor += (n_nor / num_clu);
                    }

                    if (debug_draw) {
                        debugRenderer->addLine(
                            tmpLF->m_Position-n_nor, tmpLF->m_Position+n_nor,
                            debugColor, debugColor, 3.0f);
                    }
                }
            }
        }
        if (debug_draw) {
            debugRenderer->endRenderJob();
        }
    }

    int smallClusters = std::min<float>(100, (float)numLF * 0.01); // remove small clusters (often short sections) really helps pruning false intersections
    const int nFeaturesBefore = lfs->getNumFeatures();
    std::vector<unsigned> updated_indices = lfs->removeSmallClusters(smallClusters, ps_ref);
    const int nFeaturesAfter = lfs->getNumFeatures();
    debugOutput << "remaining feature number: " << nFeaturesAfter << " [" << nFeaturesBefore << "] with threshold " << smallClusters << "\n";

    return lfs;
}

LBaseSet* detectIntersection(PointSet* ps, LineFeatureSet* lfs, const float& searchRadius, const float& intersect_ring_ratio, const float& intersectionAngle)
{
    std::vector<LBase*> resultBases;
    UnstructuredInCorePointCloud * upc = lfs->getFeaturesAsPointCloud();
    AAT posAAT = upc->getAAT("position");
    FastSphereQuerry querry(upc);

    const bool debug_draw = false;
    const float searchRadiusX2 = searchRadius * intersect_ring_ratio;
    std::set<unsigned> baseMap;
    const float intersectionTollerance = fabs(cos(intersectionAngle * M_PI / 180.0f));
    for (int cii = 0; cii < lfs->m_LineFeatures.size(); ++cii) {
        const unsigned& currentClusterID = lfs->m_ClusterID[cii];
        LineFeature* cf = lfs->m_LineFeatures[cii];
        Vector3f cPos = cf->m_Position;

        if (debug_draw) {
            debugRenderer->beginRenderJob_OneFrame("intersections_", DR_FRAME++);
            debugRenderer->addLine(cPos, cPos+cf->m_LineDirection,
                makeVector3f(1, 0, 0), makeVector3f(1, 0, 0),
                3.f);
        }
        mpcard numIndices;
        mpcard* indices;
        std::set< unsigned > false_set; // ring neighborhood: directional estimation within a very small radius can be very unstable
        {
            querry.querry(cPos, searchRadiusX2, &indices, numIndices);
            for (int nnii = 0; nnii < numIndices; ++nnii) {
                false_set.insert(indices[nnii]);
            }
        }
        querry.querry(cPos, searchRadius, &indices, numIndices);
        for (int nnii = 0; nnii < numIndices; ++nnii) {
            const unsigned& nIndex = indices[nnii];
            if (cii == nIndex) continue; // same line
            if (false_set.find(nIndex) != false_set.end()) continue; // within inner part of the ring

            const unsigned& nClusterID = lfs->m_ClusterID[nIndex];
            if (currentClusterID == nClusterID) continue; // same cluster
            const unsigned& baseIndex = currentClusterID * (unsigned)lfs->m_ClusterID.size() + nClusterID;

            LineFeature * nf = lfs->m_LineFeatures[nIndex];
            Vector3f nPos = upc->getPointSet()->get3f(indices[nnii], posAAT);

            if (( fabs(cf->m_LineDirection*nf->m_LineDirection) < intersectionTollerance)
                && (baseMap.find(baseIndex) == baseMap.end())) 
            {
                LBase * base = new LBase;
                base->setup(cf,nf,cii,nIndex,1);
                resultBases.push_back(base);
                baseMap.insert(baseIndex);

                if (debug_draw) {
                    debugRenderer->addLine(nPos, nPos+nf->m_LineDirection,
                        makeVector3f(0, 1, 0), makeVector3f(0, 1, 0),
                        3.f);
                    //debugRenderer->addSphere(
                    //    base->getBasePoint(), 0.2f,
                    //    makeVector3f(1.f, 0.f, 0.f));
                }
            }
            else {
                if (debug_draw) {
                    debugRenderer->addLine(nPos, nPos+nf->m_LineDirection,
                        makeVector3f(0, 0, 1), makeVector3f(0, 0, 1),
                        3.f);
                }
            }
        }
        if (debug_draw) {
            debugRenderer->endRenderJob();
        }
    }

    const int nBases = (int)resultBases.size();
    LBaseSet* lbs = new LBaseSet;
    lbs->setup( LBaseSet::getDefaultName(), AttachedData::ADF_PERSISTENT, nullptr);
    lbs->setBaseVector(resultBases);

    //debugRenderer->beginRenderJob_OneFrame("intersections_", DR_FRAME++);
    //for (int index = 0; index < resultBases.size(); ++index) {
    //    debugRenderer->addSphere(
    //        resultBases[index]->getBasePoint(), 0.2f,
    //        makeVector3f(1.f, 0.f, 0.f));
    //}
    //debugRenderer->endRenderJob();

    return lbs;
}

PointCloud *C13MSFeatDetectCrossPoint::detectFeatures(MultiScalePointCloud *source)
{
    MultiScaleInCorePointCloud *incore = dynamic_cast<MultiScaleInCorePointCloud*>(source);
    if (!incore) {
        error("C13MSFeatDetectCrossPoint::detectFeatures - needs MultiScaleInCorePointCloud at this point.");
        return nullptr;
    }
    if (incore->getNumLevels() == 0) {
        error("C13MSFeatDetectCrossPoint::detectFeatures - source is empty");
        return nullptr;
    }

    VertexDescriptor vd;
    vd.pushAttrib( mVAD("position", 3, VAD::DATA_FORMAT_FLOAT32) );
    vd.pushAttrib( mVAD("normal", 3, VAD::DATA_FORMAT_FLOAT32) );
    vd.pushAttrib( mVAD("tangent_v", 3, VAD::DATA_FORMAT_FLOAT32) );
    vd.pushAttrib( mVAD("scale",    1, VAD::DATA_FORMAT_FLOAT32) );
    vd.pushAttrib( mVAD("level",    1, VAD::DATA_FORMAT_CARD8) );
    vd.pushAttrib( mVAD("index",    1, VAD::DATA_FORMAT_CARD32) );

    MultiScaleInCorePointCloud *resultMSPC = new MultiScaleInCorePointCloud();
    resultMSPC->clearAndSetup(&vd);
    std::deque<PointSet*> psvec;
    std::deque<LineFeatureSet*> lfsvec;
    std::deque<LBaseSet*> lbsvec;
    float sampleSpacingFact = 2.f;

    card32 level = 0;
    card32 endLevel;
    if (multiScaleMode == MSMODE_MIN_LEVEL_ONLY) {
        endLevel = level + 1;
    } else {
        endLevel = std::min(maxLevel+1, incore->getNumLevels());
    }
    while (level < endLevel) {

        const PointSet *currIncorePS = incore->getLevel(level);
        if (!currIncorePS->getDescr()->providesAttribute("curv2", 2, VAD::DATA_FORMAT_FLOAT32)) {
            error("C13MSFeatDetectCrossPoint::detectFeatures - source has no curv2 channel");
            return nullptr;
        }
        PlaneClusterExtractor* plane_cluster;
        Plane3f first_plane;
        float plane_space;
        if (use1stPlane) { // only use the first plane in case of major floor plane
            plane_cluster = dynamic_cast<PlaneClusterExtractor*>(
                incore->getAttachedData(level, PlaneClusterExtractor::getDefaultName()));
            if (!plane_cluster) {
                error("C13MSFeatDetectCrossPoint::detectFeatures - attachment PlaneClusterExtractor missing.");
                return nullptr;
            }
            plane_space = plane_cluster->space_th_;
            const PlaneCluster& cluster = (*plane_cluster)[0];
            first_plane = cluster.plane_;
        }

        float32 levelExp = float32(level);
        if (levelExp <= 0) levelExp = 0;
        float32 levelFact = pow(incore->getMultiScaleParameters()->levelFactor, levelExp);
        float32 resampleSpacing = incore->getMultiScaleParameters()->baseGridSpacing * levelFact * sampleSpacingFact;
        UnstructuredInCorePointCloud *singleLevelPC = new UnstructuredInCorePointCloud;
        singleLevelPC->clearAndSetup(&vd, currIncorePS->getNumEntries());
        PointSet *singleLevelPS = singleLevelPC->getPointSet();
        AAT sourcePositionAAT = currIncorePS->getAAT("position", 3, VAD::DATA_FORMAT_FLOAT32);
        AAT   sourceNormalAAT = currIncorePS->getAAT("normal", 3, VAD::DATA_FORMAT_FLOAT32);
        AAT    sourceCurv2AAT = currIncorePS->getAAT("curv2", 2, VAD::DATA_FORMAT_FLOAT32);
        AAT sourceTangentVAAT = currIncorePS->getAAT("tangent_v", 3, VAD::DATA_FORMAT_FLOAT32);
        AAT resultPositionAAT = singleLevelPS->getAAT("position", 3, VAD::DATA_FORMAT_FLOAT32);
        AAT   resultNormalAAT = singleLevelPS->getAAT("normal", 3, VAD::DATA_FORMAT_FLOAT32);
        AAT    resultScaleAAT = singleLevelPS->getAAT("scale", 1, VAD::DATA_FORMAT_FLOAT32);
        AAT    resultLevelAAT = singleLevelPS->getAAT("level", 1, VAD::DATA_FORMAT_CARD8);
        AAT    resultIndexAAT = singleLevelPS->getAAT("index", 1, VAD::DATA_FORMAT_CARD32);
        AAT resultTangentVAAT = singleLevelPS->getAAT("tangent_v", 3, VAD::DATA_FORMAT_FLOAT32);

        mpcard n = currIncorePS->getNumEntries();
        mpcard actualNumberOfEntries = 0;
        for (mpcard i=0; i<n; i++) {
            Vector2f curv2;
            curv2[0] = currIncorePS->get2f(i, sourceCurv2AAT)[0];
            curv2[1] = currIncorePS->get2f(i, sourceCurv2AAT)[1];
            if (abs(curv2[0]) < curvThreshold
                || abs(curv2[0]) / abs(curv2[1]) < curvRatio
                ) continue;

            const Vector3f& pos = currIncorePS->get3f(i, sourcePositionAAT);
            if (use1stPlane) {
                if (first_plane.getDistanceFromPlane(pos) < plane_space) continue;
            }

            singleLevelPS->set3f(actualNumberOfEntries, resultPositionAAT, pos);
            singleLevelPS->set3f(actualNumberOfEntries, resultNormalAAT, currIncorePS->get3f(i, sourceNormalAAT));
            singleLevelPS->set3f(actualNumberOfEntries, resultTangentVAAT, currIncorePS->get3f(i, sourceTangentVAAT));
            singleLevelPS->set1f(actualNumberOfEntries, resultScaleAAT, resampleSpacing);
            singleLevelPS->set1ub(actualNumberOfEntries, resultLevelAAT, level);
            singleLevelPS->set1u(actualNumberOfEntries, resultIndexAAT, i);
            ++actualNumberOfEntries;
        }
        debugOutput << "On level " << level << ": selected " << actualNumberOfEntries << " out of " << n << " features\n";
        singleLevelPS->changeHeight(actualNumberOfEntries);
        BoundingBox3f bb_ps = singleLevelPS->getBoundingBox();

        /*add boundary points*/
        //{
        //    PCCBoundaryDetector cmd;
        //    UnstructuredInCorePointCloud* bPC = new UnstructuredInCorePointCloud;
        //    bPC->clearAndSetup(&vd, currIncorePS->getNumEntries());
        //    PointSet *bPS = (PointSet*)currIncorePS->copy();
        //    bPC->setPointSet(bPS);
        //    cmd.detectBoundaries(bPC);
        //    bPS = bPC->getPointSet();
        //    AAT bPosAAT = bPS->getAAT("position");
        //    AAT bNormAAT = bPS->getAAT("normal");
        //    AAT bBoundaryAAT = bPS->getAAT("boundary");
        //    AAT bBoundaryTangentsAAT = bPS->getAAT("boundaryTangent");
        //    char* buffer = new char[currIncorePS->getDescr()->getSize()];
        //    for (mpcard i = 0; i < bPC->getNumPoints(); ++i) {
        //        float boundary = bPS->get1f(i, bBoundaryAAT);
        //        if (boundary < .8f) continue;
        //        singleLevelPS->addPoint(buffer);
        //        singleLevelPS->set3f(actualNumberOfEntries, resultPositionAAT, currIncorePS->get3f(i, bPosAAT));
        //        singleLevelPS->set3f(actualNumberOfEntries, resultNormalAAT, currIncorePS->get3f(i, bNormAAT));
        //        singleLevelPS->set3f(actualNumberOfEntries, resultTangentVAAT, currIncorePS->get3f(i, bBoundaryTangentsAAT));
        //        singleLevelPS->set1u(actualNumberOfEntries, resultIndexAAT, actualNumberOfEntries);
        //        ++actualNumberOfEntries;
        //    }
        //    delete bPC;
        //    delete buffer;
        //}

        /*MS Filtering*/
        {
            PointSet* newps = applyMSFilter(singleLevelPS, resampleSpacing, msAngleTollerance, msMaxNIter, debugdraw);
            delete singleLevelPS;
            singleLevelPS = newps;
            debugOutput << "On level " << level << ": " << newps->getNumEntries() << " points after MS filtering\n";
        }

        /*Cluster line features*/
        {
            const float cluster_radius = resampleSpacing * cluster_ratio;
            const float intersect_radius = resampleSpacing * intersect_ratio;
            debugOutput << "cluster radius: " << cluster_radius << "; intersection radius: " << intersect_radius << "\n";
            LineFeatureSet* lfs = clusterLineFeature(&singleLevelPS, cluster_radius, msAngleTollerance);
            LBaseSet* lbs = detectIntersection(singleLevelPS, lfs, intersect_radius, intersect_ring_ratio, intersectionAngle);
            lfsvec.push_back(lfs);
            lbsvec.push_back(lbs);
        }

        psvec.push_back(singleLevelPS);

        /*attach supporting point indices*/
        {
            C13DescriptorAttachment* att = new C13DescriptorAttachment;
            att->setup(C13DescriptorAttachment::getDefaultName(), AttachedData::ADF_PERSISTENT);
            att->level = level;
            att->scale = resampleSpacing;
            att->lowerCorner = bb_ps.lowerCorner;
            att->upperCorner = bb_ps.upperCorner;
            incore->attachData(att, level);
        }

        ++level;
    }

    MultiScalePCParams *params = (MultiScalePCParams*)incore->getMultiScaleParameters()->copy();
    resultMSPC->build(params, psvec);
    delete params;
    for (unsigned l = 0; l < endLevel; ++l) {
        resultMSPC->attachData(lfsvec[l], l);
        resultMSPC->attachData(lbsvec[l], l);
    }

    return resultMSPC;
}
