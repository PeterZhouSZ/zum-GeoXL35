//---------------------------------------------------------------------------
#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "C13MultiScaleFeatureDescriptor.h"
#include "C13DescriptorAttachment.h"
//---------------------------------------------------------------------------
#include "CopyObjectProperties.h"
#include "RegressionPlane.h"
#include "FastSphereQuerry.h"
#include <float.h>
#include "PointSetKNNQuery.h"
#include "MultiScaleInCorePointCloud.h"
#include "SVD.h"
#include "Util/FastEMD/emd_hat.hpp"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

static std::string describeMultiScaleModes(card32 i) {
    if (i==0) return "MSMODE_FIXED_NUMBINS_EXPANDING_SUPPORT";
    if (i==1) return "MSMODE_FIXED_SUPPORT_SHRINKING_NUMBINS";
    if (i==2) return "MSMODE_INDEPENDENT_DESCRIPTORS_PER_LEVEL";
    return "undefined";
}

IMPLEMENT_ABSTRACT_CLASS( C13MultiScaleFeatureDescriptor , 0 )
{
    BEGIN_CLASS_INIT( C13MultiScaleFeatureDescriptor );
}

IMPLEMENT_CLASS( C13MSFeatDescrHOG , 0 )
{
    BEGIN_CLASS_INIT( C13MSFeatDescrHOG );
    ADD_CARD32_PROP_UPDATE_DESCR(multiScaleMode, 0, 0, describeMultiScaleModes, 3, nullptr)
        ADD_CARD32_PROP(numOrientationBins, 0)
        ADD_CARD32_PROP(numSpatialBins, 0)
        ADD_CARD32_PROP(numVerticalBins, 0)
        ADD_FLOAT32_PROP(smoothness, 0)
        ADD_BOOLEAN_PROP(normalizeDescriptor, 0)
        ADD_CARD32_PROP(pcaDimension, 0)
        ADD_VECTOR3F_PROP(canonicalUpward, 0)
        ADD_BOOLEAN_PROP(useCanonicalUpward, 0)
        ADD_BOOLEAN_PROP(debugVis, 0)
        ADD_BOOLEAN_PROP(useDiffusion, 0)
        ADD_FLOAT32_PROP(diffusionDelta, 0)
}

// ---


void C13MultiScaleFeatureDescriptor::assign( const Object* obj, COPY_CONTEXT *context ) 
{
    const C13MultiScaleFeatureDescriptor *other = dynamic_cast<const C13MultiScaleFeatureDescriptor*>(obj);
    if (other) {
        copyObjectProperties( obj, this );
    } else {
        pAssert(false);
    }
}




C13MSFeatDescrHOG::C13MSFeatDescrHOG(const bool useCurvature)
{
    multiScaleMode = MSMODE_INDEPENDENT_DESCRIPTORS_PER_LEVEL;
    numOrientationBins = 8;
    numSpatialBins = 4;
    smoothness = 1.0f;
    numVerticalBins = 1;
    normalizeDescriptor = true;
    canonicalUpward = makeVector3f(0,1,0);
    useCanonicalUpward = true;
    pcaDimension = 16;
    debugVis = false;
    useDiffusion = false;
    diffusionDelta = 0.1f;
    this->useCurvature = useCurvature;
}


C13MSFeatDescrHOG::~C13MSFeatDescrHOG()
{
}

void updateChannel(const DMatrixF& descriptor, PointSet* featurePS)
{
    AAT descriptorsAAT = featurePS->getAAT("descriptors");
    AAT colorAAT = featurePS->getAAT("color", 3, VAD::DATA_FORMAT_FLOAT32);

    float maxdc = FLT_MIN, mindc = FLT_MAX;
    mpcard numFeatures = descriptor.getColumns();
    mpcard descrStorageDim = descriptor.getRows();
    for (mpcard i=0; i<numFeatures; i++) {
        card8 *ptr = (card8*)featurePS->getDataPointer(i);
        ptr += descriptorsAAT.getOffset();
        float32 *fptr = (float32*)ptr;
        for (mpcard j=0; j<descrStorageDim; j++) {
            *fptr = descriptor[i][j];
            if (*fptr < mindc) mindc = *fptr;
            if (*fptr > maxdc) maxdc = *fptr;
            //if (i == 0) debugOutput << *fptr << " ";
            fptr++;
        }
    }
    //debugOutput << "\n";

    float rangedc = maxdc - mindc;
    for (mpcard i=0; i<numFeatures; i++) {
        card8 *ptr = (card8*)featurePS->getDataPointer(i);
        ptr += descriptorsAAT.getOffset();
        float32 *fptr = (float32*)ptr;
        Vector3f color = makeVector3f(
            (fptr[0]-mindc)/rangedc, (fptr[1]-mindc)/rangedc, (fptr[2]-mindc)/rangedc);
        featurePS->set3f(i, colorAAT, color);
    }
}

void C13MSFeatDescrHOG::computeDescriptors(MultiScalePointCloud *geometry, PointCloud *featurePoints)
{
    if (multiScaleMode != MSMODE_INDEPENDENT_DESCRIPTORS_PER_LEVEL) {
        notImplemented();
    }
    if (!useCanonicalUpward) {
        notImplemented();
    }
    UnstructuredInCorePointCloud *featureUPC = dynamic_cast<UnstructuredInCorePointCloud*>(featurePoints);
    MultiScaleInCorePointCloud *featureMSPC = dynamic_cast<MultiScaleInCorePointCloud*>(featurePoints);
    if (!featureUPC && !featureMSPC) {
        error("MSFeatDescrHOG::computeDescriptors - feature missing.");
        return;
    }
    if (featurePoints->providesAttribute("descriptors")) {
        warning("descriptor channel exists already - if the dimension changed, the code might not work (please remove channel manually).");
    }
    MultiScaleInCorePointCloud *geometryIPC = dynamic_cast<MultiScaleInCorePointCloud*>(geometry);
    if (!geometryIPC) {
        error("MSFeatDescrHOG::computeDescriptors - sorry, i can currently only handle in-core multi-scale point clouds as geometry.");
        return;
    }
    mpcard numLevels = geometryIPC->getNumLevels();
    if (numLevels==0) {
        error("MSFeatDescrHOG::computeDescriptors - no ms-levels in geometry");
        return;
    }
    std::vector<MSHOGLevelInfo> levelInfos(numLevels);
    for (mpcard i=0; i<numLevels; i++) {
        levelInfos[i].upc = new UnstructuredInCorePointCloud;
        levelInfos[i].upc->setPointSet((PointSet*)geometryIPC->getLevel(i)->copy());
        levelInfos[i].detector = new SingleScaleDetector(levelInfos[i].upc);
    }

    mpcard descrDim = numOrientationBins*numSpatialBins*numSpatialBins*numVerticalBins;
    card32 descrStorageDim = pcaDimension;
    if (descrStorageDim > descrDim) descrStorageDim = descrDim;
    if (descrStorageDim > 255) descrStorageDim = 255; // technical limitation; channel can have only dim up to 255 
    if (featureUPC) {
        checkAttribute(featureUPC, "descriptors", descrStorageDim, VAD::DATA_FORMAT_FLOAT32);
        checkAttribute(featureUPC, "color", 3, VAD::DATA_FORMAT_FLOAT32);
        checkAttribute(featureUPC, "radius", 1, VAD::DATA_FORMAT_FLOAT32);
        PointSet* featurePS = featureUPC->getPointSet(); // update after channel change
        C13DescriptorAttachment* att = dynamic_cast<C13DescriptorAttachment*>(
            geometryIPC->getAttachments()->getData(C13DescriptorAttachment::getDefaultName()));
        if (att == nullptr) {
            error("MSFeatDescrHOG::computeDescriptors - attachment not found.");
            return;
        } else {
            computeDescriptors(levelInfos, featurePS, att, descrDim, descrStorageDim);
            DMatrixF descriptor = att->getDescriptor();
            updateChannel(descriptor, featurePS);
        }
    } else if (featureMSPC) {
        checkAttribute(featureMSPC, "descriptors", descrStorageDim, VAD::DATA_FORMAT_FLOAT32);
        checkAttribute(featureMSPC, "color", 3, VAD::DATA_FORMAT_FLOAT32);
        checkAttribute(featureMSPC, "radius", 1, VAD::DATA_FORMAT_FLOAT32);
        for (unsigned ii = 0; ii < featureMSPC->getNumLevels(); ++ii) {
            PointSet *featurePS = featureMSPC->getLevel(ii);
            C13DescriptorAttachment* att = dynamic_cast<C13DescriptorAttachment*>(
                geometryIPC->getAttachedData(ii, C13DescriptorAttachment::getDefaultName()));
            if (att == nullptr) {
                error("MSFeatDescrHOG::computeDescriptors - attachment not found.");
                return;
            }
            computeDescriptors(levelInfos, featurePS, att, descrDim, descrStorageDim);
            DMatrixF descriptor = att->getDescriptor();
            updateChannel(descriptor, featurePS);
        }
    }

    for (mpcard i=0; i<numLevels; i++) {
        delete levelInfos[i].detector;
        delete levelInfos[i].upc;
    }
}

void C13MSFeatDescrHOG::computeDescriptors(
    const std::vector<MSHOGLevelInfo>& levelInfos, PointSet *featurePS, C13DescriptorAttachment* att,
    mpcard descrDim, mpcard descrStorageDim)
{
    mpcard numLevels = levelInfos.size();
    mpcard numFeatures = featurePS->getNumEntries();
    if (descrStorageDim > numFeatures-1) {
        warning("C13MSFeatDescrHOG::computeDescriptors - too few feature points");
        return;
    }
    AAT positionAAT = featurePS->getAAT("position", 3, VAD::DATA_FORMAT_FLOAT32);
    AAT    scaleAAT = featurePS->getAAT("scale",    1, VAD::DATA_FORMAT_FLOAT32);
    AAT    levelAAT = featurePS->getAAT("level",    1, VAD::DATA_FORMAT_CARD8);
    AAT   radiusAAT = featurePS->getAAT("radius",   1, VAD::DATA_FORMAT_FLOAT32);
    bool firstWarning = true;
    DVectorF resultDescr;
    DMatrixF results(numFeatures,descrDim);
    float localRadius;
    for (mpcard f=0; f<numFeatures; f++) {
        Vector3f pos   = featurePS->get3f (f, positionAAT);
        float32  scale = featurePS->get1f (f, scaleAAT);
        card32   level = featurePS->get1ub(f, levelAAT);
        if (level >= numLevels) {
            if (firstWarning) {warning("level out of range in feature point cloud"); firstWarning = false;}
            level = numLevels-1;
        }
        float32 fittingRadius = scale*numSpatialBins;
        Vector3f normal = levelInfos[level].detector->getAverageNormal(pos, fittingRadius);
        Vector3f tangentU = canonicalUpward.crossProduct(normal);
        {
            float32 lu=norm(tangentU);
            if (lu >0.01) {
                tangentU *= 1.0f/lu;
            } else {
                if (firstWarning) {warning("normal parallel to upward direction"); firstWarning = false;}
                tangentU = makeVector3f(1,0,0); // Make something up so we can continue somewhat gracefully
            }
        }
        float32 radius = scale*numSpatialBins; // diameter is 2x radius, so no x2 here
        levelInfos[level].detector->getDescriptor(
            pos, normal, tangentU, radius, 
            resultDescr, localRadius,
            numSpatialBins, numVerticalBins, numOrientationBins, normalizeDescriptor, smoothness,
            debugVis,
            //((f%200)==42)?debugVis:0,
            useCurvature);
        results[f] = resultDescr;
        featurePS->set1f(f, radiusAAT, localRadius);
    }

    DMatrixF basis;
    DVectorF singularValues;
    DMatrixF descriptor(numFeatures,descrStorageDim);
    if (useDiffusion) {
        DMatrixF affinity;
        affinity.setDimension(numFeatures, numFeatures);

        //std::vector< std::vector<double> > cost_emd;
        //cost_emd.resize(descrDim);
        //for (unsigned ii = 0; ii < descrDim; ++ii) {
        //    cost_emd[ii].resize(descrDim);
        //}
        //unsigned k1 = 0;
        //for (int d1=0; d1<numVerticalBins; d1++) {
        //    for (int v1=0; v1<numSpatialBins; v1++) {
        //        for (int u1=0; u1<numSpatialBins; u1++) {
        //            for (int o1=0; o1<numOrientationBins; o1++) {
        //                unsigned k2 = 0;
        //                for (int d2=0; d2<numVerticalBins; d2++) {
        //                    for (int v2=0; v2<numSpatialBins; v2++) {
        //                        for (int u2=0; u2<numSpatialBins; u2++) {
        //                            for (int o2=0; o2<numOrientationBins; o2++) {
        //                                cost_emd[k1][k2] = abs(d1-d2)+abs(u1-u2)+abs(v1-v2)+(abs(o1-o2)%numOrientationBins);
        //                                ++k2;
        //                            }
        //                        }
        //                    }
        //                }
        //                ++k1;
        //            }
        //        }
        //    }
        //}
        for (unsigned c = 0; c < numFeatures; ++c) {
            //std::vector<double> fc;
            //fc.resize(descrDim);
            //for (unsigned ii = 0; ii < descrDim; ++ii) fc[ii] = results[c][ii];
            for (unsigned r = c; r < numFeatures; ++r) {
                //std::vector<double> fr;
                //fr.resize(descrDim);
                //for (unsigned ii = 0; ii < descrDim; ++ii) fr[ii] = results[r][ii];
                //double val= emd_hat_gd_metric<double>()(fr,fc, cost_emd);
                double val = normQuad(results[c]-results[r]);
                affinity[r][c] = exp(-val/(2*diffusionDelta));
                affinity[c][r] = affinity[r][c];
            }
        }
        for (unsigned c = 0; c < numFeatures; ++c) {
            float s = 0;
            for (unsigned r = 0; r < numFeatures; ++r) {
                s += affinity[c][r];
            }
            for (unsigned r = 0; r < numFeatures; ++r) {
                affinity[c][r] /= s;
            }
        }
        SVD<float32>::computeSVD_NoMatV(affinity, basis, singularValues);
        //if (debugVis) {
        debugOutput << "eigenvalues of affinity: " << singularValues << "\n";
        //}
        int ts = 3;
        DVectorF eval(singularValues.size());
        for (unsigned ii = 0; ii < singularValues.size(); ++ii)
            eval[ii] = power(singularValues[ii], ts);
        for (mpcard i=0; i<numFeatures; i++) {
            for (mpcard j=0; j<descrStorageDim; j++) {
                descriptor[i][j] = eval[j+1]*basis[j+1][i]; // skip the first
            }
        }
    } else {
        SVD<float32>::computeSVD_NoMatV(results, basis, singularValues);
        //if (debugVis) {
        debugOutput << "eigenvalues of pca: " << singularValues << "\n";
        //}
        for (mpcard i=0; i<numFeatures; i++) {
            for (mpcard j=0; j<descrStorageDim; j++) {
                descriptor[i][j] = results[i]*basis[j];
            }
        }
    }
    DMatrixF basisRed(descrStorageDim, descrDim);
    for (mpcard i=0; i<descrStorageDim; i++) {
        basisRed[i] = basis[i];
    }

    att->setPcaBasis(basisRed);
    att->setEigValue(singularValues);
    att->setDescriptor(descriptor);
}


// ----


C13MSFeatDescrHOG::SingleScaleDetector::SingleScaleDetector( UnstructuredInCorePointCloud *upc )
{
    if (upc->getPointSet()->providesAttribute("position", 3, VAD::DATA_FORMAT_FLOAT32)
        && upc->getPointSet()->providesAttribute("curv2", 2, VAD::DATA_FORMAT_FLOAT32)
        && upc->getPointSet()->providesAttribute("tangent_u", 3, VAD::DATA_FORMAT_FLOAT32)
        && upc->getPointSet()->providesAttribute("normal", 3, VAD::DATA_FORMAT_FLOAT32)) 
    {
        this->upc = upc;
        ps = upc->getPointSet();
        tangentUAAT = ps->getAAT("tangent_u");
        positionAAT = ps->getAAT("position");
        curv2AAT    = ps->getAAT("curv2");
        normalAAT   = ps->getAAT("normal");
        query = new FastSphereQuerry(upc);
        knnOctree = PointSetKNNQuery::createOctree(ps);
        knn = new PointSetKNNQuery(ps, knnOctree, NULL_VECTOR3F, positionAAT);
    } else {
        throw ETypeMismatch("MSFeatDescrHOG::SingleScaleDetector::SingleScaleDetector() - channel missing. Need: position:3f, curv2:2f, tangent_u:3f, normal:3f.");
    }
    singleWarning = true;
}

void C13MSFeatDescrHOG::SingleScaleDetector::getDescriptor(
    const Vector3f& pos, const Vector3f& normal, const Vector3f &tangentU, float32 radius, 
    DVectorF &result, float32 &localRadius,
    card32 numSpatialBins, card32 numVerticalBins, card32 numOrientationBins,
    bool normalizeDescriptor, float32 smoothness,
    bool debugVis,
    bool useCurvature)
{
    Vector3f tangentV = normalize( normal.crossProduct(tangentU) );

    /// building descriptor
    const float32  spatialBinSpacing = 2*radius / numSpatialBins;
    const float32  verticalRadius    = numVerticalBins*spatialBinSpacing/2.0f;
    const Vector3f lowerCorner       = pos - tangentU*radius - tangentV*radius - normal*verticalRadius;
    const Vector3f upperCorner       = pos + tangentU*radius + tangentV*radius + normal*verticalRadius;
    const Vector3f center			 = (lowerCorner + upperCorner) * 0.5f;
    const float32  binRadius         = spatialBinSpacing * 2.0f * smoothness;
    const float32  oBinSpacing       = 1.0f / numOrientationBins;

    result.setDim(numSpatialBins*numSpatialBins*numVerticalBins*numOrientationBins);
    result.setZero();

    // First find all points that the hog should be build over
    mpcard numAllPoints;
    mpcard *allIndices;
    float32 allRadius = binRadius + radius; // TODO (daniel): This is a bit too large.
    localRadius = binRadius;
    query->querry(center, allRadius, &allIndices, numAllPoints);
    std::vector<mpcard> buffer(numAllPoints);
    std::vector<Vector3f> allPositions(numAllPoints);
    for (size_t i = 0; i < numAllPoints; ++i) {
        Vector3f position = ps->get3f(allIndices[i], positionAAT);
        if (numVerticalBins == 1) {
            // Project positions onto plane
            position -= (normal * (position * normal));
        }
        allPositions[i] = position;
    }

    Vector3f binPosN = lowerCorner;
    card32 baseIndex = 0;
    for (card32 nd=0; nd<numVerticalBins; nd++) {
        Vector3f binPosV = binPosN;
        for (card32 v=0; v<numSpatialBins; v++) {
            Vector3f binPosU = binPosV;
            for (card32 u=0; u<numSpatialBins; u++) {

                mpcard numLocalPoints;
                mpcard *localIndices;
                /*query->querry(binPosU, binRadius, &localIndices, numLocalPoints);*/
                {
                    numLocalPoints = 0;
                    localIndices = (numAllPoints > 0) ? (&buffer[0]) : nullptr;
                    Vector3f queryCenter = binPosU;
                    if (numVerticalBins == 1) {
                        // Project positions onto plane
                        queryCenter -= (normal * (queryCenter * normal));
                    }
                    for (size_t i = 0; i < numAllPoints; ++i) {
                        float32 dist = (allPositions[i] - queryCenter).getNorm();
                        if (dist < binRadius) {
                            localIndices[numLocalPoints++] = allIndices[i];
                        }
                    }
                }

                for (mpcard j=0; j<numLocalPoints; j++) {
                    Vector3f localPosVec  = ps->get3f(localIndices[j], positionAAT);

                    float32  magnitude;
                    Vector2f localCoords;
                    if (useCurvature) {
                        Vector3f localCurvVec = ps->get3f(localIndices[j], tangentUAAT);
                        if (norm(localCurvVec) < 1e-5) continue; // just in case, should not happen
                        localCoords  = makeVector2f(tangentU*localCurvVec, tangentV*localCurvVec);
                        Vector2f curv2 = ps->get2f(localIndices[j], curv2AAT);
                        magnitude = curv2[0]-curv2[1];
                        if (magnitude < 0) magnitude = -magnitude;
                        if (_isnan(magnitude)) {
                            magnitude = 0;
                            if (singleWarning) {
                                warning("NaN principal curvature encountered - this is not in Indian restaurant!");
                                singleWarning = false;
                            }
                        }
                    } else {
                        magnitude = 1.0f;
                        Vector3f localNormalVec = ps->get3f(localIndices[j], normalAAT);
                        if (norm(localNormalVec) < 1e-5) continue;
                        localCoords  = makeVector2f(tangentU*localNormalVec, tangentV*localNormalVec);
                    }

                    float32 sBinWeight = exp(-((localPosVec-binPosU)*(localPosVec-binPosU))/sqr(spatialBinSpacing*smoothness));

                    float32 angle = atan2(localCoords[0],localCoords[1]) / (float32)M_PI;
                    if (angle > 1.0f) angle -= 1.0f;
                    for (card32 oBin=0; oBin<numOrientationBins; oBin++) {
                        float32 oBinAngle = (float32)oBin / (float32)numOrientationBins;
                        float32 oBinWeight = exp(-sqr(angle-oBinAngle)/sqr(oBinSpacing*smoothness));
                        result[baseIndex+oBin] += oBinWeight*sBinWeight*magnitude; // TODO (daniel): Support 360deg bins using the sign of the magnitude.
                    }
                }
                baseIndex += numOrientationBins;
                binPosU += spatialBinSpacing*tangentU;
            }      
            binPosV += spatialBinSpacing*tangentV;
        }
        binPosN += spatialBinSpacing*normal;
    }
    if (normalizeDescriptor) {
        float32 m = 0;
        for (mpcard i=0; i<result.getDim(); i++) {
            m += result[i];
        }
        if (m>= 1e-7) result /= m;
    }

    if (debugVis) {
        debugRenderer->beginRenderJob_OneFrame("HOC_descriptors_", DR_FRAME++);
        //debugRenderer->beginRenderJob("HOC descriptors");
        Matrix3f frameRot;
        frameRot[0] = tangentU;
        frameRot[1] = tangentV;
        frameRot[2] = normal;

        Vector3f binPosN = lowerCorner;
        card32 baseIndex = 0;

        for (card32 nd=0; nd<numVerticalBins; nd++) {
            Vector3f binPosV = binPosN;
            for (card32 v=0; v<numSpatialBins; v++) {
                debugRenderer->addLine(
                    pos-tangentU*radius-tangentV*radius+normal*verticalRadius, 
                    pos+tangentU*radius-tangentV*radius+normal*verticalRadius, 
                    makeVector3f(0,0.5f,0), makeVector3f(0,0.5f,0), 2);
                debugRenderer->addLine(
                    pos+tangentU*radius-tangentV*radius+normal*verticalRadius, 
                    pos+tangentU*radius+tangentV*radius+normal*verticalRadius, 
                    makeVector3f(0,0.5f,0), makeVector3f(0,0.5f,0), 2);
                debugRenderer->addLine(
                    pos+tangentU*radius+tangentV*radius+normal*verticalRadius, 
                    pos-tangentU*radius+tangentV*radius+normal*verticalRadius, 
                    makeVector3f(0,0.5f,0), makeVector3f(0,0.5f,0), 2);
                debugRenderer->addLine(
                    pos-tangentU*radius+tangentV*radius+normal*verticalRadius, 
                    pos-tangentU*radius-tangentV*radius+normal*verticalRadius, 
                    makeVector3f(0,0.5f,0), makeVector3f(0,0.5f,0), 2);

                Vector3f binPosU = binPosV;
                for (card32 u=0; u<numSpatialBins; u++) {

                    debugRenderer->addLine(
                        binPosU, 
                        binPosU+spatialBinSpacing*tangentU, 
                        makeVector3f(0.5f,0,0), makeVector3f(0.5f,0,0), 1);
                    debugRenderer->addLine(
                        binPosU+spatialBinSpacing*tangentU, 
                        binPosU+spatialBinSpacing*tangentU+spatialBinSpacing*tangentV, 
                        makeVector3f(0.5f,0,0), makeVector3f(0.5f,0,0), 1);
                    debugRenderer->addLine(
                        binPosU+spatialBinSpacing*tangentU+spatialBinSpacing*tangentV, 
                        binPosU+spatialBinSpacing*tangentV, 
                        makeVector3f(0.5f,0,0), makeVector3f(0.5f,0,0), 1);
                    debugRenderer->addLine(
                        binPosU+spatialBinSpacing*tangentV, 
                        binPosU, 
                        makeVector3f(0.5f,0,0), makeVector3f(0.5f,0,0), 1);
                    for (card32 oBin=0; oBin<numOrientationBins; oBin++) {
                        float32 oBinAngle = (float32)oBin / (float32)numOrientationBins * (float32)M_PI;
                        Vector3f startPoint = binPosU + frameRot*((XAXIS_VECTOR3F+YAXIS_VECTOR3F)*spatialBinSpacing*0.5f);
                        float32 magnitude = result[baseIndex+oBin]*numOrientationBins*numSpatialBins*0.5f;
                        Vector3f endPoint = startPoint + frameRot*((XAXIS_VECTOR3F*cos(oBinAngle) + YAXIS_VECTOR3F*sin(oBinAngle))*(magnitude*spatialBinSpacing));
                        startPoint = startPoint - (endPoint-startPoint);
                        debugRenderer->addLine(startPoint, endPoint, makeVector4f(1,1,0.3f,1),makeVector4f(1,1,0.7f,1), 1);
                    }

                    baseIndex += numOrientationBins;
                    binPosU += spatialBinSpacing*tangentU;
                }      
                binPosV += spatialBinSpacing*tangentV;
            }
            binPosN += spatialBinSpacing*normal;
        }
        debugRenderer->endRenderJob();
    }
}


Vector3f C13MSFeatDescrHOG::SingleScaleDetector::getAverageNormal(const Vector3f &pos, const float32 fittingRadius)
{
    knn->setTargetPoint(pos);
    card32 index;
    {   
        bool found;
        knn->getNextNearestPointIndex(index, found);
        if (!found) throw EInvalidState("MSFeatDescrHOG::SingleScaleDetector::getLocalFrameFromFixedUpDir - point set empty");
    }
    mpcard *indices = nullptr;
    mpcard numPoints = 0;
    query->querry(pos, fittingRadius, &indices, numPoints);

    /// tangent space
    PCA3f pca;
    for (mpcard i=0; i<numPoints; i++) {
        Vector3f cPos = ps->get3f(indices[i], positionAAT);
        pca.addPoint(cPos);
    }
    Vector3f eigenvalues;
    Matrix3f eigenvectors;
    Vector3f centroid;
    pca.analyze(eigenvalues, eigenvectors, centroid);

    Vector3f normal = normalize(eigenvectors[2]);
    if (normal*ps->get3f(index, normalAAT) <0) normal *= -1;
    return normal;
}

C13MSFeatDescrHOG::SingleScaleDetector::~SingleScaleDetector ()
{
    delete query;
    delete knnOctree;
    delete knn;
}
