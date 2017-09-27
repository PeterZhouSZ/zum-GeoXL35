#include "StdAfx.h"
#include <opencv\cxcore.h>
//---------------------------------------------------------------------------
#include "DampingICL.h"
//---------------------------------------------------------------------------
#include <set>
#include <functional>
#include <boost/heap/priority_queue.hpp>
#include <boost/format.hpp>
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

DampingICL::DampingICL(void)
{
    m_MaxNumIterations = 10;
    m_AngleSigma = 0.2f;
    m_NormalAngleSigma = 0.1f;
    m_LineDistanceSigma = 0.1f;
    m_Visualize = false;
    m_hIt = nullptr;
    m_LineFeatureSet = nullptr;
    m_CurvatureSigma = 0.15f;
    m_VisualizeInWorldSpace = true;
    m_LineFeatureSet_Flags = nullptr;
    m_RedrawFunction = nullptr;
}

DampingICL::~DampingICL(void)
{
    delete m_hIt;
}

bool DampingICL::align( const std::vector<LineFeature*>& subset, Matrix4f& relativeTransformation, float& score )
{
    Matrix4f initialTransformation = relativeTransformation;
    if (subset.size() < 2)
        return false;

    if (m_LineFeatureSet == nullptr)
        throw PException("DampingICL::align() was called without calling initialization method begin()!");

    LineFeatureSet* lfs = m_LineFeatureSet;

    // compute boundingbox of subset
    BoundingBox3f bb;
    bb.lowerCorner = bb.upperCorner = subset[0]->m_Position;
    for (unsigned i = 1; i < subset.size(); ++i)
        bb.addPoint(subset[i]->m_Position);
    BoundingBox3f bbSubset = bb;
    bbSubset.enlarge(1.15f);
    bb.makeBoundingCube();

    // compute transformation from world space to normed local space [-1,1]?
    Matrix3f relativeRotation = shrink4To3(relativeTransformation);
    Matrix4f sourceFrame = makeTranslation4f(bb.getCenter());
    Matrix4f initialSourceFrame = sourceFrame;
    Matrix4f destFrame = relativeTransformation * sourceFrame;

    float scaleFactor = bb.getSideLength(0) / 2.0f;
    Matrix4f destToLocal = makeScale4f(makeVector3f(1,1,1) / scaleFactor) * invertFrame(destFrame);
    Matrix4f localToDest = destFrame * makeScale4f(makeVector3f(1,1,1) * scaleFactor);
    Matrix4f refToLocal = makeScale4f(makeVector3f(1,1,1) / scaleFactor) * invertFrame(sourceFrame);
    Matrix4f localToRef = sourceFrame * makeScale4f(makeVector3f(1,1,1) * scaleFactor);

    score = 0;
    float lastScore = 1e20f;
    for (unsigned pass = 0; pass < m_MaxNumIterations; ++pass) {
        // visualization
        if (m_Visualize) {
            debugRenderer->beginRenderJob_OneFrame("icl_align_", DR_FRAME++);

            //debugRenderer->clearDebugData("asdf");
            //debugRenderer->beginRenderJob("asdf");
            debugRenderer->setSmoothLine(true);
            Vector3f srceColor = makeVector3f(0, .5f, 1);
            Vector3f destColor = makeVector3f(1, 0.75f, 0);
            debugRenderer->addFineBoundingBox(IDENTITY4F, bbSubset, srceColor, false);
            debugRenderer->addFineBoundingBox(relativeTransformation, bbSubset, destColor, false);

            for (unsigned i = 0; i < subset.size(); ++i) {
                LineFeature* lf = subset[i];
                Vector3f p1 = lf->m_Position + normalize(lf->m_LineDirection) * lf->m_Sigma;
                Vector3f p2 = lf->m_Position - normalize(lf->m_LineDirection) * lf->m_Sigma;
                if (!m_VisualizeInWorldSpace) {
                    p1 = transformVector3f(refToLocal, p1);
                    p2 = transformVector3f(refToLocal, p2);
                } else {
                    debugRenderer->addLine(p1, p2, srceColor, srceColor, 4);
                    p1 = transformVector3f(relativeTransformation, p1);
                    p2 = transformVector3f(relativeTransformation, p2);
                }
                debugRenderer->addLine(p1, p2, destColor, destColor, 4);
            }

            for (unsigned i = 0; i < lfs->m_LineFeatures.size(); ++i) {
                LineFeature* lf = lfs->m_LineFeatures[i];
                Vector3f p1 = lf->m_Position + normalize(lf->m_LineDirection) * lf->m_Sigma;
                Vector3f p2 = lf->m_Position - normalize(lf->m_LineDirection) * lf->m_Sigma;
                if (!m_VisualizeInWorldSpace) {
                    p1 = transformVector3f(destToLocal, p1);
                    p2 = transformVector3f(destToLocal, p2);
                }
                debugRenderer->addLine(p1, p2, makeVector3f(1,1,1)*0.5f, makeVector3f(1,1,1)*0.5f, 2);
            }
        }

        lastScore = score;
        score = 0;
        float summed_weight = 0;
        OptMat3 zeroMat = IDENTITY3F * 0;
        OptMat3 P = zeroMat;
        OptMat3 S = zeroMat;
        OptMat3 R = zeroMat;
        OptVec6 resB = NULL_VECTOR6F;
        float inv_angle_sigma_2 = 1.0f / m_AngleSigma / m_AngleSigma;
        float inv_linepos_sigma_2 = 1.0f / m_LineDistanceSigma / m_LineDistanceSigma;

        // calculate corresponding points
        for (unsigned i = 0; i < subset.size(); ++i) {
            LineFeature* lf = subset[i];
            Vector3f featurePos = lf->m_Position;
            Vector3f transformedFeaturePos = transformVector3f(relativeTransformation, featurePos);
            Vector3f transformedLineDirection = normalize(relativeRotation * lf->m_LineDirection);
            Vector3f transformedNormal = relativeRotation * lf->m_Normal;

            // seek closest line
            m_hIt->setSeekPointAndReset(transformedFeaturePos);
            card32 numNearLines = 0;
            while (!m_hIt->atEnd()) {
                //m_hIt->next(); // only on the closest line feature
                LineFeature* closestFeature = lfs->m_LineFeatures[m_hIt->getCurrentPointIndex()];
                Vector3f closestFeature_LineDirection = normalize( closestFeature->m_LineDirection );

                // compute weight
                Matrix3f localFrame = calcTangentSystem(relativeRotation.transpose() * closestFeature_LineDirection);
                float cos_angle = fabs(transformedLineDirection * closestFeature_LineDirection);
                float cos_normalAngle = transformedNormal * closestFeature->m_Normal;
                const float maxAngle = cos(M_PI / 8.0f);
                Vector3f pointDistance = transformedFeaturePos - closestFeature->m_Position;
                Vector3f lineDistance = pointDistance -
                    closestFeature_LineDirection * (pointDistance * closestFeature_LineDirection);
                const float curvDiff = lf->m_Curvature - closestFeature->m_Curvature;
                const float normDiff = std::max(0.0f, 1.0f - cos_normalAngle) / m_NormalAngleSigma;
                OptFloatType weight = 
                    (cos_angle - maxAngle) / (1.0f - maxAngle) * 
                    exp( -normQuad(lineDistance) * inv_linepos_sigma_2 ) *
                    exp( -curvDiff * curvDiff / m_CurvatureSigma/ m_CurvatureSigma ) *
                    exp( -normDiff * normDiff );

                if (weight < 0.005f)
                    break;

                // compute local quadratic approximation
                // M =  n n^t
                // F(x) = (x-p)^tM(x-p) = x^tMx - 2x^tMp + ...
                Vector3f relP = transformVector3f(refToLocal, lf->m_Position);
                Matrix3f matM = outerProduct(localFrame[0], localFrame[0] )
                    + outerProduct(localFrame[1], localFrame[1] );
                //+ outerProduct(localFrame[2], localFrame[2] ) * 0.025f;
                //Matrix3f matM = IDENTITY3F - outerProduct()
                Vector3f destPosLocal = transformVector3f(destToLocal, closestFeature->m_Position);
                Vector3f b = matM * destPosLocal * (-2.0f);

                if (m_Visualize) {
                    //debugRenderer->addLine(relP, destPosLocal, makeVector3f(0,0,0.6f), makeVector3f(1,0,0), 3);
                    //debugRenderer->addLine(transformVector3f(destToLocal, closestFeature->m_Position + lineDistance), destPosLocal, makeVector3f(0,0,0.6f), makeVector3f(1,1,0), 3);

                    float t = std::max(0.0f, (lastScore - m_Visualization_MinScore / 2.0f) / m_Visualization_MinScore * 2.0f);

                    Vector3f col = makeVector3f(1, 0, 0) * (1.0f - t) + makeVector3f(1, 1, 1) * t;
                    if (lastScore > m_Visualization_MinScore)
                        col = makeVector3f(1, 1, 1);
                    col *= weight;

                    if (!m_VisualizeInWorldSpace) {
                        Vector3f tmp = (relativeRotation.transpose() * closestFeature->m_Normal).crossProduct(localFrame[2]);
                        tmp = normalize(tmp);
                        float s = closestFeature->m_Sigma / scaleFactor;
                        debugRenderer->addGlowingNormalDitribution2D(relP, localFrame[2] * s * 3.0f, tmp * s, col);
                    } else {
                        Vector3f tmp = (closestFeature->m_Normal).crossProduct(closestFeature_LineDirection);
                        tmp = normalize(tmp);
                        float s = closestFeature->m_Sigma ;
                        debugRenderer->addGlowingNormalDitribution2D(transformedFeaturePos, closestFeature_LineDirection * s * 3.0f, tmp * s, col);
                    }
                }

                // F(x) = Ax?+ Bxy + Cy?+ Dxz + Eyz + Fz?+ Gx + Hy + Iz (Mitra et al 2004)
                OptFloatType A = matM[0][0];
                OptFloatType B = matM[1][0] + matM[0][1];
                OptFloatType C = matM[1][1];
                OptFloatType D = matM[2][0] + matM[0][2];
                OptFloatType E = matM[2][1] + matM[1][2];
                OptFloatType F = matM[2][2];
                OptFloatType G = b[0];
                OptFloatType H = b[1];
                OptFloatType I = b[2];

                // compute derivatives
                OptFloatType x = relP[0];
                OptFloatType y = relP[1];
                OptFloatType z = relP[2];
                OptFloatType J = 2.0f * (C*x*x - B*x*y + A*y*y);
                OptFloatType K = 2.0f * (F*x*x - D*x*z + A*z*z);
                OptFloatType L = 2.0f * (F*y*y - E*y*z + C*z*z);
                OptFloatType M = -E*x*x + D*x*y + B*x*z - 2.0f*A*y*z;
                OptFloatType N = -D*y*y + E*x*y -2.0f*C*x*z + B*y*z;
                OptFloatType T = -B*z*z -2.0f*F*x*y + E*x*z + D*y*z;
                OptFloatType U = B*(x*x-y*y) + 2.0f*(C-A)*x*y + E*x*z - D*y*z + H*x - G*y;
                OptFloatType V = D*(z*z-x*x) - E*x*y + 2.0f*(A-F)*x*z + B*y*z -I*x + G*z;
                OptFloatType W = E*(y*y-z*z) + D*x*y - B*x*z + 2.0f*(F-C)*y*z + I*y - H*z;

                // setup linear system
                P += makeMatrix3f(J, M, N, M, K, T, N, T, L) * weight;
                S += makeMatrix3f(B*x - A*y*2.0f, C*x*2.0f - B*y, E*x - D*y,
                    -D*x + A*z*2.0f, -E*x + B*z, -2.0f*F*x + D*z,
                    D*y - B*z, E*y - 2.0f*C*z, 2.0f*F*y - E*z) * weight;
                R += makeMatrix3f(2.0f*A, B, D, 
                    B, 2.0f*C, E, 
                    D, E, 2.0f*F) * weight;
                resB -= makeVector6f(U,V,W,
                    2.0f*A*x + B*y + D*z + G,
                    B*x + 2.0f*C*y + E*z + H,
                    D*x + E*y + 2.0f*F*z + I) * weight;

                // current residual
                if (numNearLines == 0)
                    score += weight;
                summed_weight +=  weight;

                ++numNearLines;
                if (numNearLines > 0)
                    break;
            }
        }

        if (m_Visualize) {
            if (m_RedrawFunction) {
                (*m_RedrawFunction)();
                //Sleep(500);
                //if (pass + 1 < m_MaxNumIterations)
                //	debugRenderer->clearDebugData("asdf");
            }
            debugRenderer->endRenderJob();
        }

        if (m_Visualize)
            debugOutput << "Score: " << score << "\n";

        if (score < lastScore) { // must improve without outlier checking
            if (m_Visualize)
                debugOutput << ">> diverge due to decreasing score.\n";
            return true;
        }

        const float min_weight = (float)subset.size() * 0.01f;
        if (summed_weight < min_weight) {
            if (m_Visualize)
                debugOutput << ">> diverge due to too few inliers: "
                << summed_weight << " [" << 100*summed_weight/min_weight << "%].\n";
            return false;
        }

        // setup linear system
        OptMat3 S_T = S.transpose();
        OptMat6 matM;
        for (unsigned a = 0; a < 3; ++a) {
            for (unsigned b = 0; b < 3; ++b) {
                matM[a][b] = P[a][b];
                matM[a+3][b] = S[a][b];
                matM[a][b+3] = S_T[a][b];
                matM[a+3][b+3] = R[a][b];
            }
        }

        CvMat cvMatInput, cvMatOutput;
        OptMat6 matM_inv;
        cvInitMatHeader( &cvMatInput, 6, 6, CV_32FC1, matM.data());
        cvInitMatHeader( &cvMatOutput, 6, 6, CV_32FC1, matM_inv.data());
        cvInvert(&cvMatInput, &cvMatOutput,CV_SVD_SYM);

        OptVec6 coeffs = matM_inv * resB * 0.5;

        float cos_a = cos(coeffs[0]);
        float sin_a = sin(coeffs[0]);
        float cos_b = cos(coeffs[1]);
        float sin_b = sin(coeffs[1]);
        float cos_c = cos(coeffs[2]);
        float sin_c = sin(coeffs[2]);
        Matrix3f delta_rotation = 
            makeMatrix3f(cos_a, -sin_a,0,
            sin_a, cos_a, 0,
            0,0,1) * 
            makeMatrix3f(cos_b, 0, sin_b,
            0,1,0,
            -sin_b, 0, cos_b) *
            makeMatrix3f(1, 0, 0,
            0, cos_c, -sin_c,
            0, sin_c, cos_c );

        Matrix4f deltaTransformation = makeTranslation4f(makeVector3f(coeffs[3],coeffs[4],coeffs[5]))
            * expand3To4(delta_rotation);

        refToLocal = deltaTransformation * refToLocal;
        sourceFrame = invertFrame(localToRef*refToLocal)*initialSourceFrame;

        relativeTransformation = destFrame * invertFrame(sourceFrame);			 
        relativeRotation = shrink4To3(relativeTransformation);

        if (fabs(score - lastScore) < 0.01f) {
            if (m_Visualize)
                debugOutput << ">> score threshold: ICL converged after " << pass << " iterations\n";
            return true;
        }
    }
    if (m_Visualize)
        debugOutput << ">> failed to converge in " << m_MaxNumIterations << " iterations.\n";
    return true;
}

void DampingICL::begin( LineFeatureSet * lfs, unsigned * lfs_flags )
{
    m_LineFeatureSet = lfs;
    m_LineFeatureSet_Flags = lfs_flags;
    m_hIt = new HierarchicalKNNIterator(lfs->getFeaturesAsPointCloud(),32,nullptr);
}

void DampingICL::end()
{
    m_LineFeatureSet = nullptr;
    m_LineFeatureSet_Flags = nullptr;
    delete m_hIt;
    m_hIt = nullptr;
}

bool DampingICL::findCorrespondingLine( LineFeature * l, const Matrix4f &relativeTransformation, unsigned * index )
{
    if( m_LineFeatureSet == nullptr )
        throw PException("DampingICL::findCorrespondingLine() was called without calling initialization method begin()!");

    Vector3f transformedSourcePos = transformVector3f(relativeTransformation, l->m_Position);
    m_hIt->setSeekPointAndReset(transformedSourcePos);
    card32 nIndex = m_hIt->getCurrentPointIndex();

    if( m_LineFeatureSet_Flags )
        if( m_LineFeatureSet_Flags[nIndex] != 0 )
            return false;

    LineFeature * destLF = m_LineFeatureSet->m_LineFeatures[nIndex];

    if( fabs(l->m_Curvature - destLF->m_Curvature) > m_CurvatureSigma*2.0f )
        return false;

    if( fabs(norm(l->m_RotationDirection) - norm(destLF->m_RotationDirection)) > 0.2f )
        return false;

    if( norm(transformedSourcePos - destLF->m_Position) > 2.0f * l->m_Sigma )
        return false;

    Vector3f transformedSourceNormal = shrink4To3(relativeTransformation) * l->m_Normal;
    if( transformedSourceNormal * destLF->m_Normal < 1 - m_NormalAngleSigma ) 
        return false;

    Vector3f nearest_pos = destLF->m_Position + destLF->m_LineDirection * (destLF->m_LineDirection * (transformedSourcePos - destLF->m_Position));
    if( norm(nearest_pos - transformedSourcePos) > 2.0f * m_LineDistanceSigma )
        return false;

    Matrix3f rotMat = shrink4To3(relativeTransformation);
    if( fabs( normalize(destLF->m_LineDirection) * normalize(rotMat*l->m_LineDirection)) < 1.0f - m_AngleSigma )
        return false;

    if( index )
        *index = m_hIt->getCurrentPointIndex();

    return true;
}

bool DampingICL::findCorrespondingLineBIO( LineFeature * l, const Matrix4f &relativeTransformation, unsigned * index )
{
    //get line feature closest to the projected position 
    Vector3f transformedSourcePos = transformVector3f(relativeTransformation, l->m_Position);
    m_hIt->setSeekPointAndReset(transformedSourcePos);
    card32 nIndex = m_hIt->getCurrentPointIndex();
    LineFeature * destLF = m_LineFeatureSet->m_LineFeatures[nIndex];

    if( norm(transformedSourcePos - destLF->m_Position) > (2.0f * l->m_Sigma) )
        return false;

    Vector3f nearest_pos = destLF->m_Position + destLF->m_LineDirection * (destLF->m_LineDirection * (transformedSourcePos - destLF->m_Position));
    if( norm(nearest_pos - transformedSourcePos) > 2.0f * m_LineDistanceSigma )
        return false;

    Matrix3f rotMat = shrink4To3(relativeTransformation);
    if( fabs( normalize(destLF->m_LineDirection) * normalize(rotMat*l->m_LineDirection)) < 1.0f - m_AngleSigma )
        return false;

    if( index )
        *index = m_hIt->getCurrentPointIndex(); // == nIndex?

    return true;
}
