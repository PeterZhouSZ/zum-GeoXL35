#include "StdAfx.h"
#include <opencv\cxcore.h>
//---------------------------------------------------------------------------
#include "DampingICP.h"
//---------------------------------------------------------------------------
#include <set>
#include <functional>
#include <boost/heap/priority_queue.hpp>
#include <boost/format.hpp>
//---------------------------------------------------------------------------

DampingICP::DampingICP(PointCloud* pc)
{
    hIt = new HierarchicalKNNIterator(pc, 32, nullptr);
    NORMAL= pc->getAAT("normal");
    POSITION= pc->getAAT("position");

    setupParameters();
}

DampingICP::~DampingICP(void)
{
    delete hIt;
}

void DampingICP::setupParameters(
    unsigned max_num_icp /*= 10*/,              // number of iterations
    float median_point_dist /*= 0.01f*/,        // median point distance
    float outlier_dist_factor /*= 10*/,         // outlier distance factor
    float min_inlier_ratio /*= 0.8*/,           // minimum inlier percentage
    float max_allowed_dist/*= 0.1f */,          // allowed move distance
    float residual_tolerance/*= 0.01f*/,        // maximal residual tolerance
    float converge_difference /*= 0.001f*/,     // convergence difference threshold
    bool visualization/*= false*/
    )
{

    this->max_num_icp = max_num_icp;
    this->median_point_dist = median_point_dist;
    this->outlier_dist_factor = outlier_dist_factor;
    this->min_inlier_ratio = min_inlier_ratio;
    this->max_allowed_dist = max_allowed_dist;
    this->residual_tolerance = residual_tolerance;
    this->converge_difference = converge_difference;
    this->visualization = visualization;
}

float DampingICP::calculateResidualAfterICP(
    PointSet* startPS, Matrix4f& originalTransformation
    )
{
    std::deque<unsigned> inlier_indices;
    return calculateResidualAfterICP(startPS, originalTransformation, inlier_indices, true);
}

float DampingICP::calculateResidualAfterICP(
    PointSet* startPS, Matrix4f& originalTransformation,
    std::deque<unsigned>& inlier_indices,
    bool final_snap
    )
{
    if (!startPS) return 1e20f;
    const unsigned numPoints = startPS->getNumEntries();
    if (numPoints < 4) return 1e20f;

    Matrix4f trans = originalTransformation;
    Matrix4f initialTransformation = trans;

    float residual = 0;
    float lastResidual = 1e20f;

    if (median_point_dist < std::numeric_limits<float>::epsilon()) {
        error("DampingICP::calculateResidualAfterICP() - Median Point Distance is not defined!");
        return 1e20f;
    }

    // compute bounding box of sartPS
    BoundingBox3f bb(startPS->get3f(0,POSITION));
    for (unsigned ii = 1; ii < numPoints; ++ii)
        bb.addPoint(startPS->get3f(ii, POSITION));
    startPScenter = bb.getCenter();
    startPScenterOrgTransformed = transformVector3f(originalTransformation, startPScenter);
    bb.makeBoundingCube();


    // compute transformation from world space to normed local space [-1,1]?
    //Matrix3f relativeRotation = shrink4To3(initialTransformation);
    Matrix4f sourceFrame = makeTranslation4f(bb.getCenter());
    Matrix4f initialSourceFrame = sourceFrame;
    Matrix4f destFrame = initialTransformation * sourceFrame;

    const float scaleFactor = bb.getSideLength(0) / 2.0f;
    Matrix4f destToLocal = makeScale4f(makeVector3f(1,1,1) / scaleFactor) * invertFrame(destFrame);
    Matrix4f localToDest = destFrame * makeScale4f(makeVector3f(1,1,1) * scaleFactor);
    Matrix4f refToLocal = makeScale4f(makeVector3f(1,1,1) / scaleFactor)* invertFrame(sourceFrame);
    Matrix4f localToRef = sourceFrame * makeScale4f(makeVector3f(1,1,1) * scaleFactor);
    std::string icpPoints = "debug icp";

    // in case of final snap, do not chech inliers, and use larger outlier threshold
    const unsigned num_inliers =
        final_snap ?
        0 :
    min_inlier_ratio * (float)numPoints;
    const float outlier_dist =
        //final_snap ?
        //median_point_dist * outlier_dist_factor * 2 :
        median_point_dist * outlier_dist_factor;
    {
        //const unsigned num_inliers_last = num_inliers - 1;
        //std::vector< float > resi_in_vec(num_inliers);
    }
    if( visualization ) debugOutput << "    icp: ";
    const bool debug_visualization = visualization; // may cause crash if using mult-threaded interface.
    // to prevent crash, just DO NOT let the drawing area refresh, e.g. do not hover the mouse over it, when it has focus.
    const unsigned draw_frame = DebugRenderer::DR_FRAME;
    if (debug_visualization) {
        DebugRenderer::DR_FRAME++;
    }
    for (unsigned pass = 0; pass < max_num_icp; ++pass)
    {
        if (debug_visualization) {
            const std::string draw_name_base = "icp_residual_calculation_";
            const std::string draw_name = str( boost::format( "%1%%2%" ) % draw_name_base % draw_frame );
            debugRenderer->clearDebugData( draw_name );
            debugRenderer->beginRenderJob_OneFrame(draw_name_base, draw_frame);
        }

        residual = 0;
        float summed_weight = 0;
        Matrix3f zeroMat = IDENTITY3F * 0;
        Matrix3f P = zeroMat;
        Matrix3f S = zeroMat;
        Matrix3f R = zeroMat;
        Vector6f resB = NULL_VECTOR6F;

        {
            //boost::heap::priority_queue< float > resi_in_vec; // largest on top
            //unsigned curr_pi = 0;
        }
        // calculate corresponding points
        inlier_indices.clear();
        for (unsigned ii = 0; ii < numPoints; ++ii) {
            const Vector3f point = startPS->get3f(ii, POSITION);
            const Vector3f transformedPos = transformVector3f(trans, point);

            // seek closest point
            hIt->setSeekPointAndReset(transformedPos);
            const Vector3f closestPoint = hIt->get3f(POSITION);
            const Vector3f closestPointNormal = hIt->get3f(NORMAL);

            const float closestDist = norm(closestPoint-transformedPos);
            if (closestDist > outlier_dist) {
                if (debug_visualization) {
                    debugRenderer->addPoint(point,
                        makeVector3f(0.f, 1.f, 0.f));
                }
                continue;
            }

            inlier_indices.push_back(ii);
            //const float weight = 1.0f;
            float weight = (closestDist > outlier_dist) ? 0.f : 1.f;
            {
                summed_weight += weight;

                // compute local quadratic approximation
                // M =  n n^t
                // F(x) = (x-p)^tM(x-p) = x^tMx - 2x^tMp + ...
                Vector3f relP = transformVector3f(refToLocal, point);
                Matrix3f matM = outerProduct(closestPointNormal, closestPointNormal );
                Vector3f destPosLocal = transformVector3f(destToLocal, closestPoint);
                Vector3f b = matM * destPosLocal * (-2.0f);

                // F(x) = Ax?+ Bxy + Cy?+ Dxz + Eyz + Fz?+ Gx + Hy + Iz (Mitra et al 2004)
                float A = matM[0][0];
                float B = matM[1][0] + matM[0][1];
                float C = matM[1][1];
                float D = matM[2][0] + matM[0][2];
                float E = matM[2][1] + matM[1][2];
                float F = matM[2][2];
                float G = b[0];
                float H = b[1];
                float I = b[2];

                // compute derivates
                float x = relP[0];
                float y = relP[1];
                float z = relP[2];
                float J = 2.0f * (C*x*x - B*x*y + A*y*y);
                float K = 2.0f * (F*x*x - D*x*z + A*z*z);
                float L = 2.0f * (F*y*y - E*y*z + C*z*z);
                float M = -E*x*x + D*x*y + B*x*z - 2.0f*A*y*z;
                float N = -D*y*y + E*x*y -2.0f*C*x*z + B*y*z;
                float T = -B*z*z -2.0f*F*x*y + E*x*z + D*y*z;
                float U = B*(x*x-y*y) + 2.0f*(C-A)*x*y + E*x*z - D*y*z + H*x - G*y;
                float V = D*(z*z-x*x) - E*x*y + 2.0f*(A-F)*x*z + B*y*z -I*x + G*z;
                float W = E*(y*y-z*z) + D*x*y - B*x*z + 2.0f*(F-C)*y*z + I*y - H*z;

                // setup linear system
                P += makeMatrix3f(	J, M, N, 
                    M, K, T, 
                    N, T, L) * weight;
                S += makeMatrix3f(	B*x - A*y*2.0f	, C*x*2.0f - B*y	, E*x - D*y,
                    -D*x + A*z*2.0f	, -E*x + B*z		, -2.0f*F*x + D*z,
                    D*y - B*z		, E*y - 2.0f*C*z	, 2.0f*F*y - E*z) * weight;
                R += makeMatrix3f(	2.0f*A, B, D, 
                    B, 2.0f*C, E, 
                    D, E, 2.0f*F) * weight;
                resB -= makeVector6f(U,V,W,
                    2.0f*A*x + B*y + D*z + G,
                    B*x + 2.0f*C*y + E*z + H,
                    D*x + E*y + 2.0f*F*z + I) * weight;
            }

            // project onto surfel
            Vector3f projectedPoint = transformedPos - closestPointNormal * (closestPointNormal * (transformedPos- closestPoint));
            float projectedDistance = normQuad(transformedPos - projectedPoint);
            if (debug_visualization) {
                debugRenderer->addPoint(point,
                    makeVector3f(1.f, 0.f, 0.f));
                debugRenderer->addPoint(transformedPos,
                    makeVector3f(0.f, 0.f, 1.f));
                debugRenderer->addLine(
                    transformedPos, closestPoint,
                    makeVector3f(0.8f, 0.8f, 0.f), makeVector3f(0.f, 0.8f, 0.8f), 1.0f);
                debugRenderer->addLine(
                    transformedPos, projectedPoint,
                    makeVector3f(0.8f, 0.8f, 0.f), makeVector3f(0.8f, 0.f, 0.8f), 1.0f);
            }

            // current residual
            residual += projectedDistance; 
            {
                //if (curr_pi <= num_inliers) {
                //    //resi_in_vec[curr_pi] = projectedDistance;
                //    resi_in_vec.push(projectedDistance);
                //    ++curr_pi;
                //    //} else if (curr_pi == num_inliers) {
                //    //    std::sort(resi_in_vec.begin(), resi_in_vec.end());
                //} else {
                //    //if (projectedDistance < resi_in_vec[num_inliers_last]) {
                //    //    resi_in_vec[num_inliers_last] = projectedDistance;
                //    //    std::sort(resi_in_vec.begin(), resi_in_vec.end());
                //    //}
                //    if (projectedDistance < resi_in_vec.top()) {
                //        resi_in_vec.push(projectedDistance);
                //    }
                //}
            }
        }
        {
            //for (float pd : resi_in_vec) {
            //    residual += pd;
            //}
        }

        if (summed_weight < num_inliers) {
            if( visualization ) debugOutput << "diverge due to too few inliers: "
                << summed_weight << " [" << 100*summed_weight/num_inliers << "%].\n";
            return 1e20f;
        }

        //residual /= (float)numPoints;
        residual /= summed_weight;
        residual = sqrt(residual);

        if (visualization) debugOutput << residual << ", ";

        if (residual_tolerance > residual) {
            if( visualization ) debugOutput << "converged after " << pass
                << " steps with residual: " << residual << "\n";
            originalTransformation = trans;
            return residual;
        }

        //if (lastResidual < residual){ // may happen because of outlier checking
        //    if( visualization ) debugOutput << "diverge with increasing residual\n";
        //    return 1e20f;
        //}

        if (!checkAllowedMovement(trans)){
            if( visualization ) debugOutput << "diverge outside of allowed movement\n";
            return 1e20f;
        }

        if(pass>0 && fabs(lastResidual-residual) < converge_difference )
        {

            if (visualization) debugOutput << "converged after " << pass
                << " steps with residual difference: " << fabs(lastResidual-residual) << "\n";
            originalTransformation = trans;
            return residual;

        }


        // setup linear system
        Matrix3f S_T = S.transpose();
        Matrix6f matM;
        for( unsigned a = 0;a<3;a++ )
        {
            for( unsigned b = 0;b<3;b++ )
            {
                matM[a][b] = P[a][b];
                matM[a+3][b] = S[a][b];
                matM[a][b+3] = S_T[a][b];
                matM[a+3][b+3] = R[a][b];
            }
        }

        CvMat cvMatInput, cvMatOutput;
        Matrix6f matM_inv;
        cvInitMatHeader( &cvMatInput, 6, 6, CV_32FC1, matM.data());
        cvInitMatHeader( &cvMatOutput, 6, 6, CV_32FC1, matM_inv.data());
        cvInvert(&cvMatInput, &cvMatOutput,CV_SVD_SYM);

        Vector6f coeffs = matM_inv * resB * 0.5;

        float cos_a = cos(coeffs[0]);
        float sin_a = sin(coeffs[0]);
        float cos_b = cos(coeffs[1]);
        float sin_b = sin(coeffs[1]);
        float cos_c = cos(coeffs[2]);
        float sin_c = sin(coeffs[2]);
        Matrix3f delta_rotation = 
            makeMatrix3f(
            cos_a, -sin_a, 0,
            sin_a, cos_a, 0,
            0, 0, 1) * 
            makeMatrix3f(
            cos_b, 0, sin_b,
            0, 1, 0,
            -sin_b, 0, cos_b) *
            makeMatrix3f(
            1, 0, 0,
            0, cos_c, -sin_c,
            0, sin_c, cos_c );

        Matrix3f rot = makeMatrix3f(
            1, -coeffs[0], coeffs[1],
            coeffs[0], 1, -coeffs[2],
            -coeffs[1], coeffs[2], 1);
        // 		debugOutput << "\nvorher:\n";
        // 		debugOutput << rot;

        Matrix3f U,V;
        Vector3f eigenValues;
        CvMat cvMatInputR, cvMat_U, cvMat_W, cvMat_Vt;
        cvInitMatHeader( &cvMatInputR, 3, 3, CV_32FC1, rot.data());
        cvInitMatHeader( &cvMat_W, 3, 1, CV_32FC1, eigenValues.data() );
        cvInitMatHeader( &cvMat_U, 3, 3, CV_32FC1, U.data() );
        cvInitMatHeader( &cvMat_Vt, 3, 3, CV_32FC1, V.data() );



        // Do SVD
        cvSVD(&cvMatInputR, &cvMat_W, &cvMat_U, &cvMat_Vt, CV_SVD_MODIFY_A | CV_SVD_U_T );

        delta_rotation = U*V;
        delta_rotation = delta_rotation.transpose();

        // 		debugOutput << "\nnachher:\n";
        // 		debugOutput << delta_rotation << "\n";

        Matrix4f deltaTransformation = makeTranslation4f(makeVector3f(coeffs[3],coeffs[4],coeffs[5]))
            * expand3To4(delta_rotation);

        refToLocal = deltaTransformation * refToLocal;
        sourceFrame = invertFrame(localToRef*refToLocal)*initialSourceFrame;
        trans = destFrame * invertFrame(sourceFrame);	
        lastResidual=residual;

        if (debug_visualization) {
            debugRenderer->endRenderJob();
        }
    }

    originalTransformation = trans;  // xxx remove


    if( visualization ) debugOutput << "not converged in prescribed number of iterations\n";
    //return 1e20f;
    return residual; // keep the possibility of out-of-iterations convergence
}

bool DampingICP::checkAllowedMovement(const Matrix4f &trans)
{
    Vector3f transformedCenter = transformVector3f(trans, startPScenter);

    Vector3f moveVecA = transformedCenter - startPScenterOrgTransformed;
    float moveDist = norm(moveVecA);

    if (moveDist > max_allowed_dist ) return false;
    else return true;
}
