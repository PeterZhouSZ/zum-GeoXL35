#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Handle3DGizmo.h"
#include "Util\numerical\EigenAdaptor.h"
#include "Util\ColorSchemer.hpp"
#include "Util\ExtraGL.hpp"
#include <random>
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

Handle3DGizmo::Handle3DGizmo(const std::deque<unsigned>& points, const card32& width, const card32& height) :
gizmoMove(CreateMoveGizmo()), gizmoRotate(CreateRotateGizmo()), gizmoScale(CreateScaleGizmo())
{
    isActive_ = false;
    colActive_ = makeVector3f(0.9f, 1.0f, 0.0f);
    colDormant_ = makeVector3f(0.6f, 0.6f, 0.6f);

    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_real_distribution<> dist01(0.f, 1.f);
    color_ = ColorSchemer::GetJetColor(dist01(rng));

    initMatrix_ = editMatrix_ = IDENTITY4F;
    gizmo = gizmoMove;
    //gizmoMove->SetLocation(IGizmo::LOCATE_WORLD);
    //gizmoRotate->SetLocation(IGizmo::LOCATE_WORLD);
    gizmoMove->SetLocation(IGizmo::LOCATE_LOCAL);
    gizmoRotate->SetLocation(IGizmo::LOCATE_LOCAL);
    gizmoScale->SetLocation(IGizmo::LOCATE_LOCAL);
    gizmoMove->SetDisplayScale(0.8f);
    gizmoRotate->SetDisplayScale(0.6f);
    gizmoScale->SetDisplayScale(0.8f);
    gizmoMove->SetScreenDimension(width, height);
    gizmoRotate->SetScreenDimension(width, height);
    gizmoScale->SetScreenDimension(width, height);

    gizmoMove->SetEditMatrix(editMatrix_.data());
    gizmoRotate->SetEditMatrix(editMatrix_.data());
    gizmoScale->SetEditMatrix(editMatrix_.data());

    std::copy(points.begin(), points.end(), std::back_inserter(points_));
}

const Vector3f Handle3DGizmo::getCenter(void) const
{
    return shrink4To3(editMatrix_[3]);
}

const Vector3f Handle3DGizmo::getTranslation(void) const
{
    return shrink4To3(editMatrix_[3]) - shrink4To3(initMatrix_[3]);
}

const Matrix4f Handle3DGizmo::getTransformation(void) const
{
    return editMatrix_ * invertFrame(initMatrix_);
}

void Handle3DGizmo::setTransformation(const Matrix4f& trans)
{
    editMatrix_ = initMatrix_ = trans;
}

void Handle3DGizmo::UpdateStep(void)
{
    initMatrix_ = editMatrix_;
}

void Handle3DGizmo::Update(const std::vector<Vector3f>& deformed)
{
    //Vector3f center;
    //Matrix3f coordinate;
    //{
    //    const unsigned& num_points = points_.size();
    //    center = NULL_VECTOR3F;
    //    SparseMatrixD cov(num_points); // build covariance matrix
    //    for (unsigned pi = 0; pi < num_points; ++pi) {
    //        const Vector3f& pos = deformed[points_[pi]];
    //        center = center + pos;
    //        for (unsigned d = 0; d < 3; ++d) {
    //            cov[pi].addEntryBinary(d, pos[d]);
    //        }
    //    }
    //    center = center / (float)num_points;

    //    for (unsigned pi = 0; pi < num_points; ++pi) {
    //        for (unsigned d = 0; d < 3; ++d) {
    //            cov[pi].addEntryBinary(d, -center[d]);
    //        }
    //    }
    //    std::vector<double> U, S, V_T;
    //    ClapackAdaptor::dgesvd(cov, &U, &S, &V_T, 3, false);

    //    for (unsigned ai = 0; ai < 3; ++ai){
    //        for (unsigned d = 0; d < 3; ++d) {
    //            coordinate[ai][d] = V_T[d * 3 + ai];
    //        }
    //        coordinate[ai] = normalize(coordinate[ai]);
    //    }
    //}
    //initMatrix_ = expand3To4(coordinate);
    //initMatrix_[3] = expand3To4(center);

    // only update center, coordinate frame is reset
    initMatrix_ = editMatrix_;
    Vector3f center;
    const unsigned& num_points = points_.size();
    {
        center = NULL_VECTOR3F;
        for (unsigned pi = 0; pi < num_points; ++pi) {
            const Vector3f& pos = deformed[points_[pi]];
            center = center + pos;
        }
        center = center / (float)num_points;
    }
    initMatrix_[3] = expand3To4(center);
}

void Handle3DGizmo::set_difference(const std::deque<unsigned>& rhs)
{
    std::deque<unsigned> res(points_.size() + rhs.size());
    std::deque<unsigned>::iterator it;

    std::sort(points_.begin(), points_.end());

    it = std::set_difference(points_.begin(), points_.end(), rhs.begin(), rhs.end(), res.begin());
    points_.resize(it - res.begin());
    std::copy(res.begin(), it, points_.begin());
}

void Handle3DGizmo::BuildCoordinate(const PointSet& PS)
{
    if (!isActive_) return;

    const AAT& posAAT = PS.getAAT("position");
    Vector3f center = NULL_VECTOR3F;
    Matrix3f coordinate;
    {
        const unsigned& num_points = points_.size();
        //center = NULL_VECTOR3F;
        //SparseMatrixD cov(num_points); // build covariance matrix
        //for (unsigned pi = 0; pi < num_points; ++pi) {
        //    const Vector3f& pos = PS.get3f(points_[pi], posAAT);
        //    center = center + pos;
        //    for (unsigned d = 0; d < 3; ++d) {
        //        cov[pi].addEntryBinary(d, pos[d]);
        //    }
        //}
        //center = center / (float)num_points;

        //for (unsigned pi = 0; pi < num_points; ++pi) {
        //    for (unsigned d = 0; d < 3; ++d) {
        //        cov[pi].addEntryBinary(d, -center[d]);
        //    }
        //}

        Eigen::MatrixXf COV(3, num_points);
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for (unsigned ii = 0; ii < num_points; ++ii) {
            const Eigen::Vector3f posI = EigenAdaptor::ToEigen(PS.get3f(points_[ii], posAAT));
            COV.col(ii) = posI;
            centroid += posI;
        }
        centroid /= (float)num_points;

        for (unsigned ii = 0; ii < num_points; ++ii) {
            COV.col(ii) -= centroid;
        }

        Eigen::JacobiSVD<Eigen::MatrixXf> svd(COV, Eigen::ComputeThinU);
        Eigen::Matrix3f U = svd.matrixU();

        //std::vector<double> U, S, V_T;
        //Clapack::ClapackAdaptor::dgesvd(cov, &U, &S, &V_T, 3, false);

        for (unsigned ai = 0; ai < 3; ++ai){
            //for (unsigned d = 0; d < 3; ++d) {
            //    coordinate[ai][d] = V_T[d * 3 + ai];
            //}
            //coordinate[ai] = normalize(coordinate[ai]);
            coordinate[ai] = EigenAdaptor::FromEigen(static_cast<Eigen::Vector3f>(U.col(ai)));
            coordinate[ai].normalize();
        }
    }
    editMatrix_ = expand3To4(coordinate);
    editMatrix_[3] = expand3To4(center);
    initMatrix_ = editMatrix_;
    //gizmo->SetEditMatrix(editMatrix_.data());

    //debugRenderer->beginRenderJob_OneFrame("debug_", DR_FRAME++);
    //debugRenderer->addPoint(center, makeVector3f(1, 1, 0));
    //debugRenderer->addLine(
    //    center, center + coordinate[0],
    //    makeVector3f(1, 0, 0),
    //    makeVector3f(1, 0, 0),
    //    1);
    //debugRenderer->addLine(
    //    center, center + coordinate[1],
    //    makeVector3f(0, 1, 0),
    //    makeVector3f(0, 1, 0),
    //    1);
    //debugRenderer->addLine(
    //    center, center + coordinate[2],
    //    makeVector3f(0, 0, 1),
    //    makeVector3f(0, 0, 1),
    //    1);
    //debugRenderer->endRenderJob();
}

void Handle3DGizmo::SetControlMode(const Mode& controlMode)
{
    switch (controlMode)
    {
    case INACTIVE:
        isActive_ = false;
        //gizmo = nullptr;
        break;
    case MOVE:
        isActive_ = true;
        gizmo = gizmoMove;
        break;
    case ROTATE:
        isActive_ = true;
        gizmo = gizmoRotate;
        break;
    case SCALE:
        isActive_ = true;
        gizmo = gizmoScale;
        break;
    default:
        error("Handle3DGizmo::SetControlMode - not supported");
    }
}

void DrawAxis(const Vector3f &orig, const Vector3f &axis,
    const Vector3f &vtx, const Vector3f &vty,
    float fct, float fct2, const Vector4f &col)
{
    const static float ZPI = 3.14159265358979323846f;

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glColor4fv(col.data());
    glBegin(GL_LINES);
    glVertex3fv(orig.data());
    glVertex3f(orig[0] + axis[0], orig[1] + axis[1], orig[2] + axis[2]);
    glEnd();

    glBegin(GL_TRIANGLE_FAN);
    for (int i = 0; i <= 30; i++)
    {
        Vector3f pt;
        pt = vtx * cos(((2 * ZPI) / 30.0f)*i)*fct;
        pt += vty * sin(((2 * ZPI) / 30.0f)*i)*fct;
        pt += axis*fct2;
        pt += orig;
        glVertex3fv(&pt[0]);
        pt = vtx * cos(((2 * ZPI) / 30.0f)*(i + 1))*fct;
        pt += vty * sin(((2 * ZPI) / 30.0f)*(i + 1))*fct;
        pt += axis*fct2;
        pt += orig;
        glVertex3fv(&pt[0]);
        glVertex3f(orig[0] + axis[0], orig[1] + axis[1], orig[2] + axis[2]);
    }
    glEnd();
}

void Handle3DGizmo::glDraw(const PointSet& PS) const
{
    if (points_.empty()) return;

    const AAT& posAAT = PS.getAAT("position");
    glEnable(GL_ALPHA_TEST);
    glAlphaFunc(GL_NOTEQUAL, 0);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    { // outer ring
        (isActive_) ? glColor3fv(colActive_.data()) : glColor3fv(colDormant_.data());
        glPointSize(14.0);
        glBegin(GL_POINTS);
        for (unsigned pi = 0; pi < points_.size(); ++pi) {
            const Vector3f& pos = PS.get3f(points_[pi], posAAT);
            glVertex3fv(pos.data());
        }
        glEnd();
    }
    { // inner core
        glColor3fv(color_.data());
        glPointSize(8.0);
        glBegin(GL_POINTS);
        for (unsigned pi = 0; pi < points_.size(); ++pi) {
            const Vector3f& pos = PS.get3f(points_[pi], posAAT);
            glVertex3fv(pos.data());
        }
        glEnd();
    }
    glDisable(GL_POINT_SMOOTH);
    glBlendFunc(GL_NONE, GL_NONE);
    glDisable(GL_BLEND);

    if (isActive_)
    {
        float viewMat[16];
        float projMat[16];

        glGetFloatv (GL_MODELVIEW_MATRIX, viewMat );  
        glGetFloatv (GL_PROJECTION_MATRIX, projMat );  

        gizmo->SetCameraMatrix( viewMat, projMat );
        gizmo->Draw();
    }

    //if (gizmo == gizmoRotate)
    //{
    //    DrawAxis(shrink4To3(editMatrix_[3]), shrink4To3(editMatrix_[0]),
    //        shrink4To3(editMatrix_[1]), shrink4To3(editMatrix_[2]),
    //        0.05f, 0.83f, makeVector4f(1, 0, 0, 1));
    //    DrawAxis(shrink4To3(editMatrix_[3]), shrink4To3(editMatrix_[1]),
    //        shrink4To3(editMatrix_[2]), shrink4To3(editMatrix_[0]),
    //        0.05f, 0.83f, makeVector4f(0, 1, 0, 1));
    //    DrawAxis(shrink4To3(editMatrix_[3]), shrink4To3(editMatrix_[2]),
    //        shrink4To3(editMatrix_[0]), shrink4To3(editMatrix_[1]),
    //        0.05f, 0.83f, makeVector4f(0, 0, 1, 1));
    //}
}

void Handle3DGizmo::glDraw(void) const
{
    if (points_.empty()) return;

    if (isActive_)
    {
        float viewMat[16];
        float projMat[16];

        glGetFloatv(GL_MODELVIEW_MATRIX, viewMat);
        glGetFloatv(GL_PROJECTION_MATRIX, projMat);

        gizmo->SetCameraMatrix(viewMat, projMat);
        gizmo->Draw();
    }
}

void Handle3DGizmo::areaResize(card32 width, card32 height)
{
    gizmoMove->SetScreenDimension( width, height );
    gizmoRotate->SetScreenDimension( width, height );
    gizmoScale->SetScreenDimension( width, height );
}

bool Handle3DGizmo::mouseDown(int32 x, int32 y)
{
    if (!isActive_) return false;
    return gizmo->OnMouseDown( x, y );
}

void Handle3DGizmo::mouseMoved(int32 x, int32 y)
{
    if (!isActive_) return;
    gizmo->OnMouseMove( x, y );
}

void Handle3DGizmo::mouseUp(int32 x, int32 y)
{
    if (!isActive_) return;
    gizmo->OnMouseUp( x, y );

    //Vector3f center0 = shrink4To3(initMatrix_[3]);
    //Vector3f center1 = shrink4To3(editMatrix_[3]);
    //debugRenderer->beginRenderJob_OneFrame("debug_", DR_FRAME++);
    //{
    //    Matrix3f coordinate = shrink4To3(initMatrix_);
    //    debugRenderer->addLine(
    //        center0, center0 + coordinate[0],
    //        makeVector3f(1, 0, 0),
    //        makeVector3f(1, 0, 0),
    //        1);
    //    debugRenderer->addLine(
    //        center0, center0 + coordinate[1],
    //        makeVector3f(0, 1, 0),
    //        makeVector3f(0, 1, 0),
    //        1);
    //    debugRenderer->addLine(
    //        center0, center0 + coordinate[2],
    //        makeVector3f(0, 0, 1),
    //        makeVector3f(0, 0, 1),
    //        1);
    //}
    //{
    //    Matrix3f coordinate = shrink4To3(editMatrix_);
    //    debugRenderer->addLine(
    //        center1, center1 + coordinate[0],
    //        makeVector3f(1, 0, 0),
    //        makeVector3f(1, 0, 0),
    //        1);
    //    debugRenderer->addLine(
    //        center1, center1 + coordinate[1],
    //        makeVector3f(0, 1, 0),
    //        makeVector3f(0, 1, 0),
    //        1);
    //    debugRenderer->addLine(
    //        center1, center1 + coordinate[2],
    //        makeVector3f(0, 0, 1),
    //        makeVector3f(0, 0, 1),
    //        1);
    //}
    //debugRenderer->addPoint(center0, makeVector3f(1, 1, 0));
    //debugRenderer->addPoint(center1, makeVector3f(0, 1, 1));
    //debugRenderer->addLine(
    //    center0, center1,
    //    makeVector3f(1, 1, 0),
    //    makeVector3f(0, 1, 1),
    //    1);
    //debugRenderer->endRenderJob();
}
