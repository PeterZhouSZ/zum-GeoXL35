#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "EditSynGizmoHandler.h"
#include "PCIEditSyn.h"
#include "Util\ColorSchemer.hpp"
#include "Util\ExtraGL.hpp"
#include "RepBoxLoader.h"
//---------------------------------------------------------------------------
#include "GeometricRange.h"
#include "SelectionTools.h"
#include "SceneGraphState.h"
//---------------------------------------------------------------------------

EditSynGizmoHandlerSelect::EditSynGizmoHandlerSelect(PCIEditSyn* pciTarget) :
EditSynGizmoHandler(pciTarget, SELECT)
{
    leftMBdown = false;
    rightMBdown = false;

    leftMBdownX = 0;
    leftMBdownY = 0;
    leftMBmoveY = 0;
    leftMBmoveX = 0;

    w_ = 100;
    h_ = 100;
};

void EditSynGizmoHandlerSelect::activate(void)
{
}
void EditSynGizmoHandlerSelect::hibernate(void)
{
}

int EditSynGizmoHandlerSelect::mouseMoved(
    int32 x, int32 y)
{
    leftMBmoveX = x;
    leftMBmoveY = y;

    return 0;
}

//----------------------------------------------------------------------
int EditSynGizmoHandlerSelect::mouseDown(
    int32 x, int32 y,
    MouseButtons buttonsState, Modifiers modifiersState)
{
    if (buttonsState.getLeft()) leftMBdown = true;
    if (buttonsState.getRight()) rightMBdown = true;

    leftMBdownX = x;
    leftMBdownY = y;
    leftMBmoveX = x;
    leftMBmoveY = y;

    return 0;
}

//----------------------------------------------------------------------
int EditSynGizmoHandlerSelect::mouseUp(
    int32 x, int32 y,
    MouseButtons buttonsState, Modifiers modifiersState)
{
    leftMBmoveX = x;
    leftMBmoveY = y;

    if (leftMBmoveX == leftMBdownX && leftMBmoveX == leftMBdownX) pointSelect();
    else rangeSelect();

    if (!handle_->empty())
    {
        const std::deque<unsigned>& points = handle_->getPoints();
        const PointSet& guidePS = *pciTarget_->RetrieveUICPC(pciTarget_->guide_name)->getPointSet();
        const AAT posAAT = guidePS.getAAT("position");
        const std::deque<RepBox::Ptr>& guideBoxes = pciTarget_->guideBoxes;
        const RepBox::Ptr box = guideBoxes[points[0]];
        Matrix4f trans = expand3To4(box->basis);
        trans[3] = expand3To4(guidePS.get3f(points[0], posAAT));
        handle_->setTransformation(trans);
    }

    leftMBdown = false;
    rightMBdown = false;

    return 0;
}

//----------------------------------------------------------------------
void EditSynGizmoHandlerSelect::rangeSelect(void)
{
    enum { XOF_SELECT, XOF_REMOVE };
    const int selrem = (leftMBdown) ? XOF_SELECT : XOF_REMOVE;

    UICPC* guidePC = pciTarget_->RetrieveUICPC(pciTarget_->guide_name);
    if (nullptr == guidePC) return;

    GeometricRange* range = getRayPyramidalRangeFromMouseClick(
        leftMBdownX, leftMBdownY, leftMBmoveX, leftMBmoveY, w_, h_,
        &(pciTarget_->getScene()->getRootState()->staticState->camera),
        &(pciTarget_->getScene()->getRootState()->staticState->viewFrustum));
    if (nullptr == range) return;

    SceneObjectIterator *newItSO = guidePC->createIterator(SOIT(SOIT::CAP_BASIC_PC | SOIT::CAP_HIERARCHICAL));
    PointCloudIterator *newIt = dynamic_cast<PointCloudIterator*>(newItSO);
    if ((nullptr == newIt) || (nullptr == newItSO)) { delete newItSO; delete range; return; };
    range->setTransformation(guidePC->getTransformation());
    IteratorAdaptor *rangeIt = getGeoRangeIteratorAdaptor(guidePC, newIt, range);
    if (nullptr == rangeIt) { delete newItSO; delete range; return; }

    std::deque<unsigned> selectedId;
    while (!rangeIt->atEnd())
    {
        selectedId.push_back(rangeIt->getCurrentPointIndex());
        rangeIt->next();
    }

    if (!selectedId.empty())
    {
        if (XOF_SELECT == selrem) {
            handle_->setPoints(selectedId);
        }
        else {
            std::sort(selectedId.begin(), selectedId.end());
            handle_->set_difference(selectedId);
        }
    }

    delete rangeIt;
    delete newItSO;
    delete range;
}

//----------------------------------------------------------------------
void EditSynGizmoHandlerSelect::pointSelect(void)
{
    enum { XOF_SELECT, XOF_REMOVE };
    const int selrem = (leftMBdown) ? XOF_SELECT : XOF_REMOVE;

    UICPC* guidePC = pciTarget_->RetrieveUICPC(pciTarget_->guide_name);
    if (nullptr == guidePC) return;
    const AAT posAAT = guidePC->getPointSet()->getAAT("position");

    Vector3f pointPosition;
    std::string cloudName;
    if (!
        getClosestPointPositionFromAllPoints(
        leftMBdownX, leftMBdownY, w_, h_, 5,
        &(pciTarget_->getScene()->getRootState()->staticState->camera),
        &(pciTarget_->getScene()->getRootState()->staticState->viewFrustum),
        pciTarget_->getScene(),
        pointPosition, cloudName)
        ) return;

    std::deque<unsigned> selectedId;
    PointSetANNQuery knn(guidePC->getPointSet(), 1);
    float distSqr;
    selectedId.push_back(knn.getNearestPointIndexAndSqrDistance(pointPosition, distSqr));
    if (5 < distSqr) return;

    if (XOF_SELECT == selrem) {
        handle_->setPoints(selectedId);
    }
    else if (nullptr != handle_) {
        std::sort(selectedId.begin(), selectedId.end());
        handle_->set_difference(selectedId);
    }
}

//----------------------------------------------------------------------
void EditSynGizmoHandlerSelect::areaResize(
    card32 width, card32 height)
{
    w_ = width;
    h_ = height;
    handle_->areaResize(width, height);
}

//----------------------------------------------------------------------
void EditSynGizmoHandlerSelect::glDrawTool(GLContext *glContext)
{
    forget(glContext);

    if (!handle_->empty()) {
        const std::deque<RepBox::Ptr>& boxes = pciTarget_->guideBoxes;
        const Vector3f color = handle_->getColor();
        const float radius = pciTarget_->wireWidth_ * .045f;
        const float wireWidth = pciTarget_->wireWidth_ * 2.f;
        SolidSphere sphere(radius);
        glColor3fv(color.data());
        for (unsigned pid : handle_->getPoints()) {
            const Vector3f cen = boxes[pid]->centroid;
            sphere.draw(cen[0], cen[1], cen[2]);
            RepBoxLoader::glDrawBox(boxes[pid], color, wireWidth);
        }
        handle_->glDraw();
    }
    if (false == leftMBdown && false == rightMBdown) return;

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glDisable(GL_LIGHTING);
    glLineWidth(1.0f);

    float32 posX =   (leftMBmoveX-w_*0.5f) / (w_*0.5f);
    float32 posY = -((leftMBmoveY-h_*0.5f) / (h_*0.5f));

    float32 posX2 =  (leftMBdownX-w_*0.5f) / (w_*0.5f);
    float32 posY2 =-((leftMBdownY-h_*0.5f) / (h_*0.5f));

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    if (leftMBdown) glColor4f(1.0f,0.9f,0.1f,0.4f);
    else glColor4f(0.1f,0.9f,1.0f,0.4f);
    {
        glBegin(GL_QUADS);
        glVertex2f(posX2, posY2);
        glVertex2f(posX2, posY);
        glVertex2f(posX, posY);
        glVertex2f(posX, posY2);
        glEnd();
    }
    if (leftMBdown) glColor4f(1.0f,0.9f,0.0f,0.9f);
    else glColor4f(0.0f,0.9f,1.0f,0.9f);
    {
        glBegin(GL_LINE_LOOP);
        glVertex2f(posX2, posY2);
        glVertex2f(posX2, posY);
        glVertex2f(posX, posY);
        glVertex2f(posX, posY2);
        glEnd();
    }

    glPopAttrib();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}
