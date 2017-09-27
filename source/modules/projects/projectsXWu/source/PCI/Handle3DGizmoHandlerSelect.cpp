#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Handle3DGizmoHandler.h"
#include "PCI/PCISymmDefield.h"
//---------------------------------------------------------------------------
#include "GeometricRange.h"
#include "SelectionTools.h"
#include "SceneGraphState.h"
//---------------------------------------------------------------------------

Handle3DGizmoHandlerSelect::Handle3DGizmoHandlerSelect(PCISymmDefield* pciSymmDefield) :
    Handle3DGizmoHandler(pciSymmDefield, SELECT)
{
    handle3DList_ = pciSymmDefield_->handle3DList_;
    handle3DListWidget_ = pciSymmDefield_->handle3DListWidget_;

    leftMBdown = false;
    rightMBdown = false;

    leftMBdownX = 0;
    leftMBdownY = 0;
    leftMBmoveY = 0;
    leftMBmoveX = 0;

    w_ = 100;
    h_ = 100;
};

void Handle3DGizmoHandlerSelect::activate()
{
    if (pciSymmDefield_->uiMode_ == this->uiMode_) {
        handle3DListWidget_->BuildItemList();
        handle3DListWidget_->show();
        handle3DListWidget_->raise();
    }
    //else handle3DListWidget_->hide();
}

int Handle3DGizmoHandlerSelect::mouseMoved(
    int32 x, int32 y)
{
    leftMBmoveX = x;
    leftMBmoveY = y;

    return 0;
}

//----------------------------------------------------------------------
int Handle3DGizmoHandlerSelect::mouseDown(
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
int Handle3DGizmoHandlerSelect::mouseUp(
    int32 x, int32 y,
    MouseButtons buttonsState, Modifiers modifiersState)
{
    leftMBmoveX = x;
    leftMBmoveY = y;

    if (leftMBmoveX == leftMBdownX && leftMBmoveX == leftMBdownX) pointSelect();
    else rangeSelect();

    leftMBdown = false;
    rightMBdown = false;

    return 0;
}

//----------------------------------------------------------------------
void Handle3DGizmoHandlerSelect::rangeSelect(void)
{
    enum { XOF_SELECT, XOF_REMOVE };
    const int selrem = (leftMBdown) ? XOF_SELECT : XOF_REMOVE;

    UICPC* deformPC = pciSymmDefield_->RetrieveUICPC(pciSymmDefield_->outputSymmName);
    if (nullptr == deformPC) return;

    GeometricRange* range = getRayPyramidalRangeFromMouseClick(
        leftMBdownX, leftMBdownY, leftMBmoveX, leftMBmoveY, w_, h_,
        &(pciSymmDefield_->getScene()->getRootState()->staticState->camera),
        &(pciSymmDefield_->getScene()->getRootState()->staticState->viewFrustum));
    if (nullptr == range) return;

    SceneObjectIterator *newItSO = deformPC->createIterator(SOIT(SOIT::CAP_BASIC_PC | SOIT::CAP_HIERARCHICAL));
    PointCloudIterator *newIt = dynamic_cast<PointCloudIterator*>(newItSO);
    if ((nullptr == newIt) || (nullptr == newItSO)) { delete newItSO; delete range; return; };
    range->setTransformation(deformPC->getTransformation());
    IteratorAdaptor *rangeIt = getGeoRangeIteratorAdaptor(deformPC, newIt, range);
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
            Handle3DGizmo handle(selectedId, w_, h_);
            //if (handle3DList_->empty())
                handle.setActive(true);
            handle3DList_->push_back(handle);
            handle3DListWidget_->BuildItemList();
        } else {
            std::sort(selectedId.begin(), selectedId.end());
            std::deque<Handle3DGizmo> newHandles;
            for (Handle3DGizmo& handle : handle3DList_->getHandles()) {
                handle.set_difference(selectedId);
                if (!handle.empty()) newHandles.push_back(handle);
            }
            handle3DList_->reassign(newHandles);
            handle3DListWidget_->BuildItemList();
        }
    }

    delete rangeIt;
    delete newItSO;
    delete range;
}

//----------------------------------------------------------------------
void Handle3DGizmoHandlerSelect::pointSelect(void)
{
    enum { XOF_SELECT, XOF_REMOVE };
    const int selrem = (leftMBdown) ? XOF_SELECT : XOF_REMOVE;

    UICPC* deformPC = pciSymmDefield_->RetrieveUICPC(pciSymmDefield_->outputSymmName);
    if (nullptr == deformPC) return;
    const AAT posAAT = deformPC->getPointSet()->getAAT("position");

    Vector3f pointPosition;
    std::string cloudName;
    if (!
        getClosestPointPositionFromAllPoints(
        leftMBdownX, leftMBdownY, w_, h_, 5,
        &(pciSymmDefield_->getScene()->getRootState()->staticState->camera),
        &(pciSymmDefield_->getScene()->getRootState()->staticState->viewFrustum),
        pciSymmDefield_->getScene(),
        pointPosition, cloudName)
        ) return;

    std::deque<unsigned> selectedId;
    PointSetANNQuery knn(deformPC->getPointSet(), 1);
    float distSqr;
    selectedId.push_back(knn.getNearestPointIndexAndSqrDistance(pointPosition, distSqr));
    if (5 < distSqr) return;

    if (XOF_SELECT == selrem) {
        Handle3DGizmo handle(selectedId, w_, h_);
        //if (handle3DList_->empty())
            handle.setActive(true);
        handle3DList_->push_back(handle);
        handle3DListWidget_->BuildItemList();
    } else {
        std::sort(selectedId.begin(), selectedId.end());
        std::deque<Handle3DGizmo> newHandles;
        for (Handle3DGizmo& handle : handle3DList_->getHandles()) {
            handle.set_difference(selectedId);
            if (!handle.empty()) newHandles.push_back(handle);
        }
        if (1 == newHandles.size()) newHandles.front().setActive(true);
        handle3DList_->reassign(newHandles);
        handle3DListWidget_->BuildItemList();
    }
}

//----------------------------------------------------------------------
void Handle3DGizmoHandlerSelect::areaResize(
    card32 width, card32 height)
{
    w_ = width;
    h_ = height;
}

//----------------------------------------------------------------------
void Handle3DGizmoHandlerSelect::glDrawTool(GLContext *glContext)
{
    forget(glContext);

    handle3DList_->glDraw();
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
