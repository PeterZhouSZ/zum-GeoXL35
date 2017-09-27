//---------------------------------------------------------------------------
#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "EditSynGizmoHandler.h"
#include "PCIEditSyn.h"
#include "Util\ColorSchemer.hpp"
#include "Util\ExtraGL.hpp"
//---------------------------------------------------------------------------
#include <iomanip>
//---------------------------------------------------------------------------

EditSynGizmoHandlerControl::EditSynGizmoHandlerControl(PCIEditSyn* pciTarget) :
EditSynGizmoHandler(pciTarget, CONTROL)
{
    handleLocked = false;
}

void EditSynGizmoHandlerControl::activate(void)
{
    handle_->SetControlMode(pciTarget_->controlMode_);
}
void EditSynGizmoHandlerControl::hibernate(void)
{
    handle_->SetControlMode(pciTarget_->controlMode_);
}

int EditSynGizmoHandlerControl::mouseMoved(
    int32 x, int32 y)
{
    if (!handleLocked) return 1;
    handle_->mouseMoved(x, y);

    return 0;
}

//----------------------------------------------------------------------
int EditSynGizmoHandlerControl::mouseDown(
    int32 x, int32 y,
    MouseButtons buttonsState, Modifiers modifiersState)
{
    if (handleLocked) return 1;
    if (handle_->mouseDown(x, y)) handleLocked = true;
    return 0;
}

//----------------------------------------------------------------------
int EditSynGizmoHandlerControl::mouseUp(
    int32 x, int32 y,
    MouseButtons buttonsState, Modifiers modifiersState)
{
    if (!handleLocked) return 1;
    handle_->mouseUp(x, y);
    handleLocked = false;
    return 0;
}

//----------------------------------------------------------------------
void EditSynGizmoHandlerControl::areaResize(
    card32 width, card32 height)
{
    handle_->areaResize(width, height);
}

//----------------------------------------------------------------------
void EditSynGizmoHandlerControl::glDrawTool(GLContext *glContext)
{
    if (!pciTarget_->showGizmo) return;
    if (!handle_->empty()) {
        const std::deque<RepBox::Ptr>& boxes = pciTarget_->guideBoxes;
        const Vector3f color = handle_->getColor();
        const float radius = pciTarget_->wireWidth_ * .045f;
        const float wireWidth = pciTarget_->wireWidth_ * 2.f;
        const float lineWidth = pciTarget_->wireWidth_ * 0.8f;
        SolidSphere sphere(radius);
        const Vector3f trans = handle_->getTranslation();
        glColor3fv(color.data());
        // illustrate the hints
        for (unsigned pid : handle_->getPoints()) {
            RepBox::Ptr box = boost::shared_ptr<RepBox>(new RepBox(*boxes[pid]));
            const Vector3f cenOrig = box->centroid;
            box->centroid = cenOrig + trans;
            const Vector3f cenCurr = box->centroid;
            sphere.draw(cenCurr[0], cenCurr[1], cenCurr[2]);
            RepBoxLoader::glDrawBox(box, color, wireWidth);
            {
                glPushAttrib(GL_ENABLE_BIT);
                glLineStipple(1, 0x1C47); /* dash/dot/dash */
                glEnable(GL_LINE_STIPPLE);
                glLineWidth(lineWidth);
                glBegin(GL_LINES);
                glVertex3fv(cenOrig.data());
                glVertex3fv(cenCurr.data());
                glEnd();
                glPopAttrib();
            }
        }
        handle_->glDraw();
    }
}
