#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Handle3DGizmoHandler.h"
#include "PCI/PCISymmDefield.h"
//---------------------------------------------------------------------------
#include <iomanip>
//---------------------------------------------------------------------------

Handle3DGizmoHandlerControl::Handle3DGizmoHandlerControl(PCISymmDefield* pciSymmDefield) :
    Handle3DGizmoHandler(pciSymmDefield, CONTROL)
{
    handle3DList_ = pciSymmDefield_->handle3DList_;

    handleLocked = false;
}

void Handle3DGizmoHandlerControl::activate()
{
    if (pciSymmDefield_->uiMode_ == this->uiMode_) {
        handle3DList_->SetControlMode(pciSymmDefield_->controlMode_);
        handle3DList_->BuildCoordinate();
        pciSymmDefield_->solver_->Refactor();
        /*if (pciSymmDefield_->bRealtime)*/ pciSymmDefield_->Deform(); // pre-factorization
    }
}

int Handle3DGizmoHandlerControl::mouseMoved(
    int32 x, int32 y)
{
    handle3DList_->mouseMoved(x, y);
    if (handleLocked && pciSymmDefield_->bRealtime)
    {
        pciSymmDefield_->Deform();
    }
    return 0;
}

//----------------------------------------------------------------------
int Handle3DGizmoHandlerControl::mouseDown(
    int32 x, int32 y,
    MouseButtons buttonsState, Modifiers modifiersState)
{
    pciSymmDefield_->Initialize();
    if (handle3DList_->mouseDown(x, y)) handleLocked = true;
    return 0;
}

//----------------------------------------------------------------------
int Handle3DGizmoHandlerControl::mouseUp(
    int32 x, int32 y,
    MouseButtons buttonsState, Modifiers modifiersState)
{
    handle3DList_->mouseUp(x, y);
    handleLocked = false;
    return 0;
}

//----------------------------------------------------------------------
void Handle3DGizmoHandlerControl::areaResize(
    card32 width, card32 height)
{
    handle3DList_->areaResize(width, height);
}

//----------------------------------------------------------------------
void Handle3DGizmoHandlerControl::glDrawTool(GLContext *glContext)
{
    handle3DList_->glDraw();
}
