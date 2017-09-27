#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Handle3DList.h"
#include "Util\ColorSchemer.hpp"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

void Handle3DList::clear(void)
{
    handles_.clear();
    deformPC = nullptr;
}

void Handle3DList::BuildCoordinate()
{
    if (nullptr == deformPC) return;
    const PointSet& deformPS = *deformPC->getPointSet();
    for (Handle3DGizmo& handle : handles_) {
        handle.BuildCoordinate(deformPS);
    }
}

void Handle3DList::SetControlMode(const Handle3DGizmo::Mode& controlMode)
{
    for (Handle3DGizmo& handle : handles_) {
        handle.SetControlMode(controlMode);
    }
}

void Handle3DList::Update(const std::vector<Vector3f>& deformed)
{
    for (Handle3DGizmo& handle : handles_) {
        handle.Update(deformed);
    }
}

void Handle3DList::glDraw() const
{
    if (nullptr == deformPC) return;
    const PointSet& deformPS = *deformPC->getPointSet();
    for (unsigned hi = 0; hi < handles_.size(); ++hi) {
        handles_[hi].glDraw(deformPS);
    }
}

void Handle3DList::areaResize(card32 width, card32 height)
{
    for (Handle3DGizmo& handle : handles_) {
        handle.areaResize(width, height);
    }
}

bool Handle3DList::mouseDown(int32 x, int32 y)
{
    bool ret = false;
    for (Handle3DGizmo& handle : handles_) {
        if (handle.mouseDown( x, y )) ret = true;
    }
    return ret;
}

void Handle3DList::mouseMoved(int32 x, int32 y)
{
    for (Handle3DGizmo& handle : handles_) {
        handle.mouseMoved( x, y );
    }
}

void Handle3DList::mouseUp(int32 x, int32 y)
{
    for (Handle3DGizmo& handle : handles_) {
        handle.mouseUp( x, y );
    }
}
