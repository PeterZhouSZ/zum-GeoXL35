#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "EditSynHandle.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

EditSynHandle::EditSynHandle(const std::deque<unsigned>& points, const card32& width, const card32& height) :
Handle3DGizmo(points, width, height)
{
}
