#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Handle3DGizmoHandler.h"
#include "PCI/PCISymmDefield.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

Handle3DGizmoHandler::Handle3DGizmoHandler(PCISymmDefield* pciSymmDefield, const Mode& uiMode) :
    pciSymmDefield_(pciSymmDefield), uiMode_(uiMode)
{
};
