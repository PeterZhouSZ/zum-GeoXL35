#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "EditSynGizmoHandler.h"
#include "PCIEditSyn.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

EditSynGizmoHandler::EditSynGizmoHandler(PCIEditSyn* pciTarget, const Mode& uiMode) :
pciTarget_(pciTarget), uiMode_(uiMode)
{
    handle_ = pciTarget->handle_;
}
