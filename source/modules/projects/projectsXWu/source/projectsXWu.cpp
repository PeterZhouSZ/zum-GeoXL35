#include "StdAfx.h"

#include "projectsXWu.h"

#include <TaggingMacros.h>
#include <X4Types.h>

#include "projectsXWu_rcc.h"

#include "PCICL3DRepetitionGroundTruth2015EditorWidget.h"
#include "PCICL3DRepetitionGroundTruth2015.h"

using namespace X4;

#include "projectsXWu.init.cpp"



BEGIN_INIT_MODULE
    qInitResources();
	((GUIDescriptor*)PCICL3DRepetitionGroundTruth2015::getClass()->getGUIDescriptor())->addClassEditor(
	PCICL3DRepetitionGroundTruth2015Editor::getClass(), true);
END_INIT_MODULE 

BEGIN_SHUTDOWN_MODULE
END_SHUTDOWN_MODULE 

BEGIN_X4_LOADABLE_MODULE(projectsXWu)
	#include "projectsXWu.tags.h"
END_X4_LOADABLE_MODULE(projectsXWu)

