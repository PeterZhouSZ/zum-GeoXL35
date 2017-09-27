#include "StdAfx.h"

#include "projectsEG2014.h"

#include <TaggingMacros.h>
#include <X4Types.h>

#include "projectsEG2014_rcc.h"

using namespace X4;

#include "projectsEG2014.init.cpp"



BEGIN_INIT_MODULE
    qInitResources();
END_INIT_MODULE 

BEGIN_SHUTDOWN_MODULE
END_SHUTDOWN_MODULE 

BEGIN_X4_LOADABLE_MODULE(projectsEG2014)
	#include "projectsEG2014.tags.h"
END_X4_LOADABLE_MODULE(projectsEG2014)


