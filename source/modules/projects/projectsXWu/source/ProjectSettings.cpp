//---------------------------------------------------------------------------
#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "ProjectSettings.h"
//---------------------------------------------------------------------------
#include "CopyObjectProperties.h"
#include "PropertyTableProperty.h"
#include "SeparatorClassProperty.h"
//---------------------------------------------------------------------------

ProjectSettings projectSettings;

IMPLEMENT_CLASS(ProjectSettings, 0)
{
    BEGIN_CLASS_INIT(ProjectSettings);
    ADD_SEPARATOR("hack");
    ADD_BOOLEAN_PROP(bFlipNormal, 0);
}

ProjectSettings::ProjectSettings() 
{
    bFlipNormal = true;
}

ProjectSettings::~ProjectSettings()
{

}

void ProjectSettings::assign(const Object* obj, X4CopyContext *context)
{
	copyObjectProperties(obj, this);
}
