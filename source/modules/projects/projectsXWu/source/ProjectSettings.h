//---------------------------------------------------------------------------
#ifndef ProjectSettings_H
#define ProjectSettings_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------

class PROJECTSXWU_API ProjectSettings : public Persistent
{
    DEFINE_CLASS(ProjectSettings)
public:

public:
    ProjectSettings();
    ~ProjectSettings();
    virtual void assign(const Object* obj, X4CopyContext *context = NULL);

public:
    // hack
    bool bFlipNormal;
};

extern ProjectSettings projectSettings;

#endif
