#ifndef PCILpCVT_H
#define PCILpCVT_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
#include "PCInteractionTool.h"
//---------------------------------------------------------------------------

class PROJECTSXWU_API PCILpCVT :
    public PCInteractionTool
{
    DEFINE_CLASS( PCILpCVT );

public:
    typedef boost::shared_ptr< PCILpCVT > Ptr;
    typedef boost::shared_ptr< const PCILpCVT > ConstPtr;

public:
    PCILpCVT(void);
    ~PCILpCVT(void);

public:
    virtual void mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
    virtual void mouseMoved(int32 x, int32 y);
    virtual void mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
    virtual void areaResize(card32 width, card32 height);
    virtual void mouseWheelRotated(int32 rotatedDelta, Modifiers modifiersState);
    virtual void keyDown(GeneralKey key);
    virtual void keyUp(GeneralKey key);
    virtual void glDrawTool(GLContext *glContext);
    virtual void connectToSceneImpl( Scene *scene, OnSceneChangeCallback *onChange, QWidget *sceneEditorWidget, QWidget* toolWidget = NULL );

public:

private:
};

#endif
