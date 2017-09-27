#ifndef PCISelDelMark_H
#define PCISelDelMark_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "RepBoxLoader.h"
#include "PCwithGrid.h"
//---------------------------------------------------------------------------
#include "PCInteractionTool.h"
//---------------------------------------------------------------------------

class PROJECTSXWU_API PCISelDelMark : public PCInteractionTool
{
    DEFINE_CLASS(PCISelDelMark);

public:
    enum { XF_REMOVE = -1, XF_NORMAL = 0, XF_BOUNDARY = 1 };

public:
    void MarkSelAsBoundary(void);

public: /** basics */
    PCISelDelMark(void);
    ~PCISelDelMark(void);
    virtual void assign(const Object* obj, COPY_CONTEXT *context = nullptr);

public:
    virtual void keyDown(GeneralKey key);
    virtual void keyUp(GeneralKey key) { keyUpPassToCam(key); }
    virtual void mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
    virtual void mouseMoved(int32 x, int32 y);
    virtual void mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
    virtual void areaResize(card32 width, card32 height);
    virtual void mouseWheelRotated(int32 rotatedDelta, Modifiers modifiersState) { mouseWheelRotatedPassToCam(rotatedDelta, modifiersState); }
    virtual void glDrawTool(GLContext *glContext);
private:
    virtual void connectToSceneImpl(Scene *scene, OnSceneChangeCallback *onChange, QWidget *sceneEditorWidget, QWidget* toolWidget = NULL);

private:
    void rangeSelect(void);

protected:
    card32 w_, h_;
    bool leftMBdown, rightMBdown;
    int32 leftMBdownX, leftMBdownY;
    int32 leftMBmoveX, leftMBmoveY;

    std::string data_folder;
    std::string example_name;
    float boundary_ratio;
};

#endif
