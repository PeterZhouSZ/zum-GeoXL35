//---------------------------------------------------------------------------
#ifndef EditSynGizmoHandlerH
#define EditSynGizmoHandlerH
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "Interface/Handle3DGizmo.h"
//---------------------------------------------------------------------------
#include "MouseButtons.h"
//---------------------------------------------------------------------------

class PCIEditSyn;

class PROJECTSXWU_API EditSynGizmoHandler
{
public:
    typedef boost::shared_ptr< EditSynGizmoHandler > Ptr;
    typedef boost::shared_ptr< const EditSynGizmoHandler > ConstPtr;

public:
    enum Mode { CONTROL, SELECT };

public:
    explicit EditSynGizmoHandler(PCIEditSyn* pciTarget, const Mode& uiMode);
    virtual ~EditSynGizmoHandler() {}

    //void SetControlMode(const HandlerControlMode& controlMode) { controlMode_ = controlMode; }
    virtual void activate(void) = 0;
    virtual void hibernate(void) = 0;

public:
    virtual int mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState) = 0;
    virtual int mouseMoved(int32 x, int32 y) = 0;
    virtual int mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState) = 0;
    virtual void areaResize(card32 width, card32 height) = 0;
    virtual void glDrawTool(GLContext *glContext) = 0;

protected:
    PCIEditSyn* pciTarget_;
    Handle3DGizmo::Ptr handle_;

public:
    Mode uiMode_;
    //HandlerControlMode controlMode_;
};

class PROJECTSXWU_API EditSynGizmoHandlerControl : public EditSynGizmoHandler
{
public:
    typedef boost::shared_ptr< EditSynGizmoHandlerControl > Ptr;
    typedef boost::shared_ptr< const EditSynGizmoHandlerControl > ConstPtr;

public:
    explicit EditSynGizmoHandlerControl(PCIEditSyn* pciTarget);

    virtual void activate(void);
    virtual void hibernate(void);

public:
    virtual int mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
    virtual int mouseMoved(int32 x, int32 y);
    virtual int mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
    virtual void areaResize(card32 width, card32 height);
    virtual void glDrawTool(GLContext *glContext);

protected:
    bool handleLocked;
};

class PROJECTSXWU_API EditSynGizmoHandlerSelect : public EditSynGizmoHandler
{
public:
    typedef boost::shared_ptr< EditSynGizmoHandlerSelect > Ptr;
    typedef boost::shared_ptr< const EditSynGizmoHandlerSelect > ConstPtr;

public:
    explicit EditSynGizmoHandlerSelect(PCIEditSyn* pciTarget);

    virtual void activate(void);
    virtual void hibernate(void);

public:
    virtual int mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
    virtual int mouseMoved(int32 x, int32 y);
    virtual int mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
    virtual void areaResize(card32 width, card32 height);
    virtual void glDrawTool(GLContext *glContext);

private:
    void rangeSelect(void);
    void pointSelect(void);

protected:

protected:
    card32 w_, h_;
    bool leftMBdown, rightMBdown;
    int32 leftMBdownX, leftMBdownY;
    int32 leftMBmoveX, leftMBmoveY;
};

#endif
