//---------------------------------------------------------------------------
#ifndef Handle3DGizmoHandlerH
#define Handle3DGizmoHandlerH
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "Interface/Handle3DListWidget.h"
//---------------------------------------------------------------------------
#include "MouseButtons.h"
//---------------------------------------------------------------------------

class PCISymmDefield;

class PROJECTSXWU_API Handle3DGizmoHandler
{
public:
    typedef boost::shared_ptr< Handle3DGizmoHandler > Ptr;
    typedef boost::shared_ptr< const Handle3DGizmoHandler > ConstPtr;

public:
    enum Mode { CONTROL, SELECT, SYMCON, SYMSEL };

public:
    explicit Handle3DGizmoHandler(PCISymmDefield* pciSymmDefield, const Mode& uiMode);
    virtual ~Handle3DGizmoHandler() {}

    //void SetControlMode(const HandlerControlMode& controlMode) { controlMode_ = controlMode; }
    virtual void activate() {}

public:
    virtual int mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState) = 0;
    virtual int mouseMoved(int32 x, int32 y) = 0;
    virtual int mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState) = 0;
    virtual void areaResize(card32 width, card32 height) = 0;
    virtual void glDrawTool(GLContext *glContext) = 0;

protected:
    PCISymmDefield* pciSymmDefield_;
    Handle3DList::Ptr handle3DList_;

protected:
    Mode uiMode_;
    //HandlerControlMode controlMode_;
};

class PROJECTSXWU_API Handle3DGizmoHandlerControl : public Handle3DGizmoHandler
{
public:
    typedef boost::shared_ptr< Handle3DGizmoHandlerControl > Ptr;
    typedef boost::shared_ptr< const Handle3DGizmoHandlerControl > ConstPtr;

public:
    explicit Handle3DGizmoHandlerControl(PCISymmDefield* pciSymmDefield);

    virtual void activate();

public:
    virtual int mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
    virtual int mouseMoved(int32 x, int32 y);
    virtual int mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
    virtual void areaResize(card32 width, card32 height);
    virtual void glDrawTool(GLContext *glContext);

protected:
    bool handleLocked;
};

class PROJECTSXWU_API Handle3DGizmoHandlerSelect : public Handle3DGizmoHandler
{
public:
    typedef boost::shared_ptr< Handle3DGizmoHandlerSelect > Ptr;
    typedef boost::shared_ptr< const Handle3DGizmoHandlerSelect > ConstPtr;

public:
    explicit Handle3DGizmoHandlerSelect(PCISymmDefield* pciSymmDefield);

    virtual void activate();

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
    Handle3DListWidget::Ptr handle3DListWidget_;

protected:
    card32 w_, h_;
    bool leftMBdown, rightMBdown;
    int32 leftMBdownX, leftMBdownY;
    int32 leftMBmoveX, leftMBmoveY;
};

class PROJECTSXWU_API Handle3DGizmoHandlerSymCon : public Handle3DGizmoHandler
{
public:
    typedef boost::shared_ptr< Handle3DGizmoHandlerSymCon > Ptr;
    typedef boost::shared_ptr< const Handle3DGizmoHandlerSymCon > ConstPtr;

public:
    explicit Handle3DGizmoHandlerSymCon(PCISymmDefield* pciSymmDefield);

    virtual void activate();

public:
    virtual int mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
    virtual int mouseMoved(int32 x, int32 y);
    virtual int mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
    virtual void areaResize(card32 width, card32 height);
    virtual void glDrawTool(GLContext *glContext);

protected:
    bool handleLocked;
};

class PROJECTSXWU_API Handle3DGizmoHandlerSymSel : public Handle3DGizmoHandler
{
public:
    typedef boost::shared_ptr< Handle3DGizmoHandlerSymSel > Ptr;
    typedef boost::shared_ptr< const Handle3DGizmoHandlerSymSel > ConstPtr;

public:
    explicit Handle3DGizmoHandlerSymSel(PCISymmDefield* pciSymmDefield);

    virtual void activate();

public:
    virtual int mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
    virtual int mouseMoved(int32 x, int32 y);
    virtual int mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
    virtual void areaResize(card32 width, card32 height);
    virtual void glDrawTool(GLContext *glContext);

private:
    void rangeSelect(void);

protected:
    Handle3DListWidget::Ptr handle3DListWidget_;

protected:
    card32 w_, h_;
    bool leftMBdown, rightMBdown;
    int32 leftMBdownX, leftMBdownY;
    int32 leftMBmoveX, leftMBmoveY;
};

#endif
