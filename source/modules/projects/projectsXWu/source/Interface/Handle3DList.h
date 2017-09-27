//---------------------------------------------------------------------------
#ifndef Handle3DListH
#define Handle3DListH
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "Handle3DGizmo.h"
//---------------------------------------------------------------------------
#include "Persistent.h"
//---------------------------------------------------------------------------


class PROJECTSXWU_API Handle3DList : public Persistent
{
    friend class Handle3DListWidget;

public:
    typedef boost::shared_ptr< Handle3DList > Ptr;
    typedef boost::shared_ptr< const Handle3DList > ConstPtr;

public:
    Handle3DList() : deformPC(nullptr) {}
    void clear(void);
    void setPC(UICPC* PC) { deformPC = PC; }
    const UICPC* getPC(void) const { return deformPC; }
    void push_back(const Handle3DGizmo& handle) { handles_.push_back(handle); }
    bool empty(void) const { return handles_.empty(); }
    void reassign(const std::deque<Handle3DGizmo>& newHandles) { handles_ = newHandles; }

    std::deque<Handle3DGizmo>& getHandles(void) { return handles_; }

    void BuildCoordinate();
    void SetControlMode(const Handle3DGizmo::Mode& controlMode);
    void Update(const std::vector<Vector3f>& deformed);

public:
    bool mouseDown(int32 x, int32 y);
    void mouseMoved(int32 x, int32 y);
    void mouseUp(int32 x, int32 y);
    void areaResize(card32 width, card32 height);
    void glDraw() const;

private:
    std::deque<Handle3DGizmo> handles_;
    UICPC* deformPC;
};

#endif
