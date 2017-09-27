//---------------------------------------------------------------------------
#ifndef Handle3DGizmoH
#define Handle3DGizmoH
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "LibGizmo\IGizmo.h"
//---------------------------------------------------------------------------
#include "Persistent.h"
//---------------------------------------------------------------------------

class PROJECTSXWU_API Handle3DGizmo : public Persistent
{
public:
    enum Mode { INACTIVE, MOVE, ROTATE, SCALE };

public:
    typedef boost::shared_ptr< Handle3DGizmo > Ptr;
    typedef boost::shared_ptr< const Handle3DGizmo > ConstPtr;

public:
    explicit Handle3DGizmo(const std::deque<unsigned>& points, const card32& width, const card32& height);
    void setPoints(const std::deque<unsigned>& points) { points_.assign(points.begin(), points.end());  }
    const std::deque<unsigned>& getPoints(void) const { return points_; }
    size_t size(void) const { return points_.size(); }
    bool empty(void) const { return points_.empty(); }
    void setActive(const bool& active) { isActive_ = active; };
    void invertActive(void) { isActive_ = !isActive_; };
    bool isActive(void) const { return isActive_; }

    const Vector3f getColor(void) const { return color_; }
    const Vector3f getCenter(void) const;
    const Vector3f getTranslation(void) const;
    const Matrix4f getTransformation(void) const;
    void setTransformation(const Matrix4f& trans);
    void UpdateStep(void);
    void Update(const std::vector<Vector3f>& deformed);
    void set_difference(const std::deque<unsigned>& rhs);

    void BuildCoordinate(const PointSet& PS);
    void SetControlMode(const Mode& controlMode);

public:
    bool mouseDown(int32 x, int32 y);
    void mouseMoved(int32 x, int32 y);
    void mouseUp(int32 x, int32 y);
    void areaResize(card32 width, card32 height);
    virtual void glDraw(const PointSet& PS) const;
    virtual void glDraw(void) const;

protected:
    bool isActive_;
    Vector3f colActive_;
    Vector3f colDormant_;
    Vector3f color_;

    IGizmo::Ptr gizmo;
    IGizmo::Ptr gizmoMove, gizmoRotate, gizmoScale;

    std::deque<unsigned> points_;
    Matrix4f initMatrix_;
    Matrix4f editMatrix_;
};

#endif
