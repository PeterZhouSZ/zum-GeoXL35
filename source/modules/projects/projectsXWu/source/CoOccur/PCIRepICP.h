#ifndef PCIRepICP_H
#define PCIRepICP_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "RepBoxLoader.h"
//---------------------------------------------------------------------------
#include "PCInteractionTool.h"
//---------------------------------------------------------------------------

class PROJECTSXWU_API PCIRepICP :
    public PCInteractionTool
{
    DEFINE_CLASS( PCIRepICP );

public:
    typedef boost::shared_ptr< PCIRepICP > Ptr;
    typedef boost::shared_ptr< const PCIRepICP > ConstPtr;

public:
    PCIRepICP(void);
    ~PCIRepICP(void);

    void loadGT(void);
    void saveGT(void);

private:
    void drawBox(RepBox::Ptr box, Vector3f color, float32 lindwidth);

    void drawQuad(
        const Vector3f &p1, const Vector3f &p2, const Vector3f &p3, const Vector3f &p4,
        const Vector3f &color, const float &linewidth);

    void drawCubeTR(
        const Vector3f &pbegin, const Vector3f &pend,
        const Vector3f &color, const float &linewidth,
        Vector3f T, Matrix3f R
        );

public:
    virtual void mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
    virtual void mouseMoved(int32 x, int32 y);
    virtual void mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
    virtual void mouseWheelRotated(int32 rotatedDelta, Modifiers modifiersState);
    virtual void areaResize(card32 width, card32 height);
    virtual void keyDown(GeneralKey key);
    virtual void keyUp(GeneralKey key);
    virtual void glDrawTool(GLContext *glContext);
private:
    virtual void connectToSceneImpl(Scene *scene, OnSceneChangeCallback *onChange, QWidget *sceneEditorWidget, QWidget* toolWidget = NULL);

public:
    void GeoAlign(void);

private:
    std::deque< std::deque<RepBox::Ptr> > boxRecord;
};

#endif
