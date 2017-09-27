//---------------------------------------------------------------------------
#ifndef PCIStatiCosyn_H
#define PCIStatiCosyn_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "VoxelJointGrid.h"
//---------------------------------------------------------------------------
#include "PCInteractionTool.h"
//---------------------------------------------------------------------------

class PROJECTSXWU_API PCIStatiCosyn : public PCInteractionTool
{
    DEFINE_CLASS(PCIStatiCosyn)

public:
    PCIStatiCosyn();
    ~PCIStatiCosyn();

    // interaction tool interface
    virtual void keyDown(GeneralKey key);
    virtual void keyUp(GeneralKey key) {}
    virtual void mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
    virtual void mouseMoved(int32 x, int32 y);
    virtual void mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
    virtual void mouseWheelRotated(int32 rotatedDelta, Modifiers modifiersState);
    virtual void areaResize(card32 width, card32 height) {}
    virtual void glDrawTool(GLContext *glContext) {}
    virtual void connectToSceneImpl( Scene *scene, OnSceneChangeCallback *onChange, QWidget *sceneEditorWidget, QWidget* toolWidget = NULL );

    // synthesis
    void VoxelizeScene(void);
    void Synthesis(void);
    void Reset(void);

    // debug
    void DrawVoxelsZ(void);

private:
    std::string dataFolder;
    std::string dataName;
    Vector3f oDelta_;
    Vector3f gendir_;
    float voxize_;
    Vector2i margin_;
    Vector2i marginXY_d;
    PointDistributor::Ptr pointVoxer_;
};

#endif