#ifndef PCISupres_H
#define PCISupres_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "RepBoxLoader.h"
#include "ProjectSettings.h"
//---------------------------------------------------------------------------
#include "PCInteractionTool.h"
//---------------------------------------------------------------------------

class PROJECTSXWU_API PCISupres : public PCInteractionTool
{
    DEFINE_CLASS(PCISupres);

public: /** transformation generation */
    void InitRepBox(void);
    void PreProc(void);
    void GenerateParchTrans(void);
    void ValidateTransformations(void);
    void TestPlaneGrid(void);
    void CompleteScene(void);
    void CompleteSceneSymmetry(void);
    void ExtractGroup(void);

public: /** basics */
    PCISupres(void);
    ~PCISupres(void);
    virtual void assign(const Object* obj, COPY_CONTEXT *context = nullptr);

public:
    virtual void keyDown(GeneralKey key) { keyDownPassToCam(key); }
    virtual void keyUp(GeneralKey key) { keyUpPassToCam(key); }
    virtual void mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState) { mouseDownPassToCam(x, y, buttonsState, modifiersState); }
    virtual void mouseMoved(int32 x, int32 y) { mouseMovedPassToCam(x, y); }
    virtual void mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState) { mouseUpPassToCam(x, y, buttonsState, modifiersState); }
    virtual void areaResize(card32 width, card32 height) { areaResizePassToCam(width, height); }
    virtual void mouseWheelRotated(int32 rotatedDelta, Modifiers modifiersState) { mouseWheelRotatedPassToCam(rotatedDelta, modifiersState); }
    //virtual void glDrawTool(GLContext *glContext);
private:
    virtual void connectToSceneImpl(Scene *scene, OnSceneChangeCallback *onChange, QWidget *sceneEditorWidget, QWidget* toolWidget = NULL);

protected:
    RepBoxLoader::Ptr repBoxLoader;

protected:
    std::string data_folder;
    std::string example_name;
    std::string guide_name;
    std::string box_cloud;

    unsigned num_infer_trans;
    float cell_size;
    unsigned cell_subdiv;
    unsigned verbose;

protected:
    ProjectSettings* projectSettings_;
    float match_ratio;
    float outlier_ratio;
    float inlier_ratio;
    unsigned num_icp;
    float cost_keep_empty;
    float mean_error_th;
};

#endif
