#ifndef PCIEditSyn_H
#define PCIEditSyn_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "RepBoxLoader.h"
#include "ProjectSettings.h"
#include "Interface/EditSynHandle.h"
#include "EditSynGizmoHandler.h"
#include "DenseGrid3.h"
#include "KNNPaternGentor.h"
//---------------------------------------------------------------------------
#include "PCInteractionTool.h"
//---------------------------------------------------------------------------

class PROJECTSXWU_API PCIEditSyn : public PCInteractionTool
{
    DEFINE_CLASS(PCIEditSyn);
    friend class EditSynGizmoHandler;
    friend class EditSynGizmoHandlerControl;
    friend class EditSynGizmoHandlerSelect;

public:
    bool Initialize(void);
    void StateChangePrint(void);
    void ProliferatePattern(void);

public: /** basics */
    PCIEditSyn(void);
    ~PCIEditSyn(void);
    virtual void assign(const Object* obj, COPY_CONTEXT *context = nullptr);

public:
    virtual void keyDown(GeneralKey key);
    virtual void keyUp(GeneralKey key);
    virtual void mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
    virtual void mouseMoved(int32 x, int32 y);
    virtual void mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
    virtual void areaResize(card32 width, card32 height);
    virtual void mouseWheelRotated(int32 rotatedDelta, Modifiers modifiersState);
    virtual void glDrawTool(GLContext *glContext);
private:
    virtual void connectToSceneImpl(Scene *scene, OnSceneChangeCallback *onChange, QWidget *sceneEditorWidget, QWidget* toolWidget = NULL);

private:
    UICPC* RetrieveUICPC(std::string const& node, UICPC* reference = 0);
    void initProbGrid(void);
    void proliferateOnce(void);
    void initProbs(void);
    void addUserHypos(void);

private:
    EditSynGizmoHandler::Mode uiMode_;
    Handle3DGizmo::Mode controlMode_;
    typedef std::map<EditSynGizmoHandler::Mode, EditSynGizmoHandler::Ptr> GizmoHandlerMap;
    GizmoHandlerMap gizmoHandlerMap_;
    Handle3DGizmo::Ptr handle_; // shared storage
    bool showGizmo;
    bool initialized_;

    float wireWidth_;

protected:
    std::string data_folder;
    std::string example_name;
    std::string guide_name;

protected:
    RepBoxLoader::Ptr repBoxLoader;
    std::deque<RepBox::Ptr> guideBoxes;

protected:
    DenseGrid3f::Ptr gridExample;
    bool showProb;
    float cellSize;
    KNNPaternGentor::Ptr knnPaternGentor;
    int numIter;
};

#endif
