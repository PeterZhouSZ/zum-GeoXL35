#ifndef PCISymmDefield_H
#define PCISymmDefield_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "SPSolver/SymmSpaceSolver.h"
#include "SPSolver/HandleConstraints.h"
#include "SPSolver/SymmetryConstraints.h"
#include "SPSolver/SlideConstraints.h"
#include "Handle3DGizmoHandler.h"
#include "Interface/Handle3DList.h"
//---------------------------------------------------------------------------
#include "PCInteractionTool.h"
//---------------------------------------------------------------------------

class PROJECTSXWU_API PCISymmDefield :
    public PCInteractionTool
{
    DEFINE_CLASS( PCISymmDefield );
    friend class Handle3DGizmoHandler;
    friend class Handle3DGizmoHandlerControl;
    friend class Handle3DGizmoHandlerSelect;
    friend class Handle3DGizmoHandlerSymCon;
    friend class Handle3DGizmoHandlerSymSel;

public:
    typedef boost::shared_ptr< PCISymmDefield > Ptr;
    typedef boost::shared_ptr< const PCISymmDefield > ConstPtr;

public:
    PCISymmDefield(void);
    ~PCISymmDefield(void);

public:
    virtual void mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
    virtual void mouseMoved(int32 x, int32 y);
    virtual void mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
    virtual void areaResize(card32 width, card32 height);
    virtual void mouseWheelRotated(int32 rotatedDelta, Modifiers modifiersState);
    virtual void keyDown(GeneralKey key);
    virtual void keyUp(GeneralKey key);
    virtual void glDrawTool(GLContext *glContext);

public:
    void OrganizeFiles(void);
    int InitializeSymmGroup();
    void Initialize(void);
    void Deform(void);
    void Reset(void);
    void Rebuild(void);

public:
    void ShowSampleGraph(void);

private:
    UICPC* RetrieveUICPC(std::string const& node, UICPC* reference = 0);
    virtual void connectToSceneImpl( Scene *scene, OnSceneChangeCallback *onChange, QWidget *sceneEditorWidget, QWidget* toolWidget = NULL );

private:
    Handle3DGizmoHandler::Mode uiMode_;
    Handle3DGizmo::Mode controlMode_;
    typedef std::map<Handle3DGizmoHandler::Mode, Handle3DGizmoHandler::Ptr> GizmoHandlerMap;
    GizmoHandlerMap gizmoHandlerMap_;
    Handle3DList::Ptr handle3DList_;
    Handle3DListWidget::Ptr handle3DListWidget_;
    Handle3DList::Ptr handle3DSymList_;
    Handle3DListWidget::Ptr handle3DSymListWidget_;

private:
    typedef std::array<int, 2> key_type;
    typedef std::map<key_type, std::deque<unsigned> > map_type;
    map_type symPointMap;

private:
    std::string symmWorkNameBase;
    std::string symmGroupNameBase;
    std::string inputSymmName;
    std::string outputSymmName;
    std::string resultName;
    std::string dataFolder;
    std::string partFolder;

private:
    bool showMesh_;
    bool showSample_;
    bool showGizmo_;
    unsigned uiStep_;

private:
    bool useNullSpace;
    bool useFixedRigion;
    SymmSpaceSolver::Ptr solver_;
    HandleConstraints::Ptr handleConstraints_;
    SlideConstraints::Ptr slideConstraints_;
    SymmetryConstraints::Ptr symmetryConstraints_;

    bool bIntialized_;
    bool bRealtime;
    bool bShowGraph;
    Timer rtTimer_;
    float rtTime_;

    unsigned iteration_;
    card32 numIterations;
    bool optimize_;

    float sampleEps;
    card32 minClique;
    float gridResolution;
    bool sampleOnLines;
    float topRadiusMultiplier;
    float basePoissonResolution;

    bool useSubdivision;
    float interpoMultiplier;

    bool seperateGroup;
    bool useInertia;

    bool useSymmetry;
    bool useColinearity;
    bool useCoplanarity;
    card32 minPlaneSize;
    float regularizerWeight;
    float symmetryWeight;
    float handleWeight;
    float colinearityWeight;
    float coplanarityWeight;
    unsigned startDRFrame;

    float sampleEpsReset_;
    card32 minCliqueReset_;
    float gridResolutionReset_;
    bool sampleOnLinesReset_;
    float topRadiusMultiplierReset_;
    bool seperateGroupReset_;
};

//struct SymmetrySamplePointsMap : public AttachedData
//{
//    DEFINE_CLASS( SymmetrySamplePointsMap );
//    typedef std::map<Vector2i, std::deque<unsigned> > map_type;
//    ~SymmetrySamplePointsMap() {}
//    static std::string getDefaultName() { return "SymmetrySamplePointsMapAttachment"; }
//    map_type symPointMap;
//};

#endif
