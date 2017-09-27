#ifndef PCITestCentr_H
#define PCITestCentr_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
#include "PCInteractionTool.h"
//---------------------------------------------------------------------------

class PROJECTSXWU_API PCITestCentr : public PCInteractionTool {
private:
    DEFINE_CLASS( PCITestCentr )

public:
    PCITestCentr();

    virtual void mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState) {}
    virtual void mouseMoved(int32 x, int32 y) {}
    virtual void mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState) {}
    virtual void areaResize(card32 width, card32 height) {}
    virtual void keyDown(GeneralKey key) {}
    virtual void keyUp(GeneralKey key) {}
    virtual void glDrawTool(GLContext *glContext) {}

public:
    void TestTrimeshUnorderedGrid(void);

    void TestTrimeshStatic(void);
    void TestWeldTrimesh(void);

    void TestOrderedPointSet(void);

    void TestPatternRotation(void);
    void TestPatternTranslation(void);

    void TestSymmFeatFrame(void);

    void TestSymmetryToolBox(void);

    void TestGraphicalModel(void);

    void TestExperiments(void);

    //void TestSuper4PCS(void);

private:
    bool get1stTrimesh(UnstructuredInCoreTriangleMesh*& mesh);
};

#endif
