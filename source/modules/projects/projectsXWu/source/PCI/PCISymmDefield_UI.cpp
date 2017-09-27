#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "PCI/PCISymmDefield.h"
//---------------------------------------------------------------------------
#include "SceneEditorWidget.h"
#include "LCHierarchyRenderer.h"
#include "GLShaderMaterial.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

//---------------------------------------------------------------------------
void PCISymmDefield::ShowSampleGraph(void)
{
    UICPC* samplePC  = RetrieveUICPC(inputSymmName);
    if (!samplePC)
    {
        error("PCISymmDefield::ShowSampleGraph() - Sample node not found.");
        return;
    }
    UICPC* deformPC = RetrieveUICPC(outputSymmName);
    if (!deformPC)
    {
        error("PCISymmDefield::ShowSampleGraph() - Deformed node not found.");
        return;
    }
    const PointSet& deformPS = *deformPC->getPointSet();
    SymmSampleGraph* symSamGraph = dynamic_cast<SymmSampleGraph*>(
        samplePC->getAttachments()->getData(SymmSampleGraph::getDefaultName()));
    if (!symSamGraph) {
        error("PCISymmDefield::ShowSampleGraph() - SymmSampleGraph missing.");
        return;
    }
    DR_FRAME = 0;
    symSamGraph->DrawWithDR(deformPS, DR_FRAME);
}

void PCISymmDefield::mouseMoved(
    int32 x, int32 y)
{
    rtTime_ = rtTimer_.getDeltaValue();
    if (0 == gizmoHandlerMap_[uiMode_]->mouseMoved(x, y)) return;
    mouseMovedPassToCam(x, y);
}

//----------------------------------------------------------------------
void PCISymmDefield::mouseDown(
    int32 x, int32 y,
    MouseButtons buttonsState, Modifiers modifiersState)
{
    rtTime_ = rtTimer_.getDeltaValue();
    if (0 == gizmoHandlerMap_[uiMode_]->mouseDown(x, y, buttonsState, modifiersState)) return;
    mouseDownPassToCam(x, y, buttonsState, modifiersState);
}

//----------------------------------------------------------------------
void PCISymmDefield::mouseUp(
    int32 x, int32 y,
    MouseButtons buttonsState, Modifiers modifiersState)
{
    if (0 == gizmoHandlerMap_[uiMode_]->mouseUp(x, y, buttonsState, modifiersState)) return;
    mouseUpPassToCam(x, y, buttonsState, modifiersState);
}

//----------------------------------------------------------------------
void PCISymmDefield::areaResize(
    card32 width, card32 height)
{
    for (GizmoHandlerMap::iterator it = gizmoHandlerMap_.begin(); it != gizmoHandlerMap_.end(); ++it) {
        it->second->areaResize(width, height);
    }

    areaResizePassToCam(width, height);
}

//----------------------------------------------------------------------
void PCISymmDefield::glDrawTool(GLContext *glContext)
{
    if (!showGizmo_) return;

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    gizmoHandlerMap_[uiMode_]->glDrawTool(glContext);

    glPopAttrib();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
}

//----------------------------------------------------------------------
void PCISymmDefield::mouseWheelRotated(
    int32 rotatedDelta,
    Modifiers modifiersState)
{
    mouseWheelRotatedPassToCam(rotatedDelta, modifiersState);
}

//----------------------------------------------------------------------
void PCISymmDefield::keyDown(GeneralKey key)
{
    Initialize();

    const char& keyAlphaNum = key.getkeyAlphaNumKey();
    switch (keyAlphaNum)
    {
    case 'q':
    case 'Q':
        uiMode_ = key.getModifiers().getAlt() ? Handle3DGizmoHandler::SYMSEL : Handle3DGizmoHandler::SELECT;
        controlMode_ = Handle3DGizmo::INACTIVE;
        break;
    case 'w':
    case 'W':
        controlMode_ = Handle3DGizmo::MOVE;
        uiMode_ = key.getModifiers().getAlt() ? Handle3DGizmoHandler::SYMCON : Handle3DGizmoHandler::CONTROL;
        break;
    case 'e':
    case 'E':
        controlMode_ = Handle3DGizmo::ROTATE;
        uiMode_ = Handle3DGizmoHandler::CONTROL;
        break;
        //case 'r':
        //case 'R':
        //    controlMode_ = SCALE;
        //    uiMode_ = CONTROL;
        //    break;
    case 'a':
    case 'A':
        showMesh_ = !showMesh_;
        {
            SGObjectNode* node = getSceneGraphObject(scene, resultName);
            if (node)
            {
                if (showMesh_) {
                    node->setVisible(true);
                } else {
                    node->setVisible(false);
                }
            }
        }
        return;
    case 's':
    case 'S':
        showSample_ = !showSample_;
        {
            SGObjectNode* node = getSceneGraphObject(scene, outputSymmName);
            if (node)
            {
                if (showSample_) {
                    node->setVisible(true);
                } else {
                    node->setVisible(false);
                }
            }
        }
        return;
    case 'd':
    case 'D':
        showGizmo_ = !showGizmo_;
        return;
    case 'z':
    case 'Z':
        ++uiStep_;
        switch (uiStep_)
        {
        case 1:
            {
                SGObjectNode* node = getSceneGraphObject(scene, resultName);
                node->setVisible(false);
            }
            {
                SGObjectNode* node = getSceneGraphObject(scene, outputSymmName);
                node->setVisible(true);
            }
            {
                showGizmo_ = true;
            }
            return;
        case 2:
            {
                SGObjectNode* node = getSceneGraphObject(scene, resultName);
                node->setVisible(true);
            }
            {
                SGObjectNode* node = getSceneGraphObject(scene, outputSymmName);
                node->setVisible(false);
            }
            {
                showGizmo_ = true;
            }
            return;
        case 3:
            {
                SGObjectNode* node = getSceneGraphObject(scene, resultName);
                node->setVisible(true);
            }
            {
                SGObjectNode* node = getSceneGraphObject(scene, outputSymmName);
                node->setVisible(false);
            }
            {
                showGizmo_ = false;
            }
            return;
        }
        return;
    default:
        keyDownPassToCam(key);
        return;
    }

    debugOutput << "\n";
    debugOutput << "########################################\n";
    std::string modeString;
    switch (uiMode_)
    {
    case Handle3DGizmoHandler::CONTROL: modeString = "CONTROL"; break;
    case Handle3DGizmoHandler::SELECT: modeString = "SELECT"; break;
    case Handle3DGizmoHandler::SYMCON: modeString = "SYMCON"; break;
    case Handle3DGizmoHandler::SYMSEL: modeString = "SYMSEL"; break;
    }
    debugOutput << "UI mode changed to " << modeString << "\n";
    debugOutput << "\n";

    for (GizmoHandlerMap::iterator it = gizmoHandlerMap_.begin(); it != gizmoHandlerMap_.end(); ++it) {
        it->second->activate();
    }
}

//----------------------------------------------------------------------
void PCISymmDefield::keyUp(GeneralKey key)
{
    keyUpPassToCam(key);
}

//----------------------------------------------------------------------
void PCISymmDefield::connectToSceneImpl(Scene * scene,OnSceneChangeCallback *onChange, QWidget *sceneEditorWidget, QWidget* toolWidget)
{
    SceneEditorWidget* sceneEditorW = dynamic_cast<NAMESPACE_VERSION::SceneEditorWidget*>(sceneEditorWidget);
    sceneEditorW->on_action_Auto_Layout_triggered(true);
    scene->getRootState()->staticState->backgroundColor = makeVector4f(1.0f, 1.0f, 1.0f, 1.0f); // white
    //scene->getRootState()->staticState->backgroundColor = makeVector4f(0.6f, 0.6f, 0.9f, 1.0f); // bright
    scene->getRootState()->staticState->rfShowLocalCoords = false;
    LCHierarchyRenderer* lchRenderer = dynamic_cast<NAMESPACE_VERSION::LCHierarchyRenderer*>(sceneEditorW->getCurrentRenderer());
    if (lchRenderer)
    {
        lchRenderer->getState()->getTraversalState()->PARAM_drawCloudSelection = false;
    }
    for ( unsigned int n = 0; n < scene->getNumGLMaterials(); n++ ){
        GLShaderMaterial* mat = dynamic_cast<GLShaderMaterial*>( scene->getGLMaterial( n ) );
        if ( mat != NULL ){
            mat->setForceTriangleNormal(true);
        }
    }
}
