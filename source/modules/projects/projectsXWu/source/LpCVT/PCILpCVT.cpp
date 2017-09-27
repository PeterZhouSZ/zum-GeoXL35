#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "LpCVT/PCILpCVT.h"
//---------------------------------------------------------------------------
#include "SceneEditorWidget.h"
#include "LCHierarchyRenderer.h"
#include "GLShaderMaterial.h"

namespace {
    int DR_FRAME = 0;
}

//---------------------------------------------------------------------------
IMPLEMENT_CLASS( PCILpCVT, 0 )
{
    BEGIN_CLASS_INIT( PCILpCVT );
}
//---------------------------------------------------------------------------


PCILpCVT::PCILpCVT(void)
{
}

PCILpCVT::~PCILpCVT(void)
{
}

void PCILpCVT::mouseMoved(
    int32 x, int32 y)
{
    mouseMovedPassToCam(x, y);
}

//----------------------------------------------------------------------
void PCILpCVT::mouseDown(
    int32 x, int32 y,
    MouseButtons buttonsState, Modifiers modifiersState)
{
    mouseDownPassToCam(x, y, buttonsState, modifiersState);
}

//----------------------------------------------------------------------
void PCILpCVT::mouseUp(
    int32 x, int32 y,
    MouseButtons buttonsState, Modifiers modifiersState)
{
    mouseUpPassToCam(x, y, buttonsState, modifiersState);
}

//----------------------------------------------------------------------
void PCILpCVT::areaResize(
    card32 width, card32 height)
{
    areaResizePassToCam(width, height);
}

//----------------------------------------------------------------------
void PCILpCVT::glDrawTool(GLContext *glContext)
{
}

//----------------------------------------------------------------------
void PCILpCVT::mouseWheelRotated(
    int32 rotatedDelta,
    Modifiers modifiersState)
{
    mouseWheelRotatedPassToCam(rotatedDelta, modifiersState);
}

//----------------------------------------------------------------------
void PCILpCVT::keyDown(GeneralKey key)
{
    keyDownPassToCam(key);
}

//----------------------------------------------------------------------
void PCILpCVT::keyUp(GeneralKey key)
{
    keyUpPassToCam(key);
}

//----------------------------------------------------------------------
void PCILpCVT::connectToSceneImpl(Scene * scene,OnSceneChangeCallback *onChange, QWidget *sceneEditorWidget, QWidget* toolWidget)
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
