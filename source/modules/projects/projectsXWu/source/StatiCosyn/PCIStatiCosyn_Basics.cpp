//---------------------------------------------------------------------------
#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "PCIStatiCosyn.h"
//---------------------------------------------------------------------------
#include "PropertyTableProperty.h"
#include "SeparatorClassProperty.h"
#include "SceneEditorWidget.h"
#include "LCHierarchyRenderer.h"
#include "GLShaderMaterial.h"
//---------------------------------------------------------------------------
using namespace X4;
//---------------------------------------------------------------------------
IMPLEMENT_CLASS( PCIStatiCosyn ,0)
{
    BEGIN_CLASS_INIT( PCIStatiCosyn );
    INIT_PROPERTY_TABLE();
    ADD_VECTOR3F_PROP(gendir_, 0);
    ADD_VECTOR3F_PROP(oDelta_, 0);
    ADD_FLOAT32_PROP( voxize_, 0 );
    ADD_VECTOR2I_PROP(margin_, 0);

    ADD_NOARGS_METHOD( VoxelizeScene );
    ADD_NOARGS_METHOD( Synthesis );
    ADD_NOARGS_METHOD( Reset );

    ADD_SEPARATOR_METHOD("Debug");
    ADD_NOARGS_METHOD( DrawVoxelsZ );
}

PCIStatiCosyn::~PCIStatiCosyn()
{
}

void PCIStatiCosyn::connectToSceneImpl(Scene * scene,OnSceneChangeCallback *onChange, QWidget *sceneEditorWidget, QWidget* toolWidget)
{
    SceneEditorWidget* sceneEditorW = dynamic_cast<NAMESPACE_VERSION::SceneEditorWidget*>(sceneEditorWidget);
    sceneEditorW->on_action_Auto_Layout_triggered(true);
    //scene->getRootState()->staticState->backgroundColor = makeVector4f(1.0f, 1.0f, 1.0f, 1.0f); // white
    //scene->getRootState()->staticState->backgroundColor = makeVector4f(0.6f, 0.6f, 0.9f, 1.0f); // bright
    //scene->getRootState()->staticState->rfShowLocalCoords = false;
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

void PCIStatiCosyn::keyDown(GeneralKey key) {
	if ((key.getKey() == ::keyRight) & (!key.getModifiers().getShift())) {
		debugOutput<<"expandX"<<"\n";
	}
	else if ((key.getKey() == ::keyLeft) & (!key.getModifiers().getShift()))
	{
		debugOutput<<"shrinkX"<<"\n";
	}
	else if (key.getKey() == ::keyUp)
	{
		debugOutput<<"expandY"<<"\n";
	}
	else if (key.getKey() == ::keyDown)
	{
		debugOutput<<"shrinkY"<<"\n";
	}
	else if ((key.getKey() == ::keyRight) & (key.getModifiers().getShift()))
	{
		debugOutput<<"expandZ"<<"\n";
	}
	else if ((key.getKey() == :: keyLeft) & (key.getModifiers().getShift()))
	{
		debugOutput<<"shrinkZ"<<"\n";
	}
	else if (key.getkeyAlphaNumKey() == 'G')
	{
		debugOutput<<"showSynGrid"<<"\n";
	}
	else if(key.getkeyAlphaNumKey() == 'S'){

	}
	else if(key.getkeyAlphaNumKey() == 'M'){
	}
	else if(key.getkeyAlphaNumKey() == 'D'){
	}
	else if (key.getkeyAlphaNumKey() == '0'){
	}
	else keyDownPassToCam(key);
}

void PCIStatiCosyn::mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState){
	mouseDownPassToCam(x, y, buttonsState, modifiersState);
}

void PCIStatiCosyn::mouseMoved(int32 x, int32 y){
	mouseMovedPassToCam(x, y);
}

void PCIStatiCosyn::mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState){
	mouseUpPassToCam(x, y, buttonsState, modifiersState);
}

void PCIStatiCosyn::mouseWheelRotated(int32 rotatedDelta, Modifiers modifiersState){
	mouseWheelRotatedPassToCam(rotatedDelta, modifiersState);
}
