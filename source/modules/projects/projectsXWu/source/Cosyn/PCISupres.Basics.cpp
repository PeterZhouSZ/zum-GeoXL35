#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "PCISupres.h"
#include "TransformationValidator.h"

//---------------------------------------------------------------------------
#include "CopyObjectProperties.h"
#include "Timer.h"
#include "ProgressWindow.h"
#include "PropertyTableProperty.h"
#include "SeparatorClassProperty.h"
#include "ObjectClassProperty.h"

#include "SceneEditorWidget.h"
#include "SceneGraphTools.h"
#include "SGListNode.h"
#include "SGRelativeTimeAnimationNode.h"
#include "MHSelectionIteratorAdaptor.h"
#include "LCHierarchyRenderer.h"
#include "GLShaderMaterial.h"

//---------------------------------------------------------------------------

IMPLEMENT_CLASS( PCISupres ,0)
{
    BEGIN_CLASS_INIT(PCISupres);
    INIT_PROPERTY_TABLE();
    ADD_OBJECT_PROP(projectSettings_, 0, ProjectSettings::getClass(), true)
    ADD_FLOAT32_PROP(cell_size, 0);
    ADD_CARD64_PROP(cell_subdiv, 0);
    ADD_FLOAT32_PROP(match_ratio, 0);
    ADD_FLOAT32_PROP(outlier_ratio, 0);
    ADD_FLOAT32_PROP(inlier_ratio, 0);
    ADD_CARD32_PROP(num_icp, 0);
    //ADD_FLOAT32_PROP(cost_keep_empty, 0);
    ADD_FLOAT32_PROP(mean_error_th, 0);

    ADD_NOARGS_METHOD(InitRepBox);
    //ADD_NOARGS_METHOD(PreProc);
    //ADD_NOARGS_METHOD(GenerateParchTrans);
    ADD_NOARGS_METHOD(ValidateTransformations);
    //ADD_NOARGS_METHOD(TestPlaneGrid);
    ADD_NOARGS_METHOD(CompleteScene);
    ADD_NOARGS_METHOD(CompleteSceneSymmetry);
    ADD_NOARGS_METHOD(ExtractGroup);

    ADD_SEPARATOR_METHOD("Debug");
}

PCISupres::PCISupres(void)
{
    repBoxLoader = boost::shared_ptr<RepBoxLoader>(new RepBoxLoader);

    data_folder = "data";
    example_name = "example";
    guide_name = "guide";
    box_cloud = "boxCloud";
    num_infer_trans = 10;
    cell_size = 1.29f;
    cell_subdiv = 10;
    verbose = 0;

    projectSettings_ = new ProjectSettings();
    match_ratio = 10;
    outlier_ratio = 4.f;
    inlier_ratio = .7f;
    num_icp = 20;
    cost_keep_empty = 0.f;
    mean_error_th = 0.6f;
}

PCISupres::~PCISupres(void)
{
    delete projectSettings_;
    projectSettings_ = nullptr;
}

void PCISupres::assign(const Object* obj, COPY_CONTEXT *context)
{
    PCInteractionTool::assign(obj,context);
    const PCISupres * o = dynamic_cast<const PCISupres*>(obj);
    pAssert( o != nullptr );

    copyObjectProperties(obj,this);
}

void PCISupres::connectToSceneImpl(Scene * scene, OnSceneChangeCallback *onChange, QWidget *sceneEditorWidget, QWidget* toolWidget)
{
    SceneEditorWidget* sceneEditorW = dynamic_cast<NAMESPACE_VERSION::SceneEditorWidget*>(sceneEditorWidget);
    sceneEditorW->on_action_Auto_Layout_triggered(true);
    //scene->getRootState()->staticState->backgroundColor = makeVector4f(.95f, .95f, .95f, 1.0f); // white
    scene->getRootState()->staticState->backgroundColor = makeVector4f(0.6f, 0.6f, 0.9f, 1.0f); // bright
    scene->getRootState()->staticState->rfShowLocalCoords = false;
    LCHierarchyRenderer* lchRenderer = dynamic_cast<NAMESPACE_VERSION::LCHierarchyRenderer*>(sceneEditorW->getCurrentRenderer());
    if (lchRenderer)
    {
        lchRenderer->getState()->getTraversalState()->PARAM_drawCloudSelection = false;
    }
    for (unsigned int n = 0; n < scene->getNumGLMaterials(); n++){
        GLShaderMaterial* mat = dynamic_cast<GLShaderMaterial*>(scene->getGLMaterial(n));
        if (mat != NULL){
            mat->setForceTriangleNormal(true);
        }
    }
}
