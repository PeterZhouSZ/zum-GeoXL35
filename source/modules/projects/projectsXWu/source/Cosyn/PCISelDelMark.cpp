#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "PCISelDelMark.h"

//---------------------------------------------------------------------------
#include "CopyObjectProperties.h"
#include "Timer.h"
#include "ProgressWindow.h"
#include "PropertyTableProperty.h"
#include "SeparatorClassProperty.h"

#include "SceneEditorWidget.h"
#include "SceneGraphTools.h"
#include "SGListNode.h"
#include "SGRelativeTimeAnimationNode.h"
#include "MHSelectionIteratorAdaptor.h"
#include "LCHierarchyRenderer.h"
#include "GLShaderMaterial.h"
#include "GeometricRange.h"
#include "SelectionTools.h"

//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

IMPLEMENT_CLASS(PCISelDelMark, 0)
{
    BEGIN_CLASS_INIT(PCISelDelMark);
    INIT_PROPERTY_TABLE();
    ADD_FLOAT32_PROP(boundary_ratio, 0);

    ADD_NOARGS_METHOD(MarkSelAsBoundary);
}

PCISelDelMark::PCISelDelMark(void)
{
    data_folder = "data";
    example_name = "example";
    boundary_ratio = 5.f;

    leftMBdown = false;
    rightMBdown = false;

    leftMBdownX = 0;
    leftMBdownY = 0;
    leftMBmoveY = 0;
    leftMBmoveX = 0;

    w_ = 100;
    h_ = 100;
}

PCISelDelMark::~PCISelDelMark(void)
{
}

void PCISelDelMark::assign(const Object* obj, COPY_CONTEXT *context)
{
    PCInteractionTool::assign(obj, context);
    const PCISelDelMark * o = dynamic_cast<const PCISelDelMark*>(obj);
    pAssert(o != nullptr);

    copyObjectProperties(obj, this);
}

void PCISelDelMark::connectToSceneImpl(Scene * scene, OnSceneChangeCallback *onChange, QWidget *sceneEditorWidget, QWidget* toolWidget)
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

//----------------------------------------------------------------------
void PCISelDelMark::MarkSelAsBoundary(void)
{
    debugOutput << "marking selected points as boundary ... \n";
    UICPC* examplePC = dynamic_cast<UICPC*>(
        getPointCloud(getScene(), "root/" + example_name));
    if (nullptr == examplePC) {
        warning("PCISelDelMark::keyDown() - no example point cloud.");
        return;
    }
    if (nullptr == examplePC) return;
    if (!examplePC->providesAttribute("boundary")) {
        checkAttribute(examplePC, "boundary", 1, VAD::DATA_FORMAT_INT32);
    }
    AAT flgAAT = examplePC->getAAT("flags");
    AAT bonAAT = examplePC->getAAT("boundary");
    PointSet& examplePS = *examplePC->getPointSet();
    const size_t& numEntries = examplePS.getNumEntries();
    for (unsigned ii = 0; ii < numEntries; ++ii) {
        const int32 flg = examplePS.get1i(ii, flgAAT);
        const int32 bon = examplePS.get1i(ii, bonAAT);
        if (PF_FLAG_SELECTED == flg) {
            examplePS.set1i(ii, bonAAT, XF_BOUNDARY);
        }
        else {
            examplePS.set1i(ii, bonAAT, XF_NORMAL);
        }
    }
    debugOutput << "done.\n";
}

//----------------------------------------------------------------------
void PCISelDelMark::mouseMoved(
    int32 x, int32 y)
{
    leftMBmoveX = x;
    leftMBmoveY = y;
}

//----------------------------------------------------------------------
void PCISelDelMark::mouseDown(
    int32 x, int32 y,
    MouseButtons buttonsState, Modifiers modifiersState)
{
    if (buttonsState.getLeft()) leftMBdown = true;
    if (buttonsState.getRight()) rightMBdown = true;

    leftMBdownX = x;
    leftMBdownY = y;
    leftMBmoveX = x;
    leftMBmoveY = y;
}

//----------------------------------------------------------------------
void PCISelDelMark::mouseUp(
    int32 x, int32 y,
    MouseButtons buttonsState, Modifiers modifiersState)
{
    leftMBmoveX = x;
    leftMBmoveY = y;

    rangeSelect();

    leftMBdown = false;
    rightMBdown = false;
}

//----------------------------------------------------------------------
void PCISelDelMark::rangeSelect(void)
{
    UICPC* examplePC = dynamic_cast<UICPC*>(
        getPointCloud(getScene(), "root/" + example_name));
    if (nullptr == examplePC) {
        warning("PCISelDelMark::rangeSelect() - no example point cloud.");
        return;
    }
    if (nullptr == examplePC) return;
    AAT bonAAT;
    if (!examplePC->providesAttribute("boundary")) {
        checkAttribute(examplePC, "boundary", 1, VAD::DATA_FORMAT_INT32);
        bonAAT = examplePC->getAAT("boundary");
        PointSet& examplePS = *examplePC->getPointSet();
        const size_t& numEntries = examplePS.getNumEntries();
        for (unsigned ii = 0; ii < numEntries; ++ii) {
            examplePS.set1i(ii, bonAAT, XF_NORMAL);
        }
    }
    else {
        bonAAT = examplePC->getAAT("boundary");
    }
    const float boundaryWidth = getMedianPointDistance(examplePC) * abs(boundary_ratio);

    {
        const float ulX = std::min(leftMBdownX, leftMBmoveX) - boundaryWidth;
        const float ulY = std::min(leftMBdownY, leftMBmoveY) - boundaryWidth;
        const float brX = std::max(leftMBdownX, leftMBmoveX) + boundaryWidth;
        const float brY = std::max(leftMBdownY, leftMBmoveY) + boundaryWidth;
        GeometricRange* range = getRayPyramidalRangeFromMouseClick(
            ulX, ulY, brX, brY, w_, h_,
            &(getScene()->getRootState()->staticState->camera),
            &(getScene()->getRootState()->staticState->viewFrustum));
        if (nullptr == range) return;

        SceneObjectIterator *newItSO = examplePC->createIterator(SOIT(SOIT::CAP_BASIC_PC | SOIT::CAP_HIERARCHICAL));
        PointCloudIterator *newIt = dynamic_cast<PointCloudIterator*>(newItSO);
        if ((nullptr == newIt) || (nullptr == newItSO)) { delete newItSO; delete range; return; };
        range->setTransformation(examplePC->getTransformation());
        IteratorAdaptor *rangeIt = getGeoRangeIteratorAdaptor(examplePC, newIt, range);
        if (nullptr == rangeIt) { delete newItSO; delete range; return; }

        while (!rangeIt->atEnd())
        {
            rangeIt->set1i(bonAAT, XF_BOUNDARY);
            rangeIt->next();
        }

        delete rangeIt;
        delete newItSO;
        delete range;
    }

    {
        GeometricRange* range = getRayPyramidalRangeFromMouseClick(
            leftMBdownX, leftMBdownY, leftMBmoveX, leftMBmoveY, w_, h_,
            &(getScene()->getRootState()->staticState->camera),
            &(getScene()->getRootState()->staticState->viewFrustum));
        if (nullptr == range) return;

        SceneObjectIterator *newItSO = examplePC->createIterator(SOIT(SOIT::CAP_BASIC_PC | SOIT::CAP_HIERARCHICAL));
        PointCloudIterator *newIt = dynamic_cast<PointCloudIterator*>(newItSO);
        if ((nullptr == newIt) || (nullptr == newItSO)) { delete newItSO; delete range; return; };
        range->setTransformation(examplePC->getTransformation());
        IteratorAdaptor *rangeIt = getGeoRangeIteratorAdaptor(examplePC, newIt, range);
        if (nullptr == rangeIt) { delete newItSO; delete range; return; }

        while (!rangeIt->atEnd())
        {
            rangeIt->set1i(bonAAT, XF_REMOVE);
            rangeIt->next();
        }

        delete rangeIt;
        delete newItSO;
        delete range;
    }
}

void PCISelDelMark::keyDown(GeneralKey key)
{
    if (key.getkeyAlphaNumKey() == 'D')
    {
        debugOutput << "showSynGrid" << "\n";
    }
    else if (key.getkeyAlphaNumKey() == 'G')
    {
        debugRenderer->beginRenderJob_OneFrame("draw_boundary_", DR_FRAME++);
        UICPC* examplePC = dynamic_cast<UICPC*>(
            getPointCloud(getScene(), "root/" + example_name));
        if (nullptr == examplePC) {
            warning("PCISelDelMark::keyDown() - no example point cloud.");
            return;
        }
        if (nullptr == examplePC) return;
        AAT posAAT = examplePC->getAAT("position");
        AAT bonAAT = examplePC->getAAT("boundary");
        PointSet& examplePS = *examplePC->getPointSet();
        const size_t& numEntries = examplePS.getNumEntries();
        for (unsigned ii = 0; ii < numEntries; ++ii) {
            const Vector3f pos = examplePS.get3f(ii, posAAT);
            const int32 bon = examplePS.get1i(ii, bonAAT);
            if (bon == XF_BOUNDARY) {
                debugRenderer->addPoint(pos, makeVector3f(1, 1, 0));
            }
            else if (bon == XF_REMOVE) {
                debugRenderer->addPoint(pos, makeVector3f(1, 0, 0));
            }
        }
        debugRenderer->endRenderJob();
    }
    else keyDownPassToCam(key);
}

//----------------------------------------------------------------------
void PCISelDelMark::areaResize(
    card32 width, card32 height)
{
    w_ = width;
    h_ = height;
}

//----------------------------------------------------------------------
void PCISelDelMark::glDrawTool(GLContext *glContext)
{
    forget(glContext);

    if (false == leftMBdown && false == rightMBdown) return;

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glDisable(GL_LIGHTING);
    glLineWidth(1.0f);

    float32 posX = (leftMBmoveX - w_*0.5f) / (w_*0.5f);
    float32 posY = -((leftMBmoveY - h_*0.5f) / (h_*0.5f));

    float32 posX2 = (leftMBdownX - w_*0.5f) / (w_*0.5f);
    float32 posY2 = -((leftMBdownY - h_*0.5f) / (h_*0.5f));

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    if (leftMBdown) glColor4f(1.0f, 0.9f, 0.1f, 0.4f);
    else glColor4f(0.1f, 0.9f, 1.0f, 0.4f);
    {
        glBegin(GL_QUADS);
        glVertex2f(posX2, posY2);
        glVertex2f(posX2, posY);
        glVertex2f(posX, posY);
        glVertex2f(posX, posY2);
        glEnd();
    }
    if (leftMBdown) glColor4f(1.0f, 0.9f, 0.0f, 0.9f);
    else glColor4f(0.0f, 0.9f, 1.0f, 0.9f);
    {
        glBegin(GL_LINE_LOOP);
        glVertex2f(posX2, posY2);
        glVertex2f(posX2, posY);
        glVertex2f(posX, posY);
        glVertex2f(posX, posY2);
        glEnd();
    }

    glPopAttrib();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}
