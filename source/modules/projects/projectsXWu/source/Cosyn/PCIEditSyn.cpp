#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "PCIEditSyn.h"
#include "Util/SceneAndPointCloudTools.h"
#include "Util\ColorSchemer.hpp"
#include "Util\ExtraGL.hpp"

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

//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

namespace {
    struct Kernal5x5 {
        typedef boost::multi_array<float, 3> array_type;
        array_type ker;
        Kernal5x5(void) {
            ker.resize(boost::extents[5][5][5]);
            ker.reindex(-2);
            fillGauss();
            //fillUnity();
        }
        void fillGauss(void)
        {
            for (int ii = -2; ii <= 2; ++ii) {
                for (int jj = -2; jj <= 2; ++jj) {
                    for (int kk = -2; kk <= 2; ++kk) {
                        if (2 == abs(ii) || 2 == abs(jj) || 2 == abs(kk)) {
                            ker[ii][jj][kk] = 0.1f;
                            //ker[ii][jj][kk] = 0.0f;
                        }
                        else {
                            ker[ii][jj][kk] = 0.4f;
                            //ker[ii][jj][kk] = 0.0f;
                        }
                    }
                }
            }
            ker[0][0][0] = 1.f;
        }
        void fillUnity(void)
        {
            std::fill(ker.data(), ker.data() + ker.num_elements(), 0.f);
            ker[0][0][0] = 1.f;
        }
    };
}

IMPLEMENT_CLASS(PCIEditSyn, 0)
{
    BEGIN_CLASS_INIT(PCIEditSyn);
    INIT_PROPERTY_TABLE();
    ADD_FLOAT32_PROP(cellSize, 0);
    ADD_BOOLEAN_PROP(showGizmo, 0);
    ADD_BOOLEAN_PROP(showProb, 0);
    ADD_INT32_PROP(numIter, 0);
    ADD_NOARGS_METHOD(ProliferatePattern);
    ADD_SEPARATOR_METHOD("Debug");
}

void PCIEditSyn::ProliferatePattern(void)
{
    initProbs();
    for (int ii = 0; ii < numIter; ++ii) {
        proliferateOnce();
        initProbs();
    }
}

void PCIEditSyn::proliferateOnce(void)
{
    const float cutTh = 0.6f;
    Kernal5x5 ker5;
    typedef std::array<int, 3> key_type;
    const DenseGrid3f::array_type dataGrid(gridExample->array_);
    DenseGrid3f::shape_type shape = gridExample->shape_;
    DenseGrid3f::bases_type bases = gridExample->bases_;

    UICPC* examplePC = RetrieveUICPC(example_name);
    const AAT posAAT = examplePC->getAAT("position");
    //const DenseGrid3f::key_type key_test = gridExample->GetCellIndex(
    //    examplePC->getPointSet()->get3f(0, posAAT)
    //    );

    // enumerate through 3-dims
    for (int e0 = bases[0]; e0 < bases[0] + shape[0]; ++e0) {
        for (int e1 = bases[1]; e1 < bases[1] + shape[1]; ++e1) {
            for (int e2 = bases[2]; e2 < bases[2] + shape[2]; ++e2) {
                const float val = dataGrid[e0][e1][e2];
                if (cutTh > val) continue;
                const DenseGrid3f::key_type key = { e0, e1, e2 };

                // impose every pattern
                const std::deque< std::deque< key_type > >& offVec =
                    knnPaternGentor->offVec_;
                for (const std::deque< key_type >& off : offVec) {
                    for (const key_type& arr : off) {
                        const DenseGrid3f::key_type key_n = {
                            key[0] + arr[0],
                            key[1] + arr[1],
                            key[2] + arr[2]
                        };
                        if (!gridExample->InRange(key_n)) continue;
                        //if (key_test == key_n)
                        //    debugOutput << gridExample->array_
                        //    [key_n[0]][key_n[1]][key_n[2]] << "\t";
                        // convolution
                        for (int ii = -2; ii <= 2; ++ii) {
                            for (int jj = -2; jj <= 2; ++jj) {
                                for (int kk = -2; kk <= 2; ++kk) {
                                    gridExample->array_
                                        [key_n[0] + ii][key_n[1] + jj][key_n[2] + kk] +=
                                        (ker5.ker[ii][jj][kk] * val);
                                }
                            }
                        }
                        //if (key_test == key_n)
                        //debugOutput << gridExample->array_
                        //    [key_n[0]][key_n[1]][key_n[2]] << "\n";
                    }
                }
            }
        }
    }
    gridExample->Rescale();
}

bool PCIEditSyn::Initialize(void)
{
    if (initialized_) return false;

    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "# initializing from pre-detected repetition boxes \n";
    debugOutput << "########################################\n";
    debugOutput << "\n";

    repBoxLoader = boost::shared_ptr<RepBoxLoader>(new RepBoxLoader);
    repBoxLoader->LoadGT();

    UICPC* guidePC = new UnstructuredInCorePointCloud;
    guidePC->clearAndSetup(0, true, false);
    checkAttribute(guidePC, "group", 1, VAD::DATA_FORMAT_INT32);
    guidePC->setMaterialIndex(4);

    PointSet& guidePS = *guidePC->getPointSet();
    const AAT& posAAT = guidePS.getAAT("position");
    const AAT& grpAAT = guidePS.getAAT("group");
    const AAT& clrAAT = guidePS.getAAT("color");

    guideBoxes.clear();
    unsigned groupID = 0;
    for (const std::deque<RepBox::Ptr>& boxes : repBoxLoader->boxRecord) {
        for (const RepBox::Ptr& box : boxes) {
            const size_t& numEntries = guidePS.getNumEntries();
            guidePS.changeHeight(numEntries + 1);
            guidePS.set3f(numEntries, posAAT, box->centroid);
            guidePS.set1i(numEntries, grpAAT, groupID);
            guidePS.set3f(numEntries, clrAAT, repBoxLoader->MapGroupColor(groupID));
            guideBoxes.push_back(box);
        }
        ++groupID;
    }

    addOrReplacePointCloud(scene, guide_name, guidePC);

    initProbGrid();

    StateChangePrint();

    knnPaternGentor = boost::make_shared<KNNPaternGentor>(5);
    knnPaternGentor->Generate(guidePC->getPointSet());
    knnPaternGentor->Discretize(guidePC->getPointSet(), cellSize);
    //knnPaternGentor->ShowPattern(guidePC->getPointSet()); // debug draw

    initialized_ = true;

    return true;
}

void PCIEditSyn::initProbGrid(void)
{
    const int padding = 2;

    UICPC* examplePC = RetrieveUICPC(example_name);
    const BoundingBox3f bbExample = examplePC->getPointSet()->getBoundingBox();
    gridExample = boost::make_shared<DenseGrid3f>(
        bbExample, cellSize, padding
        );

    // debug draw
    {
        //debugRenderer->beginRenderJob_OneFrame("show_grid_", DR_FRAME++);
        //gridExample->DrawWithDR(ColorSchemer::GetJetColor(0.6f));
        //debugRenderer->endRenderJob();

        debugRenderer->beginRenderJob_OneFrame("show_first_grid_", DR_FRAME++);
        const BoundingBox3f& bb = gridExample->GetCellBBox(gridExample->bases_);
        debugRenderer->addBoundingBox(bb, ColorSchemer::GetJetColor(0.6f), 1);
    }
}

void PCIEditSyn::initProbs(void)
{
    Kernal5x5 ker5;

    DenseGrid3f::array_type& dataGrid = gridExample->array_;
    DenseGrid3f::array_type::index_gen indices;
    for (RepBox::Ptr box : guideBoxes){
        const Vector3f cen = box->centroid;
        if (!gridExample->InRange(cen)) {
            error("PCIEditSyn::initProbGrid - guide box not in range");
            return;
        }
        DenseGrid3f::key_type key = gridExample->GetCellIndex(cen);
        //DenseGrid3f::array_type::array_view<3>::type sub = dataGrid[indices
        //    [DenseGrid3f::i_range(key[0] - padding, key[0] + padding)]
        //[DenseGrid3f::i_range(key[1] - padding, key[1] + padding)]
        //[DenseGrid3f::i_range(key[2] - padding, key[2] + padding)]
        //];

        /**/
        dataGrid[key[0]][key[1]][key[2]] += 1.f;
        /**
        // convolution
        for (int ii = -2; ii <= 2; ++ii) {
        for (int jj = -2; jj <= 2; ++jj) {
        for (int kk = -2; kk <= 2; ++kk) {
        dataGrid[key[0] + ii][key[1] + jj][key[2] + kk] +=
        ker5.ker[ii][jj][kk];
        }
        }
        }
        //**/
    }

    addUserHypos();

    gridExample->Rescale();
    //gridExample->CutOff(0.6f);
}

void PCIEditSyn::addUserHypos(void)
{
    UICPC* guidePC = RetrieveUICPC(guide_name);
    const PointSet& guidePS = *guidePC->getPointSet();
    const AAT& posAAT = guidePS.getAAT("position");
    const std::deque<unsigned>& points = handle_->getPoints();
    if (points.empty()) return;
    const Vector3f trans = handle_->getTranslation();
    //debugOutput << trans << "\n";
    Kernal5x5 ker5;
    DenseGrid3f::array_type& dataGrid = gridExample->array_;
    for (unsigned pid : points) {
        const Vector3f cen = guidePS.get3f(pid, posAAT);
        const Vector3f pos = cen + trans;
        if (!gridExample->InRange(pos)) {
            error("PCIEditSyn::addUserHypos - user hint not in range");
            return;
        }
        DenseGrid3f::key_type key = gridExample->GetCellIndex(pos);
        // convolution
        for (int ii = -2; ii <= 2; ++ii) {
            for (int jj = -2; jj <= 2; ++jj) {
                for (int kk = -2; kk <= 2; ++kk) {
                    dataGrid[key[0] + ii][key[1] + jj][key[2] + kk] +=
                        ker5.ker[ii][jj][kk];
                }
            }
        }
    }
    //gridExample->Rescale();
    //gridExample->CutOff(0.6f);
}

void PCIEditSyn::StateChangePrint(void)
{
    debugOutput << "\n";
    debugOutput << "########################################\n";
    std::string modeString;
    switch (uiMode_)
    {
    case EditSynGizmoHandler::CONTROL: modeString = "CONTROL";
        if (Handle3DGizmo::MOVE == controlMode_) modeString += ": MOVE";
        else if (Handle3DGizmo::ROTATE == controlMode_) modeString += ": ROTATE";
        break;
    case EditSynGizmoHandler::SELECT: modeString = "SELECT"; break;
    }
    debugOutput << "UI mode changed to " << modeString << "\n";
    debugOutput << "\n";

    for (GizmoHandlerMap::iterator it = gizmoHandlerMap_.begin(); it != gizmoHandlerMap_.end(); ++it) {
        if (uiMode_ == it->first) it->second->activate();
        else it->second->hibernate();
    }
}

PCIEditSyn::PCIEditSyn(void)
{
    data_folder = "root/data";
    example_name = "root/example";
    guide_name = "root/guide";
    repBoxLoader = nullptr;
    knnPaternGentor = nullptr;

    std::deque<unsigned> tempQ;
    handle_ = boost::shared_ptr<EditSynHandle>
        (new EditSynHandle(tempQ, 100, 100));
    showGizmo = true;
    initialized_ = false;

    wireWidth_ = 4.f;

    uiMode_ = EditSynGizmoHandler::SELECT;
    controlMode_ = Handle3DGizmo::INACTIVE;
    gizmoHandlerMap_[EditSynGizmoHandler::CONTROL] = boost::make_shared<EditSynGizmoHandlerControl>(this);
    gizmoHandlerMap_[EditSynGizmoHandler::SELECT] = boost::make_shared<EditSynGizmoHandlerSelect>(this);

    gridExample = nullptr;
    showProb = true;
    cellSize = 0.3f;
    numIter = 20;
}

PCIEditSyn::~PCIEditSyn(void)
{
}

UICPC* PCIEditSyn::RetrieveUICPC(
    std::string const& node, UICPC* reference)
{
    UICPC* uicpc = dynamic_cast<UICPC*>(getPointCloud(scene, node));

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if (!uicpc)
    {
        if (!reference) { return 0; }

        uicpc = dynamic_cast<UICPC*>(reference->copy());

        addPointCloud(scene, uicpc, node);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if (uicpc && uicpc->getAAT("position") == NULL_AAT)
    {
        debugOutput << "PCIEditSyn::RetrieveUICPC() - "
            "\"position\" attribute missing.\n";
        uicpc = 0;
    }

    //// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //if (uicpc && uicpc->getAAT("normal"  ) == NULL_AAT)
    //{
    //    debugOutput << "PCIEditSyn::RetrieveUICPC() - "
    //        "\"normal\" attribute missing.\n";
    //    uicpc = 0;
    //}

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    return uicpc;
}

void PCIEditSyn::assign(const Object* obj, COPY_CONTEXT *context)
{
    PCInteractionTool::assign(obj, context);
    const PCIEditSyn * o = dynamic_cast<const PCIEditSyn*>(obj);
    pAssert(o != nullptr);

    copyObjectProperties(obj, this);
}

void PCIEditSyn::connectToSceneImpl(Scene * scene, OnSceneChangeCallback *onChange, QWidget *sceneEditorWidget, QWidget* toolWidget)
{
    SceneEditorWidget* sceneEditorW = dynamic_cast<NAMESPACE_VERSION::SceneEditorWidget*>(sceneEditorWidget);
    sceneEditorW->on_action_Auto_Layout_triggered(true);
    //scene->getRootState()->staticState->backgroundColor = makeVector4f(.95f, .95f, .95f, 1.0f); // white
    scene->getRootState()->staticState->backgroundColor = makeVector4f(.05f, .05f, .25f, 1.0f); // dark
    //scene->getRootState()->staticState->backgroundColor = makeVector4f(0.6f, 0.6f, 0.9f, 1.0f); // bright
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
void PCIEditSyn::mouseMoved(
    int32 x, int32 y)
{
    if (0 == gizmoHandlerMap_[uiMode_]->mouseMoved(x, y)) return;
    //mouseMovedPassToCam(x, y);
}

//----------------------------------------------------------------------
void PCIEditSyn::mouseDown(
    int32 x, int32 y,
    MouseButtons buttonsState, Modifiers modifiersState)
{
    Initialize();

    if (0 == gizmoHandlerMap_[uiMode_]->mouseDown(x, y, buttonsState, modifiersState)) return;
    //mouseDownPassToCam(x, y, buttonsState, modifiersState);
}

//----------------------------------------------------------------------
void PCIEditSyn::mouseUp(
    int32 x, int32 y,
    MouseButtons buttonsState, Modifiers modifiersState)
{
    if (0 == gizmoHandlerMap_[uiMode_]->mouseUp(x, y, buttonsState, modifiersState)) return;
    //mouseUpPassToCam(x, y, buttonsState, modifiersState);
}

//----------------------------------------------------------------------
void PCIEditSyn::areaResize(
    card32 width, card32 height)
{
    for (GizmoHandlerMap::iterator it = gizmoHandlerMap_.begin(); it != gizmoHandlerMap_.end(); ++it) {
        it->second->areaResize(width, height);
    }
    areaResizePassToCam(width, height);
}

//----------------------------------------------------------------------
void PCIEditSyn::glDrawTool(GLContext *glContext)
{
    if (showGizmo)
    {
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

    // draw reference boxes
    //const Vector3f color = ColorSchemer::GetJetColor(0.25);
    const Vector3f color = StandardColors<40>::color_grey;
    for (RepBox::Ptr box : guideBoxes) {
        RepBoxLoader::glDrawBox(box, color, wireWidth_);
    }

    // draw probability grid
    if (showProb && initialized_) {
        glDisable(GL_BLEND);
        DenseGrid3f::array_type& dataGrid = gridExample->array_;
        //for (auto ii = dataGrid.data();
        //    ii < (dataGrid.data() + dataGrid.num_elements()); ++ii) {
        //    if (0.08 > *ii) continue;
        //}
        DenseGrid3f::shape_type shape = gridExample->shape_;
        DenseGrid3f::bases_type bases = gridExample->bases_;
        for (int e0 = bases[0]; e0 < bases[0] + shape[0]; ++e0) {
            for (int e1 = bases[1]; e1 < bases[1] + shape[1]; ++e1) {
                for (int e2 = bases[2]; e2 < bases[2] + shape[2]; ++e2) {
                    const float val = dataGrid[e0][e1][e2];
                    if (0.08 > val) continue;
                    const DenseGrid3f::key_type key = { e0, e1, e2 };
                    const Vector3f pos = gridExample->GetCellCenter(key);
                    const float radius = val * cellSize * 0.5;
                    const Vector3f color = ColorSchemer::GetJetColor(val);
                    SolidSphere sphere(radius);
                    glColor3fv(color.data());
                    sphere.draw(pos[0], pos[1], pos[2]);
                }
            }
        }
    }
}

//----------------------------------------------------------------------
void PCIEditSyn::mouseWheelRotated(
    int32 rotatedDelta,
    Modifiers modifiersState)
{
    mouseWheelRotatedPassToCam(rotatedDelta, modifiersState);
}

//----------------------------------------------------------------------
void PCIEditSyn::keyDown(GeneralKey key)
{
    Initialize();

    const char& keyAlphaNum = key.getkeyAlphaNumKey();
    switch (keyAlphaNum)
    {
    case 'q':
    case 'Q':
        uiMode_ = EditSynGizmoHandler::SELECT;
        controlMode_ = Handle3DGizmo::INACTIVE;
        break;
    case 'w':
    case 'W':
        uiMode_ = EditSynGizmoHandler::CONTROL;
        controlMode_ = Handle3DGizmo::MOVE;
        break;
    case 'e':
    case 'E':
        uiMode_ = EditSynGizmoHandler::CONTROL;
        controlMode_ = Handle3DGizmo::ROTATE;
        break;
    case 'd':
    case 'D':
        showGizmo = !showGizmo;
        return;
    default:
        keyDownPassToCam(key);
        return;
    }

    StateChangePrint();
}

//----------------------------------------------------------------------
void PCIEditSyn::keyUp(GeneralKey key)
{
    keyUpPassToCam(key);
}
