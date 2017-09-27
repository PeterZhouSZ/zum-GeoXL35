//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "PCIConstrainedICP.h"
//----------------------------------------------------------------------
#include "GlobalMacros.h" 
#include "ClassMethods.h"
//----------------------------------------------------------------------
#include "SeparatorClassProperty.h"
//----------------------------------------------------------------------
#include "ARSettings.h"
#include "ARConstraints.h"
#include "ICPConstraints.h"
//----------------------------------------------------------------------
#include "SceneEditorWidget.h"
//----------------------------------------------------------------------
#include "CICPHandle.h"
#include "CICPPersistentHandles.h"
//----------------------------------------------------------------------
#include "BinaryObjectStreams.h"
#include "FileDialogs.h"
//----------------------------------------------------------------------

namespace X4
{
//======================================================================
// Constructors
//----------------------------------------------------------------------
PCIConstrainedICP::PCIConstrainedICP()
  : input                 ("root/input"      )
  , result                ("root/result"     )
  , secondary             ("root/secondary"  )
  , icpTarget             ("root/target"     )
  , coplanarityMesh       ("root/coplanarity")
  , symmetryBaseName      ("pc1"             )
//, settings              (new ARSettings()  )
  , smoothed              (false             )
  , useICP                (true              )
  , useHandle             (false             )
  , useSymmetry           (false             )
  , useFeatureLine        (false             )
  , useCoplanarity        (false             )
  ,      handleWeight     (1000.0f           )
  ,    symmetryWeight     (   3.0f           )
  , coplanarityWeight     (   3.0f           )
  , featureLineWeight     (   3.0f           )
  , angleThreshold        (1.0f              )
  ,  sizeThreshold        (0.01f             )
  , iterations            (1                 )
  , buildConstraints      (true              )
  , optimize              (true              )
  , useYZSymmetry         (false             )
  , useRTSymmetry         (false             )
  , rotation              (false             )
//, reverse               (true              )
  , tsGridSpacing         (0.1f              )
  , tsIdentityWeight      (0.0f              )
  , tsRegularizerWeight   (1.0f              )
  , cgNumIterations       (10000             )
  , cgConvergenceThreshold(0.00001f          )
  , cgDebugOutput         (false             )
//, reweightSymmetry      (false             )
  , icpWeight             (1.0f              )
  , icpFOutlierPercentile (0.2f              )
  , icpROutlierPercentile (0.2f              )
  , icpFOutlierThreshold  (1.0f              )
  , icpROutlierThreshold  (1.0f              )
  , icpBuildForward       (true              )
  , icpBuildReverse       (false             )
  , icpNormalThreshold    (0.7f              )
  , icpCullNormalDirect   (false             )
  , icpCullNormalTarget   (false             )
  , icpNearFieldThreshold (0.05f             )
  , icpUseL1Norm          (false             )
  , icpL1Threshold        (0.05f             )
  , icpL1Epsilon          (0.05f             )
  , icpL1Power            (1                 )
  , samplingModifier      (0.25f             )
  , handleChanged_        (false             )
  , inputPC_              (0                 )
  , mesh_                 (0                 )
  , isMesh_               (false             )
  , symmetryInitialized_  (false             )
  , hasOldSolver_         (false             )
  , iteration_            (1                 )
{
//  settings->gridSize             = 2;
//  settings->correspondenceParam  = 1;
//  settings->rigidityParam        = 1;
//  settings->icpPercentileCulling = false;
//  settings->icpConstraintsMode   = ICP_CONSTRAINT_POINT_TO_POINT;
//  settings->correspondenceParam  = 1;
//  settings->tsCachedCellMatrices = false;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  handler_.setHandles(&handles_);
  handler_.addChangesListener(this);
}

//======================================================================
// Destructor
//----------------------------------------------------------------------
PCIConstrainedICP::~PCIConstrainedICP()
{
//  delete settings;
}

//======================================================================
// Member functions
//----------------------------------------------------------------------
IMPLEMENT_X4_CLASS_DESCR(PCIConstrainedICP, 7, "Constrained ICP")
{
	BEGIN_CLASS_INIT(PCIConstrainedICP)
	
  ADD_BOOLEAN_PROP(buildConstraints, 6);
  ADD_BOOLEAN_PROP(optimize,         6);
  ADD_BOOLEAN_PROP(smoothed,         6);
  
  ADD_CARD32_PROP (iterations,       6);

  ADD_FLOAT32_PROP(samplingModifier, 7);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  ADD_SEPARATOR("Nodes");
  ADD_STRING_PROP(input,     6);
  ADD_STRING_PROP(result,    6);
  ADD_STRING_PROP(secondary, 6);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  ADD_SEPARATOR("Components");
  ADD_BOOLEAN_PROP(useICP,         6);
  ADD_BOOLEAN_PROP(useSymmetry,    6);
  ADD_BOOLEAN_PROP(useFeatureLine, 6);
  ADD_BOOLEAN_PROP(useCoplanarity, 6);
  ADD_BOOLEAN_PROP(useHandle,      6);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  ADD_SEPARATOR("ICP settings");
  ADD_STRING_PROP(icpTarget,              6);
  ADD_BOOLEAN_PROP(icpBuildForward,       6);
  ADD_BOOLEAN_PROP(icpBuildReverse,       6);
  ADD_FLOAT32_PROP(icpWeight,             6);
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  ADD_SEPARATOR("ICP percentile culling");
  ADD_FLOAT32_PROP(icpFOutlierPercentile, 6);
  ADD_FLOAT32_PROP(icpROutlierPercentile, 6);
  ADD_FLOAT32_PROP(icpFOutlierThreshold,  6);
  ADD_FLOAT32_PROP(icpROutlierThreshold,  6);
  ADD_FLOAT32_PROP(icpNearFieldThreshold, 6);  
  ADD_BOOLEAN_PROP(icpCullNormalDirect,   6);
  ADD_BOOLEAN_PROP(icpCullNormalTarget,   6);
  ADD_FLOAT32_PROP(icpNormalThreshold,    6);
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  ADD_SEPARATOR("ICP L1 norm");
  ADD_BOOLEAN_PROP(icpUseL1Norm,   7);
  ADD_FLOAT32_PROP(icpL1Threshold, 7);
  ADD_FLOAT32_PROP(icpL1Epsilon,   7);
  ADD_CARD32_PROP (icpL1Power,     7);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  ADD_SEPARATOR("Symmetry settings");
  ADD_STRING_PROP (symmetryBaseName, 6);
  ADD_FLOAT32_PROP(symmetryWeight,   6);
//ADD_BOOLEAN_PROP(useYZSymmetry,    6);
//ADD_BOOLEAN_PROP(useRTSymmetry,    6);
//ADD_BOOLEAN_PROP(rotation,         6);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  ADD_SEPARATOR("Feature line settings");
  ADD_FLOAT32_PROP(featureLineWeight, 6);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  ADD_SEPARATOR("Coplanarity settings");
  ADD_STRING_PROP (coplanarityMesh,   6);
  ADD_FLOAT32_PROP(coplanarityWeight, 6);
  ADD_FLOAT32_PROP(angleThreshold,    6);
  ADD_FLOAT32_PROP( sizeThreshold,    6);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  ADD_SEPARATOR("Handle settings");
  ADD_FLOAT32_PROP(handleWeight, 6);
    
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  ADD_SEPARATOR("Solver");
//ADD_OBJECT_PROP(settings, 0, ARSettings::getClass(), true);
  ADD_FLOAT32_PROP(tsGridSpacing,       6);
  ADD_FLOAT32_PROP(tsIdentityWeight,    6);
  ADD_FLOAT32_PROP(tsRegularizerWeight, 6);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  ADD_SEPARATOR("Conjugate gradient settings");
  ADD_CARD32_PROP (cgNumIterations,        6);
  ADD_FLOAT32_PROP(cgConvergenceThreshold, 6);
  ADD_BOOLEAN_PROP(cgDebugOutput,          6);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#ifdef USE_THREADED_TOOL_FOR_CICP
  ADD_NOARGS_METHOD_THREADED(Deform);
  ADD_NOARGS_METHOD_THREADED(EvaluateSecondary);
//ADD_NOARGS_METHOD_THREADED(SelectSymmetries);
  ADD_NOARGS_METHOD_THREADED(Reset);
  ADD_NOARGS_METHOD_THREADED(SaveHandles);
  ADD_NOARGS_METHOD_THREADED(LoadHandles);
  ADD_NOARGS_METHOD_THREADED(ClearHandles);
  ADD_NOARGS_METHOD_THREADED(SaveSettings);
  ADD_NOARGS_METHOD_THREADED(LoadSettings);
#else
  ADD_NOARGS_METHOD(Deform);
  ADD_NOARGS_METHOD(EvaluateSecondary);
//ADD_NOARGS_METHOD(SelectSymmetries);
  ADD_NOARGS_METHOD(Reset);
  ADD_NOARGS_METHOD(SaveHandles);
  ADD_NOARGS_METHOD(LoadHandles);
  ADD_NOARGS_METHOD(ClearHandles);
  ADD_NOARGS_METHOD(SaveSettings);
  ADD_NOARGS_METHOD(LoadSettings);
#endif
}

//----------------------------------------------------------------------
void PCIConstrainedICP::mouseMoved(int32 x, int32 y)
{
  SceneEditorWidget* widget =
    dynamic_cast<SceneEditorWidget*>(sceneEditorWidget);

  if (widget &&
      widget->mapGLAreaToGlobal(QPoint(x, y)) != QCursor::pos())
  {
    return;
  }

  handleChanged_ = false;

  handler_.setParams(
    getScene()->getRootState()->staticState->camera,
    getScene()->getRootState()->staticState->viewFrustum,
    IDENTITY4F);

  handler_.mouseMoved(x, y);

  if (!handleChanged_)
  {
    mouseMovedPassToCam(x, y);
  }
}

//----------------------------------------------------------------------
void PCIConstrainedICP::mouseDown(int32 x, int32 y,
  MouseButtons buttonsState, Modifiers modifiersState)
{
  if (buttonsState.getMiddle())
  {
    mouseDownPassToCam(x, y, buttonsState, modifiersState);
    
    return;
  }
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  PreprocessInput();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  UICPC* inputPC  = RetrieveUICPC(input);

  if (!inputPC)
  {
    warning("PCIConstrainedICP::mouseDown() - "
            "Input node not found or attributes missing.");

    mouseDownPassToCam(x, y, buttonsState, modifiersState);

    return;
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  UICPC* resultPC = RetrieveUICPC(result, inputPC);

  if (!resultPC)
  {
    warning("PCIConstrainedICP::mouseDown() - "
            "Result node not found or created or attributes missing.");

    mouseDownPassToCam(x, y, buttonsState, modifiersState);

    return;
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  handler_.setParams(
    getScene()->getRootState()->staticState->camera,
    getScene()->getRootState()->staticState->viewFrustum,
    IDENTITY4F);
  
  MouseButtons hButtons(buttonsState.getLeft () ||
                        buttonsState.getRight(), false, false);

  handler_.mouseDown(x, y, hButtons, modifiersState);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (buttonsState.getLeft())
  {
    if (handler_.selected()) { return; }

    mpcard id = 0;

    if (resultPC                                      &&
        PickPoint(x, y, resultPC, id)                 &&
        id < resultPC->getPointSet()->getNumEntries())
    {
      AAT const  inputPositionAAT =  inputPC->getAAT("position");
      AAT const resultPositionAAT = resultPC->getAAT("position");

      Vector3f const origin =
         inputPC->getPointSet()->get3f(id,  inputPositionAAT);
      
      Vector3f const target =
        resultPC->getPointSet()->get3f(id, resultPositionAAT);

      handles_.push_back(new CICPHandle(origin, target, id));

      handler_.mouseDown(x, y, buttonsState, modifiersState);
    }
    else
    {
      mouseDownPassToCam(x, y, buttonsState, modifiersState);
    }
  }
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  else if (buttonsState.getRight())
  {
    bool selected = false;
    
    card32 h = 0;
    card32 p = 0;

    handler_.getSelection(selected, h, p);

    if (selected && h < handles_.size())
    {
      delete handles_.at(h);

      handles_.erase(handles_.begin() + h);
    }
    else
    {
      mouseDownPassToCam(x, y, buttonsState, modifiersState);
    }
  }
}

//----------------------------------------------------------------------
void PCIConstrainedICP::mouseUp(int32 x, int32 y,
  MouseButtons buttonsState, Modifiers modifiersState)
{
  bool working = handler_.selected();

  handler_.setParams(
    getScene()->getRootState()->staticState->camera,
    getScene()->getRootState()->staticState->viewFrustum,
    IDENTITY4F);

  MouseButtons hButtons(buttonsState.getLeft () ||
                        buttonsState.getRight(), false, false);

  handler_.mouseUp(x, y, hButtons, modifiersState);

  if (!working)
  {
    mouseUpPassToCam(x, y, buttonsState, modifiersState);
  }
}

//----------------------------------------------------------------------
void PCIConstrainedICP::areaResize(card32 width, card32 height)
{
  handler_.setParams(
    getScene()->getRootState()->staticState->camera,
    getScene()->getRootState()->staticState->viewFrustum,
    IDENTITY4F);

  handler_.areaResize(width, height);
  
  areaResizePassToCam(width, height);
}

//----------------------------------------------------------------------
void PCIConstrainedICP::mouseWheelRotated(int32     rotatedDelta,
                                          Modifiers modifiersState)
{
  mouseWheelRotatedPassToCam(rotatedDelta, modifiersState);
}

//----------------------------------------------------------------------
void PCIConstrainedICP::keyDown(GeneralKey key)
{
  keyDownPassToCam(key);
}

//----------------------------------------------------------------------
void PCIConstrainedICP::keyUp(GeneralKey key)
{
  keyUpPassToCam(key);
}

//----------------------------------------------------------------------
void PCIConstrainedICP::glDrawTool(GLContext *glContext)
{
  for (card32 i = 0; i < handles_.size(); ++i)
  {
    handles_.at(i)->glDraw(glContext);
  }
}

//----------------------------------------------------------------------
void PCIConstrainedICP::handleChanged(card32 which, card32 point)
{
  handleChanged_ = true;
}

//----------------------------------------------------------------------
bool PCIConstrainedICP::PickPoint(
  int32 const x, int32 const y, PointCloud* pc, mpcard& id)
{
  UnstructuredInCorePointCloud* uicpc =
    dynamic_cast<UnstructuredInCorePointCloud*>(pc);

  if (!uicpc) { return false; }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  PointSet* ps = uicpc->getPointSet();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  Vector3f clickPoint = NULL_VECTOR3F;

  if (!guessClickPoint(x, y, 3, clickPoint)) { return false; }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  HierarchicalKNNIterator knn(pc, 16, NULL);

  knn.setSeekPointAndReset(clickPoint);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  AAT const positionAAT = uicpc->getAAT("position");
  
  float32 dist = norm(knn.get3f(positionAAT) - clickPoint);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  id = knn.getPointSetPointNumber();

  return dist < 5.0f; //MAX_POINT_DIST
}

//----------------------------------------------------------------------
PCIConstrainedICP::UICPC* PCIConstrainedICP::RetrieveUICPC(
  std::string const& node, UICPC* reference)
{
  UICPC* uicpc = dynamic_cast<UICPC*>(getPointCloud(getScene(), node));

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (!uicpc)
  {
    if (!reference) { return 0; }

    iteration_ = 1; //HACK
  
    uicpc = dynamic_cast<UICPC*>(reference->copy());
  
    addPointCloud(getScene(), uicpc, node);
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (uicpc && uicpc->getAAT("position") == NULL_AAT)
  {
    debugOutput << "PCIConstrainedICP::RetrieveUICPC() - "
                   "\"position\" attribute missing.\n";
    uicpc = 0;
  }
    
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (uicpc && uicpc->getAAT("normal"  ) == NULL_AAT)
  {
    debugOutput << "PCIConstrainedICP::RetrieveUICPC() - "
                   "\"normal\" attribute missing.\n";
    uicpc = 0;
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return uicpc;
}

//----------------------------------------------------------------------
void PCIConstrainedICP::PreprocessInput(bool const force)
{
  if (force)
  {
    symmetryInitialized_ = false;
  }

  InitializeSymmetry(symmetryBaseName);
  
  UICPC* inputPC = RetrieveUICPC(input);

  if (!inputPC)
  {
    debugOutput << "PCIConstrainedICP::PreprocessInput() - "
                   "Null pointer.\n";
    return;
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  UICTM* mesh = 0;
  
  if (useCoplanarity)
  {
    mesh = dynamic_cast<UICTM*>(RetrieveUICPC(coplanarityMesh));
  }

  isMesh_ = mesh;
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (!force && inputPC == inputPC_ && (!useCoplanarity || (mesh == mesh_)))
  {
    return;
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  replacePointCloud(getScene(), result,
    dynamic_cast<PointCloud*>(inputPC->copy()));

  inputPC_ = getPointCloud(getScene(), input);  

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (!isMesh_) { return; }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  coplanarity_ = CoplanarityConstraints(angleThreshold, sizeThreshold);  

  coplanarity_.Initialize(*mesh);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  mesh_ = mesh;
}

//----------------------------------------------------------------------
void PCIConstrainedICP::Reset()
{
  debugOutput << "\n";
  debugOutput << "########################################\n";
  debugOutput << "################ Reset #################\n";
  debugOutput << "########################################\n";
  debugOutput << "\n";

  iteration_ = 1;

  hasOldSolver_ = false;

  deletePointCloud(getScene(), result);

  symmetry_.Reset();

  PreprocessInput(true);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  UICPC* inputPC = RetrieveUICPC(input);

  if (!inputPC)
  {
    warning("PCIConstrainedICP::Reset() - "
            "Input node not found or attributes missing.");
    return;
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (!RetrieveUICPC(result, inputPC))
  {
    warning("PCIConstrainedICP::Reset() - "
            "Result node not found or created or attributes missing.");
  }
}

//----------------------------------------------------------------------
void PCIConstrainedICP::SelectSymmetries()
{
  //symmetry_.RaiseDialog(useFeature);
}

//----------------------------------------------------------------------
void PCIConstrainedICP::SaveSettings()
{
  std::string const filename = FileDialogs::getSaveFileName(
    sceneEditorWidget, "Save CICP settings to file", "*.object");

  if (!filename.empty())
  {
    BinaryOutputObjectStream out(filename.c_str());

    out.writeObject(this);
  }
}

//----------------------------------------------------------------------
void PCIConstrainedICP::LoadSettings()
{
  std::string const filename = FileDialogs::getOpenFileName(
    sceneEditorWidget, "Load CICP settings to file", "*.object");

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (!filename.empty())
  {
		PCInteractionTool* s = 0;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    BinaryInputObjectStream in(filename.c_str());
		
    in.readAnyObject(s);
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		PCIConstrainedICP* cicp = dynamic_cast<PCIConstrainedICP*>(s);

    if (!cicp)
    {
      QMessageBox::information(getToolWidget(), "Error",
        "Object is not a PCIConstrainedICP object.");

      delete s;

      return;
    }

    CopySettings(*cicp);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //delete settings;
    //settings = cicp->settings;
    //cicp->settings = 0;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    delete s;
  
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    ObjectViewsTable::update(this);
  }
}

//----------------------------------------------------------------------
void PCIConstrainedICP::SaveHandles()
{
  std::string const extension = ".handle.object";

  std::string filename = FileDialogs::getSaveFileName(
    getToolWidget(), "Save CICP handles", "*" + extension);

  if (filename.empty()) { return; }

  if (std::search( filename.begin(),  filename.end(),
                  extension.begin(), extension.end()) == filename.end())
  {
    filename += extension;
  }

  BinaryOutputObjectStream out(filename.c_str());

  CICPPersistentHandles* p = new CICPPersistentHandles(handles_);

  out.writeObject(p);

  delete p;
}

//----------------------------------------------------------------------
void PCIConstrainedICP::LoadHandles()
{
  std::string filename = FileDialogs::getOpenFileName(
    getToolWidget(), "Load CICP handles", "*.handle.object");

  if (filename.empty()) { return; }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  BinaryInputObjectStream in(filename.c_str());

  Persistent* instance = 0;

  in.readObject(instance);

  CICPPersistentHandles* hInstance =
    dynamic_cast<CICPPersistentHandles*>(instance);

  if (!hInstance)
  {
    QMessageBox::information(getToolWidget(), "Error",
                             "Object is not a handle object.");
    delete instance;
    
    return;
  }
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  handles_.clear();

  for (mpcard i = 0; i < hInstance->getNumhandless(); ++i)
  {
    Handle3D* h =
      dynamic_cast<Handle3D*>(hInstance->gethandles(i)->copy());

    CICPHandle* handle = dynamic_cast<CICPHandle*>(h);

    if (handle)
    {
      //disable topological constraints after load -> might vary
      handle->id_ = ARConstraintAnchor::NO_TOP_ANCHOR;
    }

    handles_.push_back(h);
  }

  ObjectViewsTable::update(this);
}

//----------------------------------------------------------------------
void PCIConstrainedICP::ClearHandles()
{
  handles_.clear();

  ObjectViewsTable::update(this);
}

//----------------------------------------------------------------------
void PCIConstrainedICP::CopySettings(PCIConstrainedICP const& other)
{
  input                  = other.input;
  result                 = other.result;
  secondary              = other.secondary;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  icpTarget              = other.icpTarget;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  coplanarityMesh        = other.coplanarityMesh;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  symmetryBaseName       = other.symmetryBaseName;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //settings is copied elsewhere -> pointer stuff
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  smoothed               = other.smoothed;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  useICP                 = other.useICP;
  useHandle              = other.useHandle;
  useSymmetry            = other.useSymmetry;
  useCoplanarity         = other.useCoplanarity;
  useFeatureLine         = other.useFeatureLine;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  handleWeight           = other.handleWeight;
  symmetryWeight         = other.symmetryWeight;
  coplanarityWeight      = other.coplanarityWeight;
  featureLineWeight      = other.featureLineWeight;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  angleThreshold         = other.angleThreshold;
  sizeThreshold          = other.sizeThreshold;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  iterations             = other.iterations;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  buildConstraints       = other.buildConstraints;
  optimize               = other.optimize;
  useYZSymmetry          = other.useYZSymmetry;
  useRTSymmetry          = other.useRTSymmetry;
  rotation               = other.rotation;
//reverse                = other.reverse;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  tsGridSpacing          = other.tsGridSpacing;
  tsIdentityWeight       = other.tsIdentityWeight;
  tsRegularizerWeight    = other.tsRegularizerWeight;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  cgNumIterations        = other.cgNumIterations;
  cgConvergenceThreshold = other.cgConvergenceThreshold;
  cgDebugOutput          = other.cgDebugOutput;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//reweightSymmetry       = other.reweightSymmetry;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  icpWeight              = other.icpWeight;
  icpFOutlierPercentile  = other.icpFOutlierPercentile;
  icpROutlierPercentile  = other.icpROutlierPercentile;
  icpFOutlierThreshold   = other.icpFOutlierThreshold;
  icpROutlierThreshold   = other.icpROutlierThreshold;
  icpBuildForward        = other.icpBuildForward;
  icpBuildReverse        = other.icpBuildReverse;
  
  icpNormalThreshold     = other.icpNormalThreshold;
  icpCullNormalDirect    = other.icpCullNormalDirect;
  icpCullNormalTarget    = other.icpCullNormalTarget;

  icpNearFieldThreshold  = other.icpNearFieldThreshold;

  icpUseL1Norm           = other.icpUseL1Norm;
  icpL1Threshold         = other.icpL1Threshold;
  icpL1Epsilon           = other.icpL1Epsilon;
  icpL1Power             = other.icpL1Power;
  
  samplingModifier       = other.samplingModifier;
}

} //namespace X4
