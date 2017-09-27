//----------------------------------------------------------------------
#ifndef PCIConstrainedICP_h_
#define PCIConstrainedICP_h_
//----------------------------------------------------------------------
#include "PCInteractionTool.h"
#include "PCInteractionToolThreaded.h"
//----------------------------------------------------------------------
#include "Camera.h"
#include "ViewFrustum.h"
#include "x3base/x3deformation/source/userInterface/handles3d/Handle3DHandler.h"
//----------------------------------------------------------------------
#include "TemplateDeform/Constraints/SymmetryConstraints.h"
#include "TemplateDeform/Constraints/CoplanarityConstraints.h"
//----------------------------------------------------------------------
#include "TemplateDeform/TPSSolver/TPSSolver.h"

namespace X4
{
//======================================================================
// Forward declarations
//----------------------------------------------------------------------
class ARSettings;

//#define USE_THREADED_TOOL_FOR_CICP

//======================================================================
// PCIConstrainedICP
//----------------------------------------------------------------------
class PROJECTSXWU_API PCIConstrainedICP :
#ifdef USE_THREADED_TOOL_FOR_CICP
  public PCInteractionToolThreaded,
#else
  public PCInteractionTool,
#endif
  public Handle3DUpdatesListener
{
  X4_CLASS(PCIConstrainedICP)

public:
  //====================================================================
  // Nested
  //--------------------------------------------------------------------
  typedef UnstructuredInCorePointCloud   UICPC;
  typedef UnstructuredInCoreTriangleMesh UICTM;

public:
  //====================================================================
  // Constructors
  //--------------------------------------------------------------------
  PCIConstrainedICP();
  
  //====================================================================
  // Destructor
  //--------------------------------------------------------------------
  ~PCIConstrainedICP();

  //====================================================================
  // Member functions
  //--------------------------------------------------------------------
  virtual void mouseMoved(int32 x, int32 y);
  virtual void mouseDown (int32 x, int32 y,
    MouseButtons buttonsState, Modifiers modifiersState);
  virtual void mouseUp   (int32 x, int32 y,
    MouseButtons buttonsState, Modifiers modifiersState);
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  virtual void areaResize(card32 width, card32 height);
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  virtual void mouseWheelRotated(int32     rotatedDelta,
                                 Modifiers modifiersState);
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  virtual void keyDown(GeneralKey key);
  virtual void keyUp  (GeneralKey key);
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  virtual void glDrawTool(GLContext *glContext);

  //--------------------------------------------------------------------
  // Handle3DUpdatesListener
  //--------------------------------------------------------------------
  virtual void handleChanged(card32 which, card32 point);

protected:
  //====================================================================
  // Member functions
  //--------------------------------------------------------------------
  void Deform();
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  void SelectSymmetries();
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  void Reset();
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  void SaveHandles();
  void LoadHandles();
  void ClearHandles();
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  void SaveSettings();
  void LoadSettings();
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  void EvaluateSecondary();

  //====================================================================
  // Member variables
  //--------------------------------------------------------------------
  std::string input;
  std::string result;
  std::string secondary;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  std::string icpTarget;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  std::string coplanarityMesh;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  std::string symmetryBaseName;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //ARSettings* settings;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  bool smoothed;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  bool useICP;
  bool useHandle;
  bool useSymmetry;
  bool useCoplanarity;
  bool useFeatureLine;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  float32      handleWeight;
  float32    symmetryWeight;
  float32 coplanarityWeight;
  float32 featureLineWeight;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  float32 angleThreshold;
  float32  sizeThreshold;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  card32 iterations;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  bool buildConstraints;
  bool optimize;
  bool useYZSymmetry;
  bool useRTSymmetry;
  bool rotation;
//bool reverse;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  float32 tsGridSpacing;
  float32 tsIdentityWeight;
  float32 tsRegularizerWeight;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  card32  cgNumIterations;
  float32 cgConvergenceThreshold;
  bool    cgDebugOutput;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //bool reweightSymmetry;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  float32 icpWeight;
  float32 icpFOutlierPercentile;
  float32 icpROutlierPercentile;
  float32 icpFOutlierThreshold;
  float32 icpROutlierThreshold;
  bool icpBuildForward;
  bool icpBuildReverse;

  float32 icpNormalThreshold;
  bool icpCullNormalDirect;
  bool icpCullNormalTarget;

  float32 icpNearFieldThreshold;

  bool icpUseL1Norm;
  float32 icpL1Threshold;
  float32 icpL1Epsilon;
  card32 icpL1Power;

  float32 samplingModifier;

private:  
  //====================================================================
  // Member functions
  //--------------------------------------------------------------------
  bool PickPoint(
    int32 const x, int32 const y, PointCloud* pc, mpcard& id);
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  UICPC* RetrieveUICPC(std::string const& node, UICPC* reference = 0);
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  void PreprocessInput(bool const force = false);

  void CopySettings(PCIConstrainedICP const& other);

  void InitializeSymmetry(std::string const& name);

  //====================================================================
  // Member variables
  //--------------------------------------------------------------------
  std::vector<Handle3D*> handles_;
  Handle3DHandler        handler_;
  bool                   handleChanged_;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  PointCloud* inputPC_;
  UICTM* mesh_;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
     SymmetryConstraints symmetry_;
  CoplanarityConstraints coplanarity_;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  bool isMesh_;
  bool symmetryInitialized_;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  TPSSolver oldSolver_;
  bool hasOldSolver_;
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  card32 iteration_;
};

} //namespace X4

//----------------------------------------------------------------------
#endif //PCIConstrainedICP_h_
//----------------------------------------------------------------------
