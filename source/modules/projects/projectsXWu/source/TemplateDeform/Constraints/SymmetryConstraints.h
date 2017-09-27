//----------------------------------------------------------------------
#ifndef SymmetryConstraints_h_
#define SymmetryConstraints_h_
//----------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//----------------------------------------------------------------------
#include "TemplateDeform/Constraints/CICPSymmetry.h"

namespace X4
{

//======================================================================
// Forward declarations
//----------------------------------------------------------------------
class TPSSolver;
class PointSet;

//======================================================================
// SymmetryConstraints
//----------------------------------------------------------------------
class SymmetryConstraints
{
public:
  //====================================================================
  // Constructors
  //--------------------------------------------------------------------
  //SymmetryConstraints();

  //====================================================================
  // Accessors
  //--------------------------------------------------------------------
  std::vector<CICPRotation      > const& Rotations      () const;
  std::vector<CICPReflection    > const& Reflections    () const;
  std::vector<CICPLatticeElement> const& LatticeElements() const;
  std::vector<CICPFeatureLine   > const& FeatureLines   () const;
  std::vector<CICPLattice       > const& Lattices       () const;
  
  //====================================================================
  // Mutators
  //--------------------------------------------------------------------
  std::vector<CICPRotation      >& Rotations      ();
  std::vector<CICPReflection    >& Reflections    ();
  std::vector<CICPLatticeElement>& LatticeElements();
  std::vector<CICPFeatureLine   >& FeatureLines   ();
  std::vector<CICPLattice       >& Lattices       ();
  
  //====================================================================
  // Member functions
  //--------------------------------------------------------------------
  void Clear();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  void Update(TPSSolver& evaluator);
  void Reset();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  void Build(TPSSolver& receiver, float32 const weight);
  
  void BuildFeatureLines(TPSSolver& receiver, float32 const weight);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  void BuildConstraintsYZ(TPSSolver      & receiver,
                          PointSet  const& reference,
                          float32   const  weight);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  void BuildConstraintsRTTranslation(TPSSolver      & receiver,
                                     PointSet  const& reference,
                                     float32   const  weight);
  void BuildConstraintsRTRotation   (TPSSolver      & receiver,
                                     PointSet  const& reference,
                                     float32   const  weight);
  
private:
  //====================================================================
  // Member functions
  //--------------------------------------------------------------------

  //====================================================================
  // Member variables
  //--------------------------------------------------------------------
  std::vector<CICPRotation      > rotations_;
  std::vector<CICPReflection    > reflections_;
  std::vector<CICPLatticeElement> latticeElements_;
  std::vector<CICPFeatureLine   > featureLines_;
  std::vector<CICPLattice       > lattices_;
};

} //namespace X4

//----------------------------------------------------------------------
#endif //SymmetryConstraints_h_
//----------------------------------------------------------------------
