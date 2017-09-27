//----------------------------------------------------------------------
#ifndef CICPConstraints_h_
#define CICPConstraints_h_
//----------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//----------------------------------------------------------------------
#include "PointSet.h"
//----------------------------------------------------------------------
#include "GeometricTools.h"
//----------------------------------------------------------------------
#include "TemplateDeform/Misc/Downsampler.h"
#include "TemplateDeform/Misc/Downsampler.inline.h"
//----------------------------------------------------------------------
#include "TemplateDeform/TPSSolver/TPSSolver.h"

//#include "LinearAlgebra.h"
//#include "LinearAlgebra.inline.h"
//#include "DynamicLinearAlgebra.h"
//#include "DynamicLinearAlgebra.inline.h"
//#include "TemplateDeform/Misc/OStream.h"

#include "DebugRenderer.h"



namespace X4
{
//======================================================================
// Forward declarations
//----------------------------------------------------------------------
class PointSet;

class CICPSymmetry
{
public:
  virtual Matrix4f Transformation() const = 0;
  
  virtual void Build(TPSSolver& receiver, float32 const weight) const
  {
    Matrix4f const transformation = Transformation();
    
    Matrix3f const rotation    = shrink4To3(transformation   );
    Vector3f const translation = shrink4To3(transformation[3]);

    for (mpcard i = 0; i < sampledSource_.size(); ++i)
    {
      Vector3f const& source = sampledSource_.at(i);
      Vector3f const& target = sampledTarget_.at(i);

      receiver.AddSymmetryConstraint(source, target,
        rotation, translation, weight);
    }
  }

  virtual void Update(TPSSolver& deformation)
  {
    source_ = Deform(initialSource_, deformation);
    target_ = Deform(initialTarget_, deformation);
  }

  virtual void Reset()
  {
    source_ = initialSource_;
    target_ = initialTarget_;
  }

private:
  std::vector<Vector3f> Deform(
    std::vector<Vector3f> const& points,
    TPSSolver                  & deformation)
  {
    std::vector<Vector3f> result;

    for (std::vector<Vector3f>::const_iterator it = points.begin();
      it != points.end(); ++it)
    {
      result.push_back(deformation.Evaluate(*it));
    }

    return result;
  }

protected:
  std::vector<Vector3f> initialSource_;
  std::vector<Vector3f> initialTarget_;

  std::vector<Vector3f> source_;
  std::vector<Vector3f> target_;

  std::vector<Vector3f> sampledSource_;
  std::vector<Vector3f> sampledTarget_;
  
};

class CICPReflection : public CICPSymmetry
{
public:
  CICPReflection(PointSet            const& reference,
                 std::vector<mpcard> const& points,
                 Matrix4f            const& transformation,
                 float32             const  gridSpacing,
                 Plane3f               const& plane)
  {
    AAT const positionAAT = reference.getAAT("position");

    if (positionAAT == NULL_AAT)
    {
      throw PException("CICPReflection::CICPReflection() - Attribute missing.");
    }

    float32 const d = plane.getNormal() * plane.getPoint();

    for (std::vector<mpcard>::const_iterator it = points.begin();
         it != points.end(); ++it)
    {
      Vector3f const&  position = reference.get3f(*it, positionAAT);
      Vector3f const  tPosition =
        shrink4To3(transformation * expand3To4(position));

      float32 const distance = position * plane.getNormal() - d;

      if      (distance >  0.001f) //HACK
      {
        initialSource_.push_back( position);
        initialTarget_.push_back(tPosition);
      }
      else if (distance < -0.001f) //HACK
      {
        initialSource_.push_back(tPosition);
        initialTarget_.push_back( position);
      }
    }

    source_ = initialSource_;
    target_ = initialTarget_;

    Downsampler sampler(gridSpacing);

    sampler.AddPoints(initialSource_);

    sampledSource_ = sampler.Points();

    for (std::vector<Vector3f>::const_iterator it =
      sampledSource_.begin(); it != sampledSource_.end(); ++it)
    {
      sampledTarget_.push_back(shrink4To3(transformation * expand3To4(*it)));
    }
  }

  virtual Matrix4f Transformation() const
  {
    Vector3f sourceMean = NULL_VECTOR3F;
    Vector3f targetMean = NULL_VECTOR3F;

    for (std::vector<Vector3f>::const_iterator it = source_.begin();
      it != source_.end(); ++it)
    {
      sourceMean += *it;
    }

    sourceMean /= source_.size();

    for (std::vector<Vector3f>::const_iterator it = target_.begin();
      it != target_.end(); ++it)
    {
      targetMean += *it;
    }

    targetMean /= target_.size();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    Vector3f n = targetMean - sourceMean;

    float32 const length = norm(n);

    if (1e-6 > length)
    {
      throw PException("Unabel to calculate reflection.");
    }

    n /= length;

    if      (numeric_limits<float32>::epsilon() < abs(n[0]))
    {
      if (0 > n[0])
      {
        n *= -1.0f;
      }
    }
    else if (numeric_limits<float32>::epsilon() < abs(n[1]))
    {
      if (0 > n[1])
      {
        n *= -1.0f;
      }
    }
    else if (numeric_limits<float32>::epsilon() > n[2])
    {
      n *= -1.0f;
    }

    Vector3f const c = (sourceMean + targetMean) * 0.5f;

    Matrix3f t = IDENTITY3F;
    {
      t[0][0] = 1 - 2 * n[0] * n[0];
      t[0][1] =   - 2 * n[0] * n[1];
      t[0][2] =   - 2 * n[0] * n[2];

      t[1][0] =   - 2 * n[0] * n[1];
      t[1][1] = 1 - 2 * n[1] * n[1];
      t[1][2] =   - 2 * n[1] * n[2];

      t[2][0] =   - 2 * n[0] * n[2];
      t[2][1] =   - 2 * n[1] * n[2];
      t[2][2] = 1 - 2 * n[2] * n[2];
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    return makeTranslation4f(c) * expand3To4(t) * makeTranslation4f(-c);
  }
};

class CICPRotation : public CICPSymmetry
{
public:
  CICPRotation(PointSet            const& reference,
               std::vector<mpcard> const& points,
               Matrix4f            const& transformation,
               float32             const  gridSpacing)
  {
    AAT const positionAAT = reference.getAAT("position");

    if (positionAAT == NULL_AAT)
    {
      throw PException("CICPRotation::CICPRotation() - Attribute missing.");
    }
   
//    debugRenderer->beginRenderJob("sym");

    for (std::vector<mpcard>::const_iterator it = points.begin();
         it != points.end(); ++it)
    {
      Vector3f const&  position = reference.get3f(*it, positionAAT);
      Vector3f const  tPosition =
        shrink4To3(transformation * expand3To4(position));

//      debugRenderer->addLine(position, tPosition);

      initialSource_.push_back( position);
      initialTarget_.push_back(tPosition);
    }

    source_ = initialSource_;
    target_ = initialTarget_;

//    debugRenderer->endRenderJob();

    Downsampler sampler(gridSpacing);

    sampler.AddPoints(reference, points);

    sampledSource_ = sampler.Points();

    for (std::vector<Vector3f>::const_iterator it =
      sampledSource_.begin(); it != sampledSource_.end(); ++it)
    {
      sampledTarget_.push_back(shrink4To3(transformation * expand3To4(*it)));
    }
  }

  virtual Matrix4f Transformation() const
  {
    return computeRigidTransformationFromCorrespondences(target_, source_);
  }
};

class CICPLatticeElement : public CICPSymmetry
{
public:
  CICPLatticeElement(PointSet            const& reference,
                     std::vector<mpcard> const& points,
                     Matrix4f            const& transformation,
                     float32             const  gridSpacing)
  {
    AAT const positionAAT = reference.getAAT("position");

    if (positionAAT == NULL_AAT)
    {
      throw PException("CICPRotation::CICPRotation() - Attribute missing.");
    }
   
    for (std::vector<mpcard>::const_iterator it = points.begin();
         it != points.end(); ++it)
    {
      Vector3f const&  position = reference.get3f(*it, positionAAT);
      Vector3f const  tPosition =
        shrink4To3(transformation * expand3To4(position));

      initialSource_.push_back( position);
      initialTarget_.push_back(tPosition);
    }

    source_ = initialSource_;
    target_ = initialTarget_;

    Downsampler sampler(gridSpacing);

    sampler.AddPoints(reference, points);

    sampledSource_ = sampler.Points();

    for (std::vector<Vector3f>::const_iterator it =
      sampledSource_.begin(); it != sampledSource_.end(); ++it)
    {
      sampledTarget_.push_back(shrink4To3(transformation * expand3To4(*it)));
    }
  }

  virtual Matrix4f Transformation() const
  {
    return computeRigidTransformationFromCorrespondences(target_, source_);
  }
};

class CICPLattice : public CICPSymmetry
{
public:
  CICPLattice(PointSet            const& reference,
              std::vector<mpcard> const& points,
              Vector3f            const& u,
//              Vector3f            const& v,
              int32               const  uMin,
              int32               const  uMax,
//              int32               const  vMin,
//              int32               const  vMax,
              float32             const  gridSpacing)
  {
    AAT const positionAAT = reference.getAAT("position");

    if (positionAAT == NULL_AAT)
    {
      throw PException("CICPRotation::CICPRotation() - Attribute missing.");
    }
   
    for (std::vector<mpcard>::const_iterator it = points.begin();
         it != points.end(); ++it)
    {
      Vector3f const& position = reference.get3f(*it, positionAAT);
      
      for (int32 offset = uMin; offset < uMax; ++offset)
      {
        initialSource_.push_back(position + u *  offset     );
        initialTarget_.push_back(position + u * (offset + 1));
      }
    }

    source_ = initialSource_;
    target_ = initialTarget_;

    Downsampler sampler(gridSpacing);

    sampler.AddPoints(initialSource_);

    sampledSource_ = sampler.Points();

    for (std::vector<Vector3f>::const_iterator it =
      sampledSource_.begin(); it != sampledSource_.end(); ++it)
    {
      sampledTarget_.push_back(*it + u);
    }
  }

  virtual Matrix4f Transformation() const
  {
    return computeRigidTransformationFromCorrespondences(target_, source_);
  }
};

class CICPFeatureLine : public CICPSymmetry
{
public:
  CICPFeatureLine(Vector3f const& a,
                  Vector3f const& b,
                  int32    const  segments)
  {
    Vector3f const ab = b - a;
    
    Vector3f direction(ab);

    direction.normalize();

    float32 const offset = ab.getNorm() / segments;

    for (int32 i = 0; i < segments; ++i)
    {
      initialSource_.push_back(a + direction * offset *  i     );
      initialTarget_.push_back(a + direction * offset * (i + 1));
    }

    source_ = initialSource_;
    target_ = initialTarget_;

    sampledSource_ = initialSource_;
    sampledTarget_ = initialTarget_;
  }

  virtual Matrix4f Transformation() const
  {
    Vector3f translation = NULL_VECTOR3F;

    for (mpcard i = 0; i < source_.size(); ++i)
    {
      Vector3f const source = source_.at(i);
      Vector3f const target = target_.at(i);

      translation += target - source;
    }

    translation /= source_.size();
    
    return makeTranslation4f(translation);
  }
};

} //namespace X4

//----------------------------------------------------------------------
#endif //CICPConstraints_h_
//----------------------------------------------------------------------
