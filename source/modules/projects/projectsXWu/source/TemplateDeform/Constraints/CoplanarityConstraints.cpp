//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "CoplanarityConstraints.h"
//----------------------------------------------------------------------
#include "ARConstraintsInterface.h"
#include "DeformationEvaluator.h"
#include "PointSet.h"
//----------------------------------------------------------------------
#include "UnstructuredInCoreTriangleMesh.h"
//----------------------------------------------------------------------
#include "ARConstraints.h"
//----------------------------------------------------------------------
#include "TemplateDeform/Misc/Downsampler.h"
#include "TemplateDeform/Misc/Downsampler.inline.h"
//----------------------------------------------------------------------
#include "TemplateDeform/Misc/MeshStructure.h"
//----------------------------------------------------------------------
#include "TemplateDeform/Misc/nr/nr3.h"
#include "TemplateDeform/Misc/nr/svd.h"
//----------------------------------------------------------------------
#include "DebugRenderer.h"
//----------------------------------------------------------------------
#include <PCCTriangleMeshSampler.h>
//----------------------------------------------------------------------
#include <Timer.h>
//----------------------------------------------------------------------

namespace X4
{

//======================================================================
// Constructors
//----------------------------------------------------------------------
CoplanarityConstraint::CoplanarityConstraint(
  std::vector<Vector3f> const& points)
  : reference_(points)
  , deformed_ (points)
{}

//======================================================================
// Member functions
//----------------------------------------------------------------------
void CoplanarityConstraint::Update(DeformationEvaluator& evaluator)
{
  deformed_.clear();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (std::vector<Vector3f>::const_iterator it = reference_.begin();
    it != reference_.end(); ++it)
  {
    deformed_.push_back(evaluator.getPosition(ARConstraintAnchor(*it)));
  }
}

//----------------------------------------------------------------------
void CoplanarityConstraint::Reset()
{
  deformed_ = reference_;
}

//----------------------------------------------------------------------
void CoplanarityConstraint::Build(
  ARConstraintsInterface      & receiver,
  float32                const  gridSpacing,
  float32                const  weight)
{
  Vector3f mean = NULL_VECTOR3F;

  for (std::vector<Vector3f>::const_iterator it = deformed_.begin();
    it != deformed_.end(); ++it)
  {
    mean += *it;
  }

  mean /= deformed_.size();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //numerical recipes SVD stuff
  MatDoub matrix(deformed_.size(), 3, 0.0);

  for (card32 j = 0; j < deformed_.size(); ++j)
  {
    Vector3f const point = deformed_.at(j) - mean;

    matrix[j][0] = point[0];
    matrix[j][1] = point[1];
    matrix[j][2] = point[2];
  }

  nr::SVD svd(matrix);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // obtain plane parameters
  Vector3f const normal = makeVector3f(svd.v[0][2], svd.v[1][2], svd.v[2][2]);

  Matrix3f quadric;
  {
    quadric[0][0] = svd.v[0][0];
    quadric[0][1] = svd.v[1][0];
    quadric[0][2] = svd.v[2][0];
    
    quadric[1][0] = svd.v[0][1];
    quadric[1][1] = svd.v[1][1];
    quadric[1][2] = svd.v[2][1];
    
    quadric[2]    = normal;
  }
  
  Matrix3f I3;
  {
    I3.setZero();
    I3[2][2] = weight;
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // add constraints while taking into account grip spacing
  Downsampler sampler(gridSpacing);

  sampler.AddPoints(reference_);

  std::vector<Vector3f> const sources = sampler.Points();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (std::vector<Vector3f>::const_iterator it = sources.begin();
    it != sources.end(); ++it)
  {
    Vector3f const& source = *it;
    Vector3f const  target = source + ((mean - source) * normal) * normal;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    PositionARConstraint c;

    c.anchor     = ARConstraintAnchor(source);
    c.covariance = quadric * I3 * quadric.transpose();
    c.time       = 0;
    c.toPoint    = target;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    receiver.addPositionConstraint(&c);
  }
}

//======================================================================
// Constructors
//----------------------------------------------------------------------
CoplanarityConstraints::CoplanarityConstraints(
  float32 const angleThreshold,
  float32 const  sizeThreshold)
  : angleThreshold_(angleThreshold)
  ,  sizeThreshold_( sizeThreshold)
{}

//======================================================================
// Member functions
//----------------------------------------------------------------------
void CoplanarityConstraints::Initialize(
  UnstructuredInCoreTriangleMesh& mesh)
{
  constraints_.clear();

  X4_TIMER_START(CoplanarityInitialization);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  TrimeshStatic sMesh(&mesh);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // Calculate total area of the mesh to have a more meaningful threshold
  // for the desired size of the planes.
  float32 overallArea = 0.0f;

  for (mpcard i = 0; i < sMesh.GetNumFaces(); ++i)
  {
    Vector3i const faceVertices = sMesh.GetFaceVertices(i);

    Vector3f const a = sMesh.GetVertPosition(faceVertices[0]);
    Vector3f const b = sMesh.GetVertPosition(faceVertices[1]);
    Vector3f const c = sMesh.GetVertPosition(faceVertices[2]);

    Vector3f const ab = b - a;
    Vector3f const ac = c - a;

    overallArea += ab.crossProduct(ac).getNorm() * 0.5f;
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //card32 const sizeThreshold = sMesh.GetNumFaces() * sizeThreshold_;
  float32 sizeThreshold = overallArea * sizeThreshold_;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  std::set<mpcard> overall;


  debugRenderer->beginRenderJob("MeshNormals");

  for (mpcard i = 0; i < sMesh.GetNumFaces(); ++i)
  {
    if (overall.find(i) != overall.end()) { continue; }

//    Vector3f const normal = sMesh.GetFaceNormal(i);
    Vector3f normal = NULL_VECTOR3F;
    
    {
      Vector3i const faceVertices = sMesh.GetFaceVertices(i);

      Vector3f const a = sMesh.GetVertPosition(faceVertices[0]);
      Vector3f const b = sMesh.GetVertPosition(faceVertices[1]);
      Vector3f const c = sMesh.GetVertPosition(faceVertices[2]);

      Vector3f const ab = b - a;
      Vector3f const ac = c - a;

      normal = ab.crossProduct(ac);

      if (normal.getNorm() < 1e-6) { continue; }

      normal.normalize();

      Vector3f const base = (a + b + c) / 3.0f;
      //debugRenderer->addLine(base, base + normal * 0.1f,
      //  makeVector3f(1.0f, 0.0f, 0.0f), makeVector3f(1.0f, 0.0f, 0.0f), 4.0f);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    std::set<mpcard> current;
    std::set<mpcard> processed;
    
    std::vector<mpcard> stack;

    stack.push_back(i);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    while (!stack.empty())
    {
      mpcard const face = stack.back();

      stack.pop_back();

      if (  overall.find(face) !=   overall.end()) { continue; }
      if (processed.find(face) != processed.end()) { continue; }

      processed.insert(face);

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      //Vector3f const faceNormal = sMesh.GetFaceNormal(face);
      Vector3f faceNormal = NULL_VECTOR3F;

      {
        Vector3i const faceVertices = sMesh.GetFaceVertices(face);

        Vector3f const a = sMesh.GetVertPosition(faceVertices[0]);
        Vector3f const b = sMesh.GetVertPosition(faceVertices[1]);
        Vector3f const c = sMesh.GetVertPosition(faceVertices[2]);

        Vector3f const ab = b - a;
        Vector3f const ac = c - a;

        faceNormal = ab.crossProduct(ac);

        faceNormal.normalize();

        Vector3f const base = (a + b + c) / 3.0f;
        //debugRenderer->addLine(base, base + faceNormal * 0.1f,
        //  makeVector3f(0.0f, 0.0f, 1.0f), makeVector3f(0.0f, 0.0f, 1.0f), 4.0f);
      }

      float32 angle  = (normal           * faceNormal          );
              angle /= (normal.getNorm() * faceNormal.getNorm());
              angle  = acos(angle);
              angle *= 180.0f / M_PI;

      if (angleThreshold_ < angle)                  { continue; }

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      current.insert(face);

      std::vector<mpcard> const adjacent = sMesh.GetFaceAdjacentFaces(face);

      stack.insert(stack.end(), adjacent.begin(), adjacent.end());
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Calculate area of current plane.
    float32 area = 0.0f;

    for (std::set<mpcard>::const_iterator it = current.begin();
      it != current.end(); ++it)
    {
      Vector3i const faceVertices = sMesh.GetFaceVertices(*it);

      Vector3f const a = sMesh.GetVertPosition(faceVertices[0]);
      Vector3f const b = sMesh.GetVertPosition(faceVertices[1]);
      Vector3f const c = sMesh.GetVertPosition(faceVertices[2]);

      Vector3f const ab = b - a;
      Vector3f const ac = c - a;

      area += ab.crossProduct(ac).getNorm() * 0.5f;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //if (current.size() < sizeThreshold) { continue; }
    if (area < sizeThreshold) { continue; }

    debugRenderer->beginRenderJob_MultiFrame("planes", constraints_.size() + 1, 1);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    overall.insert(current.begin(), current.end());
    
    std::vector<mpcard> triangles(current.begin(), current.end());

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    UnstructuredInCoreTriangleMesh mesh;

    mesh.clearAndSetupDefault(triangles.size() * 3, triangles.size());

    AAT const positionAAT = mesh.getAAT("position");
    AAT const    indexAAT = mesh.getTriangles()->getAAT("index");

    for (mpcard j = 0; j < triangles.size(); ++j)
    {
      Vector3i const faceVertices = sMesh.GetFaceVertices(triangles.at(j));

        Vector3f const a = sMesh.GetVertPosition(faceVertices[0]);
        Vector3f const b = sMesh.GetVertPosition(faceVertices[1]);
        Vector3f const c = sMesh.GetVertPosition(faceVertices[2]);

        Vector3f const ab = b - a;
        Vector3f const ac = c - a;

        Vector3f faceNormal = ab.crossProduct(ac);

        faceNormal.normalize();

        Vector3f const base = (a + b + c) / 3.0f;
        debugRenderer->addLine(base, base + faceNormal * 0.1f,
          makeVector3f(0.0f, 0.0f, 1.0f), makeVector3f(0.0f, 0.0f, 1.0f), 4.0f);

      mesh.getPointSet()->set3f(j * 3 + 0, positionAAT, sMesh.GetVertPosition(faceVertices[0]));
      mesh.getPointSet()->set3f(j * 3 + 1, positionAAT, sMesh.GetVertPosition(faceVertices[1]));
      mesh.getPointSet()->set3f(j * 3 + 2, positionAAT, sMesh.GetVertPosition(faceVertices[2]));

      mesh.getTriangles()->set3i(j, indexAAT, makeVector3i(j * 3, j * 3 + 1, j * 3 + 2));
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    UnstructuredInCorePointCloud& sampled =
      *PCCTriangleMeshSampler::sampleMeshPoisson(&mesh, 0.007f, 0);

    AAT const sPositionAAT = sampled.getAAT("position");

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    std::vector<Vector3f> points;

    for (mpcard j = 0; j < sampled.getPointSet()->getNumEntries(); ++j)
    {
      points.push_back(sampled.getPointSet()->get3f(j, sPositionAAT));
    }

    for (std::vector<Vector3f>::const_iterator it = points.begin();
      it != points.end(); ++it)
    {
      debugRenderer->addPoint(*it, makeVector3f(1.0f, 0.3f, 0.3f));
    }

    debugRenderer->endRenderJob();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    constraints_.push_back(CoplanarityConstraint(points));
  }

  debugRenderer->endRenderJob();

  X4_TIMER_STOP(CoplanarityInitialization);

  debugOutput << "co-plane constraints: " << constraints_.size() << "\n";
}

//----------------------------------------------------------------------
void CoplanarityConstraints::Update(DeformationEvaluator& evaluator)
{
  for (std::vector<CoplanarityConstraint>::iterator it = constraints_.begin();
    it != constraints_.end(); ++it)
  {
    it->Update(evaluator);
  }
}

//----------------------------------------------------------------------
void CoplanarityConstraints::Build(
  ARConstraintsInterface      & receiver,
  float32                const  gridSpacing,
  float32                const  weight)
{
  for (std::vector<CoplanarityConstraint>::iterator it = constraints_.begin();
    it != constraints_.end(); ++it)
  {
    it->Build(receiver, gridSpacing, weight);
  }
}

} //namespace X4
