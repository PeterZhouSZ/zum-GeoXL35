//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "PCIConstrainedICP.h"
//----------------------------------------------------------------------
#include "ARDeformationOptimizationContext.h"
#include "AREvaluateDeformation.h"
#include "ARSettings.h"
//----------------------------------------------------------------------
#include "ICPConstraints.h"
//----------------------------------------------------------------------
#include "TemplateDeform/Constraints/HandleConstraints.h"
//----------------------------------------------------------------------
#include "TSDeformationSolver.h"
#include "TemplateDeform/TPSSolver/TPSSolver.h"
//----------------------------------------------------------------------
#include "CICPHandle.h"
//----------------------------------------------------------------------
#include "DebugRenderer.h"
//----------------------------------------------------------------------
#include "SGRelativeTimeAnimationNode.h"
#include "SymmetryGroupAttachment.h"
#include "Reflection.h"
#include "Rotation.h"
#include "Dihedral.h"
#include "Lattice.h"

#include "ProgressWindow.h"

#include "GeometricTools.h"

#include "LinearAlgebra.h"
#include "LinearAlgebra.inline.h"
#include "DynamicLinearAlgebra.h"
#include "DynamicLinearAlgebra.inline.h"
#include "TemplateDeform/Misc/OStream.h"

#include "PointSetANNQuery.h"

//----------------------------------------------------------------------
#include "TemplateDeform/Misc/Downsampler.h"
#include "TemplateDeform/Misc/Downsampler.inline.h"
//----------------------------------------------------------------------

#include "ColorTools.h"

namespace X4
{

//----------------------------------------------------------------------
struct ConstraintInformation
{
  ConstraintInformation()
    : source      (NULL_VECTOR3F)
    , target      (NULL_VECTOR3F)
    , anchor      (NULL_VECTOR3F)
    , sourceNormal(NULL_VECTOR3F)
    , targetNormal(NULL_VECTOR3F)
    , anchorNormal(NULL_VECTOR3F)
  {}

  ConstraintInformation(
    Vector3f const& source,
    Vector3f const& target,
    Vector3f const& anchor,
    Vector3f const& sourceNormal,
    Vector3f const& targetNormal,
    Vector3f const& anchorNormal)
    : source      (source      )
    , target      (target      )
    , anchor      (anchor      )
    , sourceNormal(sourceNormal)
    , targetNormal(targetNormal)
    , anchorNormal(anchorNormal)
  {}

  Vector3f source;
  Vector3f target;
  Vector3f anchor;

  Vector3f sourceNormal;
  Vector3f targetNormal;
  Vector3f anchorNormal;
};

//----------------------------------------------------------------------
void BuildICPConstraints(
  UnstructuredInCorePointCloud      & reference,
  UnstructuredInCorePointCloud      & deformed,
  UnstructuredInCorePointCloud      & target,
  ARConstraintsInterface            & receiver,
  float32                      const  gridSpacing,
  float32                      const  weight,
  float32                      const  forwardOutlierPercentile,
  float32                      const  reverseOutlierPercentile,
  float32                      const  forwardOutlierThreshold,
  float32                      const  reverseOutlierThreshold,
  bool                         const  buildForward,
  bool                         const  buildReverse,
  float32                      const  normalThreshold,
  bool                         const  cullNormalDirect,
  bool                         const  cullNormalTarget,
  float32                      const  nearFieldThreshold)
{
  PointSet& referencePS = *reference.getPointSet();
  PointSet&  deformedPS =  *deformed.getPointSet();
  PointSet&    targetPS =    *target.getPointSet();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  AAT const referencePositionAAT = referencePS.getAAT("position");
  AAT const  deformedPositionAAT =  deformedPS.getAAT("position");
  AAT const    targetPositionAAT =    targetPS.getAAT("position");

  AAT const referenceNormalAAT = referencePS.getAAT("normal");
  AAT const  deformedNormalAAT =  deformedPS.getAAT("normal");
  AAT const    targetNormalAAT =    targetPS.getAAT("normal");

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  float32 const sqrForwardOutlierThreshold =
    forwardOutlierThreshold * forwardOutlierThreshold;

  float32 const sqrReverseOutlierThreshold =
    reverseOutlierThreshold * reverseOutlierThreshold;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  std::vector<float32> forwardDistances;
  std::vector<float32> reverseDistances;
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // Assemble forward distances
  {
    //HierarchicalKNNIterator hIt(&deformed, 32, 0);
    PointSetANNQuery knn(&deformedPS, 1);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    for (mpcard i = 0; i < targetPS.getNumEntries(); ++i)
    {
      Vector3f const targetPosition = targetPS.get3f(i, targetPositionAAT);

      //hIt.setSeekPointAndReset(targetPosition);
      int32 const index = knn.getNearestPointIndex(targetPosition);

      Vector3f const sourcePosition = deformedPS.get3f(index, deformedPositionAAT);
//        deformedPS.get3f(hIt.getCurrentPointIndex(), deformedPositionAAT);

      Vector3f const vector = targetPosition - sourcePosition;

      float32 const sqrNorm = vector * vector;

      if (sqrNorm < sqrForwardOutlierThreshold)
      {
        forwardDistances.push_back(sqrNorm);
      }
    }
  }
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // Assemble reverse distances
  {
    //HierarchicalKNNIterator hIt(&target, 32, 0);
    PointSetANNQuery knn(&targetPS, 1);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    for (mpcard i = 0; i < deformedPS.getNumEntries(); ++i)
    {
      Vector3f const sourcePosition = deformedPS.get3f(i, deformedPositionAAT);

      //hIt.setSeekPointAndReset(sourcePosition);
      int32 const index = knn.getNearestPointIndex(sourcePosition);

      Vector3f const targetPosition = targetPS.get3f(index, targetPositionAAT);
//        targetPS.get3f(hIt.getCurrentPointIndex(), targetPositionAAT);

      Vector3f const vector = targetPosition - sourcePosition;

      float32 const sqrNorm = vector * vector;

      if (sqrNorm < sqrReverseOutlierThreshold)
      {
        reverseDistances.push_back(sqrNorm);
      }
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // Determine distance for pruning.
  std::sort(forwardDistances.begin(), forwardDistances.end());
  std::sort(reverseDistances.begin(), reverseDistances.end());

  float32 const maxForwardDistance =
    forwardDistances.at(
      (forwardDistances.size() - 1) * (1.0f - forwardOutlierPercentile));

  float32 const maxReverseDistance =
    reverseDistances.at(
      (reverseDistances.size() - 1) * (1.0f - reverseOutlierPercentile));
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  std::vector<ConstraintInformation> forwardConstraints;
  std::vector<ConstraintInformation> reverseConstraints;

//  debugRenderer->clearDebugData("Pruned forward constraints");
//  debugRenderer->beginRenderJob("Pruned forward constraints", false);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // Assemble forward constraints
  if (buildForward)
  {
    //HierarchicalKNNIterator hIt(&deformed, 32, 0);
    PointSetANNQuery knn(&deformedPS, 1);

    card32 count = 0;
    float32 sum = 0.0f;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    for (mpcard i = 0; i < targetPS.getNumEntries(); ++i)
    {
      Vector3f const targetPosition = targetPS.get3f(i, targetPositionAAT);
      Vector3f const targetNormal   = targetPS.get3f(i, targetNormalAAT  );

      //hIt.setSeekPointAndReset(targetPosition);
      int32 const index = knn.getNearestPointIndex(targetPosition);

      Vector3f const anchorPosition = referencePS.get3f(index, referencePositionAAT);
//        referencePS.get3f(hIt.getCurrentPointIndex(), referencePositionAAT);
      Vector3f const anchorNormal   = referencePS.get3f(index, referenceNormalAAT);
//        referencePS.get3f(hIt.getCurrentPointIndex(), referenceNormalAAT  );

      Vector3f const sourcePosition = deformedPS.get3f(index, deformedPositionAAT);
//        deformedPS.get3f(hIt.getCurrentPointIndex(), deformedPositionAAT);
      Vector3f const sourceNormal   = deformedPS.get3f(index, deformedPositionAAT);
//        deformedPS.get3f(hIt.getCurrentPointIndex(), deformedNormalAAT  );

      Vector3f const vector = targetPosition - sourcePosition;

      float32 const sqrNorm = vector * vector;

      if (sqrNorm > sqr(nearFieldThreshold))
      {
        bool cull = false;

        Vector3f color = makeVector3f(1.0f, 0.0f, 0.0f);
        
        if (sqrNorm > maxForwardDistance)
        {
          cull = true;

          if (sqrNorm > sqrForwardOutlierThreshold)
          {
            color = makeVector3f(0.0f, 1.0f, 0.0f);
          }
        }

        if (cullNormalDirect)
        {
          float32 const e = sourceNormal * targetNormal;

          if (e < normalThreshold)
          {
            cull = true;
        
            color = makeVector3f(0.0f, 0.0f, 1.0f);
          }
        }

        if (cullNormalTarget)
        {
          float32 const norm = sqrt(sqrNorm);

          float32 const e = fabs(vector * sourceNormal) / norm;

          if (e < normalThreshold)
          {
            cull = true;
        
            color = makeVector3f(0.0f, 1.0f, 1.0f);
          }
        }

        if (cull)
        {
          debugRenderer->addLine(sourcePosition, targetPosition,
            color, makeVector3f(1.0f, 1.0f, 1.0f), 1.0f);

          continue;
        }
      }

      sum += sqrNorm;
      ++count;

      debugRenderer->addLine(sourcePosition, targetPosition,
        NULL_VECTOR3F, makeVector3f(1.0f, 1.0f, 1.0f), 1.0f);

      forwardConstraints.push_back(ConstraintInformation(
        sourcePosition, targetPosition, anchorPosition,
        sourceNormal,   targetNormal,   anchorNormal));
    }

    sum /= count;

    debugOutput << "ICP RMSE: " << sqrt(sum) << "\n";
  }

//  debugRenderer->endRenderJob();

//  debugRenderer->clearDebugData("Pruned reverse constraints");
//  debugRenderer->beginRenderJob("Pruned reverse constraints", false);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // Assemble reverse constraints
  if (buildReverse)
  {
    //HierarchicalKNNIterator hIt(&target, 32, 0);
    PointSetANNQuery knn(&targetPS, 1);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    for (mpcard i = 0; i < deformedPS.getNumEntries(); ++i)
    {
      Vector3f const anchorPosition = referencePS.get3f(i, referencePositionAAT);
      Vector3f const anchorNormal   = referencePS.get3f(i, referenceNormalAAT  );

      Vector3f const sourcePosition = deformedPS.get3f(i, deformedPositionAAT);
      Vector3f const sourceNormal   = deformedPS.get3f(i, deformedNormalAAT  );

      //hIt.setSeekPointAndReset(sourcePosition);
      int32 const index = knn.getNearestPointIndex(sourcePosition);

      Vector3f const targetPosition = targetPS.get3f(index, targetPositionAAT);
//        targetPS.get3f(hIt.getCurrentPointIndex(), targetPositionAAT);

      Vector3f const targetNormal = targetPS.get3f(index, targetNormalAAT);
//        targetPS.get3f(hIt.getCurrentPointIndex(), targetNormalAAT);

      Vector3f const vector = targetPosition - sourcePosition;

      float32 const sqrNorm = vector * vector;

      if (sqrNorm > sqr(nearFieldThreshold))
      {
        bool cull = false;

        Vector3f color = makeVector3f(1.0f, 0.0f, 0.0f);
        
        if (sqrNorm > maxForwardDistance)
        {
          cull = true;

          if (sqrNorm > sqrForwardOutlierThreshold)
          {
            color = makeVector3f(0.0f, 1.0f, 0.0f);
          }
        }

        if (cullNormalDirect)
        {
          float32 const e = sourceNormal * targetNormal;

          if (e < normalThreshold)
          {
            cull = true;
        
            color = makeVector3f(0.0f, 0.0f, 1.0f);
          }
        }

        if (cullNormalTarget)
        {
          float32 const norm = sqrt(sqrNorm);

          float32 const e = fabs(vector * sourceNormal) / norm;

          if (e < normalThreshold)
          {
            cull = true;
        
            color = makeVector3f(0.0f, 1.0f, 1.0f);
          }
        }

        if (cull)
        {
          debugRenderer->addLine(sourcePosition, targetPosition,
            color, makeVector3f(1.0f, 1.0f, 1.0f), 1.0f);

          continue;
        }
      }

      debugRenderer->addLine(sourcePosition, targetPosition,
        NULL_VECTOR3F, makeVector3f(1.0f, 1.0f, 1.0f), 1.0f);

      reverseConstraints.push_back(ConstraintInformation(
        sourcePosition, targetPosition, anchorPosition,
        sourceNormal,   targetNormal,   anchorNormal));
    }
  }

//  debugRenderer->endRenderJob();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // Downsample forward constraints
  if (buildForward)
  {
    DownsamplerWithTargetAndNormal downsampler(gridSpacing);

    for (std::vector<ConstraintInformation>::const_iterator it =
      forwardConstraints.begin(); it != forwardConstraints.end(); ++it)
    {
      Vector3f const& normal = it->targetNormal;
     
      if (normal[0] != normal[0] ||
          normal[1] != normal[1] ||
          normal[2] != normal[2])
      {
        debugOutput << "BuildICPConstraints() - Removing corrupt normal.\n";
        
        continue;
      }
      
      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      downsampler.AddPoint(it->anchor, it->target, it->targetNormal);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    std::vector<Vector3f> const anchors = downsampler.Anchors();
    std::vector<Vector3f> const targets = downsampler.Targets();
    std::vector<Vector3f> const normals = downsampler.Normals();

    for (mpcard i = 0; i < anchors.size(); ++i)
    {        
      ARConstraintAnchor anchor;

      anchor.topologicalAnchor = ARConstraintAnchor::NO_TOP_ANCHOR;
      anchor.pos               = anchors.at(i);

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      Vector3f normal = normals.at(i);

      if (normal.getNorm() < 0.0001f)
      {
        debugOutput << "BuildICPConstraints() - Removing degenerate normal.\n";

        continue;
      }

      normal.normalize();

      Matrix3f const quadric = outerProduct(normal, normal);

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      PositionARConstraint constraint;
      
      constraint.setup(anchor, targets.at(i), quadric * weight, 0);

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      receiver.addPositionConstraint(&constraint);

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      Vector3f const lineColor0 = makeVector3f(1.0f, 0.50f, 0.0f);
      Vector3f const lineColor1 = makeVector3f(0.5f, 0.25f, 0.0f);

      debugRenderer->addLine(
        constraint.anchor.pos, constraint.toPoint, lineColor0, lineColor1, 4.0f);
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // Downsample reverse constraints
  if (buildReverse)
  {
    DownsamplerWithTargetAndNormal downsampler(gridSpacing);

    for (std::vector<ConstraintInformation>::const_iterator it =
      reverseConstraints.begin(); it != reverseConstraints.end(); ++it)
    {
      Vector3f const& normal = it->targetNormal;
     
      if (normal[0] != normal[0] ||
          normal[1] != normal[1] ||
          normal[2] != normal[2])
      {
        debugOutput << "BuildICPConstraints() - Removing corrupt normal.\n";
        
        continue;
      }
      
      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      downsampler.AddPoint(it->anchor, it->target, it->targetNormal);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    std::vector<Vector3f> const anchors = downsampler.Anchors();
    std::vector<Vector3f> const targets = downsampler.Targets();
    std::vector<Vector3f> const normals = downsampler.Normals();

    for (mpcard i = 0; i < anchors.size(); ++i)
    {        
      ARConstraintAnchor anchor;

      anchor.topologicalAnchor = ARConstraintAnchor::NO_TOP_ANCHOR;
      anchor.pos               = anchors.at(i);

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      Vector3f normal = normals.at(i);

      if (normal.getNorm() < 0.0001f)
      {
        debugOutput << "BuildICPConstraints() - Removing degenerate normal.\n";

        continue;
      }

      normal.normalize();

      Matrix3f const quadric = outerProduct(normal, normal);

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      PositionARConstraint constraint;
      
      constraint.setup(anchor, targets.at(i), quadric * weight, 0);

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      receiver.addPositionConstraint(&constraint);

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      Vector3f const lineColor0 = makeVector3f(0.21f, 0.46f, 0.56f);
      Vector3f const lineColor1 = makeVector3f(1.00f, 1.00f, 1.00f);

      debugRenderer->addLine(
        constraint.anchor.pos, constraint.toPoint, lineColor0, lineColor1, 4.0f);
    }
  }
}

//----------------------------------------------------------------------
void BuildICPConstraintsL1Normalized(
  UnstructuredInCorePointCloud      & reference,
  UnstructuredInCorePointCloud      & deformed,
  UnstructuredInCorePointCloud      & target,
  ARConstraintsInterface            & receiver,
  float32                      const  gridSpacing,
  float32                      const  weight,
  bool                         const  buildForward,
  bool                         const  buildReverse,
  float32                      const  l1Threshold,
  float32                      const  l1Epsilon,
  float32                      const  l1Power)
{
  PointSet& referencePS = *reference.getPointSet();
  PointSet&  deformedPS =  *deformed.getPointSet();
  PointSet&    targetPS =    *target.getPointSet();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  AAT const referencePositionAAT = referencePS.getAAT("position");
  AAT const  deformedPositionAAT =  deformedPS.getAAT("position");
  AAT const    targetPositionAAT =    targetPS.getAAT("position");

  AAT const referenceNormalAAT = referencePS.getAAT("normal");
  AAT const  deformedNormalAAT =  deformedPS.getAAT("normal");
  AAT const    targetNormalAAT =    targetPS.getAAT("normal");

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  std::vector<ConstraintInformation> forwardConstraints;
  std::vector<ConstraintInformation> reverseConstraints;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // Assemble forward constraints
  if (buildForward)
  {
    PointSetANNQuery knn(&deformedPS, 1);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    for (mpcard i = 0; i < targetPS.getNumEntries(); ++i)
    {
      Vector3f const targetPosition = targetPS.get3f(i, targetPositionAAT);
      Vector3f const targetNormal   = targetPS.get3f(i, targetNormalAAT  );

      int32 const index = knn.getNearestPointIndex(targetPosition);

      Vector3f const anchorPosition = referencePS.get3f(index, referencePositionAAT);
      Vector3f const anchorNormal   = referencePS.get3f(index, referenceNormalAAT);

      Vector3f const sourcePosition = deformedPS.get3f(index, deformedPositionAAT);
      Vector3f const sourceNormal   = deformedPS.get3f(index, deformedPositionAAT);

//      debugRenderer->addLine(sourcePosition, targetPosition,
//        NULL_VECTOR3F, makeVector3f(1.0f, 1.0f, 1.0f), 1.0f);

      forwardConstraints.push_back(ConstraintInformation(
        sourcePosition, targetPosition, anchorPosition,
        sourceNormal,   targetNormal,   anchorNormal));
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // Assemble reverse constraints
  if (buildReverse)
  {
    PointSetANNQuery knn(&targetPS, 1);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    for (mpcard i = 0; i < deformedPS.getNumEntries(); ++i)
    {
      Vector3f const anchorPosition = referencePS.get3f(i, referencePositionAAT);
      Vector3f const anchorNormal   = referencePS.get3f(i, referenceNormalAAT  );

      Vector3f const sourcePosition = deformedPS.get3f(i, deformedPositionAAT);
      Vector3f const sourceNormal   = deformedPS.get3f(i, deformedNormalAAT  );

      int32 const index = knn.getNearestPointIndex(sourcePosition);

      Vector3f const targetPosition = targetPS.get3f(index, targetPositionAAT);

      Vector3f const targetNormal = targetPS.get3f(index, targetNormalAAT);

//      debugRenderer->addLine(sourcePosition, targetPosition,
//        NULL_VECTOR3F, makeVector3f(1.0f, 1.0f, 1.0f), 1.0f);

      reverseConstraints.push_back(ConstraintInformation(
        sourcePosition, targetPosition, anchorPosition,
        sourceNormal,   targetNormal,   anchorNormal));
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // Downsample forward constraints
  if (buildForward)
  {
    DownsamplerWithTargetAndNormal downsampler(gridSpacing);

    for (std::vector<ConstraintInformation>::const_iterator it =
      forwardConstraints.begin(); it != forwardConstraints.end(); ++it)
    {
      Vector3f const& normal = it->targetNormal;
     
      if (normal[0] != normal[0] ||
          normal[1] != normal[1] ||
          normal[2] != normal[2])
      {
        debugOutput << "BuildICPConstraints() - Removing corrupt normal.\n";
        
        continue;
      }
      
      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      downsampler.AddPoint(it->anchor, it->target, it->targetNormal, it->source);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    std::vector<Vector3f> const anchors = downsampler.Anchors();
    std::vector<Vector3f> const targets = downsampler.Targets();
    std::vector<Vector3f> const normals = downsampler.Normals();
    std::vector<Vector3f> const sources = downsampler.Sources();

    for (mpcard i = 0; i < anchors.size(); ++i)
    {        
      ARConstraintAnchor anchor;

      anchor.topologicalAnchor = ARConstraintAnchor::NO_TOP_ANCHOR;
      anchor.pos               = anchors.at(i);

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      Vector3f normal = normals.at(i);

      if (normal.getNorm() < 0.0001f)
      {
        debugOutput << "BuildICPConstraints() - Removing degenerate normal.\n";

        continue;
      }

      normal.normalize();

      Matrix3f const quadric = outerProduct(normal, normal);

      Vector3f const vector = targets.at(i) - sources.at(i);

      float32 norm = vector.getNorm() /l1Threshold;

      if (norm > 1.0f)
      {
        norm = pow(norm, l1Power);
      }

      float32 const currentWeight = 1.0f / (norm + l1Epsilon);

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      PositionARConstraint constraint;
      
      constraint.setup(anchor, targets.at(i), quadric * weight * currentWeight, 0);

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      receiver.addPositionConstraint(&constraint);

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      //Vector3f const lineColor0 = makeVector3f(1.0f, 0.50f, 0.0f);
      //Vector3f const lineColor1 = makeVector3f(0.5f, 0.25f, 0.0f);
      Vector3f color = makeVector3f(1.0f, 1.0f, 1.0f);

      color[0] = vector.getNorm() / (l1Threshold * 5.0f);

      if (color[0] > 1.0f)
      {
        color[0] = 1.0f;
      }

      color[0] = 1.0f - color[0];

      Vector3f const lineColor0 = convertHSVtoRGB(color);
      Vector3f const lineColor1 = lineColor0;

      debugRenderer->addLine(
//        constraint.anchor.pos, constraint.toPoint, lineColor0, lineColor1, 4.0f);
        sources.at(i), constraint.toPoint, lineColor0, lineColor1, 4.0f);
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // Downsample reverse constraints
  if (buildReverse)
  {
    DownsamplerWithTargetAndNormal downsampler(gridSpacing);

    for (std::vector<ConstraintInformation>::const_iterator it =
      reverseConstraints.begin(); it != reverseConstraints.end(); ++it)
    {
      Vector3f const& normal = it->targetNormal;
     
      if (normal[0] != normal[0] ||
          normal[1] != normal[1] ||
          normal[2] != normal[2])
      {
        debugOutput << "BuildICPConstraints() - Removing corrupt normal.\n";
        
        continue;
      }
      
      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      downsampler.AddPoint(it->anchor, it->target, it->targetNormal, it->source);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    std::vector<Vector3f> const anchors = downsampler.Anchors();
    std::vector<Vector3f> const targets = downsampler.Targets();
    std::vector<Vector3f> const normals = downsampler.Normals();
    std::vector<Vector3f> const sources = downsampler.Sources();

    for (mpcard i = 0; i < anchors.size(); ++i)
    {        
      ARConstraintAnchor anchor;

      anchor.topologicalAnchor = ARConstraintAnchor::NO_TOP_ANCHOR;
      anchor.pos               = anchors.at(i);

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      Vector3f normal = normals.at(i);

      if (normal.getNorm() < 0.0001f)
      {
        debugOutput << "BuildICPConstraints() - Removing degenerate normal.\n";

        continue;
      }

      normal.normalize();

      Matrix3f const quadric = outerProduct(normal, normal);

      Vector3f const vector = targets.at(i) - sources.at(i);

      float32 norm = vector.getNorm() /l1Threshold;

      if (norm > 1.0f)
      {
        norm = pow(norm, l1Power);
      }

      float32 const currentWeight = 1.0f / (norm + l1Epsilon);

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      PositionARConstraint constraint;
      
      constraint.setup(anchor, targets.at(i), quadric * weight * currentWeight, 0);

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      receiver.addPositionConstraint(&constraint);

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      //Vector3f const lineColor0 = makeVector3f(0.21f, 0.46f, 0.56f);
      //Vector3f const lineColor1 = makeVector3f(1.00f, 1.00f, 1.00f);

      Vector3f color = makeVector3f(1.0f, 1.0f, 1.0f);

      color[0] = vector.getNorm() / (l1Threshold * 5.0f);

      if (color[0] > 1.0f)
      {
        color[0] = 1.0f;
      }

      color[0] = 1.0f - color[0];

      Vector3f const lineColor0 = convertHSVtoRGB(color);
      Vector3f const lineColor1 = lineColor0;

      debugRenderer->addLine(
//        constraint.anchor.pos, constraint.toPoint, lineColor0, lineColor1, 4.0f);
        sources.at(i), constraint.toPoint, lineColor0, lineColor1, 4.0f);
    }
  }
}

//======================================================================
// Member functions
//----------------------------------------------------------------------
void PCIConstrainedICP::Deform()
{  
  ProgressWindow* progress = getProgressWindow();

  PreprocessInput();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  UICPC* inputPC  = RetrieveUICPC(input);

  if (!inputPC)
  {
    warning("PCIConstrainedICP::Deform() - "
            "Input node not found or attributes missing.");
    
    return;
  }

  PointSet& inputPS = *inputPC->getPointSet();
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  UICPC* resultPC = RetrieveUICPC(result, inputPC);

  if (!resultPC)
  { 
    warning("PCIConstrainedICP::Deform() - "
            "Result node not found or created or attributes missing.");
    
    return;
  }

  PointSet* resultPS = resultPC->getPointSet();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  UICPC* targetPC = RetrieveUICPC(icpTarget);
  
  bool const hasTarget = targetPC;

  if (useICP && !hasTarget)
  {
    warning("PCIConstrainedICP::Deform() - "
            "Target node not found or attributes missing. "
            "ICP constraints will not be enforced.");
  }

  X4_TIMER_START(Overall);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // prepare TPSSolver
  X4_TIMER_START(SolverInitialize);

  TPSSolver solver(smoothed,
                   smoothed ? 4 : 2,
                   tsGridSpacing,
                   tsIdentityWeight,
                   tsRegularizerWeight);

  solver.Initialize(inputPC);

  if (hasOldSolver_)
  {
    solver.Update(oldSolver_);
  }
//  else
//  {
//    solver.Scale(); // If smoothed, weights are too high by about 12.5325f.
//  }

  debugOutput << "\n";

  X4_TIMER_STOP (SolverInitialize);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // prepare ICP constraints
#if 0
  ICPConstraints icp;

  icp.setConstraintMode        (settings->icpConstraintsMode        );
  icp.setOutliersDistanceFactor(settings->icpNearFieldRelativeCutOff);
  icp.setOutliersPercentile    (settings->icpOutlierPercentile      );
  icp.setCullNormalOutliers    (settings->icpNormalCulling          );
  icp.setOutlierNormals        (settings->icpNormalCosAngleCutOff   );
  icp.setDistanceOutliersMode  (settings->icpPercentileCulling ?
                                ICP_OUTLIERS_PERCENTILE        :
                                ICP_OUTLIERS_NONE                   );
#endif

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // prepare handle constraints
  HandleConstraints handle(handleWeight);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // check if coplanarity can be used
  if (useCoplanarity && !isMesh_)
  {
    warning("PCIConstrainedICP::Deform() - Input is not a mesh. "
            "Coplanarity constraints will not be enforced.");
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // check if feature lines can be used
  if (useSymmetry && useFeatureLine && symmetry_.FeatureLines().empty())
  {
    warning("PCIConstrainedICP::Deform() - No feature line constraints"
            "present. The grid spacing may be too high.");
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (card32 i = 0; i < iterations; ++i)
  {
    if (!buildConstraints) { break; }
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if (optimize)
    {
      debugOutput << "\n";
      debugOutput << "########################################\n";
      debugOutput << "# CICP Iteration " << iteration_ << "\n";
      debugOutput << "########################################\n";

      ++iteration_;
    }

    X4_TIMER_START(Iteration);
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -   
    debugRenderer->clearDebugData("Position_constraints");
    debugRenderer->clearDebugData("Symmetry_constraints");

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if (i)
    {
      resultPC = RetrieveUICPC(result);

      if (!resultPC)
      {
        error("PCIConstrainedICP::Deform() - Result update failed.");
        
        return;
      }

      resultPS = resultPC->getPointSet();
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    X4_TIMER_START(Regularizer);
    
    solver.beginConstraints();

    X4_TIMER_STOP(Regularizer);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // ICP constraints
    if (useICP && hasTarget)
    {
      X4_TIMER_START(ICPConstraints);
      
      debugRenderer->beginRenderJob_MultiFrame("CICP_ICP", iteration_ + 10, 1);

      if (icpUseL1Norm)
      {
        BuildICPConstraintsL1Normalized(*inputPC, *resultPC, *targetPC, solver,
          tsGridSpacing * samplingModifier, icpWeight,
          icpBuildForward, icpBuildReverse,
          icpL1Threshold, icpL1Epsilon, icpL1Power);
      }
      else
      {
        BuildICPConstraints(*inputPC, *resultPC, *targetPC, solver,
          tsGridSpacing * samplingModifier, icpWeight,
          icpFOutlierPercentile, icpROutlierPercentile,
          icpFOutlierThreshold,  icpROutlierThreshold,
          icpBuildForward, icpBuildReverse,
          icpNormalThreshold, icpCullNormalDirect, icpCullNormalTarget,
          icpNearFieldThreshold);
      }

      debugRenderer->endRenderJob();

#if 0
      if (reverse)
      {
        icp.buildReverseICPConstraints(
          settings, &solver, inputPC, resultPC, targetPC, 0, 0, true);
      }
      else
      {
        icp.buildICPConstraints(
          settings, &solver, inputPC, resultPC, targetPC, 0, 0, true, 0, true, true);
      }
#endif

      X4_TIMER_STOP(ICPConstraints);
    }
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // handle constraints
    if (useHandle)
    {
      X4_TIMER_START(HandleConstraints);

      handle.BuildConstraints(solver, inputPS, handles_);

      X4_TIMER_STOP(HandleConstraints);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // symmetry constraints
//    X4_TIMER_START(SymmetryConstraints);

    if (useSymmetry)// && !useYZSymmetry && !useRTSymmetry)
    {
      X4_TIMER_START(SymmetryConstraints);
  
      symmetry_.Build(solver, symmetryWeight);
      
      X4_TIMER_STOP(SymmetryConstraints);
    }

#if 0
    if (useSymmetry && useYZSymmetry && !useRTSymmetry)
    {
      symmetry_.BuildConstraintsYZ(solver, inputPS, symmetryWeight);
    }

    if (useSymmetry && useRTSymmetry)
    {
      if (rotation)
      {
        symmetry_.BuildConstraintsRTRotation(
          solver, inputPS, symmetryWeight);
      }
      else
      {
        symmetry_.BuildConstraintsRTTranslation(
          solver, inputPS, symmetryWeight);
      }
    }
#endif
    
//    X4_TIMER_STOP(SymmetryConstraints);
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // feature line constraints
    if (useFeatureLine)
    {
      X4_TIMER_START(FeatureLineConstraints);
      
      symmetry_.BuildFeatureLines(solver, featureLineWeight);

      X4_TIMER_STOP(FeatureLineConstraints);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // coplanarity constraints    
    if (useCoplanarity && isMesh_)
    {
      X4_TIMER_START(CoplanarityConstraints);
      
      coplanarity_.Build(solver, tsGridSpacing * samplingModifier, coplanarityWeight);
      
      X4_TIMER_STOP(CoplanarityConstraints);
    }
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if (!optimize) { break; } //for debugging

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    X4_TIMER_START(Solve);

    solver.Solve(
      cgNumIterations, cgConvergenceThreshold, cgDebugOutput, progress);

    X4_TIMER_STOP(Solve);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    X4_TIMER_START(Update);

       symmetry_.Update(solver);
    coplanarity_.Update(solver);

    X4_TIMER_STOP(Update);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    X4_TIMER_START(Evaluate);
    
    PointCloud* evaluated =
//      AREvaluateDeformation::evaluatePC(inputPC, 0, &solver, 0, settings);
      AREvaluateDeformation::evaluatePC(inputPC, 0, &solver, 0, 0);

    X4_TIMER_STOP(Evaluate);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    replacePointCloud(getScene(), result, evaluated);

    X4_TIMER_STOP(Iteration);
  }

  debugOutput << "\n";

  X4_TIMER_STOP(Overall);

  debugOutput << "\n";

  oldSolver_ = solver;
  hasOldSolver_ = true;
}

//----------------------------------------------------------------------
void PCIConstrainedICP::EvaluateSecondary()
{
  if (!hasOldSolver_)
  {
    warning("PCIConstrainedICP::EvaluateSecondary() - No old solver present.");
    return;
  }

  PointCloud* pc = getPointCloud(getScene(), secondary);

  if (!pc)
  {
    warning("PCIConstrainedICP::EvaluateSecondary() - PointCloud not found.");
    return;
  }

  PointCloud* evaluated =
    AREvaluateDeformation::evaluatePC(pc, 0, &oldSolver_, 0, 0);

  addOrReplacePointCloud(getScene(), "secondary_out", evaluated);
}

//#define WIND_TURBINE

//----------------------------------------------------------------------
void PCIConstrainedICP::InitializeSymmetry(std::string const& name)
{
  if (symmetryInitialized_) { return; }

  debugOutput << "Initializing symmetries.\n";

  SGListNode* root = dynamic_cast<SGListNode*>(getScene()->getRootNode());
	  
  SGRelativeTimeAnimationNode* aninode =
    dynamic_cast<SGRelativeTimeAnimationNode*>(
#ifdef WIND_TURBINE
    root->getChildNode(0, root->getChildIndex("data_" + name + "_groups")));
#else
    root->getChildNode(0, root->getChildIndex("root_" + name + "_groups")));
#endif

  if (!aninode)
  {
    if (useSymmetry || useFeatureLine) { error("Node \"root_" + name + "_groups\" not found."); }
    return;
  }

  SGObjectNode* pntnode = dynamic_cast<SGObjectNode*>(
    root->getChildNode(0, root->getChildIndex("__sym_work_shape__tmp_" + name)));

  if (!pntnode)
  {
    pntnode = dynamic_cast<SGObjectNode*>(
      root->getChildNode(0, root->getChildIndex("__sym_work_shape_" + name)));

    if (!pntnode)
    {
      if (useSymmetry || useFeatureLine) { error("\"__sym_work_shape_" + name + "\" not found."); }
      return;
    }
  }
  
  UICPC* points = dynamic_cast<UICPC*>(pntnode->getSceneObject());
  
  if (!points)
  {
    if (useSymmetry || useFeatureLine) { error("Points not found."); }
    return;
  }

  AAT const positionAAT = points->getPointSet()->getAAT("position");

  //SGListNode* sublist = dynamic_cast<SGListNode*>(
  //    aninode->getChildNode(0, aninode->getChildIndex("0")));
  SGListNode* sublist = dynamic_cast<SGListNode*>(aninode);

  if (!sublist)
  {
    if (useSymmetry || useFeatureLine) { error("Sublist not found."); }
    return;
  }

  symmetry_.Clear();

  debugRenderer->clearDebugData("new_sym");
  debugRenderer->beginRenderJob("new_sym");
  
  bool featureLinesBuilt = false;

  for (mpcard i = 0; i < sublist->getNumChildNodes(0); ++i)
  {
    SGListNode* subgroup = dynamic_cast<SGListNode*>(
#ifdef WIND_TURBINE
      sublist->getChildNode(0, i));

    if (!subgroup || subgroup->getName() != "subgroups") { continue; }

    for (mpcard j = 0; j < subgroup->getNumChildNodes(0); ++j)
#else
      sublist);

    mpcard const j = i;
#endif
    {
      SGObjectNode* sym = dynamic_cast<SGObjectNode*>(
        subgroup->getChildNode(0, j));

      if (!sym)
      {
        if (useSymmetry || useFeatureLine) { error("Sym not found"); }
        return;
      }

      // Check if that's the one node that we don't actually care about
      if (sym->getName() == "nonsym") { continue; }

      UICPC* pc = dynamic_cast<UICPC*>(sym->getSceneObject());

      if (!pc)
      {
        if (useSymmetry || useFeatureLine) { error("Pc not found"); }
        return;
      }

      sym::SymmetryGroupAttachment* group = dynamic_cast<sym::SymmetryGroupAttachment*>(
        pc->getAttachments()->getData("SymmetryGroup"));

      if (!group)
      {
        if (useSymmetry || useFeatureLine) { error("Group not found"); }
        return;
      }

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      // Build feature line constraints.
      if (!featureLinesBuilt)
      {
        FeatureSet& featureSet = *group->featureSet;

        for (mpcard fIndex = 0; fIndex < featureSet.getNumFeatures(); ++fIndex)
        {
          sym::FeatureLine* line =
            dynamic_cast<sym::FeatureLine*>(featureSet.getFeature(fIndex));

          Vector3f const a  = line->getPosition();
          Vector3f const b  = line->getEndPosition();

          Vector3f const ab = b - a;

          int32 const segments = ceil(ab.getNorm() / (tsGridSpacing * samplingModifier));

          if (segments < 3) { continue; } // Throws out feature lines shorter
                                          // than a grid cell.

          symmetry_.FeatureLines().push_back(CICPFeatureLine(a, b, segments));

          //debugRenderer->addLine(line->getPosition(), line->getEndPosition(),
          //  makeVector3f(float32(rand()) / RAND_MAX,
          //               float32(rand()) / RAND_MAX,
          //               float32(rand()) / RAND_MAX), NULL_VECTOR3F, 2.0f);
        }

        featureLinesBuilt = true;
      }

      // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
      sym::Reflection* reflection = dynamic_cast<sym::Reflection*>(group->group);
      sym::Rotation*   rotation   = dynamic_cast<sym::Rotation*  >(group->group);
      sym::Dihedral*   dihedral   = dynamic_cast<sym::Dihedral*  >(group->group);
      sym::Lattice*    lattice    = dynamic_cast<sym::Lattice*   >(group->group);    
      
      if (reflection)
      {
        debugOutput << "Reflective symmetry: " << group->group->getName() << "\n";

        symmetry_.Reflections().push_back(CICPReflection(
          *points->getPointSet(), group->symmetryPoints,
          reflection->getGenerator()->getWorldTransformation(1),
          tsGridSpacing * samplingModifier, reflection->getGenerator()->getPlane()));
      }
      else if (rotation)
      {
        debugOutput << "Rotational symmetry: " << group->group->getName() << "\n";

        symmetry_.Rotations().push_back(CICPRotation(
          *points->getPointSet(), group->symmetryPoints,
          rotation->getGenerator()->getWorldTransformation(1),
          tsGridSpacing * samplingModifier));

        sym::RotationH* rotationH = dynamic_cast<sym::RotationH*>(rotation);
        sym::RotationV* rotationV = dynamic_cast<sym::RotationV*>(rotation);

        if (rotationH)
        {
//          debugOutput << "This is a rotational symmetry with colinear   \n"
//                         "reflection planes. The reflections are not    \n"
//                         "enforced explicitely.                       \n\n";
          symmetry_.Reflections().push_back(CICPReflection(
            *points->getPointSet(), group->symmetryPoints,
            rotationH->getReflection()->getGenerator()->getWorldTransformation(1),
            tsGridSpacing * samplingModifier, rotationH->getReflection()->getGenerator()->getPlane()));
        }

        if (rotationV)
        {
//          debugOutput << "This is a rotational symmetry with a perpendicular \n"
//                         "reflection plane. The reflection is not enforced.\n\n";
          sym::Reflection* rVReflection =
            dynamic_cast<sym::Reflection*>(rotationV->getReflection(1));
          
            symmetry_.Reflections().push_back(CICPReflection(
              *points->getPointSet(), group->symmetryPoints,
              rVReflection->getGenerator()->getWorldTransformation(1),
              tsGridSpacing * samplingModifier, rVReflection->getGenerator()->getPlane()));
        }
      }
      else if (dihedral)
      {
        debugOutput << "Dihedral symmetry: " << group->group->getName() << "\n";

        symmetry_.Rotations().push_back(CICPRotation(
          *points->getPointSet(), group->symmetryPoints,
          dihedral->getGenerator()->getWorldTransformation(1),
          tsGridSpacing * samplingModifier));

        sym::Rotation* dRotation =
          dynamic_cast<sym::Rotation*>(dihedral->getRotation(0));

        symmetry_.Rotations().push_back(CICPRotation(
          *points->getPointSet(), group->symmetryPoints,
          dRotation->getGenerator()->getWorldTransformation(1),
          tsGridSpacing * samplingModifier));

        sym::DihedralH* dihedralH = dynamic_cast<sym::DihedralH*>(dihedral);

        if (dihedralH)
        {
//          debugOutput << "This is a dihedral symmetry with an additional     \n"
//                         "reflection plane. The reflection is not enforced.\n\n";
          symmetry_.Reflections().push_back(CICPReflection(
            *points->getPointSet(), group->symmetryPoints,
            dihedralH->getReflection()->getGenerator()->getWorldTransformation(1),
            tsGridSpacing * samplingModifier, dihedralH->getReflection()->getGenerator()->getPlane()));
        }
      }
      else if (lattice)
      {
        debugOutput << "Lattice symmetry: " << group->group->getName() << "\n";

        int32 const uMin = lattice->getMinParameterCoordinate()[0];
        int32 const uMax = lattice->getMaxParameterCoordinate()[0];

        int32 const vMin = lattice->getMinParameterCoordinate()[1];
        int32 const vMax = lattice->getMaxParameterCoordinate()[1];

        if (vMin != 0 || vMin != vMax)
        {
          throw PException("2D lattices are not yet supported.");
        }

        symmetry_.Lattices().push_back(CICPLattice(
          *points->getPointSet(), group->symmetryPoints,
          lattice->getGenerator()->u(), uMin, uMax,
          tsGridSpacing * samplingModifier));




        //for (int32 u = uMin; u <= uMax; ++u)
        //{
        //  for (int32 v = vMin; v <= vMax; ++v)
        //  {
        //    if (u == 0 && v == 0)
        //    {
        //      // That's supposed to be the center one.
        //      continue;
        //    }

        //    Vector3f const t = lattice->getGenerator()->u() * u +
        //                       lattice->getGenerator()->v() * v;

        //    
        //    symmetry_.LatticeElements().push_back(CICPLatticeElement(
        //      *points->getPointSet(), group->symmetryPoints,
        //      makeTranslation4f(t),
        //      tsGridSpacing * samplingModifier));
        //  }
        //}
      }
      else
      {
        debugOutput << group->group->getName() << "\n";
      }
    }
  }

  debugRenderer->endRenderJob();

  symmetryInitialized_ = true;
}

} //namespace X4
