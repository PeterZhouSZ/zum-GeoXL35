//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "SymmetryConstraints.h"
//----------------------------------------------------------------------
#include "TemplateDeform/TPSSolver/TPSSolver.h"
#include "PointSet.h"
//----------------------------------------------------------------------
#include "TemplateDeform/Misc/nr/nr3.h"
#include "TemplateDeform/Misc/nr/svd.h"
//----------------------------------------------------------------------
#include "DebugRenderer.h"
#include "StringHelper.h"
//----------------------------------------------------------------------

namespace X4
{
    //void BuildEmpty(
    //    SymmContainer::Ptr const &tsymm,
    //    PointSet const &deformed, AAT const& defAAT,
    //    PointSet const &reference, AAT const& refAAT,
    //    TPSSolver &receiver,
    //    float32 const weight,
    //    bool const &useFeature)
    //{
    //    return;
    //}

    //void BuildCoPlane(
    //    SymmContainer::Ptr const &tsymm,
    //    PointSet const &deformed, AAT const& defAAT,
    //    PointSet const &reference, AAT const& refAAT,
    //    TPSSolver &receiver,
    //    float32 const weight,
    //    bool const &useFeature) 
    //{
    //    SymmPlane::Ptr tsymm_cp = 
    //        boost::shared_polymorphic_downcast<SymmPlane>(tsymm);
    //    if (NULL == tsymm_cp) {
    //        warning("BuidRef: SymmReflective");
    //        return;
    //    }

    //    std::vector<mpcard> const& vertices = tsymm_cp->vindx_;

    //    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    Vector3f mean = NULL_VECTOR3F;

    //    for (mpcard j = 0; j < vertices.size(); ++j)
    //    {
    //        mean += deformed.get3f(vertices.at(j), defAAT);
    //    }

    //    mean /= vertices.size();

    //    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    //numerical recipes SVD stuff
    //    MatDoub matrix(vertices.size(), 3, 0.0);

    //    for (card32 j = 0; j < vertices.size(); ++j)
    //    {
    //        Vector3f const point =
    //            deformed.get3f(vertices.at(j), defAAT) - mean;

    //        matrix[j][0] = point[0];
    //        matrix[j][1] = point[1];
    //        matrix[j][2] = point[2];
    //    }

    //    nr::SVD svd(matrix);

    //    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    // obtain plane parameters
    //    Vector3f const normal =
    //        makeVector3f(svd.v[0][2], svd.v[1][2], svd.v[2][2]);

    //    Matrix3f mat3, I3;
    //    {
    //        mat3[0][0] = svd.v[0][0];
    //        mat3[0][1] = svd.v[1][0];
    //        mat3[0][2] = svd.v[2][0];
    //        mat3[1][0] = svd.v[0][1];
    //        mat3[1][1] = svd.v[1][1];
    //        mat3[1][2] = svd.v[2][1];
    //        mat3[2] = normal;

    //        I3.setZero();
    //        I3[2][2] = weight;
    //    }

    //    float32 const d = normal * mean;

    //    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    // add constraints
    //    for (card32 j = 0; j < vertices.size(); ++j)
    //    {
    //        mpcard const current = vertices.at(j);

    //        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //        Vector3f const refPos =
    //            reference.get3f(current, refAAT);
    //        Vector3f const defPos =
    //            deformed.get3f(current, defAAT);

    //        //debugRenderer->addPoint(point, color3);

    //        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //        //float32 const dist = defPos * normal - d;
    //        //Vector3f const target = defPos - normal * dist;

    //        Vector3f const target = refPos + ((mean - refPos) * normal) * normal;

    //        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //        PositionARConstraint c;

    //        c.anchor = ARConstraintAnchor(refPos, current);

    //        c.covariance = mat3 * I3 * mat3.transpose();
    //        c.time = 0;
    //        c.toPoint = target;

    //        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //        receiver.addPositionConstraint(&c);
    //    }
    //}

    //void BuildRotTra(
    //    SymmContainer::Ptr const &tsymm,
    //    PointSet const &deformed, AAT const& defAAT,
    //    PointSet const &reference, AAT const& refAAT,
    //    TPSSolver &receiver,
    //    float32 const weight,
    //    bool const &useFeature) 
    //{
    //    int debug_frame = 10;

    //    Matrix4f Tf, Tb;
    //    if (!tsymm->GetUpdatedTransformation(
    //        deformed, defAAT, Tf, Tb))
    //        return;

    //    int diff_r = 0;
    //    {
    //        OrbitVerticeMap::iterator mi, mj = tsymm->overt_.begin();
    //        for (mi = mj++; mj != tsymm->overt_.end(); ++mi, ++mj) {
    //            deque<mpcard> vinx0 = mi->second, vinx1 = mj->second;
    //            int diff = abs(mj->first - mi->first);
    //            diff_r += diff;
    //            //debugRenderer->beginRenderJob_OneFrame("debug_rottra_forw_", debug_frame++);
    //            deque<mpcard>::iterator vi0 = vinx0.begin(), vi1 = vinx1.begin();
    //            for (; vi0 != vinx0.end(); ++vi0, ++vi1) {
    //                Vector3f const point0_ref = reference.get3f(*vi0, refAAT);
    //                Vector3f const point1_ref = reference.get3f(*vi1, refAAT);
    //                Matrix4f T4 = IDENTITY4F;
    //                for (int j = 0; j < diff; ++j) {
    //                    T4 *= Tf;
    //                }
    //                receiver.AddSymmetryConstraint(point0_ref, point1_ref,
    //                    shrink4To3(T4), shrink4To3(T4[3]), weight);

    //                //Vector3f const point0_def = deformed.get3f(*vi0, defAAT);
    //                //Vector3f const point1_def = deformed.get3f(*vi1, defAAT);
    //                //Vector3f const point0_def_r = transformVector3f(
    //                //    expand3To4(shrink4To3(T4)), point0_def) ;
    //                //Vector3f const point0_def_t = transformVector3f(
    //                //    makeTranslation4f(shrink4To3(T4[3])), point0_def) ;
    //                //debugRenderer->addLine(
    //                //    point0_def, point0_def_r,
    //                //    makeVector3f(1, 0, 0),
    //                //    makeVector3f(0, 0, 1),
    //                //    3);
    //                //debugRenderer->addLine(
    //                //    point0_def_r, point1_def,
    //                //    makeVector3f(1, 0, 0),
    //                //    makeVector3f(0, 0, 1),
    //                //    3);
    //                //debugRenderer->addLine(
    //                //    transformVector3f(T4, point0_def), point1_def,
    //                //    makeVector3f(0, 1, 0),
    //                //    makeVector3f(0, 1, 0),
    //                //    3);
    //            }
    //            //debugRenderer->endRenderJob();
    //        }
    //    }
    //    //Matrix4f T4 = IDENTITY4F;
    //    //for (int j = 0; j < diff_r; ++j) {
    //    //    T4 *= Tb;
    //    //}
    //    //mj = tsymm->overt_.begin();
    //    {
    //        OrbitVerticeMap::reverse_iterator mi, mj = tsymm->overt_.rbegin();
    //        for (mi = mj++; mj != tsymm->overt_.rend(); ++mi, ++mj) {
    //            deque<mpcard> vinx0 = mi->second, vinx1 = mj->second;
    //            int diff = abs(mj->first - mi->first);
    //            //debugRenderer->beginRenderJob_OneFrame("debug_rottra_back_", debug_frame++);
    //            deque<mpcard>::iterator vi0 = vinx0.begin(), vi1 = vinx1.begin();
    //            for (; vi0 != vinx0.end(); ++vi0, ++vi1) {
    //                Vector3f const point0_ref = reference.get3f(*vi0, refAAT);
    //                Vector3f const point1_ref = reference.get3f(*vi1, refAAT);
    //                Matrix4f T4 = IDENTITY4F;
    //                for (int j = 0; j < diff; ++j) {
    //                    T4 *= Tb;
    //                }
    //                receiver.AddSymmetryConstraint(point0_ref, point1_ref,
    //                    shrink4To3(T4), shrink4To3(T4[3]), weight);

    //                //Vector3f const point0_def = deformed.get3f(*vi0, defAAT);
    //                //Vector3f const point1_def = deformed.get3f(*vi1, defAAT);
    //                //Vector3f const point0_def_r = transformVector3f(
    //                //    expand3To4(shrink4To3(T4)), point0_def) ;
    //                //Vector3f const point0_def_t = transformVector3f(
    //                //    makeTranslation4f(shrink4To3(T4[3])), point0_def) ;
    //                //debugRenderer->addLine(
    //                //    point0_def, point0_def_r,
    //                //    makeVector3f(1, 0, 0),
    //                //    makeVector3f(0, 0, 1),
    //                //    3);
    //                //debugRenderer->addLine(
    //                //    point0_def_r, point0_def_t,
    //                //    makeVector3f(1, 0, 0),
    //                //    makeVector3f(0, 0, 1),
    //                //    3);
    //                //debugRenderer->addLine(
    //                //    transformVector3f(T4, point0_def), point1_def,
    //                //    makeVector3f(0, 1, 0),
    //                //    makeVector3f(0, 1, 0),
    //                //    3);
    //            }
    //            //debugRenderer->endRenderJob();
    //        }
    //    }
    //}

    //void BuildRotTra_Step(
    //    SymmContainer::Ptr const &tsymm,
    //    PointSet const &deformed, AAT const& defAAT,
    //    PointSet const &reference, AAT const& refAAT,
    //    TPSSolver &receiver,
    //    float32 const weight,
    //    bool const &useFeature) 
    //{
    //    Matrix4f Tf, Tb;
    //    Tf = tsymm->Get1StepTransformation();
    //    Tb = makeTranslation4f(-shrink4To3(Tf[3]));

    //    int diff_r = 0;
    //    {
    //        OrbitVerticeMap::iterator mi, mj = tsymm->overt_.begin();
    //        for (mi = mj++; mj != tsymm->overt_.end(); ++mi, ++mj) {
    //            deque<mpcard> vinx0 = mi->second, vinx1 = mj->second;
    //            int diff = abs(mj->first - mi->first);
    //            diff_r += diff;
    //            deque<mpcard>::iterator vi0 = vinx0.begin(), vi1 = vinx1.begin();
    //            for (; vi0 != vinx0.end(); ++vi0, ++vi1) {
    //                Vector3f const point0_ref = reference.get3f(*vi0, refAAT);
    //                Vector3f const point1_ref = reference.get3f(*vi1, refAAT);
    //                Matrix4f T4 = IDENTITY4F;
    //                for (int j = 0; j < diff; ++j) {
    //                    T4 *= Tf;
    //                }
    //                receiver.AddSymmetryConstraint(point0_ref, point1_ref,
    //                    shrink4To3(T4), shrink4To3(T4[3]), weight);
    //            }
    //        }
    //        {
    //            OrbitVerticeMap::reverse_iterator mi, mj = tsymm->overt_.rbegin();
    //            for (mi = mj++; mj != tsymm->overt_.rend(); ++mi, ++mj) {
    //                deque<mpcard> vinx0 = mi->second, vinx1 = mj->second;
    //                int diff = abs(mj->first - mi->first);
    //                deque<mpcard>::iterator vi0 = vinx0.begin(), vi1 = vinx1.begin();
    //                for (; vi0 != vinx0.end(); ++vi0, ++vi1) {
    //                    Vector3f const point0_ref = reference.get3f(*vi0, refAAT);
    //                    Vector3f const point1_ref = reference.get3f(*vi1, refAAT);
    //                    Matrix4f T4 = IDENTITY4F;
    //                    for (int j = 0; j < diff; ++j) {
    //                        T4 *= Tb;
    //                    }
    //                    receiver.AddSymmetryConstraint(point0_ref, point1_ref,
    //                        shrink4To3(T4), shrink4To3(T4[3]), weight);
    //                }
    //            }
    //        }
    //    }
    //}

    //void BuildRefAdd(
    //    mpcard vindx0, mpcard vindx1,
    //    PointSet const &reference, AAT const& refAAT,
    //    PointSet const &deformed, AAT const& defAAT,
    //    float32 const weight,
    //    SymmReflective::Ptr tsymm_r, TPSSolver &receiver)
    //{
    //    Matrix4f Tf, Tb;
    //    if (!tsymm_r->GetUpdatedTransformation(
    //        deformed, defAAT, Tf, Tb))
    //        return;
    //    Matrix4f& T4 = Tf;

    //    Vector3f const point0_ref = reference.get3f(vindx0, refAAT);
    //    Vector3f const point1_ref = reference.get3f(vindx1, refAAT);
    //    Vector3f const point0_def = deformed.get3f(vindx0, defAAT);
    //    Vector3f const point1_def = tsymm_r->GetReflectedPos(point0_def);

    //    Vector3f const translation = shrink4To3(T4[3]);
    //    Matrix3f rotation = shrink4To3(T4);

    //    //debugRenderer->addLine(
    //    //    point0_ref, point1_ref,
    //    //    makeVector3f(1, 0, 0),
    //    //    makeVector3f(0, 0, 1),
    //    //    3);
    //    //debugRenderer->addLine(
    //    //    point0_def, point1_def,
    //    //    makeVector3f(1, 0, 0),
    //    //    makeVector3f(0, 0, 1),
    //    //    3);
    //    //debugRenderer->addLine(
    //    //    point0_ref, point0_def,
    //    //    makeVector3f(0, 1, 0),
    //    //    makeVector3f(0, 1, 0),
    //    //    3);
    //    //debugRenderer->addLine(
    //    //    point1_ref, point1_def,
    //    //    makeVector3f(0, 1, 0),
    //    //    makeVector3f(0, 1, 0),
    //    //    3);

    //    receiver.AddSymmetryConstraint(
    //        point0_ref, point1_ref,
    //        rotation, translation, weight);
    //}

    //void BuildRef(
    //    SymmContainer::Ptr const &tsymm,
    //    PointSet const &deformed, AAT const& defAAT,
    //    PointSet const &reference, AAT const& refAAT,
    //    TPSSolver &receiver,
    //    float32 const weight,
    //    bool const &useFeature) 
    //{
    //    SymmReflective::Ptr tsymm_r = 
    //        boost::shared_polymorphic_downcast<SymmReflective>(tsymm);
    //    if (NULL == tsymm_r) {
    //        warning("BuidRef: SymmReflective");
    //        return;
    //    }

    //    debugRenderer->beginRenderJob_OneFrame("debug_", 0);

    //    if (useFeature) {
    //        vector<SymmFeatLinePair>& symmFeatPairVec = tsymm_r->featPairs_;
    //        vector<LineDir>& lineDir_ = tsymm_r->lineDir_;
    //        for (size_t pi = 0; pi < symmFeatPairVec.size(); ++pi) {
    //            SymmFeatLinePair& symmFeatPair = symmFeatPairVec[pi];
    //            {
    //                mpcard vindx0 = symmFeatPair.first->v0;
    //                mpcard vindx1 = (SAME_DIR == lineDir_[pi]) ?
    //                    symmFeatPair.second->v0 :
    //                symmFeatPair.second->v1;

    //                BuildRefAdd(
    //                    vindx0, vindx1,
    //                    reference, refAAT,
    //                    deformed, defAAT, weight,
    //                    tsymm_r, receiver);
    //            }
    //            {
    //                mpcard vindx0 = symmFeatPair.first->v1;
    //                mpcard vindx1 = (SAME_DIR == lineDir_[pi]) ?
    //                    symmFeatPair.second->v1 :
    //                symmFeatPair.second->v0;

    //                BuildRefAdd(
    //                    vindx0, vindx1,
    //                    reference, refAAT,
    //                    deformed, defAAT, weight,
    //                    tsymm_r, receiver);
    //            }
    //        }
    //    } else {
    //        vector<SymmFeatVertexPair>& vertpairs = tsymm_r->vertPairs_;
    //        for (size_t pi = 0; pi < vertpairs.size(); ++pi) {
    //            mpcard vindx0 = vertpairs[pi].first;
    //            mpcard vindx1 = vertpairs[pi].second;

    //            BuildRefAdd(
    //                vindx0, vindx1,
    //                reference, refAAT,
    //                deformed, defAAT, weight,
    //                tsymm_r, receiver);
    //        }
    //    }

    //    debugRenderer->endRenderJob();

    //}

//======================================================================
// Constructors
//----------------------------------------------------------------------

//======================================================================
// Accessors
//----------------------------------------------------------------------
std::vector<CICPRotation> const& SymmetryConstraints::Rotations() const
{
  return rotations_;
}

//----------------------------------------------------------------------
std::vector<CICPReflection> const& SymmetryConstraints::Reflections() const
{
  return reflections_;
}

//----------------------------------------------------------------------
std::vector<CICPLatticeElement> const& SymmetryConstraints::LatticeElements() const
{
  return latticeElements_;
}

//----------------------------------------------------------------------
std::vector<CICPFeatureLine> const& SymmetryConstraints::FeatureLines() const
{
  return featureLines_;
}

//----------------------------------------------------------------------
std::vector<CICPLattice> const& SymmetryConstraints::Lattices() const
{
  return lattices_;
}

//======================================================================
// Mutators
//----------------------------------------------------------------------
std::vector<CICPRotation>& SymmetryConstraints::Rotations()
{
  return rotations_;
}

//----------------------------------------------------------------------
std::vector<CICPReflection>& SymmetryConstraints::Reflections()
{
  return reflections_;
}

//----------------------------------------------------------------------
std::vector<CICPLatticeElement>& SymmetryConstraints::LatticeElements()
{
  return latticeElements_;
}

//----------------------------------------------------------------------
std::vector<CICPFeatureLine>& SymmetryConstraints::FeatureLines()
{
  return featureLines_;
}

//----------------------------------------------------------------------
std::vector<CICPLattice>& SymmetryConstraints::Lattices()
{
  return lattices_;
}

//======================================================================
// Member functions
//----------------------------------------------------------------------
void SymmetryConstraints::Clear()
{
        rotations_.clear();
      reflections_.clear();
  latticeElements_.clear();
     featureLines_.clear();
         lattices_.clear();
}

//----------------------------------------------------------------------
void SymmetryConstraints::Build(TPSSolver& receiver, float32 const weight)
{
  for (mpcard i = 0; i < rotations_.size(); ++i)
  {
    rotations_.at(i).Build(receiver, weight);
  }

  for (mpcard i = 0; i < reflections_.size(); ++i)
  {
    reflections_.at(i).Build(receiver, weight);
  }

  for (mpcard i = 0; i < latticeElements_.size(); ++i)
  {
    //latticeElements_.at(i).Build(receiver, weight);
  }

  for (mpcard i = 0; i < lattices_.size(); ++i)
  {
    lattices_.at(i).Build(receiver, weight);
  }
}

//----------------------------------------------------------------------
void SymmetryConstraints::BuildFeatureLines(
  TPSSolver& receiver, float32 const weight)
{
  for (mpcard i = 0; i < featureLines_.size(); ++i)
  {
    featureLines_.at(i).Build(receiver, weight);
  }
}
  
//----------------------------------------------------------------------
void SymmetryConstraints::Update(TPSSolver& evaluator)
{
  for (mpcard i = 0; i < rotations_.size(); ++i)
  {
    rotations_.at(i).Update(evaluator);
  }

  for (mpcard i = 0; i < reflections_.size(); ++i)
  {
    reflections_.at(i).Update(evaluator);
  }

  for (mpcard i = 0; i < latticeElements_.size(); ++i)
  {
    latticeElements_.at(i).Update(evaluator);
  }

  for (mpcard i = 0; i < featureLines_.size(); ++i)
  {
    featureLines_.at(i).Update(evaluator);
  }

  for (mpcard i = 0; i < lattices_.size(); ++i)
  {
    lattices_.at(i).Update(evaluator);
  }
}

//----------------------------------------------------------------------
void SymmetryConstraints::Reset()
{
  for (mpcard i = 0; i < rotations_.size(); ++i)
  {
    rotations_.at(i).Reset();
  }

  for (mpcard i = 0; i < reflections_.size(); ++i)
  {
    reflections_.at(i).Reset();
  }

  for (mpcard i = 0; i < latticeElements_.size(); ++i)
  {
    latticeElements_.at(i).Reset();
  }

  for (mpcard i = 0; i < featureLines_.size(); ++i)
  {
    featureLines_.at(i).Reset();
  }

  for (mpcard i = 0; i < lattices_.size(); ++i)
  {
    lattices_.at(i).Reset();
  }
}

//----------------------------------------------------------------------
//void SymmetryConstraints::BuildCoPlaneConstraints(
//                             TPSSolver      & receiver,
//                             PointSet  const& reference,
//                             PointSet  const& deformed,
//                             float32   const  weight)
//{
//    string const id = "co-planarity_constraint_";
//    debugRenderer->clearRenderJob_AllFrames(id);
//
//    int debug_frame = 10;
//    AAT const refAAT = reference.getAAT("position");
//    AAT const defAAT = deformed.getAAT("position");
//    card32 enforced = 0;
//    TrimeshStatic::Ptr smesh = symmDetHiPrec_->strimesh;
//    vector<SymmPlane::Ptr>& copsymmvec = symmDetHiPrec_->copsymmvec;
//    for (size_t ri = 0; ri < copsymmvec.size(); ++ri) {
//
//      SymmPlane::Ptr tsymm = copsymmvec[ri];
//        if (!tsymm->show_) continue;
//        ++enforced;
//
//        BuildCoPlane(tsymm,
//            deformed, defAAT, reference, refAAT,
//            receiver, weight, false);
//
//        debugRenderer->beginRenderJob_OneFrame(id, debug_frame++);
//        tsymm->DrawWithDR(makeVector3f(1, 1, 0));
//        debugRenderer->endRenderJob();
//    }
//    debugOutput << "Enforcing " << enforced << " of "
//        << copsymmvec.size()  << " symmetry constraints.\n";
//}

//----------------------------------------------------------------------
//void SymmetryConstraints::BuildFeatLineConstraints(
//    TPSSolver      & receiver,
//    PointSet  const& reference,
//    PointSet  const& deformed,
//    float32   const  weight)
//{
//    int debug_frame = 10;
//    AAT const refAAT = reference.getAAT("position");
//    AAT const defAAT = deformed.getAAT("position");
//
//    vector<SymmFeatLine::Ptr> featLineVec = symmDetHiPrec_->lineFeatures;
//    for (size_t li = 0; li < featLineVec.size(); ++li) {
//        deque<mpcard> vertice = featLineVec[li]->vertice;
//
//        int cnt = 0;
//        Vector3f trans = NULL_VECTOR3F;
//        {
//            deque<mpcard>::iterator vi, vj = vertice.begin();
//            for (vi = vj++; vj != vertice.end(); ++vi, ++vj, ++cnt) {
//                Vector3f step =
//                    deformed.get3f(*vj, defAAT) -
//                    deformed.get3f(*vi, defAAT);
//                trans = trans + step;
//            }
//            trans = trans / cnt;
//        }
//
//        {
//            deque<mpcard>::iterator vi, vj = vertice.begin();
//            for (vi = vj++; vj != vertice.end(); ++vi, ++vj) {
//                Vector3f const point0_ref = reference.get3f(*vi, refAAT);
//                Vector3f const point1_ref = reference.get3f(*vj, refAAT);
//                receiver.AddSymmetryConstraint(point0_ref, point1_ref,
//                    IDENTITY3F, trans, weight);
//            }
//        }
//
//        trans = -trans;
//        {
//            deque<mpcard>::reverse_iterator vi, vj = vertice.rbegin();
//            for (vi = vj++; vj != vertice.rend(); ++vi, ++vj) {
//                Vector3f const point0_ref = reference.get3f(*vi, refAAT);
//                Vector3f const point1_ref = reference.get3f(*vj, refAAT);
//                receiver.AddSymmetryConstraint(point0_ref, point1_ref,
//                    IDENTITY3F, trans, weight);
//            }
//        }
//    }
//}

//----------------------------------------------------------------------
void SymmetryConstraints::BuildConstraintsYZ(
  TPSSolver      & receiver,
  PointSet  const& reference,
  float32   const  weight)
{
  AAT const positionAAT = reference.getAAT("position");

  if (positionAAT == NULL_AAT)
  {
    warning("SymmetryConstraints::BuildSymmetryConstraints() - "
            "\"position\" attribute missing from reference.");

    return;
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  card32 counter  = 0;
  card32 enforced = 0;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (card32 i = 0; i < reference.getNumEntries(); ++i)
  {
    ++counter;

    Vector3f const point = reference.get3f(i, positionAAT);

    //if (     point[0] != 0.5f) { continue; }
    //if (fabs(point[0]) < 0.1f) { continue; }

    ++enforced;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    Vector3f const mirrorPoint =
      makeVector3f(-point[0], point[1], point[2]);

    Vector3f const translation = NULL_VECTOR3F;
    //Vector3f const translation = makeVector3f(-0.3f, 0.5f, -0.7f);

    Matrix3f rotation = IDENTITY3F;

    rotation[0][0] = -1.0f;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    receiver.AddSymmetryConstraint(point, mirrorPoint,
                                   rotation, translation, weight);
  }

  debugOutput << "Enforcing " << enforced << " of "
                              << counter  << " symmetry constraints.\n";
}

//----------------------------------------------------------------------
void SymmetryConstraints::BuildConstraintsRTTranslation(
  TPSSolver& receiver, PointSet const& reference, float32 const weight) 
{
  AAT const positionAAT = reference.getAAT("position");

  if (positionAAT == NULL_AAT)
  {
    warning("SymmetryConstraints::BuildSymmetryConstraints() - "
            "\"position\" attribute missing from reference.");

    return;
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //Vector3f const translation = makeVector3f(1.5f, 0.0f, 0.0f);
  //for cube low tess:
  Vector3f const translation = makeVector3f(2.0f, 0.0f, 0.0f);

  Matrix3f const rotation = IDENTITY3F;

    for (size_t i = 0; i < reference.getNumEntries(); ++i)
    {
        Vector3f const& point0 = reference.get3f(i, positionAAT);


        if (point0[0] > 0.6f)
        {
          continue;
          
          Vector3f const point1 = point0 - translation;

            receiver.AddSymmetryConstraint(point0, point1,
                rotation, -translation, weight);

            // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            Matrix4f T4 = makeTranslation4f(-translation);
        }
        else
        {
            Vector3f const point1 = rotation * point0 + translation;

            receiver.AddSymmetryConstraint(point0, point1,
                rotation, translation, weight);
        }
    }

    return;
}

//----------------------------------------------------------------------
void SymmetryConstraints::BuildConstraintsRTRotation(
  TPSSolver& receiver, PointSet const& reference, float32 const weight) 
{
  AAT const positionAAT = reference.getAAT("position");

  if (positionAAT == NULL_AAT)
  {
    warning("SymmetryConstraints::BuildSymmetryConstraints() - "
            "\"position\" attribute missing from reference.");

    return;
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


  //Vector3f const translation = makeVector3f(1.5f, 0.3f, -0.2f);
  //Matrix3f rotation;
  //rotation[0][0] =  0.558876; rotation[1][0] =  0.129027; rotation[2][0] = 0.819152;
  //rotation[0][1] = -0.562789; rotation[1][1] =  0.784514; rotation[2][1] = 0.260398;
  //rotation[0][2] = -0.609038; rotation[1][2] = -0.606540; rotation[2][2] = 0.511060;

  //Vector3f const translation = NULL_VECTOR3F;
  //Matrix3f rotation;
  //rotation[0][0] =  0.707107; rotation[1][0] = 0.0; rotation[2][0] = 0.707107;
  //rotation[0][1] =  0.0     ; rotation[1][1] = 1.0; rotation[2][1] = 0.0     ;
  //rotation[0][2] = -0.707107; rotation[1][2] = 0.0; rotation[2][2] = 0.707107;

  Vector3f const translation = makeVector3f(1.5f, 0.3f, -0.2f);
  Matrix3f rotation;
  rotation[0][0] =  0.527980; rotation[1][0] =  0.224114; rotation[2][0] = 0.819152;
  rotation[0][1] = -0.594116; rotation[1][1] =  0.786704; rotation[2][1] = 0.167698;
  rotation[0][2] = -0.606847; rotation[1][2] = -0.575212; rotation[2][2] = 0.548514;

//#pragma omp parallel for
    for (int i = 0; i < reference.getNumEntries(); ++i)
    {
        Vector3f const& point0 = reference.get3f(i, positionAAT);


        if (point0[0] > 0.51f)
//        if (point0[2] < -0.51f)
        {
          continue;
        }
        else
        {
            Vector3f const point1 = rotation * point0 + translation;

            receiver.AddSymmetryConstraint(point0, point1,
                rotation, translation, weight);
        }
    }

    return;
}

} //namespace X4
