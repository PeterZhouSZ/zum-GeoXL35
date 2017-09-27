#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "SToolBox/DefieldSymm.h"
#include "SToolBox/DefieldSymmRotation.h"
#include "SToolBox/DefieldSymmDihedral.h"
//---------------------------------------------------------------------------
#include "GeometricTools.h"
#include "PropertyTableProperty.h"
#include "CopyObjectProperties.h"

#include <group/Lattice.h>
#include <group/Rotation.h>
#include <group/Reflection.h>
#include <group/Dihedral.h>
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

IMPLEMENT_ABSTRACT_CLASS( DefieldSymm, 0 ) 
{
    BEGIN_CLASS_INIT( DefieldSymm );
}
IMPLEMENT_CLASS( DefieldSymmVec, 0 ) 
{
    BEGIN_CLASS_INIT( DefieldSymmVec );
    INIT_PROPERTY_TABLE();
    //ADD_OBJECT_LIST_PROP( symmvec_, 0, DefieldSymm::getClass() );
}
IMPLEMENT_CLASS( DefieldSymmReflection, 0 ) 
{
    BEGIN_CLASS_INIT( DefieldSymmReflection );
    INIT_PROPERTY_TABLE();
    //ADD_OBJECT_PROP( plane_, 0, Plane::getClass(), true );
}

//============================================================================
void DefieldSymm::assign( const Object* obj, COPY_CONTEXT *context ) 
{
    const DefieldSymm *other = dynamic_cast<const DefieldSymm*>(obj);
    if (other) {
        copyObjectProperties( obj, this );
    } else {
        pAssert(false);
    }
}

void DefieldSymm::read( InputObjectStream *s )
{
    AttachedData::read(s);
    s->read(rank_);
    s->read(numtrans_);
    s->read(trans_);
    s->read(show_);

    {
        mpcard mapsize;
        s->readMaxPlatformCard(mapsize);
        for (unsigned mi = 0; mi < mapsize; ++mi) {
            int32 indx;
            s->read(indx);
            std::deque< card32 > list;
            mpcard listsize;
            s->readMaxPlatformCard(listsize); 
            for (unsigned li = 0; li < listsize; ++li) {
                card32 ordx;
                s->read(ordx);
                list.push_back(ordx);
            }
            overt_[indx] = list;
        }
    }

    {
        mpcard mapsize;
        s->readMaxPlatformCard(mapsize);
        for (unsigned mi = 0; mi < mapsize; ++mi) {
            std::deque< card32 > list;
            mpcard listsize;
            s->readMaxPlatformCard(listsize); 
            for (unsigned li = 0; li < listsize; ++li) {
                card32 ordx;
                s->read(ordx);
                list.push_back(ordx);
            }
            stack_.push_back(list);
        }
    }

    if (overt_.empty()) return;
}

void DefieldSymm::write( OutputObjectStream *s ) const
{
    AttachedData::write(s);
    s->write(rank_);
    s->write(numtrans_);
    s->write(trans_);
    s->write(show_);

    {
        s->writeMaxPlatformCard(overt_.size());
        for (OrbitVerticeMap::const_iterator it = overt_.begin(); it != overt_.end(); ++it) {
            s->write(it->first);
            const std::deque< card32 >& list = it->second;
            s->writeMaxPlatformCard(list.size());
            for (unsigned li = 0; li < list.size(); ++li) {
                s->write(list[li]);
            }
        }
    }

    {
        s->writeMaxPlatformCard(stack_.size());
        for (OrbitStack::const_iterator it = stack_.begin(); it != stack_.end(); ++it) {
            const std::deque< card32 >& list = *it;
            s->writeMaxPlatformCard(list.size());
            for (unsigned li = 0; li < list.size(); ++li) {
                s->write(list[li]);
            }
        }
    }

    if (overt_.empty()) return;
}

void DefieldSymmVec::assign( const Object* obj, COPY_CONTEXT *context ) 
{
    const DefieldSymmVec *other = dynamic_cast<const DefieldSymmVec*>(obj);
    if (other) {
        copyObjectProperties( obj, this );
    } else {
        pAssert(false);
    }
}

void DefieldSymmVec::read( InputObjectStream *s )
{
    AttachedData::read(s);

    mpcard mapsize;
    s->readMaxPlatformCard(mapsize);
    symmvec_.resize(mapsize);
    for (unsigned mi = 0; mi < mapsize; ++mi) {
        std::string className;
        s->read(className);
        if ("Reflection" == className) {
            DefieldSymmReflection* sym = new DefieldSymmReflection();
            sym->read(s);
            symmvec_[mi] = sym;
        } else if ("Rotation" == className) {
            DefieldSymmRotation* sym = new DefieldSymmRotation();
            sym->read(s);
            symmvec_[mi] = sym;
        } else if ("RotationV" == className) {
            DefieldSymmRotationV* sym = new DefieldSymmRotationV();
            sym->read(s);
            symmvec_[mi] = sym;
        } else if ("RotationH" == className) {
            DefieldSymmRotationH* sym = new DefieldSymmRotationH();
            sym->read(s);
            symmvec_[mi] = sym;
        } else if ("Dihedral" == className) {
            DefieldSymmDihedral* sym = new DefieldSymmDihedral();
            sym->read(s);
            symmvec_[mi] = sym;
        } else if ("DihedralH" == className) {
            DefieldSymmDihedralH* sym = new DefieldSymmDihedralH();
            sym->read(s);
            symmvec_[mi] = sym;
        } else { error("DefieldSymmVec::read - unknown symmetry group type"); return; }
    }
}

void DefieldSymmVec::write( OutputObjectStream *s ) const
{
    AttachedData::write(s);

    s->writeMaxPlatformCard(symmvec_.size());
    for (unsigned mi = 0; mi < symmvec_.size(); ++mi) {
        s->write(symmvec_[mi]->GetClassName());
        symmvec_[mi]->write(s);
    }
}

void DefieldSymmReflection::assign( const Object* obj, COPY_CONTEXT *context ) 
{
    const DefieldSymmReflection *other = dynamic_cast<const DefieldSymmReflection*>(obj);
    if (other) {
        copyObjectProperties( obj, this );
    } else {
        pAssert(false);
    }
}

void DefieldSymmReflection::read( InputObjectStream *s )
{
    DefieldSymm::read(s);
    plane_.read(s);
}

void DefieldSymmReflection::write( OutputObjectStream *s ) const
{
    DefieldSymm::write(s);
    plane_.write(s);
}

//============================================================================
std::string DefieldSymm::GetDescription(void)
{
    std::stringstream ss;
    ss << GetClassName() << " [" << overt_.size() << "]";
    return ss.str();
}

unsigned DefieldSymm::GetNumSubSymm(void) const
{
    return 1;
}

DefieldSymm* DefieldSymm::GetSubSymm(unsigned subs)
{
    return (0 == subs) ? this : nullptr;
}

int DefieldSymm::CreateFromSG(sym::SymmetryGroup* group, DefieldSymm::Ptr& symm)
{
    DefieldSymm* defSymm;
    int ret = DefieldSymm::CreateFromSG(group, &defSymm);
    if (0 > ret) {
        return ret;
    }
    symm.reset(defSymm);
    return ret;
}

int DefieldSymm::CreateFromSG(sym::SymmetryGroup* group, DefieldSymm** symm)
{
    sym::Reflection* reflection = dynamic_cast<sym::Reflection*>(group);
    sym::Rotation*   rotation   = dynamic_cast<sym::Rotation*  >(group);
    sym::Dihedral*   dihedral   = dynamic_cast<sym::Dihedral*  >(group);
    sym::Lattice*    lattice    = dynamic_cast<sym::Lattice*   >(group);    

    if (reflection)
    {
        DefieldSymmReflection* sym_t (new DefieldSymmReflection);
        sym_t->trans_ = reflection->getGenerator()->getWorldTransformation(1);
        sym_t->plane_ = reflection->getGenerator()->getPlane();
        *symm = sym_t;
        return 0;
    } else if (rotation)
    {
        //Matrix3f frame;
        //{
        //    frame[2] = axis;
        //    float value = 2;
        //    unsigned min_d = 0;
        //    for (unsigned d = 0; d < 3; ++d) {
        //        const float value_d = abs(frame[2][d]);
        //        if (value_d > value) continue;
        //        value = value_d;
        //        min_d = d;
        //    }
        //    Vector3f axis_min = NULL_VECTOR3F;
        //    axis_min[min_d] = 1.f;
        //    frame[0] = axis_min;
        //    frame[1] = normalize(frame[2].crossProduct(frame[0]));
        //    frame[0] = normalize(frame[1].crossProduct(frame[2]));
        //}

        //Matrix4f Tf;
        //{
        //    sym::RotationGenerator* gen = rotation->getGenerator();
        //    float angleVelocity = 2.f * M_PI / float(rotation->getNumberRotations());
        //    Matrix4f R = makeRotZ4f(angleVelocity);

        //    //const Matrix4f& T = expand3To4(gen->getFrame());
        //    const Matrix4f& T = expand3To4(frame);
        //    Tf =
        //        makeTranslation4f(center)
        //        * T 
        //        * R
        //        * invertFrame(T) 
        //        * makeTranslation4f(-center);
        //}

        const unsigned& numTrans = rotation->getNumberRotations();
        const Matrix4f& T = rotation->getGenerator()->getWorldTransformation(1);
        const Vector3f& center = rotation->getGenerator()->getRotationAxisCenter();
        const Vector3f& axis = normalize(rotation->getGenerator()->getRotationAxis());
        Matrix4f Tb;
        {
            sym::RotationGenerator* gen = rotation->getGenerator();
            float angleVelocity = 2.f * M_PI / float(rotation->getNumberRotations());
            Matrix4f R = makeRotZ4f(angleVelocity * -1.f);

            const Matrix4f& T = expand3To4(gen->getFrame());
            Tb =
                makeTranslation4f(center) 
                * T 
                * R
                * invertFrame(T) 
                * makeTranslation4f(-center);
        }


        sym::RotationH* rotationH = dynamic_cast<sym::RotationH*>(rotation);
        sym::RotationV* rotationV = dynamic_cast<sym::RotationV*>(rotation);

        if (rotationH)
        {
            DefieldSymmRotationH* sym_t (new DefieldSymmRotationH);
            sym_t->numtrans_ = rotation->getNumberRotations();
            sym_t->trans_ = rotation->getGenerator()->getWorldTransformation(1);
            sym_t->center_ = rotation->getGenerator()->getRotationAxisCenter();
            sym_t->axis_ = normalize(rotation->getGenerator()->getRotationAxis());
            sym_t->trans_b_ = Tb;

            sym::Reflection* refh = dynamic_cast<sym::Reflection*>(rotationH->getReflection());
            sym_t->reflection_h_ = new DefieldSymmReflection;
            sym_t->reflection_h_->trans_ = refh->getGenerator()->getWorldTransformation(1);
            sym_t->reflection_h_->plane_ = refh->getGenerator()->getPlane();

            *symm = sym_t;
        } else if (rotationV)
        {
            DefieldSymmRotationV* sym_t (new DefieldSymmRotationV);
            sym_t->numtrans_ = rotation->getNumberRotations();
            sym_t->trans_ = rotation->getGenerator()->getWorldTransformation(1);
            sym_t->center_ = rotation->getGenerator()->getRotationAxisCenter();
            sym_t->axis_ = normalize(rotation->getGenerator()->getRotationAxis());
            sym_t->trans_b_ = Tb;

            unsigned nref = rotationV->getNumReflections();
            sym_t->reflections_.resize(nref);
            for (unsigned ii = 0; ii < nref; ++ii) {
                sym_t->reflections_[ii] = new DefieldSymmReflection;
                sym::Reflection* refv = dynamic_cast<sym::Reflection*>(rotationV->getReflection(ii));
                sym_t->reflections_[ii]->trans_ = refv->getGenerator()->getWorldTransformation(1);
                sym_t->reflections_[ii]->plane_ = refv->getGenerator()->getPlane();
            }

            *symm = sym_t;
        } else
        {
            DefieldSymmRotation* sym_t (new DefieldSymmRotation);
            sym_t->numtrans_ = rotation->getNumberRotations();
            sym_t->trans_ = rotation->getGenerator()->getWorldTransformation(1);
            sym_t->center_ = rotation->getGenerator()->getRotationAxisCenter();
            sym_t->axis_ = normalize(rotation->getGenerator()->getRotationAxis());
            sym_t->trans_b_ = Tb;

            *symm = sym_t;
        }

        (*symm)->rank_ = (*symm)->rank_ * numTrans;

        return 0;
    } else if (dihedral)
    {
        sym::DihedralH* dihedralH = dynamic_cast<sym::DihedralH*>(dihedral);

        std::string rot_type = dihedral->getBaseRotation()->getType();
        sym::RotationV* rotv = dynamic_cast<sym::RotationV*>(dihedral->getBaseRotation());
        if (!rotv) {
            //warning("DefieldSymm::CreateFromSG - this Dihedral's base rotation is not RotationV");
            return -1;
        }
        Matrix4f Tb;
        {
            sym::RotationGenerator* gen = rotv->getGenerator();
            float angleVelocity = 2.f * M_PI / float(rotv->getNumberRotations());
            Matrix4f R = makeRotZ4f(angleVelocity * -1.f);

            const Matrix4f& T = expand3To4(gen->getFrame());
            Tb =
                makeTranslation4f(gen->getRotationAxisCenter()) 
                * T 
                * R
                * invertFrame(T) 
                * makeTranslation4f(-gen->getRotationAxisCenter());
        }
        unsigned nrot = rotv->getNumberRotations();
        unsigned nref = rotv->getNumReflections();
        if (dihedralH)
        {
            DefieldSymmDihedralH* sym_t (new DefieldSymmDihedralH);
            sym_t->numtrans_ = nrot;
            sym_t->trans_ = rotv->getGenerator()->getWorldTransformation(1);
            sym_t->trans_b_ = Tb;
            sym_t->center_ = rotv->getGenerator()->getRotationAxisCenter();
            sym_t->reflections_.resize(nref);
            for (unsigned ii = 0; ii < nref; ++ii) {
                sym_t->reflections_[ii] = new DefieldSymmReflection;
                sym::Reflection* refv = dynamic_cast<sym::Reflection*>(rotv->getReflection(ii));
                sym_t->reflections_[ii]->trans_ = refv->getGenerator()->getWorldTransformation(1);
                sym_t->reflections_[ii]->plane_ = refv->getGenerator()->getPlane();
            }

            sym::Reflection* refh = dynamic_cast<sym::Reflection*>(dihedralH->getReflection());
            sym_t->reflection_h_ = new DefieldSymmReflection;
            sym_t->reflection_h_->trans_ = refh->getGenerator()->getWorldTransformation(1);
            sym_t->reflection_h_->plane_ = refh->getGenerator()->getPlane();
            sym_t->center_ = sym_t->reflection_h_->plane_.projectPointOntoPlane(sym_t->center_);

            *symm = sym_t;
        } else
        {
            DefieldSymmDihedral* sym_t (new DefieldSymmDihedral);
            sym_t->numtrans_ = nrot;
            sym_t->trans_ = rotv->getGenerator()->getWorldTransformation(1);
            sym_t->trans_b_ = Tb;
            sym_t->center_ = rotv->getGenerator()->getRotationAxisCenter();
            sym_t->reflections_.resize(nref);
            for (unsigned ii = 0; ii < nref; ++ii) {
                sym_t->reflections_[ii] = new DefieldSymmReflection;
                sym::Reflection* refv = dynamic_cast<sym::Reflection*>(rotv->getReflection(ii));
                sym_t->reflections_[ii]->trans_ = refv->getGenerator()->getWorldTransformation(1);
                sym_t->reflections_[ii]->plane_ = refv->getGenerator()->getPlane();
            }

            *symm = sym_t;
        }
        (*symm)->numtrans_ = dihedral->getNumRotations();
        if (nrot != (*symm)->numtrans_ || nref != (*symm)->numtrans_) {
            error("CreateFromSG: incorrect dihedral");
            return -1;
        }

        (*symm)->rank_ = (*symm)->rank_ * nrot;

        return 0;
    } else
    {
        debugOutput << group->getName() << " is not supported now\n";
        return -1;
    }
}

Matrix4f DefieldSymm::GetTransformation(const int& ii) const
{
    int orx = ii % (int)DefieldSymm::GetNumTransformation();
    if (orx < 0) orx += (int)DefieldSymm::GetNumTransformation();

    Matrix4f T = IDENTITY4F;
    for (int k = 0; k < orx; ++k) T *= trans_;
    return T;
}

int DefieldSymm::AddOrbitIndice(const std::deque<unsigned>& orbitIndice, const std::deque<Vector3f>& orbitPosition)
{
    const unsigned numtrans = DefieldSymm::GetNumTransformation();
    if (numtrans != orbitIndice.size()) {
        error("DefieldSymm::AddOrbitIndice - orbit mis-match");
        return -1;
    }

    for (unsigned tt = 0; tt < numtrans; ++tt) {
        overt_[tt].push_back(orbitIndice[tt]);
    }
    stack_.push_back(orbitIndice);
    return 0;
}

Matrix4f DefieldSymm::GetTransformation(int source, int target) const
{
    return GetTransformation(target - source);
}

std::deque<OrbitPairTransformation> DefieldSymm::GetOrbitPairTransformations(const unsigned& orx) const
{
    std::deque<OrbitPairTransformation> ret;
    if (orx >= stack_.size()) return ret;
    const std::deque< unsigned >& orbit = stack_[orx];

    if (orbit.size() != numtrans_) { // possible for dihedral using its rotation base
        error("DefieldSymm::GetOrbitPairTransformations - incorrect orbit construction");
    }
    OrbitPairTransformation orbitPair;
    orbitPair.transformation = trans_;
    unsigned tt = 0;
    for (; tt < numtrans_-1; ++tt) {
        orbitPair.source.push_back(orbit[tt]);
        orbitPair.target.push_back(orbit[tt+1]);
    }
    orbitPair.source.push_back(orbit[tt]);
    orbitPair.target.push_back(orbit[0]);

    ret.push_back(orbitPair);

    return ret;
}

std::deque<OrbitPairTransformation> DefieldSymm::GetOrbitPairTransformations(void) const
{
    std::deque<OrbitPairTransformation> ret;
    for (unsigned orx = 0; orx < stack_.size(); ++orx) {
        std::deque<OrbitPairTransformation> orbit = GetOrbitPairTransformations(orx);
        std::copy(orbit.begin(), orbit.end(), std::back_inserter(ret));
    }
    return ret;
}

bool DefieldSymm::
    GetUpdatedTransformation(PointSet  const& deformed,
    AAT       const& defAAT,
    Matrix4f& Tf, Matrix4f& Tb)
{
    // (n-1)-(n-1) estimation: guaranteed to be series
    {
        //debugRenderer->beginRenderJob_OneFrame("update_transformation_", DR_FRAME++);
        std::vector<Vector3f> pts0, pts1;
        OrbitVerticeMap::iterator mi, mj = overt_.begin();
        for (mi = mj++; mj != overt_.end(); ++mi, ++mj) {
            std::deque<card32> vinx0 = mi->second, vinx1 = mj->second;
            if (vinx0.size() != vinx1.size()) {
                warning("SymmTranslation::GetUpdatedTransformation");
                continue;
            }
            std::deque<card32>::iterator vi0 = vinx0.begin(), vi1 = vinx1.begin();
            for (; vi0 != vinx0.end(); ++vi0, ++vi1) {
                pts0.push_back(deformed.get3f(*vi0, defAAT));
                pts1.push_back(deformed.get3f(*vi1, defAAT));
                //debugRenderer->addLine(
                //    pts0.back(), pts1.back(),
                //    makeVector3f(1, 0, 0),
                //    makeVector3f(0, 0, 1),
                //    1);
            }
        }
        //debugRenderer->endRenderJob();
        Tf = computeRigidTransformationFromCorrespondences(pts1, pts0);
    }
    {
        std::vector<Vector3f> pts0, pts1;
        OrbitVerticeMap::reverse_iterator mi, mj = overt_.rbegin();
        for (mi = mj++; mj != overt_.rend(); ++mi, ++mj) {
            std::deque<card32> vinx0 = mi->second, vinx1 = mj->second;
            if (vinx0.size() != vinx1.size()) {
                warning("SymmTranslation::GetUpdatedTransformation");
                continue;
            }
            std::deque<card32>::iterator vi0 = vinx0.begin(), vi1 = vinx1.begin();
            for (; vi0 != vinx0.end(); ++vi0, ++vi1) {
                pts0.push_back(deformed.get3f(*vi0, defAAT));
                pts1.push_back(deformed.get3f(*vi1, defAAT));
            }
        }
        Tb = computeRigidTransformationFromCorrespondences(pts1, pts0);
    }

    return true;
}

bool DefieldSymm::
    GetCorrespondence(
    std::vector<card32>& points0,
    std::vector<card32>& points1
    )
{
    OrbitVerticeMap::iterator mi, mj = overt_.begin();
    for (mi = mj++; mj != overt_.end(); ++mi, ++mj) {
        std::deque<card32> vinx0 = mi->second, vinx1 = mj->second;
        if (vinx0.size() != vinx1.size()) {
            warning("SymmTranslation::GetUpdatedTransformation");
            continue;
        }
        std::deque<card32>::iterator vi0 = vinx0.begin(), vi1 = vinx1.begin();
        for (; vi0 != vinx0.end(); ++vi0, ++vi1) {
            points0.push_back(*vi0);
            points1.push_back(*vi1);
        }
    }
    return true;
}

Vector3f DefieldSymm::GetNormalPosition(const Vector3f& pos, const Vector3f& ref)
{
    return pos;
}

float DefieldSymm::GetNormalDelta(const Vector3f& pos, const Vector3f& ref)
{
    const Vector3f& dirv = GetNormalDirection(ref, ref);
    const Vector3f& diff = pos - ref;
    return diff * dirv;
}

bool DefieldSymm::IsSeries(void)
{
    int cnt = 1;
    OrbitVerticeMap::iterator it, jt = overt_.begin();
    for (it = jt++; jt != overt_.end(); ++it, ++jt, ++cnt) {
        if (1 < jt->first - it->first) break;
    }
    return cnt == overt_.size();
}

void DefieldSymm::DrawWithDR(
    PointSet  const& deformed,
    AAT       const& defAAT
    )
{
    if (overt_.empty()) return;
    OrbitVerticeMap::iterator mi, mj = overt_.begin();
    for (mi = mj++; mj != overt_.end(); ++mi, ++mj) {
        std::deque<card32> vinx0 = mi->second, vinx1 = mj->second;
        std::deque<card32>::iterator vi0 = vinx0.begin(), vi1 = vinx1.begin();
        for (; vi0 != vinx0.end(); ++vi0, ++vi1) {
            Vector3f const point0_def = deformed.get3f(*vi0, defAAT);
            Vector3f const point1_def = deformed.get3f(*vi1, defAAT);
            debugRenderer->addLine(
                point0_def, point1_def,
                makeVector3f(1, 0, 0),
                makeVector3f(0, 0, 1),
                2);
        }
    }
}

//============================================================================
DefieldSymmReflection::DefieldSymmReflection(void)
{
    rank_ = REFLECTION;
    numtrans_ = 2;
    color_ = makeVector3f((float)255, (float)63, (float)13) / (float)255;
}

Vector3f DefieldSymmReflection::GetNormalDirection(const Vector3f& pos, const Vector3f& ref) const
{
    return normalize(pos - plane_.projectPointOntoPlane(ref));
}

bool DefieldSymmReflection::UpdatedTransformation(
    PointSet  const& deformed,
    AAT       const& defAAT)
{
    Matrix4f Tf, Tb;
    if (!DefieldSymm::GetUpdatedTransformation(deformed, defAAT, Tf, Tb)) return false;
    trans_ = Tf;
    return true;
}

std::deque<OrbitPairTransformation> DefieldSymmReflection::GetOrbitPairTransformations(const unsigned& orx) const
{
    std::deque<OrbitPairTransformation> ret;
    if (orx >= stack_.size()) return ret;
    const std::deque< unsigned >& orbit = stack_[orx];

    if (orbit.size() != numtrans_) { // possible for dihedral using its rotation base
        error("DefieldSymm::GetOrbitPairTransformations - incorrect orbit construction");
    }
    OrbitPairTransformation orbitPair;
    orbitPair.transformation = trans_;
    for (unsigned tt = 0; tt < numtrans_-1; ++tt) {
        orbitPair.source.push_back(orbit[tt]);
        orbitPair.target.push_back(orbit[tt+1]);
    }

    ret.push_back(orbitPair);

    return ret;
}

std::deque<OrbitPairTransformation> DefieldSymmReflection::GetOrbitPairTransformations(void) const
{
    return DefieldSymm::GetOrbitPairTransformations();
}
