#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "SToolBox/DefieldSymmRotation.h"
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

IMPLEMENT_CLASS( DefieldSymmRotation, 0 ) 
{
    BEGIN_CLASS_INIT( DefieldSymmRotation );
    INIT_PROPERTY_TABLE();
}

IMPLEMENT_CLASS( DefieldSymmRotationH, 0 ) 
{
    BEGIN_CLASS_INIT( DefieldSymmRotationH );
    INIT_PROPERTY_TABLE();
}

IMPLEMENT_CLASS( DefieldSymmRotationV, 0 ) 
{
    BEGIN_CLASS_INIT( DefieldSymmRotationV );
    INIT_PROPERTY_TABLE();
}

//============================================================================
void DefieldSymmRotationH::assign( const Object* obj, COPY_CONTEXT *context ) 
{
    const DefieldSymmRotationH *other = dynamic_cast<const DefieldSymmRotationH*>(obj);
    if (other) {
        copyObjectProperties( obj, this );
    } else {
        pAssert(false);
    }
}

void DefieldSymmRotationH::read( InputObjectStream *s )
{
    DefieldSymmRotation::read(s);
    reflection_h_ = new DefieldSymmReflection;
    reflection_h_->read(s);
}

void DefieldSymmRotationH::write( OutputObjectStream *s ) const
{
    DefieldSymmRotation::write(s);
    reflection_h_->write(s);
}

void DefieldSymmRotationV::assign( const Object* obj, COPY_CONTEXT *context ) 
{
    const DefieldSymmRotationV *other = dynamic_cast<const DefieldSymmRotationV*>(obj);
    if (other) {
        copyObjectProperties( obj, this );
    } else {
        pAssert(false);
    }
}

void DefieldSymmRotationV::read( InputObjectStream *s )
{
    DefieldSymmRotation::read(s);
    mpcard listsize;
    s->readMaxPlatformCard(listsize);
    reflections_.resize(listsize);
    for (card32 i = 0; i < reflections_.size(); ++i) {
        reflections_[i] = new DefieldSymmReflection;
        reflections_[i]->read(s);
    }
}

void DefieldSymmRotationV::write( OutputObjectStream *s ) const
{
    DefieldSymmRotation::write(s);
    s->writeMaxPlatformCard(reflections_.size());
    for (card32 i = 0; i < reflections_.size(); ++i) {
        reflections_[i]->write(s);
    }
}

void DefieldSymmRotation::assign( const Object* obj, COPY_CONTEXT *context ) 
{
    const DefieldSymmRotation *other = dynamic_cast<const DefieldSymmRotation*>(obj);
    if (other) {
        copyObjectProperties( obj, this );
    } else {
        pAssert(false);
    }
}

void DefieldSymmRotation::read( InputObjectStream *s )
{
    DefieldSymm::read(s);
    s->read(trans_b_);
    s->read(center_);
}

void DefieldSymmRotation::write( OutputObjectStream *s ) const
{
    DefieldSymm::write(s);
    s->write(trans_b_);
    s->write(center_);
}

//============================================================================
void DefieldSymmRotation::Sort(const PointSet* samPS)
{
    if (stack_.empty()) return;
    if (3 > DefieldSymmRotation::GetNumTransformation()) return;

    const AAT samPosAAT = samPS->getAAT("position");
    struct PointIndex
    {
        PointIndex(const float& dist, const unsigned& indx) : dist_(dist), indx_(indx) {}
        bool operator<(const PointIndex& rhs) const { return dist_ < rhs.dist_; }
        float dist_; unsigned indx_;
    };

    std::deque<PointIndex> sortHelp;
    for (unsigned oi = 0; oi < stack_.size(); ++oi) {
        const unsigned& elem = stack_[oi].front();
        const Vector3f& pos = samPS->get3f(elem, samPosAAT);
        const Vector3f& vec = pos - center_;
        const float dm = sqr(vec * axis_);
        sortHelp.push_back(PointIndex(abs(normQuad(vec) - dm), oi));
    }
    std::sort(sortHelp.begin(), sortHelp.end());

    OrbitStack newStack;
    for (unsigned oi = 0; oi < sortHelp.size(); ++oi) {
        newStack.push_back(stack_[sortHelp[oi].indx_]);
        //debugRenderer->beginRenderJob_OneFrame("debug_", DR_FRAME++);
        //debugRenderer->addLine(
        //    center_, center_ + axis_,
        //    makeVector3f(1, 0, 0),
        //    makeVector3f(0, 0, 1),
        //    2);
        //debugRenderer->addLine(
        //    center_, samPS->get3f(newStack[oi].front(), samPosAAT),
        //    makeVector3f(1, 0, 0),
        //    makeVector3f(0, 0, 1),
        //    2);
        //debugRenderer->endRenderJob();
    }
    stack_ = newStack;

    fixedRigion.clear();
    const float& dist0 = sortHelp.front().dist_;
    for (unsigned oi = 0; oi < sortHelp.size(); ++oi) {
        const float& dist = sortHelp[oi].dist_;
        if (10 * abs(dist0 - dist) > dist0) break;
        //debugRenderer->beginRenderJob_OneFrame("debug_", DR_FRAME++);
        //debugRenderer->addLine(
        //    center_, center_ + axis_,
        //    makeVector3f(1, 0, 0),
        //    makeVector3f(0, 0, 1),
        //    2);
        std::deque<unsigned>& orbit = stack_[oi];
        for (std::deque<unsigned>::iterator it = orbit.begin(); it != orbit.end(); ++it) {
            fixedRigion.push_back(*it);
            //debugRenderer->addLine(
            //    center_, samPS->get3f(*it, samPosAAT),
            //    makeVector3f(1, 0, 0),
            //    makeVector3f(0, 0, 1),
            //    2);
        }
        //debugRenderer->endRenderJob();
    }
}

DefieldSymmRotation::DefieldSymmRotation(void)
{
    rank_ = ROTATION;
    trans_b_ = IDENTITY4F;
    color_ = makeVector3f((float)12, (float)174, (float)232) / (float)255;
}

Vector3f DefieldSymmRotation::GetNormalDirection(const Vector3f& pos, const Vector3f& ref) const
{
    return normalize(pos - center_);
}

void DefieldSymmRotation::DrawWithDR(
    PointSet  const& deformed,
    AAT       const& defAAT
    )
{
    DefieldSymm::DrawWithDR(deformed, defAAT);
    OrbitVerticeMap::iterator mj = overt_.begin();
    for (; mj != overt_.end(); ++mj) {
        std::deque<card32> vinx1 = mj->second;
        std::deque<card32>::iterator vi1 = vinx1.begin();
        for (; vi1 != vinx1.end(); ++vi1) {
            Vector3f const point1_def = deformed.get3f(*vi1, defAAT);
            debugRenderer->addLine(
                center_, point1_def,
                makeVector3f(1, 0, 0),
                makeVector3f(0, 0, 1),
                1);
        }
    }
}

bool DefieldSymmRotation::UpdatedTransformation(
    PointSet  const& deformed,
    AAT       const& defAAT)
{
    Matrix4f Tf, Tb;
    if (!DefieldSymm::GetUpdatedTransformation(deformed, defAAT, Tf, Tb)) return false;
    trans_ = Tf;
    return true;
}

//============================================================================
DefieldSymmRotationV::DefieldSymmRotationV(void)
{
    rank_ = ROTATION_V;
    color_ = makeVector3f((float)0, (float)255, (float)255) / (float)255;
}

DefieldSymmRotationV::~DefieldSymmRotationV(void)
{
    for (unsigned ii = 0; ii < reflections_.size(); ++ii) {
        delete reflections_[ii];
    }
    reflections_.clear();
}

unsigned DefieldSymmRotationV::GetNumSubSymm(void) const
{
    return reflections_.size() + 1;
}

DefieldSymm* DefieldSymmRotationV::GetSubSymm(unsigned subs)
{
    if (0 == subs) return dynamic_cast<DefieldSymmRotation*>(this);
    else if (1 <= subs && subs < reflections_.size()+1) return reflections_[subs-1];
    else return nullptr;
}

unsigned DefieldSymmRotationV::GetNumTransformation(void) const
{
    return numtrans_ + reflections_.size();
}

Matrix4f DefieldSymmRotationV::GetTransformation(const int& ii) const
{
    unsigned orx = abs(ii) % DefieldSymmRotationV::GetNumTransformation();
    if (DefieldSymmRotation::GetNumTransformation() > orx)
        return DefieldSymmRotation::GetTransformation(ii); // first rotation
    return reflections_[orx - DefieldSymmRotation::GetNumTransformation()]->GetTransformation(); // second self-reflection
}

void DefieldSymmRotationV::DrawWithDR(
    PointSet  const& deformed,
    AAT       const& defAAT
    )
{
    DefieldSymmRotation::DrawWithDR(deformed, defAAT);
    for (unsigned ii = 0; ii < reflections_.size(); ++ii) {
        reflections_[ii]->DrawWithDR(deformed, defAAT);
    }
}

bool DefieldSymmRotationV::UpdatedTransformation(
    PointSet  const& deformed,
    AAT       const& defAAT
    )
{
    return false;
}

int DefieldSymmRotationV::AddOrbitIndice(const std::deque<unsigned>& orbitIndice, const std::deque<Vector3f>& orbitPosition)
{
    const unsigned& numtrans = DefieldSymmRotationV::GetNumTransformation();
    const unsigned& numBaseTrans = DefieldSymmRotation::GetNumTransformation();

    if (orbitPosition.size() != orbitIndice.size() || numtrans != orbitIndice.size()) {
        error("DefieldSymmDihedral::AddOrbitIndice - orbit mis-match");
        return -1;
    }

    for (unsigned tt = 0; tt < numtrans; ++tt) {
        overt_[tt].push_back(orbitIndice[tt]);
    }

    std::deque<unsigned> orbit;
    for (unsigned tt = 0; tt < numBaseTrans; ++tt) {
        orbit.push_back(orbitIndice[tt]);
    }
    stack_.push_back(orbit);

    RefMatching(orbitIndice, orbitPosition);

    return 0;
}

int DefieldSymmRotationV::RefMatching(const std::deque<unsigned>& orbitIndice, const std::deque<Vector3f>& orbitPosition)
{
    const unsigned& numtrans = DefieldSymmRotationV::GetNumTransformation();
    if (orbitPosition.size() != orbitIndice.size() || numtrans != orbitIndice.size()) {
        error("DefieldSymmDihedral::RefMatching - orbit mis-match");
        return -1;
    }
    for (unsigned ri = 0; ri < reflections_.size(); ++ri)
    {
        const Matrix4f& trans = reflections_[ri]->GetTransformation();
        std::deque<unsigned> source, target;
        std::vector<int> mark(numtrans, -1);
        for (unsigned vi = 0; vi < numtrans; ++vi) {
            if (0 <= mark[vi]) continue;
            const Vector3f& pos_vi = orbitPosition[vi];
            const Vector3f& pos_vi_t = transformVector3f(trans, pos_vi);
            if (normQuad(pos_vi_t - pos_vi) < 1e-6) { // possible on the reflection plane: self-reflection
                continue;
            }
            bool bfound = false;
            for (unsigned vj = vi+1; vj < numtrans; ++vj) {
                if (0 <= mark[vj]) continue;
                const Vector3f& pos_vj = orbitPosition[vj];
                if (normQuad(pos_vi_t - pos_vj) < 1e-6) {
                    source.push_back(vi); target.push_back(vj);
                    mark[vi] = vj; mark[vj] = vi;
                    bfound = true;
                    break;
                }
            }
            if (!bfound) {
                // this is acceptable, since orbits are generated from rotation' transformation

                //debugRenderer->beginRenderJob_OneFrame("debug_reflection_matching_", DR_FRAME++);
                //debugRenderer->addPoint(
                //    pos_vi, makeVector3f(1, 0, 0)
                //    );
                //debugRenderer->addLine(
                //    pos_vi, pos_vi_t,
                //    makeVector3f(0, 1, 0),
                //    makeVector3f(0, 0, 1),
                //    3);
                //for (unsigned vj = 0; vj < numtrans; ++vj) {
                //    const Vector3f& pos_vj = orbitPosition[vj];
                //    if (0 <= mark[vj]) {
                //        debugRenderer->addPoint(
                //            pos_vj, makeVector3f(1, 0, 1)
                //            );
                //    } else {
                //        debugRenderer->addPoint(
                //            pos_vj, makeVector3f(1, 1, 0)
                //            );
                //    }
                //}
                //debugOutput << "DefieldSymmDihedral::RefMatching - number of transformation: " << numtrans << "\n";
                //for (unsigned mi = 0; mi < source.size(); ++mi) {
                //    debugRenderer->addLine(
                //        orbitPosition[source[mi]], orbitPosition[target[mi]],
                //        makeVector3f(1, 1, 0),
                //        makeVector3f(0, 1, 1),
                //        1);
                //}
                //debugRenderer->endRenderJob();

                //error("DefieldSymmDihedral::RefMatching - dual rotation orbit not matching");
                ////debugOutput << "DefieldSymmDihedral::RefMatching - dual rotation orbit not matching\n";

                continue;
            }
        }

        for (unsigned mi = 0; mi < source.size(); ++mi) {
            std::deque<unsigned> orbit;
            if (0 > reflections_[ri]->plane_.calculateSignedDistance(orbitPosition[source[mi]])) {
                orbit.push_back(orbitIndice[target[mi]]);
                orbit.push_back(orbitIndice[source[mi]]);
            } else {
                orbit.push_back(orbitIndice[source[mi]]);
                orbit.push_back(orbitIndice[target[mi]]);
            }
            reflections_[ri]->AddOrbitIndice(orbit, orbitPosition);
        }
    }

    return 0;
}

std::deque<OrbitPairTransformation> DefieldSymmRotationV::GetOrbitPairTransformations(void) const
{
    std::deque<OrbitPairTransformation> ret;
    {
        const std::deque<OrbitPairTransformation>& orbits = DefieldSymmRotation::GetOrbitPairTransformations();
        std::copy(orbits.begin(), orbits.end(), std::back_inserter(ret));
    }
    for (unsigned ri = 0; ri < reflections_.size(); ++ri) {
        const std::deque<OrbitPairTransformation>& orbits = reflections_[ri]->GetOrbitPairTransformations();
        std::copy(orbits.begin(), orbits.end(), std::back_inserter(ret));
    }
    return ret;
}

//============================================================================
DefieldSymmRotationH::DefieldSymmRotationH(void)
{
    rank_ = ROTATION_H;
    reflection_h_ = nullptr;
    color_ = makeVector3f((float)12, (float)232, (float)170) / (float)255;
}

DefieldSymmRotationH::~DefieldSymmRotationH(void)
{
    delete reflection_h_;
    reflection_h_ = nullptr;
}

unsigned DefieldSymmRotationH::GetNumSubSymm(void) const
{
    return 2;
}

DefieldSymm* DefieldSymmRotationH::GetSubSymm(unsigned subs)
{
    if (0 == subs) return dynamic_cast<DefieldSymmRotation*>(this);
    else if (1 == subs) return reflection_h_;
    else return nullptr;
}

unsigned DefieldSymmRotationH::GetNumTransformation(void) const
{
    return 2 * numtrans_;
}

Matrix4f DefieldSymmRotationH::GetTransformation(const int& ii) const
{
    // modular to range
    int orx = ii % (int)DefieldSymmRotationH::GetNumTransformation();
    if (orx < 0) orx += (int)DefieldSymmRotationH::GetNumTransformation();

    // basic ring
    Matrix4f T = DefieldSymmRotation::GetTransformation(ii);

    // note the reflection order
    int refv = (int)((float)ii / DefieldSymmRotation::GetNumTransformation()) % 2;
    if (1 == refv) T = reflection_h_->GetTransformation() * T;
    else if (-1 == refv) T = T * reflection_h_->GetTransformation();

    return T;
}

void DefieldSymmRotationH::DrawWithDR(
    PointSet  const& deformed,
    AAT       const& defAAT
    )
{
    DefieldSymmRotation::DrawWithDR(deformed, defAAT);
    reflection_h_->DrawWithDR(deformed, defAAT);
}

bool DefieldSymmRotationH::UpdatedTransformation(
    PointSet  const& deformed,
    AAT       const& defAAT
    )
{
    return false;
}

int DefieldSymmRotationH::AddOrbitIndice(const std::deque<unsigned>& orbitIndice, const std::deque<Vector3f>& orbitPosition)
{
    const unsigned& numtrans = DefieldSymmRotationH::GetNumTransformation();
    const unsigned& numBaseTrans = DefieldSymmRotation::GetNumTransformation();

    if (orbitPosition.size() != orbitIndice.size() || numtrans != orbitIndice.size()) {
        error("DefieldSymmDihedralH::AddOrbitIndice - orbit mis-match");
        return -1;
    }

    for (unsigned tt = 0; tt < numtrans; ++tt) {
        overt_[tt].push_back(orbitIndice[tt]);
    }

    std::deque<unsigned> orbitIndice0, orbitIndice1;
    std::deque<Vector3f> orbitPosition0, orbitPosition1;
    {
        unsigned tt = 0;
        for (; tt < numBaseTrans; ++tt) {
            orbitIndice0.push_back(orbitIndice[tt]);
            orbitPosition0.push_back(orbitPosition[tt]);
        }
        for (; tt < numtrans; ++tt) {
            orbitIndice1.push_back(orbitIndice[tt]);
            orbitPosition1.push_back(orbitPosition[tt]);
        }
    }

    { // 1st rotation: 0 ~ DefieldSymmRotation::GetNumTransformation()
        std::deque<unsigned> orbit;
        for (unsigned tt = 0; tt < numBaseTrans; ++tt) {
            orbit.push_back(orbitIndice0[tt]);
        }
        stack_.push_back(orbit);
    }
    { // 2nd rotation: DefieldSymmRotation::GetNumTransformation() ~ END
        std::deque<unsigned> orbit;
        for (unsigned tt = 0; tt < numBaseTrans; ++tt) {
            orbit.push_back(orbitIndice1[tt]);
        }
        stack_.push_back(orbit);
    }

    { // reflection V: [0 : DefieldSymmRotation::GetNumTransformation()] --> [DefieldSymmRotation::GetNumTransformation() : END]
        for (unsigned tt = 0; tt < numBaseTrans; ++tt) {
            std::deque<unsigned> orbit;
            orbit.push_back(orbitIndice0[tt]);
            orbit.push_back(orbitIndice1[tt]);
            reflection_h_->AddOrbitIndice(orbit, orbitPosition);
        }
    }
    return 0;
}

std::deque<OrbitPairTransformation> DefieldSymmRotationH::GetOrbitPairTransformations(void) const
{
    std::deque<OrbitPairTransformation> ret;
    {
        const std::deque<OrbitPairTransformation>& orbits = DefieldSymmRotation::GetOrbitPairTransformations();
        std::copy(orbits.begin(), orbits.end(), std::back_inserter(ret));
    }
    {
        const std::deque<OrbitPairTransformation>& orbits = reflection_h_->GetOrbitPairTransformations();
        std::copy(orbits.begin(), orbits.end(), std::back_inserter(ret));
    }
    return ret;
}
