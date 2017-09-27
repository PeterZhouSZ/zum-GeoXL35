#include "StdAfx.h"
//---------------------------------------------------------------------------
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

IMPLEMENT_CLASS( DefieldSymmDihedral, 0 ) 
{
    BEGIN_CLASS_INIT( DefieldSymmDihedral );
    INIT_PROPERTY_TABLE();
}
IMPLEMENT_CLASS( DefieldSymmDihedralH, 0 ) 
{
    BEGIN_CLASS_INIT( DefieldSymmDihedralH );
    INIT_PROPERTY_TABLE();
}

//============================================================================
void DefieldSymmDihedral::assign( const Object* obj, COPY_CONTEXT *context ) 
{
    const DefieldSymmDihedral *other = dynamic_cast<const DefieldSymmDihedral*>(obj);
    if (other) {
        copyObjectProperties( obj, this );
    } else {
        pAssert(false);
    }
}

void DefieldSymmDihedral::read( InputObjectStream *s )
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

void DefieldSymmDihedral::write( OutputObjectStream *s ) const
{
    DefieldSymmRotation::write(s);
    s->writeMaxPlatformCard(reflections_.size());
    for (card32 i = 0; i < reflections_.size(); ++i) {
        reflections_[i]->write(s);
    }
}

void DefieldSymmDihedralH::assign( const Object* obj, COPY_CONTEXT *context ) 
{
    const DefieldSymmDihedralH *other = dynamic_cast<const DefieldSymmDihedralH*>(obj);
    if (other) {
        copyObjectProperties( obj, this );
    } else {
        pAssert(false);
    }
}

void DefieldSymmDihedralH::read( InputObjectStream *s )
{
    DefieldSymmDihedral::read(s);
    reflection_h_ = new DefieldSymmReflection;
    reflection_h_->read(s);
}

void DefieldSymmDihedralH::write( OutputObjectStream *s ) const
{
    DefieldSymmDihedral::write(s);
    reflection_h_->write(s);
}
//============================================================================
DefieldSymmDihedral::DefieldSymmDihedral(void)
{
    rank_ = DIHEDRAL;
    color_ = makeVector3f((float)232, (float)89, (float)12) / (float)255;
}
DefieldSymmDihedral::~DefieldSymmDihedral(void)
{
    for (unsigned ii = 0; ii < reflections_.size(); ++ii) {
        delete reflections_[ii];
    }
    reflections_.clear();
}

unsigned DefieldSymmDihedral::GetNumSubSymm(void) const
{
    return reflections_.size() + 1;
}

DefieldSymm* DefieldSymmDihedral::GetSubSymm(unsigned subs)
{
    if (0 == subs) return dynamic_cast<DefieldSymmRotation*>(this);
    else if (1 <= subs && subs < reflections_.size()+1) return reflections_[subs-1];
    else return nullptr;
}

Vector3f DefieldSymmDihedral::GetNormalPosition(const Vector3f& pos, const Vector3f& ref)
{
    Vector3f dirv = GetNormalDirection(ref, ref);
    Vector3f diff = pos - ref;
    return (diff * dirv) * dirv + ref;
}

unsigned DefieldSymmDihedral::GetNumTransformation(void) const
{
    return 2 * numtrans_;
}

Matrix4f DefieldSymmDihedral::GetTransformation(const int& ii) const
{
    unsigned orx = abs(ii) % DefieldSymmDihedral::GetNumTransformation();
    if (DefieldSymmRotation::GetNumTransformation() > orx)
        return DefieldSymmRotation::GetTransformation(ii); // first rotation
    return reflections_[orx - DefieldSymmRotation::GetNumTransformation()]->GetTransformation(); // second self-reflection
}

void DefieldSymmDihedral::DrawWithDR(
    PointSet  const& deformed,
    AAT       const& defAAT
    )
{
    DefieldSymmRotation::DrawWithDR(deformed, defAAT);
    for (unsigned ii = 0; ii < reflections_.size(); ++ii) {
        reflections_[ii]->DrawWithDR(deformed, defAAT);
    }
}

bool DefieldSymmDihedral::UpdatedTransformation(
    PointSet  const& deformed,
    AAT       const& defAAT)
{
    if (!DefieldSymmRotation::UpdatedTransformation(deformed, defAAT)) return false;
    for (unsigned ii = 0; ii < reflections_.size(); ++ii) {
        if (!reflections_[ii]->UpdatedTransformation(deformed, defAAT)) return false;
    }
    return true;
}

int DefieldSymmDihedral::AddOrbitIndice(const std::deque<unsigned>& orbitIndice, const std::deque<Vector3f>& orbitPosition)
{
    const unsigned& numtrans = DefieldSymmDihedral::GetNumTransformation();
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

    RotMatching(orbitIndice, orbitPosition);
    RefMatching(orbitIndice, orbitPosition);

    return 0;
}

int DefieldSymmDihedral::RotMatching(const std::deque<unsigned>& orbitIndice, const std::deque<Vector3f>& orbitPosition)
{
    const unsigned& numtrans = DefieldSymmDihedral::GetNumTransformation();
    const unsigned& numBaseTrans = DefieldSymmRotation::GetNumTransformation();

    if (orbitPosition.size() != orbitIndice.size() || numtrans != orbitIndice.size()) {
        error("DefieldSymmDihedral::RotMatching - orbit mis-match");
        return -1;
    }

    std::deque<Vector3f> posRot;
    std::deque<unsigned> indRot;
    for (unsigned ri = numBaseTrans; ri < numtrans; ++ri) {
        posRot.push_back(orbitPosition[ri]);
        indRot.push_back(orbitIndice[ri]);
    }

    const Vector3f& pos = posRot[0];
    std::deque<unsigned> orbit;
    std::vector<int> mark(numBaseTrans, -1);
    for (unsigned ri = 0; ri < numBaseTrans; ++ri) {
        const Matrix4f& trans = DefieldSymmRotation::GetTransformation(ri);
        const Vector3f& pos_t = transformVector3f(trans, pos);
        bool bfound = false;
        for (unsigned vi = 0; vi < numBaseTrans; ++vi) {
            if (0 <= mark[vi]) continue;
            const Vector3f& pos_vi = posRot[vi];
            if (normQuad(pos_vi - pos_t) < 1e-4) {
                orbit.push_back(indRot[vi]);
                bfound = true;
                mark[vi] = ri;
                break;
            }
        }
        if (!bfound) {
            //debugRenderer->beginRenderJob_OneFrame("debug_rotation_matching_", DR_FRAME++);
            //debugRenderer->addPoint(
            //    pos, makeVector3f(1, 0, 0)
            //    );
            //debugRenderer->addLine(
            //    pos, orbitPosition[0],
            //    makeVector3f(0, 1, 0),
            //    makeVector3f(0, 0, 1),
            //    3);
            //for (unsigned vj = 1; vj < numBaseTrans; ++vj) {
            //    const Vector3f& pos_vj = posRot[vj];
            //    if (0 <= mark[vj]) {
            //        debugRenderer->addPoint(
            //            pos_vj, makeVector3f(1, 0, 1)
            //            );
            //    } else {
            //        debugRenderer->addPoint(
            //            pos_vj, makeVector3f(1, 1, 0)
            //            );
            //    }
            //    const Matrix4f& trans = DefieldSymmRotation::GetTransformation(vj);
            //    const Vector3f& pos_t = transformVector3f(trans, pos);
            //    debugRenderer->addLine(
            //        pos, pos_t,
            //        makeVector3f(1, 1, 0),
            //        makeVector3f(0, 1, 1),
            //        1);
            //}
            ////for (unsigned mi = 0; mi < numBaseTrans - 1; ++mi) {
            ////    debugRenderer->addLine(
            ////        orbitPosition[mi], orbitPosition[mi + 1],
            ////        makeVector3f(1, 1, 0),
            ////        makeVector3f(0, 1, 1),
            ////        1);
            ////}
            //debugRenderer->endRenderJob();

            //error("DefieldSymmDihedral::RotMatching - secondary rotation orbit not matching");
            //return -1;

            // precision is always the problem
            continue;
        }
    }
    stack_.push_back(orbit);
    return 0;
}

int DefieldSymmDihedral::RefMatching(const std::deque<unsigned>& orbitIndice, const std::deque<Vector3f>& orbitPosition)
{
    const unsigned& numtrans = DefieldSymmDihedral::GetNumTransformation();
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
            if (normQuad(pos_vi_t - pos_vi) < 1e-4) { // possible on the reflection plane: self-reflection
                continue;
            }
            bool bfound = false;
            for (unsigned vj = vi+1; vj < numtrans; ++vj) {
                if (0 <= mark[vj]) continue;
                const Vector3f& pos_vj = orbitPosition[vj];
                if (normQuad(pos_vi_t - pos_vj) < 1e-4) {
                    source.push_back(vi); target.push_back(vj);
                    mark[vi] = vj; mark[vj] = vi;
                    bfound = true;
                    break;
                }
            }
            if (!bfound) {
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

                // this is acceptable, since orbits are generated from rotation' transformation
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

std::deque<OrbitPairTransformation> DefieldSymmDihedral::GetOrbitPairTransformations(void) const
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
DefieldSymmDihedralH::DefieldSymmDihedralH(void)
{
    rank_ = DIHEDRAL_H;
    reflection_h_ = nullptr;
    color_ = makeVector3f((float)255, (float)127, (float)0) / (float)255;
}
DefieldSymmDihedralH::~DefieldSymmDihedralH(void)
{
    delete reflection_h_;
    reflection_h_ = nullptr;
}

unsigned DefieldSymmDihedralH::GetNumSubSymm(void) const
{
    return reflections_.size() + 2;
}

DefieldSymm* DefieldSymmDihedralH::GetSubSymm(unsigned subs)
{
    if (0 == subs) return dynamic_cast<DefieldSymmRotation*>(this);
    else if (1 <= subs && subs < reflections_.size()+1) return reflections_[subs-1];
    else if (reflections_.size()+2 == subs) return reflection_h_;
    else return nullptr;
}

unsigned DefieldSymmDihedralH::GetNumTransformation(void) const
{
    return 4 * numtrans_;
}

Matrix4f DefieldSymmDihedralH::GetTransformation(const int& ii) const
{
    // modular to range
    int orx = ii % (int)DefieldSymmDihedralH::GetNumTransformation();
    if (orx < 0) orx += (int)DefieldSymmDihedralH::GetNumTransformation();

    // basic ring
    Matrix4f T = DefieldSymmDihedral::GetTransformation(ii);

    // note the reflection order
    int refv = (int)((float)ii / DefieldSymmDihedral::GetNumTransformation()) % 2;
    if (1 == refv) T = reflection_h_->GetTransformation() * T;
    else if (-1 == refv) T = T * reflection_h_->GetTransformation();

    return T;
}

void DefieldSymmDihedralH::DrawWithDR(
    PointSet  const& deformed,
    AAT       const& defAAT
    )
{
    //DefieldSymmRotation::DrawWithDR(deformed, defAAT);
    //for (unsigned ii = 0; ii < reflections_.size(); ++ii) {
    //    reflections_[ii]->DrawWithDR(deformed, defAAT);
    //}
    reflection_h_->DrawWithDR(deformed, defAAT);

    //{
    //    OrbitVerticeMap const& overt = overt_;
    //    if (overt.empty()) return;
    //    OrbitVerticeMap::const_iterator mi, mj = overt.begin();
    //    for (mi = mj++; mj != overt.end(); ++mi, ++mj) {
    //        std::deque<card32> vinx0 = mi->second, vinx1 = mj->second;
    //        std::deque<card32>::iterator vi0 = vinx0.begin(), vi1 = vinx1.begin();
    //        for (; vi0 != vinx0.end(); ++vi0, ++vi1) {
    //            Vector3f const point0_def = deformed.get3f(*vi0, defAAT);
    //            Vector3f const point1_def = deformed.get3f(*vi1, defAAT);
    //            debugRenderer->addLine(
    //                point0_def, point1_def,
    //                makeVector3f(0, 1, 0),
    //                makeVector3f(1, 0, 1),
    //                3);
    //        }
    //    }
    //}
}

bool DefieldSymmDihedralH::UpdatedTransformation(
    PointSet  const& deformed,
    AAT       const& defAAT)
{
    if (!DefieldSymmDihedral::UpdatedTransformation(deformed, defAAT)) return false;
    if (!reflection_h_->UpdatedTransformation(deformed, defAAT)) return false;
    return true;
}
int DefieldSymmDihedralH::AddOrbitIndice(const std::deque<unsigned>& orbitIndice, const std::deque<Vector3f>& orbitPosition)
{
    const unsigned& numtrans = DefieldSymmDihedralH::GetNumTransformation();
    const unsigned& numtrans_2 = DefieldSymmDihedral::GetNumTransformation();
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
        for (; tt < numtrans_2; ++tt) {
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

        RotMatching(orbitIndice0, orbitPosition0);
        RefMatching(orbitIndice0, orbitPosition0);
    }
    { // 2nd rotation: DefieldSymmDihedral::GetNumTransformation() ~ DefieldSymmDihedral::GetNumTransformation() + DefieldSymmRotation::GetNumTransformation()
        std::deque<unsigned> orbit;
        for (unsigned tt = 0; tt < numBaseTrans; ++tt) {
            orbit.push_back(orbitIndice1[tt]);
        }
        stack_.push_back(orbit);

        RotMatching(orbitIndice1, orbitPosition1);
        RefMatching(orbitIndice1, orbitPosition1);
    }

    { // reflection V: [0 : DefieldSymmDihedral::GetNumTransformation()] --> [DefieldSymmDihedral::GetNumTransformation() : 2 * DefieldSymmDihedral::GetNumTransformation()]
        for (unsigned tt = 0; tt < numtrans_2; ++tt) {
            std::deque<unsigned> orbit;
            orbit.push_back(orbitIndice0[tt]);
            orbit.push_back(orbitIndice1[tt]);
            reflection_h_->AddOrbitIndice(orbit, orbitPosition);
        }
    }
    return 0;
}

std::deque<OrbitPairTransformation> DefieldSymmDihedralH::GetOrbitPairTransformations(void) const
{
    std::deque<OrbitPairTransformation> ret;
    {
        const std::deque<OrbitPairTransformation>& orbits = DefieldSymmDihedral::GetOrbitPairTransformations();
        std::copy(orbits.begin(), orbits.end(), std::back_inserter(ret));
    }
    {
        const std::deque<OrbitPairTransformation>& orbits = reflection_h_->GetOrbitPairTransformations();
        std::copy(orbits.begin(), orbits.end(), std::back_inserter(ret));
    }
    return ret;
}
