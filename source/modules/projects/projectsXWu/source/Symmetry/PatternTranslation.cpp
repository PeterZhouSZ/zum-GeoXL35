#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Symmetry/PatternTranslation.h"
#include "Util/NoUse.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

bool PatternTranslation::SetupFrom2P(const array2vec3f& pvec, Vector2i& ivec,
                                     const float& th)
{
    direction_ = pvec[1] - pvec[0];
    orbit_.step = norm(direction_);
    if (th > orbit_.step) return false;
    direction_ /= orbit_.step;
    pmap_[0] = pvec[0];
    pmap_[1] = pvec[1];
    ivec[0] = 0; ivec[1] = 1;

    return true;
}

bool PatternTranslation::ParallelTo(const PatternTranslation::Ptr& pt)
{
    return NU::IsParallel(this->direction_, pt->direction_);
}

bool PatternTranslation::SameAs(const PatternSymmetry::Ptr& ps)
{
    int ix;
    return
        PointOnOrbit(ps->pmap_[0], ix) &&
        PointOnOrbit(ps->pmap_[1], ix);
}

bool PatternTranslation::ConsistentWith(const PatternSymmetry::Ptr& ps, const float& th)
{
    PatternTranslation::Ptr ps_d = 
        dynamic_pointer_cast<PatternTranslation>(ps);
    if (nullptr == ps_d)
        return false;

    return ParallelTo(ps_d)
        //&& !SameAs(ps_d)
        && 1e-6 > abs(orbit_.step - ps_d->orbit_.step);
}

bool PatternTranslation::AddPoint(const Vector3f& p, int& ix, const float& th)
{
    //if (!SameOrbit(p, ix, th)) return false;
    if (0 != ix && 1 != ix)
        pmap_[ix] = p;
    return true;
}

bool PatternTranslation::PointOnOrbit(const Vector3f& p, int& ix, const float& th)
{
    Vector3f v0 = p - pmap_[0], v1 = p - pmap_[1];
    float d0 = norm(v0), d1 = norm(v1);
    if (1e-6 > d0) {
        ix = 0;
        return true;
    } else if (1e-6 > d1) {
        ix = 1;
        return true;
    }
    float d = NU::PointlLineDist(p, pmap_[0], normalize(pmap_[1]-pmap_[0]));
    if (th < d) return false;

    float m = d0 / orbit_.step;
    float a, b = modf(m, &a);

    if (th < abs(b)) return false;
    if (0 < v0 * v1 && d0 < d1) a *= -1;
    ix = (int)a;
    return true;
}

void PatternTranslation::DrawWithDR(void) const
{
    OrbitPosMap::const_iterator jt, it = pmap_.begin();
    for (jt = it; ; ++it) {
        if (++jt == pmap_.end()) break;
        debugRenderer->addPoint(it->second, makeVector3f(1, 1, 0));
        if (0 == it->first) {
            debugRenderer->addLine(
                it->second, jt->second,
                makeVector3f(1, 0, 0),
                makeVector3f(0, 0, 1),
                3);
        } else {
            debugRenderer->addLine(
                it->second, jt->second,
                makeVector3f(0, 1, 1),
                makeVector3f(0, 1, 1),
                3);
        }
    }
    debugRenderer->addPoint(it->second, makeVector3f(1, 1, 0));
}
