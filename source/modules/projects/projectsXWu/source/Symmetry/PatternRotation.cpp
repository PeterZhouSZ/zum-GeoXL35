#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Symmetry/PatternRotation.h"
#include "Util/NoUse.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

bool PatternRotation::SetupFrom3P(const array3vec3f& pvec, Vector3i& ivec)
{
    Matrix3f frame;
    int maxi;
    Vector3f v1 = NULL_VECTOR3F, v2 = NULL_VECTOR3F;
    if (!generateFrame(pvec, frame, orbit_.step, orbit_.radius,
        maxi, v1, v2)) return false;
    normal_ = frame[0];
    center_ = pvec[maxi] + normalize(v1+v2) * orbit_.radius;
    float sinv = orbit_.step / 2 / orbit_.radius;
    orbit_.harc = (1 < sinv) ? M_PI_2 : asin(sinv);
    num_ = (int)floor(M_PI / orbit_.harc + 0.5f);

    ivec[0] = (maxi+2)%3; ivec[1] = maxi; ivec[2] = (maxi+1)%3;
    pmap_[capacity()-1] = pvec[ivec[0]];
    pmap_[0] = pvec[ivec[1]];
    pmap_[1] = pvec[ivec[2]];

    toWorld_ = makeTranslation4f(center_) * expand3To4(frame);
    toOYZ_ = invertFrame(toWorld_);

    OrbitPosMap::iterator it = pmap_.begin();
    for (; it != pmap_.end(); ++it) {
        pproj_[it->first] = transformVector3f(toOYZ_, it->second);
    }

    return true;
}

bool PatternRotation::generateFrame(const array3vec3f& pvec, Matrix3f& frame,
                                float& step, float& radius,
                                int& maxi, Vector3f& v1, Vector3f& v2)
{
    int i1, i2;
    array3vec3f vvec;
    float edgeMaxLength = 0.f;
    float lvec[3], ls = 0.f;
    for (unsigned j = 0; j < 3; ++j) {
        vvec[j] = pvec[(j+1)%3] - pvec[j];
        lvec[j] = norm(vvec[j]);
        if (1e-6 > lvec[j]) return false;
        ls += lvec[j];
        if (lvec[j] > edgeMaxLength) {
            maxi = (j+2)%3;
            edgeMaxLength = lvec[j];
        }
    }
    i1 = (maxi+2)%3, i2 = maxi;
    if (numeric_limits<float>::epsilon() < abs(lvec[i1] - lvec[i2])) return false;

    step = lvec[i1];
    v1 = - vvec[i1]; v2 = vvec[i2];
    if (OrbitCircle::min_arc > fabs(fabs(normalize(v1) * normalize(v2)) - 1))
        return false;

    frame[0] = normalize(v1.crossProduct(v2)); // so x-axis is aligned
    frame[1] = normalize(v1);
    frame[2] = normalize(frame[0].crossProduct(frame[1]));

    ls /= 2;
    radius = lvec[0] * lvec[1] * lvec[2];
    radius /= 4 * sqrt(ls*(ls-lvec[0])*(ls-lvec[1])*(ls-lvec[2]));

    return true;
}

void PatternRotation::DrawWithDR(void) const
{
    debugRenderer->addLine(
        center_, pmap_.begin()->second,
        makeVector3f(1, 0, 0),
        makeVector3f(0, 0, 1),
        3);
    debugRenderer->addLine(
        center_, center_+normal_,
        makeVector3f(1, 0, 0),
        makeVector3f(0, 0, 1),
        3);    Vector3f cen_t = transformVector3f(toOYZ_, center_);
    debugRenderer->addLine(
        cen_t, cen_t+makeVector3f(1, 0, 0),
        makeVector3f(1, 0, 0),
        makeVector3f(0, 0, 1),
        3);

    OrbitPosMap::const_iterator it, jt = pmap_.begin();
    for (it = jt++; it != pmap_.end(); ++it, ++jt) {
        if (jt == pmap_.end()) jt = pmap_.begin();
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
        debugRenderer->addLine(
            NULL_VECTOR3F, transformVector3f(toOYZ_, it->second),
            makeVector3f(0, 1, 0),
            makeVector3f(0, 1, 0),
            3);
    }
}

bool PatternRotation::PointOnOrbit(const Vector3f& p, int& ix, const float& th)
{
    Vector3f pt = transformVector3f(toOYZ_, p);
    if (1e-4 < abs(norm(pt) - orbit_.radius)) return false;

    float sinv = norm(pt-pproj_[0]) / 2 / orbit_.radius;
    float har = (1 < sinv) ? M_PI_2 : asin(sinv);
    float m = har/orbit_.harc;

    float a = (int)floor(m + 0.5f);
    if (th < abs(m - a)) return false;

    ix = (int)a;
    if (norm(pt-pproj_[capacity()-1]) < norm(pt-pproj_[1]))
        ix = (capacity() - ix) % capacity();
    return true;
}

bool PatternRotation::AddPoint(const Vector3f& p, int& ix, const float& th)
{
    //if (!SameOrbit(p, ix, th)) return false;
    if (capacity()-1 != ix && 0 != ix && 1 != ix) {
        pmap_[ix] = p;
        pproj_[ix] = transformVector3f(toOYZ_, p);
    }

    return true;
}

bool PatternRotation::ParallelTo(const PatternRotation::Ptr& pr)
{
    return NU::IsParallel(normal_, pr->normal_);}

bool PatternRotation::CoAxisTo(const PatternRotation::Ptr& pr, const float& th)
{
    if (!ParallelTo(pr)) return false;
    float d = NU::PointlLineDist(center_, pr->center_, pr->normal_);
    return th > abs(d);
}

bool PatternRotation::SameAs(const PatternSymmetry::Ptr& ps)
{
    int ix;
    return
        PointOnOrbit(ps->pmap_[capacity()-1], ix) &&
        PointOnOrbit(ps->pmap_[0], ix) &&
        PointOnOrbit(ps->pmap_[1], ix);
}

bool PatternRotation::ConsistentWith(const PatternSymmetry::Ptr& ps, const float& th)
{
    PatternRotation::Ptr ps_d = 
        dynamic_pointer_cast<PatternRotation>(ps);
    return CoAxisTo(ps_d, th)
        //&& !SameAs(ps_d)
        && 1e-6 > abs(orbit_.radius - ps_d->orbit_.radius)
        && 1e-6 > abs(orbit_.step - ps_d->orbit_.step)
        && 1e-6 > abs(orbit_.harc - ps_d->orbit_.harc);
}

bool PatternRotation::Concentric(const PatternRotation::Ptr& pr, const float& th)
{
    return th > norm(center_ - pr->center_) && ParallelTo(pr);
}

bool PatternRotation::GetRotatedPos(const Vector3f& pin, const int& step,
                                    Vector3f& pout, Matrix4f& tout)
{
    //if (!NU::PointOnPlane(pin, center_, normal_)) return false;

    Vector3f pin_n = ((pin - center_) * normal_) * normal_;
    float arc = orbit_.harc * 2 * step;

    tout = toWorld_ * makeTranslation4f(pin_n) *
        expand3To4(makeRotX3f(arc)) *
        makeTranslation4f(-pin_n) * toOYZ_;
    pout = transformVector3f(tout, pin);

    return true;
}

bool PatternRotation::Finalize(void)
{
    if (size() > capacity()) {
        warning("PatternRotation::Finalize");

        debugOutput << "PatternRotation::Finalize: "
            << size() << " " << capacity() << "\n";
        OrbitPosMap::iterator it;
        for (it = pmap_.begin(); it != pmap_.end(); ++it) {
            debugOutput << it->first;
        }
        debugOutput << "\n";
        for (it = pproj_.begin(); it != pproj_.end(); ++it) {
            debugOutput << it->first;
        }
        debugOutput << "\n";

        return false;
    }

    {
        //debugOutput << size() << " " << capacity() << "\n";
        //OrbitPosMap::iterator it;
        //for (it = pmap_.begin(); it != pmap_.end(); ++it) {
        //    debugOutput << it->first;
        //}
        //debugOutput << "\n";
        //for (it = pproj_.begin(); it != pproj_.end(); ++it) {
        //    debugOutput << it->first;
        //}
        //debugOutput << "\n";
    }

    //if (-1 != pproj_.begin()->first) {
    //    pmap_.erase(pmap_.begin());
    //    pproj_.erase(pproj_.begin());
    //} else if (1 != pproj_.rbegin()->first) {
    //    pmap_.erase( boost::prior( pmap_.end() ) );
    //    pproj_.erase( boost::prior( pproj_.end() ) );
    //} else {
    //    warning("PatternRotation::Finalize - error");
    //}
    
    if (size() == capacity()) {
        pmap_[-1] = pmap_[capacity()-1];
        pproj_[-1] = pproj_[capacity()-1];
        pmap_.erase( boost::prior( pmap_.end() ) );
        pproj_.erase( boost::prior( pproj_.end() ) );
    } else {
        {
            OrbitPosMap::iterator it, jt = pmap_.begin();
            for (it = jt++; jt != pmap_.end(); ++it, ++jt) {
                if (1 < jt->first - it->first) break;
            }
            if (jt == pmap_.end()) {
                warning("PatternRotation::Finalize - error");
                return false;
            }
            jt = pmap_.end();
            for (--jt; it != jt; --jt) {
                pmap_[jt->first-capacity()] = pmap_[jt->first];
            }
            pmap_.erase(++it, pmap_.end());
        }
        {
            OrbitPosMap::iterator it, jt = pproj_.begin();
            for (it = jt++; it != pproj_.end(); ++it, ++jt) {
                if (1 < jt->first - it->first) break;
            }
            if (jt == pproj_.end()) {
                warning("PatternRotation::Finalize - error");
                return false;
            }
            jt = pproj_.end();
            for (--jt; it != jt; --jt) {
                pproj_[jt->first-capacity()] = pproj_[jt->first];
            }
            pproj_.erase(++it, pproj_.end());
        }
    }

    {
        //OrbitPosMap::iterator it;
        //for (it = pmap_.begin(); it != pmap_.end(); ++it) {
        //    debugOutput << it->first;
        //}
        //debugOutput << "\n";
        //for (it = pproj_.begin(); it != pproj_.end(); ++it) {
        //    debugOutput << it->first;
        //}
        //debugOutput << "\n";
        //debugOutput << "\n";
    }

    return true;
}
