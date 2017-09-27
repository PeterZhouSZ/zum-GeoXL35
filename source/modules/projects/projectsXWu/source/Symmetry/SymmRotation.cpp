#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Symmetry/SymmRotation.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

SymmRotation::SymmRotation(PatternRotation::Ptr p0, PatternRotation::Ptr p1)
{
    patPair_.first = p0;
    patPair_.second = p1;
}

void SymmRotation::SetupVertice(const SymmFeatLine::Ptr& sfl, const int& idx)
{
    std::deque<mpcard> verts = sfl->vertice;
    overt_[idx] = verts;
}

bool SymmRotation::AddVertice(const SymmFeatLine::Ptr& sfl, int& idx)
{
    int idx1;

    if (patPair_.first->PointOnOrbit(sfl->pos0, idx) &&
        patPair_.second->PointOnOrbit(sfl->pos1, idx1))
    {
        if (idx == idx1) {
            patPair_.first->AddPoint(sfl->pos0, idx);
            patPair_.second->AddPoint(sfl->pos1, idx);
            SetupVertice(sfl, idx);
            return true;
        }
    }

    if (patPair_.first->PointOnOrbit(sfl->pos1, idx) &&
        patPair_.second->PointOnOrbit(sfl->pos0, idx1))
    {
        if (idx == idx1) {
            patPair_.first->AddPoint(sfl->pos1, idx);
            patPair_.second->AddPoint(sfl->pos0, idx);
            SymmFeatLine::Ptr line = sfl->CopyTo();
            line->Reverse();
            SetupVertice(line, idx);
            return true;
        }
    }
    return false;
}

void SymmRotation::DrawWithDR(void) const
{
    patPair_.first->DrawWithDR();
    patPair_.second->DrawWithDR();
    OrbitPosMap::const_iterator it0 = patPair_.first->pmap_.begin();
    OrbitPosMap::const_iterator it1 = patPair_.second->pmap_.begin();
    for (; it0 != patPair_.first->pmap_.end(); ++it0, ++it1) {
        debugRenderer->addLine(
            it0->second, it1->second,
            makeVector3f(1, 0, 0),
            makeVector3f(0, 0, 1),
            3);
    }
}

Matrix4f SymmRotation::Get1StepTransformation(void)
{
    PatternRotation::Ptr pat = patPair_.first;
    float arc = pat->orbit_.harc * 2;
    return
        pat->toWorld_ *
        expand3To4(makeRotX3f(arc)) *
        pat->toOYZ_;
}

std::string SymmRotation::GetDescription(void)
{
    stringstream ss;
    ss << GetName() << " [" << size() << "]";
    return ss.str();
}

std::string SymmRotation::GetName(void)
{
    return "Rotation";
}

bool SymmRotation::IsSeries(void)
{
    int cnt = 1;
    OrbitVerticeMap::iterator it, jt = overt_.begin();
    for (it = jt++; jt != overt_.end(); ++it, ++jt, ++cnt) {
        if (1 < jt->first - it->first) break;
    }
    return cnt == overt_.size(); // last one on orbit
}

bool SymmRotation::Finalize(void)
{
    if (!patPair_.first->Finalize() ||
        !patPair_.second->Finalize())
        return false;

    if (size() == capacity()) {
        overt_[-1] = overt_[capacity()-1];
        overt_.erase( boost::prior( overt_.end() ) );
    } else {
        OrbitVerticeMap::iterator it, jt = overt_.begin();
        for (it = jt++; jt != overt_.end(); ++it, ++jt) {
            if (1 < jt->first - it->first) break;
        }
        if (jt == overt_.end()) {
            warning("SymmRotation::Finalize - error");
            return false;
        }
        jt = overt_.end();
        for (--jt; it != jt; --jt) {
            overt_[jt->first-capacity()] = overt_[jt->first];
        }
        overt_.erase(++it, overt_.end());
    }

    return true;
}
