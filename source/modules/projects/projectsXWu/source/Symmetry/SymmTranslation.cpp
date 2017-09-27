#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Symmetry/SymmTranslation.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

SymmTranslation::SymmTranslation(PatternTranslation::Ptr p0, PatternTranslation::Ptr p1)
{
    patPair_.first = p0;
    patPair_.second = p1;
}

void SymmTranslation::SetupVertice(const SymmFeatLine::Ptr& sfl, const int& idx)
{
    std::deque<mpcard> verts = sfl->vertice;
    overt_[idx] = verts;
}

bool SymmTranslation::AddVertice(const SymmFeatLine::Ptr& sfl, int& idx)
{
    int idx1;

    if (patPair_.first->PointOnOrbit(sfl->pos0, idx) &&
        patPair_.second->PointOnOrbit(sfl->pos1, idx1))
    {
        if (idx == idx1) {
            SymmFeatLine::Ptr line = sfl;
            patPair_.first->AddPoint(line->pos0, idx);
            patPair_.second->AddPoint(line->pos1, idx);
            SetupVertice(line, idx);
            return true;
        }
    }

    if (patPair_.first->PointOnOrbit(sfl->pos1, idx) &&
        patPair_.second->PointOnOrbit(sfl->pos0, idx1))
    {
        if (idx == idx1) {
            SymmFeatLine::Ptr line = sfl->CopyTo();
            line->Reverse();
            patPair_.first->AddPoint(line->pos1, idx);
            patPair_.second->AddPoint(line->pos0, idx);
            SetupVertice(line, idx);
            return true;
        }
    }
    return false;
}

void SymmTranslation::DrawWithDR(void) const
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

Matrix4f SymmTranslation::Get1StepTransformation(void)
{
    PatternTranslation::Ptr pat = patPair_.first;
    return makeTranslation4f(pat->direction_ * pat->orbit_.step);
}

std::string SymmTranslation::GetDescription(void)
{
    stringstream ss;
    ss << GetName() << " [" << size() << "]";
    return ss.str();
}

std::string SymmTranslation::GetName(void)
{
    return "Translation";
}
