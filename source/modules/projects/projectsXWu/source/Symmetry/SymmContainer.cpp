#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Symmetry/SymmContainer.h"
//---------------------------------------------------------------------------
#include "GeometricTools.h"
//---------------------------------------------------------------------------

bool SymmContainer::
GetUpdatedTransformation(PointSet  const& deformed,
                         AAT       const& defAAT,
                         Matrix4f& Tf, Matrix4f& Tb)
{
    // (n-1)-(n-1) estimation: guaranteed to be series
    {
        std::vector<Vector3f> pts0, pts1;
        OrbitVerticeMap::iterator mi, mj = overt_.begin();
        for (mi = mj++; mj != overt_.end(); ++mi, ++mj) {
            std::deque<mpcard> vinx0 = mi->second, vinx1 = mj->second;
            if (vinx0.size() != vinx1.size()) {
                warning("SymmTranslation::GetUpdatedTransformation");
                continue;
            }
            std::deque<mpcard>::iterator vi0 = vinx0.begin(), vi1 = vinx1.begin();
            for (; vi0 != vinx0.end(); ++vi0, ++vi1) {
                pts0.push_back(deformed.get3f(*vi0, defAAT));
                pts1.push_back(deformed.get3f(*vi1, defAAT));
            }
        }
        //return computeRigidTransformationFromCorrespondences(pts0, pts1);
        Tf = computeRigidTransformationFromCorrespondences(pts1, pts0);
    }
    {
        std::vector<Vector3f> pts0, pts1;
        OrbitVerticeMap::reverse_iterator mi, mj = overt_.rbegin();
        for (mi = mj++; mj != overt_.rend(); ++mi, ++mj) {
            std::deque<mpcard> vinx0 = mi->second, vinx1 = mj->second;
            if (vinx0.size() != vinx1.size()) {
                warning("SymmTranslation::GetUpdatedTransformation");
                continue;
            }
            std::deque<mpcard>::iterator vi0 = vinx0.begin(), vi1 = vinx1.begin();
            for (; vi0 != vinx0.end(); ++vi0, ++vi1) {
                pts0.push_back(deformed.get3f(*vi0, defAAT));
                pts1.push_back(deformed.get3f(*vi1, defAAT));
            }
        }
        Tb = computeRigidTransformationFromCorrespondences(pts1, pts0);
    }

    return true;
}

bool SymmContainer::
GetCorrespondence(
                  std::vector<Vector3f>& points0,
                  std::vector<Vector3f>& points1
                  )
{
    return true;
}

bool SymmContainer::IsSeries(void)
{
    int cnt = 1;
    OrbitVerticeMap::iterator it, jt = overt_.begin();
    for (it = jt++; jt != overt_.end(); ++it, ++jt, ++cnt) {
        if (1 < jt->first - it->first) break;
    }
    return cnt == overt_.size();
}

void SymmContainer::DrawWithDR(
                               PointSet  const& deformed,
                               AAT       const& defAAT
                               )
{
    if (overt_.empty()) return;
    OrbitVerticeMap::iterator mi, mj = overt_.begin();
    for (mi = mj++; mj != overt_.end(); ++mi, ++mj) {
        std::deque<mpcard> vinx0 = mi->second, vinx1 = mj->second;
        std::deque<mpcard>::iterator vi0 = vinx0.begin(), vi1 = vinx1.begin();
        for (; vi0 != vinx0.end(); ++vi0, ++vi1) {
            Vector3f const point0_def = deformed.get3f(*vi0, defAAT);
            Vector3f const point1_def = deformed.get3f(*vi1, defAAT);
            debugRenderer->addLine(
                point0_def, point1_def,
                makeVector3f(1, 0, 0),
                makeVector3f(0, 0, 1),
                3);
        }
    }
}
