#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Symmetry/SymmReflective.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
void SymmReflective::Insert(const SymmFeatLinePair& sflp, const LineDir& dir)
{
    featPairs_.push_back(sflp);
    lineDir_.push_back(dir);
}

void SymmReflective::Initialize(const Matrix4f& T_, const Vector3f& N_)
{
    trans4_ = T_;
    normal_ = N_;

    std::vector<Vector3f> symPoints;
    for (size_t ii = 0; ii < featPairs_.size(); ++ii) {
        symPoints.push_back(featPairs_[ii].first->cen);
        symPoints.push_back(featPairs_[ii].second->cen);
    }
    update(symPoints);
}

void SymmReflective::DrawPlane(const Vector3f& color)
{
    Vector3f v0 = end_proj[(nn_indx+1)%3] - cen_proj;
    Vector3f v1 = end_proj[(nn_indx+2)%3] - cen_proj;
    debugRenderer->addQuad(
        cen_proj + v0 + v1,
        cen_proj + v0 - v1,
        cen_proj - v0 - v1,
        cen_proj - v0 + v1,
        color);
}

void SymmReflective::DrawWithDR(const Vector3f& color)
{
    DrawPlane(color);
	//debugRenderer->addLine(
	//	cen_proj + v0,
	//	cen_proj - v0,
	//	makeVector3f(1, 0, 0),
	//	makeVector3f(0, 0, 1),
	//	3);
	//debugRenderer->addLine(
	//	cen_proj + v1,
	//	cen_proj - v1,
	//	makeVector3f(1, 0, 0),
	//	makeVector3f(0, 0, 1),
	//	3);

    for (size_t ii = 0; ii < featPairs_.size(); ++ii) {
        debugRenderer->addLine(
            featPairs_[ii].first->pos0,
            featPairs_[ii].first->pos1,
            makeVector3f(0, 1, 0),
            makeVector3f(0, 1, 0),
            3);
        debugRenderer->addLine(
            featPairs_[ii].second->pos0,
            featPairs_[ii].second->pos1,
            makeVector3f(0, 1, 0),
            makeVector3f(0, 1, 0),
            3);
        debugRenderer->addLine(
            featPairs_[ii].first->cen,
            featPairs_[ii].second->cen,
            makeVector3f(1, 0, 0),
            makeVector3f(0, 0, 1),
            3);

        {
            //Vector3f d0_f = mSymmFeat[ii].first->pos0 - cen_proj;
            //Vector3f d1_f = mSymmFeat[ii].first->pos1 - cen_proj;
            //Vector3f d0_s = mSymmFeat[ii].second->pos0 - cen_proj;
            //Vector3f d1_s = mSymmFeat[ii].second->pos1 - cen_proj;
            //if (0 < (d0_f*mNormal)*(d1_f*mNormal) &&
            //    0 < (d0_s*mNormal)*(d1_s*mNormal)) continue;
            //debugRenderer->addLine(
            //    mSymmFeat[ii].first->pos0,
            //    mSymmFeat[ii].first->pos1,
            //    makeVector3f(0, 1, 0),
            //    makeVector3f(0, 1, 0),
            //    3);
            //debugRenderer->addLine(
            //    mSymmFeat[ii].second->pos0,
            //    mSymmFeat[ii].second->pos1,
            //    makeVector3f(0, 1, 0),
            //    makeVector3f(0, 1, 0),
            //    3);
            //debugRenderer->addLine(
            //    mSymmFeat[ii].first->cen,
            //    cen_proj,
            //    makeVector3f(1, 0, 0),
            //    makeVector3f(0, 0, 1),
            //    3);
            //debugRenderer->addLine(
            //    cen_proj,
            //    mSymmFeat[ii].second->cen,
            //    makeVector3f(1, 0, 0),
            //    makeVector3f(0, 0, 1),
            //    3);
        }

    }
}

Vector3f SymmReflective::GetReflectedPos(const Vector3f& p)
{
    // call GetUpdatedTransformation first
    return transformVector3f(trans4_, p);
}

bool SymmReflective::GenerateReflection(
                          const Vector3f& p0, const Vector3f& p1,
                          Matrix3f& T3, Vector3f& N, Vector3f& C)
{
    N = p1 - p0;
    float l = norm(N);
    if (1e-6 > l) return false;
    N /= l;
    C = (p0 + p1) / 2.f;
    if (std::numeric_limits<float>::epsilon() < abs(N[0])) {
        if (0 > N[0]) N *= -1.f;
    } else if (std::numeric_limits<float>::epsilon() < abs(N[1])) {
        if (0 > N[1]) N *= -1.f;
    } else if (std::numeric_limits<float>::epsilon() > N[2]) N *= -1.f;
    {
        T3[0][0] = 1 - 2 * N[0] * N[0];
        T3[0][1] = - 2 * N[0] * N[1];
        T3[0][2] = - 2 * N[0] * N[2];

        T3[1][0] = - 2 * N[0] * N[1];
        T3[1][1] = 1 - 2 * N[1] * N[1];
        T3[1][2] = - 2 * N[1] * N[2];

        T3[2][0] = - 2 * N[0] * N[2];
        T3[2][1] = - 2 * N[1] * N[2];
        T3[2][2] = 1 - 2 * N[2] * N[2];
    }
    return true;
}

bool SymmReflective::OnFrontSide(const Vector3f& p)
{
    return 0 > (p - cen_proj) * normal_;
}

Matrix4f SymmReflective::Get1StepTransformation(void)
{
    return trans4_;
}

bool SymmReflective::
GetUpdatedTransformation(PointSet  const& deformed,
                         AAT       const& defAAT,
                         Matrix4f& Tf, Matrix4f& Tb)
{
    std::vector<Vector3f> symPoints;
    Vector3f pts0 = NULL_VECTOR3F, pts1 = NULL_VECTOR3F;
    std::vector<SymmFeatVertexPair>::iterator mj = vertPairs_.begin();
    for (; mj != vertPairs_.end(); ++mj) {
        Vector3f p0 = deformed.get3f(mj->first, defAAT);
        Vector3f p1 = deformed.get3f(mj->second, defAAT);
        pts0 += p0;
        pts1 += p1;
        symPoints.push_back(p0);
        symPoints.push_back(p1);
    }
    pts0 /= vertPairs_.size();
    pts1 /= vertPairs_.size();
    Matrix3f T3; Vector3f N, C;
    if (!GenerateReflection(
        pts0, pts1, T3, N, C))
        return false;
    Tf = makeTranslation4f(C) * expand3To4(T3) * makeTranslation4f(-C);
    Tb = Tf;

    trans4_ = Tf;
    normal_ = N;
    update(symPoints);

    return true;
}

std::string SymmReflective::GetDescription(void)
{
    std::stringstream ss;
    ss << GetName() << " [" << size() << "]";
    return ss.str();
}

std::string SymmReflective::GetName(void)
{
    return "Reflection";
}

void SymmReflective::update(const std::vector<Vector3f>& symPoints)
{
    OOBBox3f oobb(symPoints);

    BoundingBox3f bb = oobb.getInternalBoundingBox();
    Matrix4f toW = invertFrame(oobb.getTransformation());

    Vector3f cen = transformVector3f(toW, bb.getCenter());
    cen_proj = (cen + transformVector3f(trans4_, cen)) * 0.5f;

    Vector3f p0 = transformVector3f(toW,
        bb.getCenter() + makeVector3f(1,0,0) * bb.getSideLength(0));
    Vector3f p1 = transformVector3f(toW,
        bb.getCenter() + makeVector3f(0,1,0) * bb.getSideLength(1));
    Vector3f p2 = transformVector3f(toW,
        bb.getCenter() + makeVector3f(0,0,1) * bb.getSideLength(2));
    end_proj[0] = (p0 + transformVector3f(trans4_, p0)) * 0.5f;
    end_proj[1] = (p1 + transformVector3f(trans4_, p1)) * 0.5f;
    end_proj[2] = (p2 + transformVector3f(trans4_, p2)) * 0.5f;

    nn_indx = 0;
    float shortestLength = norm(cen_proj - end_proj[0]);
    for (mpcard ii = 1; ii < 3; ++ii) {
        float l = norm(cen_proj - end_proj[ii]);
        if (l < shortestLength) {
            shortestLength = l;
            nn_indx = ii;
        }
    }
}

void SymmReflective::DrawWithDR(
                PointSet  const& deformed,
                AAT       const& defAAT
                )
{
    DrawPlane(makeVector3f(1,1,0));
    std::vector<SymmFeatVertexPair>::iterator mj = vertPairs_.begin();
    for (; mj != vertPairs_.end(); ++mj) {
        Vector3f p0 = deformed.get3f(mj->first, defAAT);
        Vector3f p1 = deformed.get3f(mj->second, defAAT);
        debugRenderer->addLine(
            p0, p1,
            makeVector3f(1, 0, 0),
            makeVector3f(0, 0, 1),
            3);
    }
}
