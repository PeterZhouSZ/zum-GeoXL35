#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Symmetry/SymmPlane.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

SymmPlane::SymmPlane(const std::vector<Vector3f>& vertices,
                     const std::vector<mpcard>& vindx)
: vertices_(vertices), vindx_(vindx)
{
    OOBBox3f oobb(vertices);

    BoundingBox3f bb = oobb.getInternalBoundingBox();
    Matrix4f toW = invertFrame(oobb.getTransformation());

    cen_ = transformVector3f(toW, bb.getCenter());

    end_[0] = transformVector3f(toW,
        bb.getCenter() + makeVector3f(1,0,0) * bb.getSideLength(0));
    end_[1] = transformVector3f(toW,
        bb.getCenter() + makeVector3f(0,1,0) * bb.getSideLength(1));
    end_[2] = transformVector3f(toW,
        bb.getCenter() + makeVector3f(0,0,1) * bb.getSideLength(2));

    nn_indx = 0;
    float shortestLength = norm(cen_ - end_[0]);
    for (mpcard ii = 1; ii < 3; ++ii) {
        float l = norm(cen_ - end_[ii]);
        if (l < shortestLength) {
            shortestLength = l;
            nn_indx = ii;
        }
    }
}

std::string SymmPlane::GetDescription(void)
{
    stringstream ss;
    ss << GetName() << " [" << size() << "]";
    return ss.str();
}

std::string SymmPlane::GetName(void)
{
    return "CoPlanarity";
}

Matrix4f SymmPlane::Get1StepTransformation(void)
{
    return IDENTITY4F;
}

void SymmPlane::DrawWithDR(const Vector3f& color) const
{
    Vector3f v0 = end_[(nn_indx+1)%3] - cen_;
    Vector3f v1 = end_[(nn_indx+2)%3] - cen_;
    debugRenderer->addQuad(
        cen_ + v0 + v1,
        cen_ + v0 - v1,
        cen_ - v0 - v1,
        cen_ - v0 + v1,
        color);

    for (size_t ii = 0; ii < vertices_.size(); ++ii) {
        debugRenderer->addPoint(
            vertices_[ii], makeVector3f(1, 0, 1));
    }
}
