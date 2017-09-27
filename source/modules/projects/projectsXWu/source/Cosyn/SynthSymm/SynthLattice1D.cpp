#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "SynthLattice1D.h"
//---------------------------------------------------------------------------
#include "GeometricTools.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

SynthLattice1D::SynthLattice1D(void)
{
    trans_ = Eigen::Matrix4f::Identity();
}

void SynthLattice1D::SetTrans(const Eigen::Matrix4f& trans)
{
    trans_ = trans;
    dir_ = trans_.col(3).head<3>();
}

void SynthLattice1D::DebugDraw(const Vector3f& color, const float32& radius)
{
    for (size_t ei = 0; ei < elem_.size(); ++ei) {
        Eigen::Vector3f pos = base_ + elem_[ei] * dir_;
        debugRenderer->addFastSphere(
            EigenAdaptor::FromEigen(pos),
            radius, color, false);
    }
    debugRenderer->addFastSphere(
        EigenAdaptor::FromEigen(base_),
        radius * 1.5, color, false);

    Eigen::Vector3f arrow = base_ + dir_;
    debugRenderer->addFineArrow(
        EigenAdaptor::FromEigen(base_),
        EigenAdaptor::FromEigen(arrow),
        color, radius * 0.4);

    Eigen::Vector3f begP = base_ + elem_[0] * dir_;
    Eigen::Vector3f endP = base_ + elem_[elem_.size()-1] * dir_;
    debugRenderer->addLine(
        EigenAdaptor::FromEigen(begP),
        EigenAdaptor::FromEigen(endP),
        color, color,
        //radius * 2
        2
        );
}

Eigen::Matrix4f SynthLattice1D::ComputeTransformation(
    const Eigen::Matrix4f& source,
    const Eigen::Matrix4f& target
    )
{
    Eigen::Matrix4f ret = Eigen::Matrix4f::Identity();
    Eigen::Vector3f cenSorc = source.col(3).head<3>();
    Eigen::Vector3f cenTarg = target.col(3).head<3>();
    ret.col(3).head<3>() = cenTarg - cenSorc;
    return ret;
}
