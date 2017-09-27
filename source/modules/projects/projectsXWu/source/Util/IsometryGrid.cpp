#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Util/IsometryGrid.h"
#include "Util/NoUse.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

IsometryGrid::IsometryGrid(const float& l_th, const float& a_th) :
length_th(l_th), angle_th(a_th)
{
}

bool IsometryGrid::Insert(const Matrix4f& M_)
{
    Vector6f v6 = NU::MatrixToVec6DF(M_);
    return Insert(v6);
}

bool IsometryGrid::Insert(const Vector6f& V_)
{
    KeyT_ key;
    {
        int ii = 0;
        for (; ii < 3; ++ii) {
            key[ii] = (int)(V_[ii] / length_th);
        }
        for (; ii < 6; ++ii) {
            key[ii] = (int)(V_[ii] / angle_th);
        }
    }

    if (mGrid.find(key) == mGrid.end()) {
        mGrid.insert(key);
        return true;
    }
    return false;
}

PlaneGrid::PlaneGrid(const float& l_th, const float& a_th) :
length_th(l_th), angle_th(a_th)
{
}

bool PlaneGrid::Insert(Vector3f N_, Vector3f P_)
{
    if (numeric_limits<float>::epsilon() < abs(N_[0])) {
        if (0 > N_[0]) N_ *= -1.f;
    } else if (numeric_limits<float>::epsilon() < abs(N_[1])) {
        if (0 > N_[1]) N_ *= -1.f;
    } else if (numeric_limits<float>::epsilon() > N_[2]) N_ *= -1.f;
    float d = P_*N_;
    KeyT_ key;
    {
        int ii = 0;
        for (; ii < 3; ++ii) {
            key[ii] = (int)(N_[ii] / angle_th);
        }
        key[ii] = (int)(d / length_th);
    }
    if (mGrid.find(key) == mGrid.end()) {
        mGrid.insert(key);
        return true;
    }
    return false;
}
