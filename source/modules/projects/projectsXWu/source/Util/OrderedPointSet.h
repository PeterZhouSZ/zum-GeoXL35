#ifndef OrderedPointSet_H
#define OrderedPointSet_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

template< typename T_=float, size_t N_=3 >
class OrderedPointSet
{
public:
    typedef boost::array< T_, N_ > ElemT;

private:
    bool near_center(const ElemT& cen, const ElemT& e0, const ElemT& e1)
    {
        T_ d0 = 0, d1 = 0;
        for (size_t ii = 0; ii < N_; ++ii) {
            T_ td = cen[ii] - e0[ii];
            d0 += td * td;
            td = cen[ii] - e1[ii];
            d1 += td * td;
        }
        return d0 < d1;
    }

public:
    OrderedPointSet()
    {
        for (size_t ii = 0; ii < N_; ++ii) {
            mCenter[ii] = 0;
        }
    }

    void Insert(ElemT& val)
    {
        float num = (float)mPoints.size();
        for (size_t ii = 0; ii < N_; ++ii) {
            mCenter[ii] =
                mCenter[ii] * num / (num+1) +
                val[ii] / (num+1);
        }
        mPoints.push_back(val);
        Sort();
    }

    void Sort(void)
    {
        std::sort(mPoints.begin(), mPoints.end(),
            boost::bind(&OrderedPointSet<T_,N_>::near_center,
            this, mCenter, _1, _2));
    }

public:
    ElemT mCenter;
    std::vector<ElemT> mPoints;
};

typedef OrderedPointSet<float, 2> OrderedPointSet2f;
typedef OrderedPointSet2f::ElemT OPoint2f;
typedef OrderedPointSet<> OrderedPointSet3f;
typedef OrderedPointSet3f::ElemT OPoint3f;

#endif
