#ifndef IsometryGrid_H
#define IsometryGrid_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class IsometryGrid
{
public:
    typedef boost::array< boost::int64_t, 6 > KeyT_;

public:
    IsometryGrid(const float& l_th, const float& a_th);
    bool Insert(const Matrix4f& M_);
    bool Insert(const Vector6f& V_);

private:
    struct hash_value : std::unary_function< KeyT_, boost::uint64_t > {
        boost::uint64_t operator() (KeyT_ const& v) const {
            return boost::hash_range(v.begin(), v.end());
        }
    };
    boost::unordered_set< KeyT_, hash_value > mGrid;
    float length_th;
    float angle_th; // angular measure
};

class PlaneGrid
{
public:
    typedef boost::array< boost::int64_t, 4 > KeyT_;

public:
    PlaneGrid(const float& l_th, const float& a_th);
    bool Insert(Vector3f N_, Vector3f P_);

private:
    struct hash_value : std::unary_function< KeyT_, boost::uint64_t > {
        boost::uint64_t operator() (KeyT_ const& v) const {
            return boost::hash_range(v.begin(), v.end());
        }
    };
    boost::unordered_set< KeyT_, hash_value > mGrid;
    float length_th;
    float angle_th; // circular measure
};

#endif
