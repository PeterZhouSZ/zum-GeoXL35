#ifndef DenseGrid3_H
#define DenseGrid3_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "boost/multi_array.hpp"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

struct CellValue1T
{
public:
    CellValue1T(void) : value_(0.f) {}
public:
    float value_;
};
//typedef boost::multi_array<CellValue1T, 3> array_type;
//array_type::extent_gen extents;
//array_type arr(extents[2][3][4]);

template< typename CellT = float >
class DenseGrid3
{
public:
    typedef boost::shared_ptr< DenseGrid3<CellT> > Ptr;
    typedef boost::shared_ptr< const DenseGrid3<CellT> > ConstPtr;
    static const int Dim = 3;

    typedef float pos_t;
    typedef StaticVector<pos_t, Dim> pos_type;

    typedef boost::multi_array<typename CellT, Dim> array_type;
    typedef typename array_type::index key_t;
    typedef boost::array<key_t, Dim> key_type;
    typedef boost::array<key_t, Dim> shape_type;
    typedef boost::array<key_t, Dim> bases_type;
    typedef boost::multi_array_types::index_range i_range;
    typedef boost::multi_array_types::extent_range e_range;
    //typedef typename array_type::array_view<3>::type view_type;
    //typedef typename view_type::reference view_ref_t;

public:
    DenseGrid3(const BoundingBox3f& box,
        const float& cellSize = 1.f,
        const int& padding = 0 // expanded range allowing easier kernal operations
        );
    ~DenseGrid3(void);

public:
    bool InRange(const key_type& key);
    bool InRange(const pos_type& pos);
    key_type GetCellIndex(const pos_type& pos);
    pos_type GetCellCenter(const key_type& key);
    pos_type GetCellCenter(const pos_type& pos);
    BoundingBox3f GetCellBBox(const key_type& key);
    BoundingBox3f GetCellBBox(const pos_type& pos);

public:
    float GetMax(void);
    void Rescale(void);
    void CutOff(const float& thresh);

public:
    void DrawWithDR(const Vector3f& color);
    void glDraw(void);

public:
    float cSize_;
    array_type array_;
    shape_type shape_;
    bases_type bases_;
    int padding_;
};

typedef DenseGrid3<float> DenseGrid3f;

template< typename CellT >
float DenseGrid3<CellT>::GetMax(void)
{
    float maxVal = std::numeric_limits<float>::min();
    for (auto ii = array_.data(); ii < array_.data() + array_.num_elements(); ++ii) {
        if (*ii > maxVal) maxVal = *ii;
    }
    return maxVal;
}

template< typename CellT >
void DenseGrid3<CellT>::Rescale(void)
{
    float maxVal = GetMax();
    debugOutput << "maxi before rescaling: " << maxVal << "\n";
    if (0.001 > maxVal) return; // maxVal = 1.f;
    for (auto ii = array_.data(); ii < array_.data() + array_.num_elements(); ++ii) {
        *ii /= maxVal;
    }
    //debugOutput << GetMax() << "\n";
}

template< typename CellT >
void DenseGrid3<CellT>::CutOff(const float& thresh)
{
    for (auto ii = array_.data(); ii < array_.data() + array_.num_elements(); ++ii) {
        if (thresh > *ii) *ii = 0.f;
    }
}

template< typename CellT >
DenseGrid3<CellT>::DenseGrid3(const BoundingBox3f& box,
    const float& cellSize,
    const int& padding
    )
{
    cSize_ = cellSize;
    padding_ = padding;
    const Vector3f lowerCorner = box.lowerCorner;
    const Vector3f upperCorner = box.upperCorner;
    const int dim0 = ceil((upperCorner[0] - lowerCorner[0]) / cSize_);
    const int dim1 = ceil((upperCorner[1] - lowerCorner[1]) / cSize_);
    const int dim2 = ceil((upperCorner[2] - lowerCorner[2]) / cSize_);
    const int bas0 = floor(lowerCorner[0] / cSize_);
    const int bas1 = floor(lowerCorner[1] / cSize_);
    const int bas2 = floor(lowerCorner[2] / cSize_);
    {
        //debugOutput << lowerCorner << "\n" << upperCorner << "\n";
        //debugOutput << dim0 << " " << dim1 << " " << dim2 << "\n";
        //debugOutput << bas0 << " " << bas1 << " " << bas2 << "\n";
    }

    shape_ = { { dim0, dim1, dim2 } };
    bases_ = { { bas0, bas1, bas2 } };
    shape_type shapePad = { { dim0 + 2 * padding, dim1 + 2 * padding, dim2 + 2 * padding } };
    bases_type basesPad = { { bas0 - padding, bas1 - padding, bas2 - padding } };
    array_.resize(shapePad);
    array_.reindex(basesPad);
    std::fill(array_.data(), array_.data() + array_.num_elements(), 0.f);
}

template< typename CellT >
DenseGrid3<CellT>::~DenseGrid3(void)
{
}

template< typename CellT >
void DenseGrid3<CellT>::DrawWithDR(const Vector3f& color)
{
    for (array_type::index ii = bases_[0]; ii < bases_[0] + shape_[0]; ++ii) {
        for (array_type::index jj = bases_[1]; jj < bases_[1] + shape_[1]; ++jj) {
            for (array_type::index kk = bases_[2]; kk < bases_[2] + shape_[2]; ++kk) {
                const key_type key = { ii, jj, kk };
                const BoundingBox3f& bb = GetCellBBox(key);
                debugRenderer->addBoundingBox(bb, color, 1);
            }
        }
    }
}

template< typename CellT >
bool DenseGrid3<CellT>::InRange(const key_type& key)
{
    return
        bases_[0] <= key[0] && key[0] < bases_[0] + shape_[0] &&
        bases_[1] <= key[1] && key[1] < bases_[1] + shape_[1] &&
        bases_[2] <= key[2] && key[2] < bases_[2] + shape_[2];
}

template< typename CellT >
bool DenseGrid3<CellT>::InRange(const pos_type& pos)
{
    key_type key = GetCellIndex(pos);
    return InRange(key);
}

template< typename CellT >
typename DenseGrid3<CellT>::key_type
DenseGrid3<CellT>::GetCellIndex(const pos_type& pos)
{
    const pos_type& pn = pos / cSize_;
    key_type key;
    for (unsigned ii = 0; ii < Dim; ++ii) key[ii] = static_cast<key_t>(floor(pn[ii]));
    return key;
}

template< typename CellT >
typename DenseGrid3<CellT>::pos_type
DenseGrid3<CellT>::GetCellCenter(const key_type& key)
{
    pos_type p;
    for (unsigned ii = 0; ii < Dim; ++ii) p[ii] = static_cast<pos_t>(key[ii]) + .5f;
    return p * cSize_;
}

template< typename CellT >
typename DenseGrid3<CellT>::pos_type
DenseGrid3<CellT>::GetCellCenter(const pos_type& pos)
{
    return GetCellCenter(GetCellIndex(pos));
}

template< typename CellT >
BoundingBox3f DenseGrid3<CellT>::GetCellBBox(const key_type& key)
{
    pos_type ccen = GetCellCenter(key);
    BoundingBox3f bb;
    bb.lowerCorner = bb.upperCorner = ccen;
    bb.addBorder(cSize_ / 2.0f);
    return bb;
}

template< typename CellT >
BoundingBox3f DenseGrid3<CellT>::GetCellBBox(const pos_type& pos)
{
    return GetCellBBox(GetCellIndex(pos));
}

#endif
