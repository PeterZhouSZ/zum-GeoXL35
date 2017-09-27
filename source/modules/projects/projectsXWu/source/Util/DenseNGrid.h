#ifndef DenseNGrid_H
#define DenseNGrid_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include <boost/bimap/unordered_set_of.hpp>
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

template< unsigned D_ = 1 >
struct CellTupleT
{
public:
    CellTupleT(void) { tuple.fill(0.f); }
public:
    std::array<float, D_> tuple;
};

template< typename CellT, unsigned D_ = 3 >
class DenseNGrid
{
public:
    static const unsigned Dim = D_;
    typedef boost::shared_ptr< DenseNGrid<CellT, D_> > Ptr;
    typedef boost::shared_ptr< const DenseNGrid<CellT, D_> > ConstPtr;

    typedef int key_t;
    typedef float pos_t;
    typedef std::array<int, D_> key_type;
    typedef StaticVector<float, D_> pos_type;

public:
    DenseNGrid(const float& cellSize = 1.f);
    ~DenseNGrid(void);

    size_t size(void) { return cell_grid.size(); }
    size_t size(unsigned id) { return size_cache[id]; }

    key_type GetCellIndex(const pos_type& pos);
    key_type GetCellIndex(const sn_type& sn);
    sn_type GetSeqNumber(const key_type& key);
    bool Contains(const key_type& key);
    CellPtr GetCellPtr(const key_type& key);
    CellPtr GetCellPtr(const sn_type& sn);
    pos_type GetCellCenter(const key_type& key);
    pos_type GetCellCenter(const pos_type& pos);
    pos_type GetCellCenter(const sn_type& sn);
    BoundingBox3f GetCellBBox(const key_type& key);
    BoundingBox3f GetCellBBox(const pos_type& pos);
    BoundingBox3f GetCellBBox(const sn_type& sn);

    void DrawWithDR(void);

public:
    float cell_size;
    std::deque< CellT > data_;
    std::array<key_t, D_> size_;
};

template< typename CellT, unsigned D_ = 3 >
DenseNGrid<CellT, D_>::DenseNGrid(const float& cellSize)
{
    Clear();
    cell_size = cellSize;
}

template< typename CellT, unsigned D_ = 3 >
DenseNGrid<CellT, D_>::~DenseNGrid(void)
{
    Clear();
}

template< typename CellT, unsigned D_ = 3 >
void DenseNGrid<CellT, D_>::Clear(void)
{
    cell_size = 1.f;
    data_.clear();
}

template< typename CellT, unsigned D_ = 3 >
void DenseNGrid<CellT, D_>::DrawWithDR(const pos_type& color)
{
}

template< typename CellT, unsigned D_ = 3 >
typename DenseNGrid<CellT, D_>::key_type DenseNGrid<CellT, D_>::GetCellIndex(const pos_type& pos)
{
    const pos_type& pn = pos / cell_size;
    key_type k;
    for (unsigned ii = 0; ii < D_; ++ii) k[ii] = static_cast<key_t>(floor(pn[ii]));
    return k;
}

template< typename CellT, unsigned D_ = 3 >
typename DenseNGrid<CellT, D_>::key_type DenseNGrid<CellT, D_>::GetCellIndex(const sn_type& sn)
{
    return var_bimap.left.at(sn);
}

template< typename CellT, unsigned D_ = 3 >
typename DenseNGrid<CellT, D_>::sn_type DenseNGrid<CellT, D_>::GetSeqNumber(const key_type& key)
{
    return var_bimap.right.at(key);
}

template< typename CellT, unsigned D_ = 3 >
bool DenseNGrid<CellT, D_>::Contains(const key_type& key)
{
    return cell_grid.find(key) != cell_grid.end();
}

template< typename CellT, unsigned D_ = 3 >
typename DenseNGrid<CellT, D_>::CellPtr DenseNGrid<CellT, D_>::GetCellPtr(const key_type& key)
{
    return cell_grid.at(key);
}

template< typename CellT, unsigned D_ = 3 >
typename DenseNGrid<CellT, D_>::CellPtr DenseNGrid<CellT, D_>::GetCellPtr(const sn_type& sn)
{
    return GetCellPtr(GetCellIndex(sn));
}

template< typename CellT, unsigned D_ = 3 >
typename DenseNGrid<CellT, D_>::pos_type DenseNGrid<CellT, D_>::GetCellCenter(const key_type& key)
{
    pos_type p;
    for (unsigned ii = 0; ii < D_; ++ii) p[ii] = static_cast<pos_t>(key[ii]) + .5f;
    return p * cell_size;
}

template< typename CellT, unsigned D_ = 3 >
typename DenseNGrid<CellT, D_>::pos_type DenseNGrid<CellT, D_>::GetCellCenter(const pos_type& pos)
{
    return GetCellCenter(GetCellIndex(pos));
}

template< typename CellT, unsigned D_ = 3 >
typename DenseNGrid<CellT, D_>::pos_type DenseNGrid<CellT, D_>::GetCellCenter(const sn_type& sn)
{
    return GetCellCenter(GetCellIndex(sn));
}

template< typename CellT, unsigned D_ = 3 >
BoundingBox3f DenseNGrid<CellT, D_>::GetCellBBox(const key_type& key)
{
    pos_type ccen = GetCellCenter(key);
    BoundingBox3f bb;
    bb.lowerCorner = bb.upperCorner = ccen;
    bb.addBorder(cell_size / 2.0f);
    return bb;
}

template< typename CellT, unsigned D_ = 3 >
BoundingBox3f DenseNGrid<CellT, D_>::GetCellBBox(const pos_type& pos)
{
    return GetCellBBox(GetCellIndex(pos));
}

template< typename CellT, unsigned D_ = 3 >
BoundingBox3f DenseNGrid<CellT, D_>::GetCellBBox(const sn_type& sn)
{
    return GetCellBBox(GetCellIndex(sn));
}

#endif
