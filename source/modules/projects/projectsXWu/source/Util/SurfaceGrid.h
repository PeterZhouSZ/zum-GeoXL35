#ifndef SurfaceGrid_H
#define SurfaceGrid_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include <boost/bimap/unordered_set_of.hpp>
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

struct CellListT
{
public:
    bool empty(void) { return points.empty(); }
public:
    std::deque<unsigned> points;
};
typedef boost::shared_ptr< CellListT > CellListPtr;

template< typename CellT, unsigned D_ = 3 >
class SurfaceGrid
{
public:
    static const unsigned Dim = D_;
    typedef boost::shared_ptr< SurfaceGrid<CellT, D_> > Ptr;
    typedef boost::shared_ptr< const SurfaceGrid<CellT, D_> > ConstPtr;

    typedef int key_t;
    typedef float pos_t;
    typedef StaticVector<int, D_> key_type;
    typedef StaticVector<float, D_> pos_type;
    typedef unsigned snr_type;

protected:
    struct hash_value : std::unary_function< key_type, boost::uint64_t > {
        boost::uint64_t operator() (key_type const& v) const {
            return boost::hash_range(v.begin(), v.end());
        }
    };

public:
    typedef CellT CellType;
    typedef boost::shared_ptr< CellT > CellPtr;
    typedef boost::unordered_map< key_type, CellPtr, hash_value > grid_map_type;
    typedef boost::bimap<
        boost::bimaps::unordered_set_of< snr_type >,
        boost::bimaps::unordered_set_of< key_type, hash_value >
    > var_bimap_type;

public:
    SurfaceGrid(void);
    SurfaceGrid(const float& cellSize);
    ~SurfaceGrid(void);

    size_t size(void) { return cell_grid.size(); }

    key_type AddPoint(const pos_type& point);
    void AddCell(const key_type& key, CellPtr cell);
    void Clear(void);
    void ClearEmptyCells(const unsigned& num = 1);

    key_type GetCellIndex(const pos_type& pos);
    key_type GetCellIndex(const snr_type& snr);
    bool Contains(const key_type& key);
    CellPtr GetCellPtr(const key_type& key);
    CellPtr GetCellPtr(const snr_type& snr);
    CellPtr operator[] (const snr_type& snr) const { return GetCellPtr(snr); }
    CellPtr operator[] (const key_type& key) const { return GetCellPtr(key); }
    pos_type GetCellCenter(const key_type& key);
    pos_type GetCellCenter(const pos_type& pos);
    pos_type GetCellCenter(const snr_type& snr);
    snr_type GetCellSeqNr(const key_type& key);
    snr_type GetCellSeqNr(const pos_type& pos);
    BoundingBox3f GetCellBBox(const key_type& key);
    BoundingBox3f GetCellBBox(const pos_type& pos);
    BoundingBox3f GetCellBBox(const snr_type& snr);

    void DrawWithDR(const Vector3f& color);

public:
    grid_map_type cell_grid;
    var_bimap_type var_bimap;
    float cell_size;
};

template< typename CellT, unsigned D_ = 3 >
SurfaceGrid<CellT, D_>::SurfaceGrid(void)
{
    Clear();
}

template< typename CellT, unsigned D_ = 3 >
SurfaceGrid<CellT, D_>::SurfaceGrid(const float& cellSize)
{
    Clear();
    cell_size = cellSize;
}

template< typename CellT, unsigned D_ = 3 >
SurfaceGrid<CellT, D_>::~SurfaceGrid(void)
{
    Clear();
}

template< typename CellT, unsigned D_ = 3 >
void SurfaceGrid<CellT, D_>::Clear(void)
{
    cell_grid.clear();
    var_bimap.clear();
    cell_size = -.1f;
}

template< typename CellT, unsigned D_ = 3 >
void SurfaceGrid<CellT, D_>::ClearEmptyCells(const unsigned& num)
{
    grid_map_type::iterator it = cell_grid.begin();
    while (it != cell_grid.end()) {
        if (num > it->second->size()) {
            cell_grid.erase(it++);
        }
        else {
            ++it;
        }
    }
    var_bimap.clear();
    for (it = cell_grid.begin(); it != cell_grid.end(); ++it) {
        var_bimap.insert(var_bimap_type::value_type(var_bimap.size(), it->first));
    }
}

template< typename CellT, unsigned D_ = 3 >
typename SurfaceGrid<CellT, D_>::key_type
SurfaceGrid<CellT, D_>::AddPoint(pos_type const& point)
{
    key_type key = GetCellIndex(point);
    if (cell_grid.count(key) == 0) {
        CellPtr pgc(new CellT);
        cell_grid[key] = pgc;
        var_bimap.insert(var_bimap_type::value_type(var_bimap.size(), key));
    }
    else {
    }
    return key;
}

template< typename CellT, unsigned D_ = 3 >
void SurfaceGrid<CellT, D_>::AddCell(const key_type& key, CellPtr cell)
{
    if (cell_grid.count(key) == 0) {
        if (nullptr == cell) {
            cell = boost::shared_ptr<CellT>(new CellT);
        }
        cell_grid[key] = cell;
        var_bimap.insert(var_bimap_type::value_type(var_bimap.size(), key));
    }
    else {
    }
}

template< typename CellT, unsigned D_ = 3 >
void SurfaceGrid<CellT, D_>::DrawWithDR(const Vector3f& color)
{
    for (grid_map_type::iterator it = cell_grid.begin();
        it != cell_grid.end(); ++it) {
            const BoundingBox3f& bb = GetCellBBox(it->first);
            debugRenderer->addBoundingBox(bb, color, 1);
    }
}

template< typename CellT, unsigned D_ = 3 >
typename SurfaceGrid<CellT, D_>::key_type
SurfaceGrid<CellT, D_>::GetCellIndex(const pos_type& pos)
{
    const pos_type& pn = pos / cell_size;
    key_type key;
    for (unsigned ii = 0; ii < D_; ++ii) key[ii] = static_cast<key_t>(floor(pn[ii]));
    return key;
}

template< typename CellT, unsigned D_ = 3 >
typename SurfaceGrid<CellT, D_>::key_type
SurfaceGrid<CellT, D_>::GetCellIndex(const snr_type& snr)
{
    return var_bimap.left.at(snr);
}

template< typename CellT, unsigned D_ = 3 >
bool SurfaceGrid<CellT, D_>::Contains(const key_type& key)
{
    return (0 < cell_grid.count(key));
}

template< typename CellT, unsigned D_ = 3 >
typename SurfaceGrid<CellT, D_>::CellPtr
SurfaceGrid<CellT, D_>::GetCellPtr(const key_type& key)
{
    return cell_grid.at(key);
}

template< typename CellT, unsigned D_ = 3 >
typename SurfaceGrid<CellT, D_>::CellPtr
SurfaceGrid<CellT, D_>::GetCellPtr(const snr_type& snr)
{
    return cell_grid.at(GetCellIndex(snr));
}

template< typename CellT, unsigned D_ = 3 >
typename SurfaceGrid<CellT, D_>::pos_type
SurfaceGrid<CellT, D_>::GetCellCenter(const key_type& key)
{
    pos_type p;
    for (unsigned ii = 0; ii < D_; ++ii) p[ii] = static_cast<pos_t>(key[ii]) + .5f;
    return p * cell_size;
}

template< typename CellT, unsigned D_ = 3 >
typename SurfaceGrid<CellT, D_>::pos_type
SurfaceGrid<CellT, D_>::GetCellCenter(const pos_type& pos)
{
    return GetCellCenter(GetCellIndex(pos));
}

template< typename CellT, unsigned D_ = 3 >
typename SurfaceGrid<CellT, D_>::pos_type
SurfaceGrid<CellT, D_>::GetCellCenter(const snr_type& snr)
{
    return GetCellCenter(GetCellIndex(snr));
}

template< typename CellT, unsigned D_ = 3 >
typename SurfaceGrid<CellT, D_>::snr_type
SurfaceGrid<CellT, D_>::GetCellSeqNr(const key_type& key)
{
    // NOTE: reverse look-up is very slow
    return var_bimap.right.at(key);
}

template< typename CellT, unsigned D_ = 3 >
typename SurfaceGrid<CellT, D_>::snr_type
SurfaceGrid<CellT, D_>::GetCellSeqNr(const pos_type& pos)
{
    return GetCellSeqNr(GetCellIndex(pos));
}

template< typename CellT, unsigned D_ = 3 >
BoundingBox3f SurfaceGrid<CellT, D_>::GetCellBBox(const key_type& key)
{
    pos_type ccen = GetCellCenter(key);
    BoundingBox3f bb;
    bb.lowerCorner = bb.upperCorner = ccen;
    bb.addBorder(cell_size / 2.0f);
    return bb;
}

template< typename CellT, unsigned D_ = 3 >
BoundingBox3f SurfaceGrid<CellT, D_>::GetCellBBox(const pos_type& pos)
{
    return GetCellBBox(GetCellIndex(pos));
}

template< typename CellT, unsigned D_ = 3 >
BoundingBox3f SurfaceGrid<CellT, D_>::GetCellBBox(const snr_type& snr)
{
    return GetCellBBox(GetCellIndex(snr));
}


#endif
