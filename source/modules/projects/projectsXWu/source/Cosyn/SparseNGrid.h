#ifndef SparseNGrid_H
#define SparseNGrid_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include <boost/bimap/unordered_set_of.hpp>
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

template< unsigned D_ = 3 >
class SparseNGrid
{
public:
    static const unsigned Dim = D_;
    typedef boost::shared_ptr< SparseNGrid<D_> > Ptr;
    typedef boost::shared_ptr< const SparseNGrid<D_> > ConstPtr;

    typedef int key_t;
    typedef float pos_t;
    typedef StaticVector<int, D_> key_type;
    typedef StaticVector<float, D_> pos_type;
    typedef unsigned sn_type;

public:
    struct GCIndexMap
    {
    public:
        size_t count(unsigned id) { return points.count(id); }
        size_t size(unsigned id) { return points[id].size(); }
    public:
        boost::unordered_map< unsigned, std::deque<unsigned> > points;
        //sn_type sn;
        //key_type key;
    };

protected:
    struct hash_value : std::unary_function< key_type, boost::uint64_t > {
        boost::uint64_t operator() (key_type const& v) const {
            return boost::hash_range(v.begin(), v.end());
        }
    };

public:
    typedef GCIndexMap CellType;
    typedef boost::shared_ptr< CellType > CellPtr;
    typedef boost::unordered_map< key_type, CellPtr, hash_value > grid_map_type;
    typedef boost::bimap<
        boost::bimaps::unordered_set_of< sn_type >,
        boost::bimaps::unordered_set_of< key_type, hash_value >
    > var_bimap_type;

public:
    SparseNGrid(void);
    SparseNGrid(const float& cellSize);
    ~SparseNGrid(void);

    size_t size(void) { return cell_grid.size(); }
    size_t size(unsigned id) { return size_cache[id]; }

    CellPtr AddPoint(pos_type const& point, key_type& key);
    void BuildVarBimap(void);
    void Clear(void);
    void CleanEmptyCells(void);

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

    void DrawWithDR(const pos_type& color);

public:
    grid_map_type cell_grid;
    var_bimap_type var_bimap;
    float cell_size;
    std::deque< unsigned > size_cache;
};

typedef SparseNGrid<3> LabelGrid;

template< unsigned D_ >
SparseNGrid<D_>::SparseNGrid(void)
{
    Clear();
}

template< unsigned D_ >
SparseNGrid<D_>::SparseNGrid(const float& cellSize)
{
    Clear();
    cell_size = cellSize;
}

template< unsigned D_ >
SparseNGrid<D_>::~SparseNGrid(void)
{
    Clear();
}

template< unsigned D_ >
void SparseNGrid<D_>::Clear(void)
{
    for (grid_map_type::iterator it = cell_grid.begin(); it != cell_grid.end(); ++it) {
        it->second.reset();
    }
    cell_grid.clear();
    var_bimap.clear();
    cell_size = -.1f;
    size_cache.clear();
}

template< unsigned D_ >
void SparseNGrid<D_>::CleanEmptyCells(void)
{
    for (grid_map_type::iterator it = cell_grid.begin(); it != cell_grid.end();) {
        if (it->points.empty()) {
            it = cell_grid.erase(it);
        } else {
            ++it;
        }
    }
    BuildVarBimap();
}

template< unsigned D_ >
typename SparseNGrid<D_>::CellPtr
    SparseNGrid<D_>::AddPoint(pos_type const& point, key_type& key)
{
    key = GetCellIndex(point);
    sn_type sn = cell_grid.size();
    if (cell_grid.count(key) == 0) {
        CellPtr pgc (new CellType);
        //pgc->sn = sn;
        //pgc->key = key;
        cell_grid[key] = pgc;
        var_bimap.insert( var_bimap_type::value_type(sn, key) );
        size_cache.push_back(0);
    } else {
        sn = GetSeqNumber(key);
    }
    ++ size_cache[sn];
    return cell_grid[key];
}

template< unsigned D_ >
void SparseNGrid<D_>::BuildVarBimap(void)
{
    //if (!var_bimap.empty()) return;
    var_bimap.clear();
    grid_map_type::const_iterator cell_it;
    cell_it = cell_grid.begin();
    for(size_t variable = 0; cell_it != cell_grid.end(); ++variable, ++cell_it) {
        var_bimap.insert( var_bimap_type::value_type(variable, cell_it->first) );
        //cell_it->second->sn = variable;
    }
}

template< unsigned D_ >
void SparseNGrid<D_>::DrawWithDR(const pos_type& color)
{
    for (grid_map_type::iterator it = cell_grid.begin();
        it != cell_grid.end(); ++it) {
            const BoundingBox3f& bb = GetCellBBox(it->first);
            debugRenderer->addBoundingBox(bb, color, 1);
    }
}

template< unsigned D_ >
typename SparseNGrid<D_>::key_type SparseNGrid<D_>::GetCellIndex(const pos_type& pos)
{
    const pos_type& pn = pos / cell_size;
    key_type k;
    for (unsigned ii = 0; ii < D_; ++ii) k[ii] = static_cast<key_t>(floor(pn[ii]));
    return k;
}

template< unsigned D_ >
typename SparseNGrid<D_>::key_type SparseNGrid<D_>::GetCellIndex(const sn_type& sn)
{
    return var_bimap.left.at(sn);
}

template< unsigned D_ >
typename SparseNGrid<D_>::sn_type SparseNGrid<D_>::GetSeqNumber(const key_type& key)
{
    return var_bimap.right.at(key);
}

template< unsigned D_ >
bool SparseNGrid<D_>::Contains(const key_type& key)
{
    return cell_grid.find(key) != cell_grid.end();
}

template< unsigned D_ >
typename SparseNGrid<D_>::CellPtr SparseNGrid<D_>::GetCellPtr(const key_type& key)
{
    return cell_grid.at(key);
}

template< unsigned D_ >
typename SparseNGrid<D_>::CellPtr SparseNGrid<D_>::GetCellPtr(const sn_type& sn)
{
    return GetCellPtr(GetCellIndex(sn));
}

template< unsigned D_ >
typename SparseNGrid<D_>::pos_type SparseNGrid<D_>::GetCellCenter(const key_type& key)
{
    pos_type p;
    for (unsigned ii = 0; ii < D_; ++ii) p[ii] = static_cast<pos_t>(key[ii]) + .5f;
    return p * cell_size;
}

template< unsigned D_ >
typename SparseNGrid<D_>::pos_type SparseNGrid<D_>::GetCellCenter(const pos_type& pos)
{
    return GetCellCenter(GetCellIndex(pos));
}

template< unsigned D_ >
typename SparseNGrid<D_>::pos_type SparseNGrid<D_>::GetCellCenter(const sn_type& sn)
{
    return GetCellCenter(GetCellIndex(sn));
}

template< unsigned D_ >
BoundingBox3f SparseNGrid<D_>::GetCellBBox(const key_type& key)
{
    pos_type ccen = GetCellCenter(key);
    BoundingBox3f bb;
    bb.lowerCorner = bb.upperCorner = ccen;
    bb.addBorder(cell_size / 2.0f);
    return bb;
}

template< unsigned D_ >
BoundingBox3f SparseNGrid<D_>::GetCellBBox(const pos_type& pos)
{
    return GetCellBBox(GetCellIndex(pos));
}

template< unsigned D_ >
BoundingBox3f SparseNGrid<D_>::GetCellBBox(const sn_type& sn)
{
    return GetCellBBox(GetCellIndex(sn));
}

#endif
