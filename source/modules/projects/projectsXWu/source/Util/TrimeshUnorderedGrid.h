#ifndef TrimeshUnorderedGrid_H
#define TrimeshUnorderedGrid_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "Util/TrimeshStatic.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

struct GCell
{
    GCell() : center(NULL_VECTOR3F) {};
    std::deque<unsigned> indice;
    std::deque<Vector3f> posVec;
    Vector3f center;
    Vector3f GetCenter(void)
    {
        Vector3f ret = NULL_VECTOR3F;
        const unsigned numPos = posVec.size();
        for (unsigned pi = 0; pi < numPos; ++pi) {
            ret += posVec[pi];
        }
        if (0 < numPos) ret /= (float)numPos;
        center = ret;
        return ret;
    }
};

//template< typename GCell=std::deque<unsigned> >
class UnorderedGrid
{
public:
    typedef boost::shared_ptr< UnorderedGrid > Ptr;
    typedef boost::shared_ptr< const UnorderedGrid > ConstPtr;

protected:
    struct hash_value : std::unary_function< Vector3i, boost::uint64_t > {
        boost::uint64_t operator() (const Vector3i& v) const {
            return boost::hash_range(v.begin(), v.end());
        }
    };

public:
    //typedef boost::unordered_set<unsigned> GCell;
    //typedef std::deque<unsigned> GCell;
	typedef boost::unordered_map< Vector3i, GCell, hash_value > grid_map_type;
    typedef grid_map_type::iterator iterator;
    typedef grid_map_type::const_iterator const_iterator;

public:
    UnorderedGrid(void);
    UnorderedGrid(const float& cellSize);
    ~UnorderedGrid(void);

    const size_t size(void) const { return cell_grid_.size(); }
    const float cell_size(void) const { return cell_size_; }

    int AddPoint(const Vector3f& point, Vector3i* p_idx);
    int AddPoint(const Vector3f& point, Vector3i* p_idx, Vector3f* p_posUpdated);
    void Clear(void);

    Vector3i GetCellIndexFromPoint(const Vector3f& pos) const;
    Vector3f GetCellCenterFromIndex(const Vector3i& idx) const;
    Vector3f GetCellCenterFromPoint(const Vector3f& pos) const;
    BoundingBox3f GetCellBBoxFromIndex(const Vector3i& idx) const;
    BoundingBox3f GetCellBBoxFromPoint(const Vector3f& pos) const;

    void DrawWithDR(const Vector3f& color = makeVector3f(0, 1, 1));
    bool CheckDrawAdjacency(void);

protected:
	grid_map_type cell_grid_;
	float cell_size_;
    unsigned num_points_;
};

class UnorderedGridUniq : public UnorderedGrid
{
public:
    typedef boost::shared_ptr< UnorderedGridUniq > Ptr;
    typedef boost::shared_ptr< const UnorderedGridUniq > ConstPtr;

public:
    UnorderedGridUniq(const float& cellSize) : UnorderedGrid(cellSize) {}

    int AddPoint(const Vector3f& point, Vector3i* p_idx);
    int AddPoint(const Vector3f& point, Vector3i* p_idx, Vector3f* p_posUpdated);
    unsigned GetOrdx(const Vector3i& idx);
    unsigned GetOrdx(const Vector3f& point);
};

class TrimeshUnorderedGrid : public UnorderedGrid
{
public:
    typedef boost::shared_ptr< TrimeshUnorderedGrid > Ptr;
    typedef boost::shared_ptr< const TrimeshUnorderedGrid > ConstPtr;

public:
    void Setup(UnstructuredInCoreTriangleMesh* mesh, const float& cellSize);

public:
    BoundingBox3f bounding_box_;
};

class TrimeshUnorderedGridStatic : public TrimeshUnorderedGrid
{
public:
    typedef boost::shared_ptr< TrimeshUnorderedGridStatic > Ptr;
    typedef boost::shared_ptr< const TrimeshUnorderedGridStatic > ConstPtr;

public:
    void Setup(TrimeshStatic::Ptr smesh, const float& cellSize);

private:
    TrimeshStatic::Ptr smesh_;
};

#endif
