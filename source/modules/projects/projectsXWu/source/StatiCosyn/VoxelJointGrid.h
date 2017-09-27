//---------------------------------------------------------------------------
#ifndef VoxelJointGrid_H
#define VoxelJointGrid_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "Util/marray.hpp"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

// represent connectivity between adjacent voxel sides
// each side of a voxel is a joint
struct SJoint
{
public:
    typedef boost::shared_ptr< SJoint > Ptr;
    typedef boost::shared_ptr< const SJoint > ConstPtr;

public:
    PointSetANNQueryPtr psKNN;
    std::deque<unsigned> indice;
    inline size_t size(void) { return indice.size(); }
};

// box shape voxel contains 6 joints
struct SVoxelJoint
{
public:
    typedef boost::shared_ptr< SVoxelJoint > Ptr;
    typedef boost::shared_ptr< const SVoxelJoint > ConstPtr;

public:
    std::deque<unsigned> dataInd;
    std::deque<SJoint::Ptr> lower; // -x, -y, -z
    std::deque<SJoint::Ptr> upper; // +x, +y, +z
    std::deque<unsigned> count; // number of points in one percent of voxel's volume
    bool countEmpty;

public:
    SVoxelJoint(void)
    {
        count.resize(1000, 0);
        for (unsigned d = 0; d < 3; ++d) {
            lower.push_back(SJoint::Ptr(new SJoint));
            upper.push_back(SJoint::Ptr(new SJoint));
        }
    }
    SJoint::Ptr getJoint(const int& d)
    {
        switch(d) {
        case 0: return lower[0]; break;
        case 1: return lower[1]; break;
        case 2: return lower[2]; break;
        case 3: return upper[0]; break;
        case 4: return upper[1]; break;
        case 5: return upper[2]; break;
        default: return nullptr;
        }
    }
    inline size_t size(void) { return dataInd.size(); }
    inline unsigned operator[] (const unsigned& x) const { return dataInd[x]; }
};

struct SGridDesc
{
public:
    Vector3i ms; // length of each dimension
    Vector3i lc, uc; // lower/upper corner
    int vol; // volume, number of voxels
    SGridDesc& operator=(const SGridDesc& other)
    {
        if (this != &other) {
            ms = other.ms;
            lc = other.lc;
            uc = other.uc;
            vol = other.vol;
        }
        return *this;
    }
public:
    int Index2SN(const Vector3i& index) const;
    Vector3i SN2Index(const int& sn) const;
    bool isValid(const Vector3i& index) const;
    bool onBorder(const Vector3i& index) const;
public:
    static bool onBorder(const Vector3i& index,
        const Vector3i& lower, const Vector3i& upper
        );
    static bool isValid(const Vector3i& index,
        const Vector3i& lower, const Vector3i& upper
        );
    static int intersectVoxels(
        const Vector3i& lowerL, const Vector3i& upperL,
        const Vector3i& lowerR, const Vector3i& upperR,
        Vector3i& lowerM, Vector3i& upperM);
};

class VoxelJointGrid
{
public:
    typedef boost::shared_ptr< VoxelJointGrid > Ptr;
    typedef boost::shared_ptr< const VoxelJointGrid > ConstPtr;

public:
    typedef std::deque< SVoxelJoint::Ptr > gridT;
    gridT voxels_;
    Vector3f gendir_;
    PointSet* zDataPS_;
    Matrix4f tParaZ_, tZback_; // align gendir to Z axis

public:
    //SGridDesc spaceDesc_;
    //SGridDesc shiftDesc_;
    //SGridDesc synthDesc_;
    //SGridDesc labelDesc_;
    //std::deque< float > pairDist_;
    //float distPairMax_;
    //float voxize_;
    //float jointRatio_;

    std::deque<int> synthLabels_;

public:
    explicit VoxelJointGrid(UICPC* dataPC,
        const Vector3f& dir, const Vector3f& delta, const float& voxize);
    ~VoxelJointGrid();
    void Voxelize(PointSet* dataPS);
    //void ComputePairDist(const PointSet& fPS, const std::string& aat, const unsigned& fDim);
    void ComputeVoxelPairDist(const PointSet& fPS, const std::string& aat, const unsigned& fDim);
	void Synthesis(int lowerB = -10000, int upperB = 10000);
	void SynthesisGM(int lowerB = -10000, int upperB = 10000);
	UICPC* CollectSynPoints(void);
    void DrawWithDR(void);

private:
    //void compareJointVolume(
    //    const Vector3i& lowerM, const Vector3i& upperM,
    //    const Vector3i& shift,
    //    const PointSet& fPS, const AAT& fAAT,
    //    float& pairdist
    //    );
    void compareJoint(
        SJoint::Ptr sj0, SJoint::Ptr sj1,
        const Vector3i& spaceIvec, const Vector3i& spaceJvec,
        const Vector3f& sideShift,
        const PointSet& fPS, const AAT& fAAT,
        const float& distEmpty,
        float& pairdist
        );

public:
    Vector3i GetCellIndexFromPoint(const Vector3f& pos);
    Vector3f GetCellCenterFromIndex(const Vector3i& idx);
    BoundingBox3f GetCellBBoxFromIndex(const Vector3i& idx);
};

class PointDistributor
{
public:
    typedef boost::shared_ptr< PointDistributor > Ptr;
    typedef boost::shared_ptr< const PointDistributor > ConstPtr;

public:
    UICPC* dataPC_;
    float voxize_;
    std::deque<Vector3f> gendirs_;
    std::deque<VoxelJointGrid::Ptr> grids_;

public:
    PointDistributor();
    ~PointDistributor();

public:
    void BindPointCloud(UICPC* dataPC);
    void PushGendir(const Vector3f& dir, const Vector3f& delta);
    void Synthesis(
        const unsigned& di, Scene* scene,
        int lowerB, int upperB
        );

public:
    void DrawVoxelsZ(void);
};

#endif