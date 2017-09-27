//---------------------------------------------------------------------------
#ifndef PointVoxelGrid_H
#define PointVoxelGrid_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------

struct SVoxel
{
public:
    typedef boost::shared_ptr< SVoxel > Ptr;
    typedef boost::shared_ptr< const SVoxel > ConstPtr;

public:
    std::deque<unsigned> dataInd_;
    std::deque<unsigned> featInd_;

public:
    size_t size(void) { return featInd_.size(); }
    unsigned operator[] (const unsigned& x) const { return featInd_[x]; }
};

struct SVoxelSlice
{
public:
    typedef boost::shared_ptr< SVoxelSlice > Ptr;
    typedef boost::shared_ptr< const SVoxelSlice > ConstPtr;
public:
    typedef Vector2i IndT;
    typedef SVoxel::Ptr ValT;
public:
	typedef boost::unordered_map< IndT, ValT > sliceMapT;
    typedef sliceMapT::iterator iterator;
    typedef sliceMapT::const_iterator const_iterator;
public:
    sliceMapT voxels_;
public:
    //SVoxelSlice();
    //~SVoxelSlice();
};

struct SSliceStack
{
public:
    typedef boost::shared_ptr< SSliceStack > Ptr;
    typedef boost::shared_ptr< const SSliceStack > ConstPtr;

    //public:
    //    typedef int IndT;
    //    typedef SVoxelSlice::Ptr ValT;
    //public:
    //	typedef boost::unordered_map< IndT, ValT > stackMapT;

//public:
//    typedef Vector3i IndT;
//    typedef SVoxel::Ptr ValT;
//private:
//    struct hash_value : std::unary_function< Vector3i, boost::uint64_t > {
//        boost::uint64_t operator() (const Vector3i& v) const {
//            return boost::hash_range(v.begin(), v.end());
//        }
//    };
//public:
//    typedef boost::unordered_map< IndT, ValT, hash_value > stackMapT;

private:
    //stackMapT slices_;
    //stackMapT voxels_;
    typedef std::deque<SVoxel::Ptr> stackMapT;
    //std::deque< std::deque< std::deque< SVoxel::Ptr > > > voxarray_;
    Vector3f gendir_;
    PointSet* zDataPS_;
    PointSet* zFeatPS_;
    Matrix4f tParaZ_, tZback_;
    float voxize_;

    stackMapT spaceVoxels_;
    Vector3i spaceMeasure_;
    Vector3i spaceLower_, spaceUpper_;
    int spaceVolume_;
    BoundingBox3f spaceBB_;
    float distPairMax_;

    Vector3i shiftMeasure_;
    Vector3i shiftLower_, shiftUpper_;
    int shiftVolume_;

    //stackMapT synVoxels_;
    Vector3i synthMeasure_;
    Vector3i synthLower_, synthUpper_;
    int synthVolume_;
    BoundingBox3f synthBB_;

    Vector3i labelMeasure_;
    Vector3i labelLower_, labelUpper_;
    int labelVolume_;
    std::deque<int> synthLabels_;

    //std::vector<float> dataCost_;
    //float* pairDist_;
    std::vector<float> pairDist_;
    //SparseMatrix<float> pairDist_;

public:
    explicit SSliceStack(
        UICPC* dataPC, UICPC* featPC,
        const Vector3f& dir, const float& voxize);
    ~SSliceStack();
    void Voxelize(PointSet* dataPS, PointSet* featPS);
    //void ComputePairHausdorff(const PointSet& fPS, const std::string& aat, const unsigned& fDim);
    void ComputeShiftHausdorff(const PointSet& fPS, const std::string& aat, const unsigned& fDim);
    void Synthesis(void);
    UICPC* CollectSynPoints(void);
    void DrawWithDR(const Vector3f& color = makeVector3f(0, 0.8f, 0.8f));

public:
    Vector3i GetCellIndexFromPoint(const Vector3f& pos);
    Vector3f GetCellCenterFromIndex(const Vector3i& idx);
    BoundingBox3f GetCellBBoxFromIndex(const Vector3i& idx);
    int Index2SN(const Vector3i& index,
        const Vector3i& measure, const Vector3i& lower);
    Vector3i SN2Index(const int& sn,
        const Vector3i& measure, const Vector3i& lower);
    bool onBorder(const Vector3i& index,
        const Vector3i& lower, const Vector3i& upper);
    bool isValid(const Vector3i& index,
        const Vector3i& lower, const Vector3i& upper);
    int intersectVoxels(
        const Vector3i& lowerL, const Vector3i& upperL,
        const Vector3i& lowerR, const Vector3i& upperR,
        Vector3i& lowerM, Vector3i& upperM);
};

class PointVoxelization
{
public:
    typedef boost::shared_ptr< PointVoxelization > Ptr;
    typedef boost::shared_ptr< const PointVoxelization > ConstPtr;

public:
    UICPC* dataPC_;
    UICPC* featPC_;
    float voxize_;
    std::deque<Vector3f> gendirs_;
    std::deque<SSliceStack::Ptr> stacks_;

public:
    PointVoxelization();
    ~PointVoxelization();

public:
    void BindPointCloud(UICPC* dataPC, UICPC* featPC);
    void PushGendir(const Vector3f& dir);
    void Synthesis(const unsigned& di, Scene* scene);

public:
    void DrawVoxelsZ(void);
};

#endif