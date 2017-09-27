#ifndef PCwithGrid_H
#define PCwithGrid_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "Util\SurfaceGrid.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

struct PatchTrans
{
    typedef boost::shared_ptr< PatchTrans > Ptr;
    typedef boost::shared_ptr< const PatchTrans > ConstPtr;

public:
    PatchTrans(void);
    ~PatchTrans(void);

    PointSet* GetPatchPS(void);
    UICPC* GetPatchPCTransformed(void);
    void Draw(void);

public:
    UICPC* data;
    Vector3f center;
    Matrix3f frame;
    float radius;
    std::deque<unsigned> points;
    Matrix4f trans;
    float score;

private:
    PointSet* patch;
};

template<typename CellT>
class PCwithGrid
{
public:
    typedef boost::shared_ptr< PCwithGrid > Ptr;
    typedef boost::shared_ptr< const PCwithGrid > ConstPtr;

    typedef CellT GridCellType;
    typedef SurfaceGrid<CellT> GridType;
    typedef typename SurfaceGrid<CellT>::Ptr GridTypePtr;
    typedef typename GridType::CellPtr GridCellPtr;
    typedef typename GridType::key_type GridKeyType;
    typedef typename GridType::pos_type GridPosType;

public:
    PCwithGrid(void);
    ~PCwithGrid(void);
    size_t size(void);

    void SetPC(UICPC* pc);
    void BuideGrid(UICPC* pc, const float& cellSize);
    void VisualizeGrid(const Vector3f& color);
    void EstimateNormal(void);

public:
    UICPC* data_;
    typename GridType::Ptr grid_;
    float cell_size_;
};

//============================================================================

template<typename CellT>
PCwithGrid<CellT>::PCwithGrid(void)
{
    cell_size_ = 0.f;
    data_ = nullptr;
    grid_ = nullptr;
}
template<typename CellT>
PCwithGrid<CellT>::~PCwithGrid(void)
{
    data_ = nullptr;
}

template<typename CellT>
size_t PCwithGrid<CellT>::size(void)
{
    return (nullptr == grid_) ? 0 : grid_->size();
}

template<typename CellT>
void PCwithGrid<CellT>::SetPC(UICPC* pc)
{
    data_ = pc;
}

template<typename CellT>
void PCwithGrid<CellT>::BuideGrid(UICPC* pc, const float& cellSize)
{
    if (0.00001f > cellSize) {
        error("PCwithGrid::BuideGrid() - cell size is too small");
    }
    data_ = pc;
    cell_size_ = cellSize;
    grid_ = boost::shared_ptr<GridType>(new GridType(cell_size_));
    debugOutput << "building grid ... \n";
    const AAT posAAT = data_->getAAT("position");
    const PointSet& basePS = *data_->getPointSet();
    const unsigned numPoints = basePS.getNumEntries();
    for (unsigned jj = 0; jj < numPoints; ++jj) {
        const Vector3f& pos = basePS.get3f(jj, posAAT);
        GridKeyType key = grid_->AddPoint(pos);
        GridCellPtr cell = grid_->GetCellPtr(key);
        cell->points.push_back(jj);
    }
    debugOutput << "done.\n";
}

template<typename CellT>
void PCwithGrid<CellT>::VisualizeGrid(const Vector3f& color)
{
    grid_->DrawWithDR(color);
}

template<typename CellT>
void PCwithGrid<CellT>::EstimateNormal(void)
{
    Timer timer;
    debugOutput << "PCwithGrid::EstimateNormal\n";
    AAT pospcAAT = data_->getAAT("position");
    checkAttribute(data_, "normal", 3, VAD::DATA_FORMAT_FLOAT32);

    SceneObjectIterator *iter = data_->createIterator(SOIT::CAP_BASIC_PC | SOIT::CAP_MODIFYABLE_PC);
    x4Assert(iter != NULL);
    BasicPointCloudIterator *bIt = dynamic_cast<BasicPointCloudIterator*>(iter);
    ModifyablePointCloudIterator *mIt = dynamic_cast<ModifyablePointCloudIterator*>(iter);
    PointSet *ps = mIt->lockBlockReadWrite();

    // resample 
    BoundingBox3f bb = getPCBBox(data_);
    const float radiusResampler = bb.getDiagonalLength() * 0.01f;
    UnstructuredInCorePointCloud * pcsampled = new UnstructuredInCorePointCloud;
    pcsampled->setPointSet(dynamic_cast<PointSet*>(ps->copy()));
    PCCResampler::resamplePC(pcsampled, radiusResampler, false);

    // pca normal on both the fullres and the lowres 
    const card32 numNN = 100;
    const card32 numNNsampled = 200;
    NormalEstimator::estimateNormals(pcsampled, numNNsampled, -1.0f, -1.0f, ENVIRONMENT_KNN, false);
    NormalUnifier::unifyNormalsHoppe(pcsampled->getPointSet(), numNNsampled);
    NormalEstimator::estimateNormals(data_, numNN, -1.0f, -1.0f, ENVIRONMENT_KNN, false);

    // check sign
    AAT normalpcAAT = data_->getAAT("normal");
    AAT normalpcsampledAAT = pcsampled->getAAT("normal");
    AttachedIndexedOctree * knnOctree_pcsampled;
    PointSetKNNQuery * knn_pcsampled;
    knnOctree_pcsampled = PointSetKNNQuery::createOctree(pcsampled->getPointSet());
    knn_pcsampled = new PointSetKNNQuery(pcsampled->getPointSet(), knnOctree_pcsampled, NULL_VECTOR3F, pospcAAT);

    for (int32 i = 0; i < data_->getNumPoints(); i++){
        Vector3f pos = ps->get3f(i, pospcAAT);
        knn_pcsampled->setTargetPoint(pos);
        card32 index; {
            bool found;
            knn_pcsampled->getNextNearestPointIndex(index, found);
            if (!found) throw EInvalidState("point set empty");
        }
        Vector3f normalpc = ps->get3f(i, normalpcAAT);
        Vector3f normalpcsampled = pcsampled->getPointSet()->get3f(index, normalpcsampledAAT);

        if (normalpc*normalpcsampled < 0){
            ps->set3f(i, normalpcAAT, -normalpc);
        }
    }

    bIt->unlock();
    delete iter;
    delete knn_pcsampled;
    delete knnOctree_pcsampled;
    delete pcsampled;
    debugOutput << "Time: " << convertTimeToString(timer.getDeltaValue()) << "\n";
}

#endif
