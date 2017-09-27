//---------------------------------------------------------------------------
#ifndef SingleCompletion_H
#define SingleCompletion_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "PCwithGrid.h"
#include "CellNN.h"
#include <opengm/opengm.hxx>
#include <opengm/graphicalmodel/graphicalmodel.hxx>
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class SingleCompletion
{
public:
    typedef boost::shared_ptr< SingleCompletion > Ptr;
    typedef boost::shared_ptr< const SingleCompletion > ConstPtr;

public:
    typedef PCwithGrid<CellNN> PCGridType;
    typedef CellNN GridCellType;
    typedef SurfaceGrid< CellNN > GridType;
    typedef GridType::Ptr GridTypePtr;
    typedef GridType::CellPtr GridCellPtr;
    typedef GridType::key_type GridKeyType;
    typedef GridType::pos_type GridPosType;
    typedef GridType::snr_type GridSnrType;

public:
    typedef double ValueType;
    typedef size_t IndexType;
    typedef size_t LabelType;
    typedef opengm::Adder OpType;
    typedef opengm::ExplicitFunction<ValueType, IndexType, LabelType> ExplicitFunction;
    typedef opengm::meta::TypeListGenerator<ExplicitFunction>::type FunctionTypeList;
    typedef opengm::DiscreteSpace<IndexType, LabelType> SpaceType;
    typedef opengm::GraphicalModel<ValueType, OpType, FunctionTypeList, SpaceType> GraphModel;
    typedef GraphModel::FunctionIdentifier FunctionIdentifier;
    typedef boost::shared_ptr< GraphModel > GraphModelPtr;

public:
    struct UnionCell
    {
        explicit UnionCell(void) {}
        typedef boost::unordered_map< LabelType, GridCellPtr > MapT;
        MapT cellMap;
    };
    typedef SurfaceGrid< UnionCell > UnionGridType;
    typedef UnionGridType::CellPtr UnionGridCellPtr;

public:
    void ComputeCostMax(const float& cellSize, const float& medDist, const float& inRatio);
    void SetBasePC(UICPC* basePC);
    void BuildShiftPatches(const std::deque< PatchTrans::Ptr >& patchTransVec);
    void BuildUnionGrid();
    void ExtractBoundary();
    void BuildGraphicalModel(void);
    void Infer(void);
    UICPC* CollectPoints(void);

public:
    void VisualizeCost(void);

private:
    inline float gaussTruncate(const float& value, const float& sigma,
        const float& mult = 4.f, const float& epsilon = 0.01f);
    inline float energyTruncate(const float& value, const float& sigma,
        const float& mult = 4.f, const float& upper = 0.99f);

    float compareCellMajor(
        const GridCellPtr& cell1, const GridCellPtr& cell2,
        const bool debugShow);
    float compareCell(
        const GridCellPtr& cell1, const GridCellPtr& cell2,
        const bool debugShow);
    float computeBinary(
        int siteI, int siteJ, LabelType labelI, LabelType labelJ,
        const int ring
        );

private:
    void InferLBP(void);
    void InferMRFLBP(void);
    void InferMRFTRWS(void);
    void InferMRFGC(void);
    void InferGCO(void);

public:
    explicit SingleCompletion();
    ~SingleCompletion();
    void DrawWithDR(void);

private:
    inline GridKeyType KeyDiff(const GridKeyType& key1, const GridKeyType& key2)
    {
        return key1 - key2;
    }

    inline int KeyDist(const GridKeyType& key1, const GridKeyType& key2)
    {
        const GridKeyType diff = KeyDiff(key1, key2);
        int dist = 0;
        for (unsigned d = 0; d < GridType::Dim; ++d) {
            //dist += abs(diff[d]); // accumulate
            if (dist < abs(diff[d])) dist = abs(diff[d]); // max absolute
        }
        return dist;
    }

public:
    float inlier_dist_;
    float cell_size_;
    float side_points_;
    float gauss_truncate_;
    float mean_error_th_;

    std::deque<typename PCGridType::Ptr> patches_;
    UICPC* baseData_;
    //GridTypePtr baseGrid_;
    //std::deque<Matrix4f> transVec_;
    std::deque<UICPC*> shiftPCs_;
    //std::deque<FastSphereQuerryPtr> rQryVec_;
    //std::deque<PointSetANNQueryPtr> nQryVec_;
    UnionGridType::Ptr unionGrid_;

    GraphModelPtr graph_model_;
    std::vector<LabelType> labels_;
    std::vector<float> dataCost_;
};

#endif
