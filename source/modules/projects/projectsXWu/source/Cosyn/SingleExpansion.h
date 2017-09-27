//---------------------------------------------------------------------------
#ifndef SingleExpansion_H
#define SingleExpansion_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "PCwithGrid.h"
#include "CellPlane.h"
//#include <opengm/opengm.hxx>
//#include <opengm/graphicalmodel/graphicalmodel.hxx>
#include "GraphicalModelProxy.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class SingleExpansion
{
public:
    typedef boost::shared_ptr< SingleExpansion > Ptr;
    typedef boost::shared_ptr< const SingleExpansion > ConstPtr;

public:
    typedef PCwithGrid<CellPlane> PCGridType;
    typedef CellPlane GridCellType;
    typedef SurfaceGrid< CellPlane > GridType;
    typedef GridType::Ptr GridTypePtr;
    typedef GridType::CellPtr GridCellPtr;
    typedef GridType::key_type GridKeyType;
    typedef GridType::pos_type GridPosType;
    typedef GridType::snr_type GridSnrType;

public:
    //typedef double ValueType;
    //typedef size_t IndexType;
    //typedef size_t LabelType;
    //typedef opengm::Adder OpType;
    //typedef opengm::ExplicitFunction<ValueType, IndexType, LabelType> ExplicitFunction;
    //typedef opengm::meta::TypeListGenerator<ExplicitFunction>::type FunctionTypeList;
    //typedef opengm::DiscreteSpace<IndexType, LabelType> SpaceType;
    //typedef opengm::GraphicalModel<ValueType, OpType, FunctionTypeList, SpaceType> GraphModel;
    //typedef GraphModel::FunctionIdentifier FunctionIdentifier;
    //typedef boost::shared_ptr< GraphModel > GraphModelPtr;

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
    void ComputeCostMax(
        const float& cellSize, const unsigned& cellSubdiv,
        const float& medDist, const float& inRatio,
        const float& costKeepEmp, const float& meanErrorTh);
    void SetBasePC(UICPC* basePC);
    void BuildShiftPatches(const std::deque< PatchTrans::Ptr >& patchTransVec);
    void BuildSubGrid(PCGridType::Ptr base);
    void BuildUnionGrid(void);
    void ExtractBoundary(const std::deque< PatchTrans::Ptr >& patchTransVec);
    void BuildGraphicalModel(void);
    void Infer(void);
    UICPC* CollectPoints(void);

public:
    void VisualizeCost(void);

private:
    inline float gaussTruncate(const float& value, const float& sigma,
        const float& mult = 4.f, const float& epsilon = 0.01f);
    //inline float energyTruncate(const float& value, const float& sigma,
    //    const float& mult = 4.f, const float& upper = 0.99f);
    struct EnergyFunc
    {
        explicit EnergyFunc(float MULT = 10.f, float EPS = 0.01f)
        {
            upper = 1 - EPS;
            mult = MULT;
        }
        inline float eva(const float& value, const float& sigma = 1.f)
        {
            float ret = value / sigma;
            ret = (ret < upper) ? ret : upper;
            return mult * (ret * ret);
        }
        inline float max(void) { return mult; }
    private:
        float upper;
        float mult;
    };

private:
    float compareCellMajor(
        const GridCellPtr& cell1, const GridCellPtr& cell2,
        const bool debugShow);
    float compareCell(
        const GridCellPtr& cell1, const GridCellPtr& cell2,
        const LabelType& labelI, const LabelType& labelJ,
        const bool debugShow);
    float computeBinary(
        const int& siteI, const int& siteJ,
        const LabelType& labelI, const LabelType& labelJ,
        const int ring
        );
    float checkBon(
        const int& siteI, const int& siteJ,
        const LabelType& labelI, const LabelType& labelJ
        );

private:
    void InferGCO(void);

public:
    explicit SingleExpansion();
    ~SingleExpansion();
    void DrawWithDR(void);
    void drawUnaryCost(void);

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
            dist += abs(diff[d]); // accumulate
            //if (dist < abs(diff[d])) dist = abs(diff[d]); // max absolute
        }
        return dist;
    }

public:
    float inlier_dist_;
    float cell_size_;
    float side_points_;
    float empty_th_;
    unsigned cell_subdiv_;
    float sub_vol_;
    float gauss_truncate_;
    float mean_error_th_;
    float cost_bon_;
    float cost_keep_empty_;

    std::deque<PCGridType::Ptr> patches_;
    UICPC* baseData_;
    std::deque<UICPC*> shiftPCs_;
    UnionGridType::Ptr unionGrid_;

    GraphModelPtr graph_model_;
    std::vector<LabelType> labels_;
    std::vector<ValueType> dataCost_;
    EnergyFunc unaryFunc_;
    EnergyFunc binaryFunc_;
};

#endif
