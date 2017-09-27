#ifndef SingleScene_H
#define SingleScene_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "GraphicalModelProxy.h"
//---------------------------------------------------------------------------
#include "HierarchicalKNNIterator.h"
//---------------------------------------------------------------------------

class PROJECTSXWU_API SingleScene
{
public:
    typedef boost::shared_ptr< SingleScene > Ptr;
    typedef boost::shared_ptr< const SingleScene > ConstPtr;

    typedef LabelGrid GridType;
    typedef LabelGrid::CellType GridCellType;
    typedef LabelGrid::CellPtr GridCellTypePtr;
    typedef GridType::grid_map_type grid_map_type;
    typedef GridType::key_type GridKeyType;
    typedef GridType::pos_type GridPosType;
    typedef GridType::sn_type sn_type;
    typedef GridType::var_bimap_type var_bimap_type;

public:
    SingleScene(const float& cellSize = 0.09f, const bool& verbose = false);
    ~SingleScene();

    void SetScanData(PointSet* ps);
    void AddCandidate(UnstructuredInCorePointCloud* pc, const std::string& name);
    void BuildGrid(void);
    void VisualizeGrid(void);

    GraphModelPtr BuildGraphicalModel(void);
    UnstructuredInCorePointCloud* ExtractCurrentPC(void);

private:
    ValueType ComputeUnaryCost(
        const size_t& variable, const size_t& state
        );
    ValueType ComputeBinaryCost(
        const size_t& variable1, const size_t& state1,
        const size_t& variable2, const size_t& state2
        );
    ValueType DistDiff2Norm(
        UnstructuredInCorePointCloud* point_cloud,
        const GridType::Ptr& query_grid,
        const size_t& state,
        HierarchicalKNNIterator* qit,
        const GridType::Ptr& variable_grid,
        const size_t& variable
        );

    void addNullCandi(void);

public:
    UnstructuredInCorePointCloud* scan_data_;
    std::deque< UnstructuredInCorePointCloud* > candi_pcs_;
    std::deque< std::string > candi_name_;
    GridType::Ptr data_grid_;
    GridType::Ptr curr_grid_;
    HierarchicalKNNIterator* scan_data_query_;
    std::deque< HierarchicalKNNIterator* > candi_pcs_query_;

    GraphModelPtr gr_model_;
    ValueType empty_cell_energy_;

    bool verbose_;
};

//inline LabelType Candi2Label(const unsigned& candi) { return candi + 1; }
//inline unsigned Label2Candi(const LabelType& label) { return label - 1; }

#endif
