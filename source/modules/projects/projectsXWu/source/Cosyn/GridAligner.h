#ifndef GridAligner_H
#define GridAligner_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "PCwithGrid.h"
#include "RepBoxLoader.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

template<typename CellT>
class GridAligner
{
public:
    typedef boost::shared_ptr< GridAligner > Ptr;
    typedef boost::shared_ptr< const GridAligner > ConstPtr;
    
    typedef typename PCwithGrid<CellT>::GridType GridType;

public:
    GridAligner(void);

    void SetGrid(
        typename PCwithGrid<CellT>::Ptr example,
        typename PCwithGrid<CellT>::Ptr guide);
    void SetTemplate(RepBox::Ptr tempBox);
    void Align(void);
    void ScoreFilter(const float& epsilon = .2f);

private:
    float estimateCost(
        PointSet* startPS, PointSet* targetPS,
        const Matrix4f& trans, PointSetANNQueryPtr KNN,
        const float& outlierDist
        );

public:
    typename PCwithGrid<CellT>::Ptr example_;
    typename PCwithGrid<CellT>::Ptr guide_;
    std::deque< PatchTrans::Ptr > patch_trans;
    RepBox::Ptr temp_box;
};

#include "GridAligner.inline.h"

#endif
