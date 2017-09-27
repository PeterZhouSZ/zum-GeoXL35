#ifndef PatchTransGenerator_H
#define PatchTransGenerator_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "PCwithGrid.h"
#include "RepBoxLoader.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

template<typename CellT>
class PatchTransGenerator
{
public:
    typedef boost::shared_ptr< PatchTransGenerator > Ptr;
    typedef boost::shared_ptr< const PatchTransGenerator > ConstPtr;

    typedef typename PCwithGrid<CellT>::GridType GridType;

public:
    PatchTransGenerator(void);

    void SetGrid(
        typename PCwithGrid<CellT>::Ptr example,
        typename PCwithGrid<CellT>::Ptr guide);
    void GenerateTempMatch(RepBoxLoader::Ptr repBoxLoader);
    void GenerateInClass(const std::deque<RepBox::Ptr>& boxList);
    void GenerateInClass(const std::deque< Eigen::Matrix4f >& symmetries);

public:
    static Matrix4f ComputeTransformation(RepBox::Ptr source, RepBox::Ptr target);

public:
    typename PCwithGrid<CellT>::Ptr example_;
    typename PCwithGrid<CellT>::Ptr guide_;
    std::deque< PatchTrans::Ptr > patch_trans;
};

#include "PatchTransGenerator.inline.h"

#endif
