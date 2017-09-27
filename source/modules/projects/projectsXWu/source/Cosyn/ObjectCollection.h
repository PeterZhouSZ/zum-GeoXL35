#ifndef ObjectCollection_H
#define ObjectCollection_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "SparseNGrid.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class PROJECTSXWU_API ObjectCollection
{
public:
    typedef boost::shared_ptr< ObjectCollection > Ptr;
    typedef boost::shared_ptr< const ObjectCollection > ConstPtr;

    typedef LabelGrid GridType;
    typedef GridType::key_type GridKeyType;
    typedef GridType::pos_type GridPosType;

public:
    void AddUPC(UnstructuredInCorePointCloud* upc);
    void AddUPC(std::deque<UnstructuredInCorePointCloud*> vec);
    void BuildGrid(GridType::Ptr& grid);

public:
    std::deque<UnstructuredInCorePointCloud*> upcvec;
};

#endif
