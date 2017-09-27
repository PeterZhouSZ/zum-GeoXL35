#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "ObjectCollection.h"
#include "Util/NoUse.h"
#include "GraphicalModelProxy.h"
//---------------------------------------------------------------------------
#include "VertexArray.h"
//---------------------------------------------------------------------------

void ObjectCollection::AddUPC(UnstructuredInCorePointCloud* upc)
{
    upcvec.push_back(upc);
}

void ObjectCollection::AddUPC(std::deque<UnstructuredInCorePointCloud*> vec)
{
    upcvec.assign(vec.begin(), vec.end());
}

void ObjectCollection::BuildGrid(GridType::Ptr& grid)
{
    for (unsigned pci = 0; pci < upcvec.size(); ++pci) {
        UnstructuredInCorePointCloud* upc = upcvec[pci];
        VertexArray points(upc);
        for (unsigned jj = 0; jj < points.getNumElements(); ++jj) {
            const Vector3f p = points.getPosition3f(jj);
            GridKeyType idx;
            GridType::CellPtr node = grid->AddPoint(p, idx);
            node->points[pci].push_back(jj);
        }
    }
}
