#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Test/PCITestCentr.h"
//---------------------------------------------------------------------------
#include "Timer.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

#include "Util/TrimeshUnorderedGrid.h"
void PCITestCentr::TestTrimeshUnorderedGrid(void)
{
    UnstructuredInCoreTriangleMesh* mesh = nullptr;
    if (!get1stTrimesh(mesh)) return;

    BoundingBox3f meshBoundingBox = getPCBBox(mesh);
    double grid_size = meshBoundingBox.getDiagonalLength() * 5e-3;

    {
        TrimeshUnorderedGrid meshgrid;
        meshgrid.Setup(mesh, grid_size);
        //const TrimeshUnorderedGrid::grid_map_type& cellgrid = meshgrid.mCellGrid;
        debugRenderer->beginRenderJob_OneFrame("mesh_grid_", DR_FRAME++);
        meshgrid.DrawWithDR(makeVector3f(0,1,0));
        debugRenderer->endRenderJob();
    }

    {
        TrimeshStatic::Ptr smesh(new TrimeshStatic(mesh));
        TrimeshUnorderedGridStatic meshgrid;
        meshgrid.Setup(smesh, grid_size);
        //const TrimeshUnorderedGrid::grid_map_type& cellgrid = meshgrid.mCellGrid;
        debugRenderer->beginRenderJob_OneFrame("mesh_grid_", DR_FRAME++);
        meshgrid.DrawWithDR(makeVector3f(0,1,0));
        debugRenderer->endRenderJob();
    }
}
