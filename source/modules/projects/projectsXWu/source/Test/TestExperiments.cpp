#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Test/PCITestCentr.h"
//---------------------------------------------------------------------------
#include "Timer.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

#include "Util/PlaneCluster.h"

void test_plane_read_write(UnstructuredInCoreTriangleMesh* mesh)
{

    PlaneCluster* plane_clu = new PlaneCluster(makeVector3f(1, 0, 0), makeVector3f(0, 0, 0));
    PlaneClusterExtractor* plane_cluster_extractor = new PlaneClusterExtractor();
    plane_cluster_extractor->addCluster(plane_clu);
    plane_cluster_extractor->setup(PlaneClusterExtractor::getDefaultName(), AttachedData::ADF_PERSISTENT);
    mesh->getAttachments()->attachData(plane_cluster_extractor);
}

void PCITestCentr::TestExperiments(void)
{
    UnstructuredInCoreTriangleMesh* mesh = nullptr;
    if (!get1stTrimesh(mesh)) return;

    test_plane_read_write(mesh);
}
