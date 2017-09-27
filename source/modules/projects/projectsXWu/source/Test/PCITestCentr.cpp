#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Test/PCITestCentr.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
IMPLEMENT_CLASS(PCITestCentr, 0)
{
    BEGIN_CLASS_INIT(PCITestCentr);
    //ADD_NOARGS_METHOD(TestTrimeshUnorderedGrid);
    //ADD_NOARGS_METHOD(TestTrimeshStatic);
    //ADD_NOARGS_METHOD(TestWeldTrimesh);
    //ADD_NOARGS_METHOD(TestOrderedPointSet);
    //ADD_NOARGS_METHOD(TestPatternRotation);
    //ADD_NOARGS_METHOD(TestPatternTranslation);
    //ADD_NOARGS_METHOD(TestSymmFeatFrame);
    //ADD_NOARGS_METHOD(TestSymmetryToolBox);
    //ADD_NOARGS_METHOD(TestGraphicalModel);
    ADD_NOARGS_METHOD(TestExperiments);
    //ADD_NOARGS_METHOD(TestSuper4PCS);
}
//----------------------------------------------------------------------

PCITestCentr::PCITestCentr()
{
}

bool PCITestCentr::get1stTrimesh(UnstructuredInCoreTriangleMesh*& mesh)
{
    std::vector<PointCloud*> pcs = getAllPointCloudsFromScene();
    if (pcs.size() == 0)
        return false;
    mesh = dynamic_cast<UnstructuredInCoreTriangleMesh*>(pcs[0]);
    if (mesh == nullptr) {
        return false;
    }
    return true;
}
