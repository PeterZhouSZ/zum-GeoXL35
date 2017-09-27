#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Test/PCITestCentr.h"
//---------------------------------------------------------------------------
#include "Timer.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

#include "Util/NoUse.h"
#include "VertexArray.h"
void PCITestCentr::TestWeldTrimesh(void)
{
    UnstructuredInCoreTriangleMesh* mesh = nullptr;
    if (!get1stTrimesh(mesh)) return;

    //{
    //    VertexArray va_vert(mesh->getPointSet());
    //    VertexArray va_face(mesh->getTriangles());
    //    for (mpcard fi = 0; fi < va_face.getNumElements(); ++fi) {
    //        Vector3i tri = va_face.getIndex3i(fi);
    //        debugRenderer->beginRenderJob_OneFrame("original_", DR_FRAME++);
    //        for (mpcard j = 0; j < 3; ++j) {
    //            debugRenderer->addLine(
    //                va_vert.getPosition3f(tri[j]),
    //                va_vert.getPosition3f(tri[(j+1)%3]),
    //                makeVector3f(1, 0, 0),
    //                makeVector3f(0, 0, 1),
    //                3);
    //        }
    //        debugRenderer->endRenderJob();
    //        //debugOutput << tri[0] << " " << tri[1] << " "  << tri[2] << "\n";
    //    }
    //}

    UnstructuredInCoreTriangleMeshPtr omesh (new UnstructuredInCoreTriangleMesh);
    omesh->assign(mesh);
    omesh->snapMesh(1e-6);
    omesh = NU::OpenMesh(&*omesh);
    debugOutput << "before welding mesh: "
        << omesh->getNumPoints() << "[" << omesh->getNumTriangles() << "]\n";
    omesh->snapMesh(1e-6);
    {
        VertexArray va_vert(omesh->getPointSet());
        VertexArray va_face(omesh->getTriangles());
        for (mpcard fi = 0; fi < va_face.getNumElements(); ++fi) {
            Vector3i tri = va_face.getIndex3i(fi);
            debugRenderer->beginRenderJob_OneFrame("opened_", DR_FRAME++);
            for (mpcard j = 0; j < 3; ++j) {
                debugRenderer->addLine(
                    va_vert.getPosition3f(tri[j]),
                    va_vert.getPosition3f(tri[(j+1)%3]),
                    makeVector3f(1, 0, 0),
                    makeVector3f(0, 0, 1),
                    3);
            }
            debugRenderer->endRenderJob();
            //debugOutput << tri[0] << " " << tri[1] << " "  << tri[2] << "\n";
        }
    }

    debugOutput << "after welding mesh: "
        << omesh->getNumPoints() << "[" << omesh->getNumTriangles() << "]\n";
}
