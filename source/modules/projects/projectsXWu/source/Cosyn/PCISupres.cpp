#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "PCISupres.h"
#include "PCwithGrid.h"
#include "Util/SceneAndPointCloudTools.h"
#include "Util\ColorSchemer.hpp"
#include "PatchTransGenerator.h"
#include "TransformationValidator.h"
#include "SingleExpansion.h"
#include "CellPlane.h"
#include "GroupGenerator.h"

//---------------------------------------------------------------------------
#include "CopyObjectProperties.h"
#include "Timer.h"
#include "ProgressWindow.h"
#include "PropertyTableProperty.h"
#include "SeparatorClassProperty.h"

#include "SceneEditorWidget.h"
#include "SceneGraphTools.h"
#include "SGListNode.h"
#include "SGRelativeTimeAnimationNode.h"
#include "MHSelectionIteratorAdaptor.h"

//---------------------------------------------------------------------------

void PCISupres::InitRepBox(void)
{
    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "# initializing from pre-detected repetition boxes \n";
    debugOutput << "########################################\n";
    debugOutput << "\n";

}

void PCISupres::PreProc(void)
{
    //UICPC* guidePC = dynamic_cast<UICPC*>(
    //    getPointCloud(getScene(), "root/" + guide_name));
    //if (nullptr == guidePC) {
    //    warning("PCISupres::GenerateParchTrans() - no guide point cloud.");
    //    return;
    //}

    //guideGrid_->SetPC(guidePC);
    //guideGrid_->EstimateNormal();
}

void PCISupres::GenerateParchTrans(void)
{
    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "# generating transformations with patch support \n";
    debugOutput << "########################################\n";
    debugOutput << "\n";

    UICPC* examplePC = dynamic_cast<UICPC*>(
        getPointCloud(getScene(), "root/" + example_name));
    if (nullptr == examplePC) {
        warning("PCISupres::GenerateParchTrans() - no example point cloud.");
        return;
    }

    UICPC* guidePC = dynamic_cast<UICPC*>(
        getPointCloud(getScene(), "root/" + guide_name));
    if (nullptr == guidePC) {
        warning("PCISupres::GenerateParchTrans() - no guide point cloud.");
        return;
    }

    // load raw detection
    repBoxLoader->SetPC(examplePC);
    repBoxLoader->LoadGT();

    typedef PCwithGrid<CellPlane> PCGridType;
    // build grid structure
    PCGridType::Ptr examplePCGrid =
        boost::shared_ptr<PCGridType>(new PCGridType());
    examplePCGrid->BuideGrid(examplePC, cell_size);
    PCGridType::Ptr guidePCGrid =
        boost::shared_ptr<PCGridType>(new PCGridType());
    guidePCGrid->BuideGrid(guidePC, cell_size);
    //guidePCGrid->EstimateNormal();

    // generate transformations for extracted patches
    PatchTransGenerator<CellPlane>::Ptr PTGentor(new PatchTransGenerator<CellPlane>);
    PTGentor->SetGrid(examplePCGrid, guidePCGrid);
    PTGentor->GenerateTempMatch(repBoxLoader);
}

void PCISupres::ValidateTransformations(void)
{
    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "# validating transformations using ICP \n";
    debugOutput << "########################################\n";
    debugOutput << "\n";

    UICPC* examplePC = dynamic_cast<UICPC*>(
        getPointCloud(getScene(), "root/" + example_name));
    if (nullptr == examplePC) {
        warning("PCISupres::GenerateParchTrans() - no example point cloud.");
        return;
    }

    // load raw detection
    repBoxLoader->SetPC(examplePC);
    repBoxLoader->LoadGT();

    // validate manual marked object classes
    TransformationValidator::Ptr transValidator(new TransformationValidator);
    transValidator->SetPC(examplePC);
    transValidator->match_ratio_ = match_ratio;
    transValidator->outlier_ratio_ = outlier_ratio;
    transValidator->inlier_ratio_ = inlier_ratio;
    transValidator->num_icp_ = num_icp;
    //transValidator->Validate(&repBoxLoader->GetClassList(0));
    transValidator->Validate(&repBoxLoader->boxRecord);

    repBoxLoader->SaveGT();
}

void PCISupres::TestPlaneGrid(void)
{
    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "# test plane approximation of grid cells \n";
    debugOutput << "########################################\n";
    debugOutput << "\n";

    UICPC* examplePC = dynamic_cast<UICPC*>(
        getPointCloud(getScene(), "root/" + example_name));
    if (nullptr == examplePC) {
        warning("PCISupres::GenerateParchTrans() - no example point cloud.");
        return;
    }
    PointSet* examplePS = examplePC->getPointSet();

    typedef PCwithGrid<CellPlane> PCGridType;
    // build grid structure
    PCGridType::Ptr examplePCGrid =
        boost::shared_ptr<PCGridType>(new PCGridType());
    examplePCGrid->BuideGrid(examplePC, cell_size);

    // plane approximation of grid points
    PCGridType::GridTypePtr exampleGrid = examplePCGrid->grid_;
    const int numCell = exampleGrid->size();
    //const int numCell = 10;
    static int DR_FRAME = 0;
    debugRenderer->beginRenderJob_OneFrame("plane_cell_", DR_FRAME++);
    progressWindow->pushStep(true, "icp validation");
    for (int ci = 0; ci < numCell; ++ci) {
        PCGridType::GridCellPtr cell = exampleGrid->GetCellPtr(ci);
        cell->EstimatePlane(examplePS);
        //cell->DrawPS(examplePS, makeVector3f(0, 0, 1));
        cell->DrawPlane(makeVector3f(0, 1, 1), cell_size * 0.4);
        progressWindow->progressf((float)ci / (float)(numCell - 1));
    }
    progressWindow->popStep();
    examplePCGrid->VisualizeGrid(makeVector3f(0.4f, 0.4f, 0.4f));
    debugRenderer->endRenderJob();
}

void PCISupres::CompleteScene(void)
{
    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "# complete scene using graph inference \n";
    debugOutput << "########################################\n";
    debugOutput << "\n";

    UICPC* examplePC = dynamic_cast<UICPC*>(
        getPointCloud(getScene(), "root/" + example_name));
    if (nullptr == examplePC) {
        warning("PCISupres::GenerateParchTrans() - no example point cloud.");
        return;
    }
    PointSet* examplePS = examplePC->getPointSet();

    typedef CellPlane CellType;
    typedef PCwithGrid<CellType> PCGridType;
    // build grid structure
    PCGridType::Ptr examplePCGrid = boost::shared_ptr<PCGridType>(new PCGridType());
    examplePCGrid->BuideGrid(examplePC, cell_size);

    // load raw detection
    repBoxLoader->SetPC(examplePC);
    repBoxLoader->LoadGT();

    // generate transformations for extracted patches
    PatchTransGenerator<CellType>::Ptr PTGentor(new PatchTransGenerator<CellType>);
    PTGentor->SetGrid(examplePCGrid, nullptr);
    PTGentor->GenerateInClass(repBoxLoader->GetClassList(0));

    // complete scene
    SingleExpansion::Ptr singleExp(new SingleExpansion);
    singleExp->ComputeCostMax(
        cell_size, cell_subdiv,
        getMedianPointDistance(examplePC),
        outlier_ratio,
        cost_keep_empty, mean_error_th
        );
    singleExp->SetBasePC(examplePC);
    singleExp->BuildShiftPatches(PTGentor->patch_trans);
    singleExp->BuildUnionGrid();
    singleExp->ExtractBoundary(PTGentor->patch_trans);
    singleExp->BuildGraphicalModel();
    singleExp->Infer();
    UICPC* synPC = singleExp->CollectPoints();
    addPointCloud(scene, synPC, "root/synPC");
    //singleExp->VisualizeCost();
}

void PCISupres::CompleteSceneSymmetry(void)
{
    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "# complete scene using graph inference \n";
    debugOutput << "########################################\n";
    debugOutput << "\n";

    UICPC* examplePC = dynamic_cast<UICPC*>(
        getPointCloud(getScene(), "root/" + example_name));
    if (nullptr == examplePC) {
        warning("PCISupres::GenerateParchTrans() - no example point cloud.");
        return;
    }
    PointSet* examplePS = examplePC->getPointSet();

    typedef CellPlane CellType;
    typedef PCwithGrid<CellType> PCGridType;
    // build grid structure
    PCGridType::Ptr examplePCGrid = boost::shared_ptr<PCGridType>(new PCGridType());
    examplePCGrid->BuideGrid(examplePC, cell_size);

    // load raw detection
    repBoxLoader->SetPC(examplePC);
    repBoxLoader->LoadSymmetries();

    // generate transformations from precomputed symmetries
    PatchTransGenerator<CellType>::Ptr PTGentor(new PatchTransGenerator<CellType>);
    PTGentor->SetGrid(examplePCGrid, nullptr);
    PTGentor->GenerateInClass(repBoxLoader->symmetries_);

    // complete scene
    SingleExpansion::Ptr singleExp(new SingleExpansion);
    singleExp->ComputeCostMax(
        cell_size, cell_subdiv,
        getMedianPointDistance(examplePC),
        outlier_ratio,
        cost_keep_empty, mean_error_th
        );
    singleExp->SetBasePC(examplePC);
    singleExp->BuildShiftPatches(PTGentor->patch_trans);
    singleExp->BuildUnionGrid();
    singleExp->ExtractBoundary(PTGentor->patch_trans);
    singleExp->BuildGraphicalModel();
    singleExp->Infer();
    UICPC* synPC = singleExp->CollectPoints();
    addPointCloud(scene, synPC, "root/synPC");
    //singleExp->VisualizeCost();
}

void PCISupres::ExtractGroup(void)
{
    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "# extract symmetry groups \n";
    debugOutput << "########################################\n";
    debugOutput << "\n";

    repBoxLoader->LoadGT();
    GroupGenerator::Ptr groupGenerator(new GroupGenerator);
    //groupGenerator->Generate(&repBoxLoader->GetClassList(0));
    groupGenerator->Generate(&repBoxLoader->boxRecord);
}
