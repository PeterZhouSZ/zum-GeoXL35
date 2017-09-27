//---------------------------------------------------------------------------
#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "PCIStatiCosyn.h"
#include "Util/NoUse.h"
#include <fstream>
#include <locale>
//---------------------------------------------------------------------------
#include "SGRelativeTimeAnimationNode.h"
#include "SceneGraphState.h"
//---------------------------------------------------------------------------
using namespace X4;
//---------------------------------------------------------------------------
namespace {
    int DR_FRAME = 0;
}

PCIStatiCosyn::PCIStatiCosyn() :
    pointVoxer_(new PointDistributor)
{
    dataFolder = "NeuesRathaus0";
    dataName = "pcFullres";
    gendir_ = makeVector3f(0.5, 0, 0);
    oDelta_ = NULL_VECTOR3F;
    voxize_ = 0.6;
    margin_ = makeVector2i(-1000, 1000);
}

void PCIStatiCosyn::VoxelizeScene(void)
{
    //UICPC *dataPC = dynamic_cast<UICPC*>(
    //    getPointCloud(getScene(), NU::PrefixRootName(dataFolder)+"/"+dataName)
    //    );
    //if (!dataPC) {
    //    error("PCIStatiCosyn::VoxelizeScene() - source point cloud not found.");
    //    return;
    //}

    const std::vector<std::string> selection = getScene()->getRootState()->staticState->getSelectedNodes();
    if (selection.empty()) {
        error("PCIStatiCosyn::VoxelizeScene() - select a input point cloud.");
        return;
    }
    UICPC * dataPC = dynamic_cast<UICPC*>(getPointCloud( getScene(), selection[0]) );
    if (!dataPC) {
        error("PCIStatiCosyn::VoxelizeScene() - source point cloud not found.");
        return;
    }

    pointVoxer_->voxize_ = voxize_;
    pointVoxer_->BindPointCloud(dataPC);

    std::ifstream genFile("C:\\Workstation\\temp\\generators.txt");
    if (genFile.is_open()) {
        std::string line;
        {
            std::getline(genFile, line);
            debugOutput << "generator read from file: " << line << "\n";
            genFile.close();
        }
        {
            char chars[] = "[],";
            for (unsigned i = 0; i < strlen(chars); ++i) {
                line.erase(std::remove(line.begin(), line.end(), chars[i]),
                    line.end());
            }
        }
        std::stringstream ss(line);
        Vector3f dir;
        bool bread = true;
        for (unsigned d = 0; d < 3; ++d) {
            if (!(ss >> dir[d])) {
                debugOutput << "wrong generator format: " << d << ", " << ss.peek() << "\n";
                bread = false;
                break;
            }
        }
        if (bread) gendir_ = dir;
    }
    debugOutput << "generator: " << gendir_ << "\n";

    ////{
    ////    gendir_ = makeVector3f(0.0097, 0.1182, -0.0044);
    ////    //gendir_ = makeVector3f(0.0097, 0.1182, -0.0044) * 2;
    ////    //gendir_ = makeVector3f(0.0097, 0.1182, -0.0044) * 4;
    ////    //gendir_ = makeVector3f(0.0097, 0.1182, -0.0044) * 8;
    ////    //gendir_ = makeVector3f(1.2, 0, 0);
    ////}
    //{
    //    gendir_ = makeVector3f(-0.0004, -0.0005, -0.1758);
    //}
    pointVoxer_->PushGendir(gendir_, oDelta_);
}

void PCIStatiCosyn::DrawVoxelsZ(void)
{
    pointVoxer_->DrawVoxelsZ();
}

void PCIStatiCosyn::Synthesis(void)
{
    pointVoxer_->Synthesis(0, scene,
        margin_[0], margin_[1]);
    updateSceneView();
}

void PCIStatiCosyn::Reset(void)
{
    pointVoxer_ = PointDistributor::Ptr(new PointDistributor);
    SGRelativeTimeAnimationNode* groupList = dynamic_cast<SGRelativeTimeAnimationNode*>(
        getSceneGraphNodeByName(scene, "root/syn_steps"));
    if (groupList) groupList->clear();
}
