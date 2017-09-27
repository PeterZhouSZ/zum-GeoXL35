//---------------------------------------------------------------------------
#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "SingleExpansion.h"
#include <omp.h>
#include "Util/ColorSchemer.hpp"
#include "Util\numerical\EigenAdaptor.h"
#include "Util/gco-v3.0/GCoptimization.h"
#include "PCISelDelMark.h"
//---------------------------------------------------------------------------
#include "ProgressWindow.h"
#include "AnnSearch.h"
#include "SGRelativeTimeAnimationNode.h"
//---------------------------------------------------------------------------
using namespace X4;
//---------------------------------------------------------------------------
namespace {
    int DR_FRAME = 0;
    static unsigned NUM_UNARY = 0;
    static unsigned NUM_BINARY = 0;
    const float PI = 3.14159265;
}

SingleExpansion::SingleExpansion(
    )
{
    inlier_dist_ = 0.1f;
    cell_size_ = 1.f;
    cell_subdiv_ = 10;
    sub_vol_ = 1e6;
    gauss_truncate_ = 0.01f;
    mean_error_th_ = 0.4f;
    baseData_ = nullptr;
    unionGrid_ = nullptr;
    graph_model_ = nullptr;
    //unaryFunc_ = EnergyFunc(10.f);
    unaryFunc_ = EnergyFunc(4.f);
    binaryFunc_ = EnergyFunc(4.f);
    cost_bon_ = 10000 * unaryFunc_.max();
    cost_keep_empty_ = 1.001;
}

SingleExpansion::~SingleExpansion()
{
    for (UICPC* pc : shiftPCs_) {
        delete pc;
    }
    shiftPCs_.clear();
}

void SingleExpansion::DrawWithDR(void)
{
}

void SingleExpansion::SetBasePC(UICPC* basePC)
{
    patches_.clear();
    PCGridType::Ptr basePatch =
        boost::shared_ptr<PCGridType>(new PCGridType);
    basePatch->BuideGrid(basePC, cell_size_);
    baseData_ = basePC;
    patches_.push_back(basePatch);
    BuildSubGrid(basePatch);
    debugOutput << "base grid built.\n\n";
}

void SingleExpansion::ComputeCostMax(
    const float& cellSize, const unsigned& cellSubdiv,
    const float& medDist, const float& inRatio,
    const float& costKeepEmp, const float& meanErrorTh)
{
    cell_size_ = cellSize;
    cell_subdiv_ = cellSubdiv;
    inlier_dist_ = medDist * inRatio;
    sub_vol_ = cell_subdiv_ * cell_subdiv_ * cell_subdiv_;
    side_points_ = cellSize / medDist;
    side_points_ *= side_points_;
    empty_th_ = 0.1;
    cost_keep_empty_ = costKeepEmp;
    mean_error_th_ = meanErrorTh;
}

void SingleExpansion::BuildShiftPatches(const std::deque< PatchTrans::Ptr >& patchTransVec)
{
    const size_t numTrans = patchTransVec.size();
    for (unsigned ti = 0; ti < numTrans; ++ti) {
    //for (unsigned ti = 0; ti < 1; ++ti) {
    //for (unsigned ti = 0; ti < 4; ++ti) {
    //for (unsigned ti = 5; ti < patchTransVec.size(); ++ti) {
        PatchTrans::Ptr patchTrans = patchTransVec[ti];
        UICPC* shiftPC = patchTrans->GetPatchPCTransformed();
        PCGridType::Ptr shiftPatch =
            boost::shared_ptr<PCGridType>(new PCGridType);
        shiftPatch->BuideGrid(shiftPC, cell_size_);
        //shiftPatch->grid_->ClearEmptyCells(side_points_ * empty_th_);
        shiftPCs_.push_back(shiftPC);
        patches_.push_back(shiftPatch);
        BuildSubGrid(shiftPatch);

        debugRenderer->beginRenderJob_OneFrame("trans_group_", 30 + ti);
        Matrix4f transGroup = IDENTITY4F;
        for (unsigned d = 0; d < 4; ++d) {
            transGroup *= patchTrans->trans;
        }
        PointSet* basePS = baseData_->getPointSet();
        const AAT posAAT = basePS->getAAT("position");
        const unsigned numPoints = basePS->getNumEntries();
        for (unsigned ii = 0; ii < numPoints; ++ii) {
            const Vector3f pos = basePS->get3f(ii, posAAT);
            const Vector3f posT = transformVector3f(transGroup, pos);
            debugRenderer->addPoint(posT, ColorSchemer::GetJetColor(ti, numTrans));
        }
        debugRenderer->endRenderJob();
    }
    debugOutput << "built " << patchTransVec.size() << " shift patches in total.\n\n";
}

void SingleExpansion::BuildSubGrid(PCGridType::Ptr base)
{
    debugOutput << "building sub-grid ... \n";
    const bool debugDraw = false;
    UICPC* baseData = base->data_;;
    GridType::Ptr baseGrid = base->grid_;
    const float subCellSize = cell_size_ / (float)cell_subdiv_;
    const unsigned numCell = baseGrid->size();
    for (unsigned ci = 0; ci < numCell; ++ci) {
        if (debugDraw) debugRenderer->beginRenderJob_OneFrame("plane_cell_subdivide_", DR_FRAME++);
        GridCellPtr cell = baseGrid->GetCellPtr(ci);
        cell->KNN = boost::shared_ptr<PointSetANNQuery>
            (new PointSetANNQuery(cell->GetCellPS(baseData), 1));
        cell->sub = boost::shared_ptr<GridType>(new GridType(subCellSize));
        GridType::Ptr subGrid = cell->sub;
        PointSet* cellPS = cell->GetCellPS(baseData);
        AAT posAAT = cellPS->getAAT("position");
        const unsigned numPoints = cellPS->getNumEntries();
        for (unsigned ii = 0; ii < numPoints; ++ii) {
            const Vector3f& pos = cellPS->get3f(ii, posAAT);
            GridKeyType key = subGrid->AddPoint(pos);
            GridCellPtr cell = subGrid->GetCellPtr(key);
            cell->points.push_back(ii);
        }
        subGrid->ClearEmptyCells(4);
        for (unsigned jj = 0; jj < subGrid->size(); ++jj) {
            GridCellPtr subCell = subGrid->GetCellPtr(jj);
            subCell->EstimateNormal(cellPS);
            subCell->points.clear();
            if (debugDraw && (500 > DR_FRAME))
                subCell->DrawPlane(makeVector3f(0, 1, 1), subCellSize * 0.4);
        }
        if (debugDraw) subGrid->DrawWithDR(makeVector3f(0.4f, 0.4f, 0.4f));
        if (debugDraw) debugRenderer->endRenderJob();
    }
    debugOutput << "done.\n";
}

void SingleExpansion::BuildUnionGrid()
{
    debugOutput << "building union grid ... \n";
    const unsigned numTrans = patches_.size();
    unionGrid_ = boost::shared_ptr<UnionGridType>(new UnionGridType(cell_size_));

    progressWindow->pushStep(true, "union grid");
    for (unsigned ti = 0; ti < numTrans; ++ti) {
        GridType::Ptr gridI = patches_[ti]->grid_;
        for (int ci = 0; ci < gridI->size(); ++ci) {
            GridCellPtr cell = gridI->GetCellPtr(ci);
            GridKeyType key = gridI->GetCellIndex(ci);
            GridPosType pos = gridI->GetCellCenter(ci);
            if (unionGrid_->Contains(key)) {
                UnionGridCellPtr uniCell = unionGrid_->GetCellPtr(key);
                uniCell->cellMap[ti] = cell;
            }
            else {
                UnionGridCellPtr uniCell(new UnionCell());
                uniCell->cellMap[ti] = cell;
                unionGrid_->AddCell(key, uniCell);
            }
        }
        progressWindow->progressf((float)ti / (float)(numTrans - 1));
    }
    progressWindow->popStep();

    // make an uniform problem domain
    for (unsigned ti = 0; ti < numTrans; ++ti) {
        GridType::Ptr gridI = patches_[ti]->grid_;
        for (unsigned ci = 0; ci < unionGrid_->size(); ++ci) {
            UnionGridCellPtr uniCell = unionGrid_->GetCellPtr(ci);
            const UnionCell::MapT& cellMap = uniCell->cellMap;
            if (0 < uniCell->cellMap.count(ti)) continue;
            const GridKeyType key = unionGrid_->GetCellIndex(ci);
            GridCellPtr cell = boost::shared_ptr<GridCellType>(new GridCellType);
            gridI->AddCell(key, cell);
            uniCell->cellMap[ti] = cell;
        }
    }

    // draw each pathes and the union grid
    if (true)
    {
        for (unsigned ti = 0; ti < numTrans; ++ti) {
            GridType::Ptr gridI = patches_[ti]->grid_;
            PointSet* PSI = patches_[ti]->data_->getPointSet();
            debugRenderer->beginRenderJob_OneFrame("shifted_patches_", 10+ti);
            const AAT posAAT = PSI->getAAT("position");
            for (unsigned ci = 0; ci < unionGrid_->size(); ++ci) {
                UnionGridCellPtr uniCell = unionGrid_->GetCellPtr(ci);
                const UnionCell::MapT& cellMap = uniCell->cellMap;
                if (0 == cellMap.count(ti)) {
                    std::ostringstream ss;
                    ss << "SingleExpansion::BuildUnionGrid - empty cell: " << ci << "[" << ti << "]";
                    debugOutput << ss.str() << "\n";
                    continue;
                }
                GridCellPtr cell = cellMap.at(ti);
                const std::deque<unsigned>& points = cell->points;
                for (unsigned ii = 0; ii < points.size(); ++ii) {
                    const Vector3f posI = PSI->get3f(points[ii], posAAT);
                    debugRenderer->addPoint(posI, ColorSchemer::GetJetColor(ti, numTrans));
                }
            }
            gridI->DrawWithDR(makeVector3f(0.4, 0.4, 0.4));
            debugRenderer->endRenderJob();
        }
    }
    debugOutput << "done.\n";
}

void SingleExpansion::ExtractBoundary(const std::deque< PatchTrans::Ptr >& patchTransVec)
{
    debugOutput << "extracting boundary ... \n";
    const size_t numPatches = patches_.size();
    const IndexType numVar = unionGrid_->size();
    if (!baseData_->providesAttribute("boundary")) {
        warning("SingleCompletion::ExtractBoundary() - missing boundary attribute");
        return;
    }
    AAT bonAAT = baseData_->getAAT("boundary");

    const bool debugDraw = false;
    for (unsigned ni = 0; ni < numPatches; ++ni) {
        GridType::Ptr gridI = patches_[ni]->grid_;
        PointSet* PSI = patches_[ni]->data_->getPointSet();
        if (debugDraw) debugRenderer->beginRenderJob_OneFrame("draw_boundary_", DR_FRAME++);
        for (int ci = 0; ci < gridI->size(); ++ci) {
            GridCellPtr cell = gridI->GetCellPtr(ci);
            const std::deque<unsigned>& points = cell->points;
            for (unsigned ii = 0; ii < points.size(); ++ii) {
                const int32 bon = PSI->get1i(points[ii], bonAAT);
                if (PCISelDelMark::XF_BOUNDARY != bon) continue;
                cell->isBon = true;
                if (debugDraw) {
                    const BoundingBox3f& bb = gridI->GetCellBBox(ci);
                    debugRenderer->addBoundingBox(bb, ColorSchemer::GetJetColor(ni, numPatches), 1);
                }
                break;
            }
        }
        if (debugDraw) unionGrid_->DrawWithDR(makeVector3f(0.4, 0.4, 0.4));
        if (debugDraw) debugRenderer->endRenderJob();
    }

    debugOutput << "done.\n";
}

float SingleExpansion::gaussTruncate(const float& value, const float& sigma,
    const float& mult, const float& epsilon)
{
    const float ret = exp(-mult * (value * value) / (sigma * sigma));
    return (ret < epsilon) ? 0 : ret;
}
//float SingleExpansion::energyTruncate(const float& value, const float& sigma,
//    const float& mult, const float& upper)
//{
//    float ret = (value < upper) ? value : upper;
//    return mult * (ret * ret) / (sigma * sigma);
//}

float SingleExpansion::compareCellMajor(
    const GridCellPtr& cell1, const GridCellPtr& cell2,
    const bool debugShow)
{
    PointSetANNQueryPtr KNN = cell2->KNN;
    const PointSet& cellPS1 = *cell1->GetCellPS(nullptr);
    const PointSet& cellPS2 = *cell2->GetCellPS(nullptr);
    const unsigned szPS1 = cellPS1.getNumEntries();
    const unsigned szPS2 = cellPS2.getNumEntries();
    const AAT nmlAAT = cellPS2.getAAT("normal");
    const AAT posAAT = cellPS2.getAAT("position");

    float costIn = 0;
    unsigned numIn = 0;
    if (debugShow) debugOutput << "\n" << NUM_UNARY++ << " cell\n";
    //if (debugShow) debugRenderer->beginRenderJob_OneFrame("draw_unary_", DR_FRAME++);
    for (unsigned ii = 0; ii < szPS1; ++ii)
    {
        const Vector3f posI = cellPS1.get3f(ii, posAAT);
        const int32 indexNN = KNN->getNearestPointIndex(posI);
        const Vector3f posN = cellPS2.get3f(indexNN, posAAT);

        const float distNN = norm(posN - posI);
        if (debugShow)
        {
            debugRenderer->addPoint(posI, makeVector3f(1, 1, 0));
            debugRenderer->addPoint(posN, makeVector3f(0, 1, 1));
            if (distNN > inlier_dist_) {
                debugRenderer->addLine(
                    posI, posN,
                    makeVector3f(1, 0, 0), makeVector3f(1, 0, 0),
                    1.f);
            }
            else {
                debugRenderer->addLine(
                    posI, posN,
                    makeVector3f(1, 1, 0), makeVector3f(0, 1, 1),
                    1.f);
            }
        }
        if (distNN > inlier_dist_) {
            continue;
        }
        ++numIn;

        //Eigen::Vector3f nmlI = EigenAdaptor::ToEigen(cellPS1.get3f(ii, nmlAAT));
        //nmlI.normalize();
        //Eigen::Vector3f nmlN = EigenAdaptor::ToEigen(cellPS2.get3f(indexNN, nmlAAT));
        //nmlN.normalize();
        //const float nin = abs(nmlI.dot(nmlN));
        //const float deg = acos(nin) * 180.0 / PI;
        //const float degNorm = std::max<float>(0.0f, std::min<float>(deg / 90, 1.0f));
        //const float value = unaryFunc_.eva(degNorm);
        //const float weight = gaussTruncate(distNN, inlier_dist_, 1, gauss_truncate_);
        ////const float weight = 1.f;
        //costIn += weight * value;

        const float value = unaryFunc_.eva(distNN, inlier_dist_); // -log

        //debugOutput << nin << " - " << degNorm << " - "
        //    << value << " (" << weight << "); ";
    }
    //if (debugShow) debugRenderer->endRenderJob();
    //debugOutput << "\n";

    const float matchLimit = (float)szPS1 * 0.1f;
    const float inLimit = unaryFunc_.eva(mean_error_th_);
    const float misCost = unaryFunc_.eva(0.7f);
    const float maxCost = unaryFunc_.max();
    const unsigned numMis1 = szPS1 - numIn;
    float cost;
    if (matchLimit > numIn) cost = maxCost;
    else cost = ((float)numMis1 * misCost + costIn) / szPS1;
    if (inLimit > cost) cost = 0;
    else cost = maxCost;

    //if (maxCost < (cost - maxCost * 1e-6)) debugOutput << "compareCellMajor: " << cost
    //    << " (" << maxCost << ")" << "\n";
    if (debugShow) debugOutput << numIn << " (" << szPS1 << ", " << szPS2 << "): "
        << (costIn / (float)numIn) << " --> " << cost;
    //const float empMul = side_points_ / szPS2;
    //cost *= empMul;
    //if (debugShow) debugOutput << " --> " << cost << " @ " << empMul;
    if (debugShow) debugOutput << "\n";

    return cost;
}

float SingleExpansion::compareCell(
    const GridCellPtr& cell1, const GridCellPtr& cell2,
    const LabelType& labelI, const LabelType& labelJ,
    const bool debugShow)
{
    const float matchLimit = (float)sub_vol_ * 0.01f;
    const float maxCost = binaryFunc_.max();
    GridType::Ptr grid1 = cell1->sub;
    GridType::Ptr grid2 = cell2->sub;
    unsigned szCell1 = grid1->size();
    unsigned szCell2 = grid2->size();
    if (matchLimit > szCell1 || matchLimit > szCell2) return maxCost;

    float costIn = 0;
    unsigned numIn = 0;
    if (debugShow) debugOutput << "\n" << NUM_BINARY++ << " cell pair\n";
    if (debugShow) debugRenderer->beginRenderJob_OneFrame("draw_binary_", DR_FRAME++);
    for (unsigned ci = 0; ci < szCell1; ++ci)
    {
        GridKeyType key = grid1->GetCellIndex(ci);
        if (!grid2->Contains(key)) continue;

        const Vector3f posI = grid1->GetCellPtr(key)->point;
        const Vector3f posN = grid2->GetCellPtr(key)->point;

        const float distNN = norm(posN - posI);
        if (debugShow)
        {
            debugRenderer->addPoint(posI, makeVector3f(1, 1, 0));
            debugRenderer->addPoint(posN, makeVector3f(0, 1, 1));
            if (distNN > inlier_dist_) {
                debugRenderer->addLine(
                    posI, posN,
                    makeVector3f(1, 0, 0), makeVector3f(1, 0, 0),
                    1.f);
            }
            else {
                debugRenderer->addLine(
                    posI, posN,
                    makeVector3f(1, 1, 0), makeVector3f(0, 1, 1),
                    1.f);
            }
        }
        if (distNN > inlier_dist_) {
            continue;
        }
        ++numIn;

        const float value = binaryFunc_.eva(distNN, inlier_dist_); // -log

        costIn += value;
    }
    if (debugShow) debugOutput << "\n";
    if (debugShow) debugRenderer->endRenderJob();

    const float inLimit = binaryFunc_.eva(mean_error_th_);
    const float misCost = binaryFunc_.eva(0.7f);
    const unsigned numMis1 = (0 == labelI) ? 0 : szCell1 - numIn;
    const unsigned numMis2 = (0 == labelJ) ? 0 : szCell2 - numIn;
    float cost;
    if (matchLimit > numIn) cost = maxCost;
    else cost = ((float)(numMis1 + numMis2) * misCost + costIn) / (float)(numMis1 + numMis2 + numIn);
    if (inLimit > cost) cost = 0;
    else cost = maxCost;

    //if (maxCost < (cost - maxCost * 1e-6)) debugOutput << "compareCell: " << cost
    //    << " (" << maxCost << ")" << "\n";
    if (debugShow) debugOutput << numIn << " (" << szCell1 << ", " << szCell2 << "): "
        << (costIn / (float)numIn) << " --> " << cost;
    //const float empMul = side_points_ / szCell2;
    //cost *= empMul;
    //if (debugShow) debugOutput << " --> " << cost << " @ " << empMul;
    if (debugShow) debugOutput << "\n";

    return cost;
}

namespace {
    float smoothFn(int siteI, int siteJ, int labelI, int labelJ)
    {
        if (labelI == labelJ) return 0;
        else return 1;
    }

    float smoothFn(int siteI, int siteJ, int labelI, int labelJ,
        std::deque<UICPC*> pcVec, std::deque<FastSphereQuerryPtr> rQryVec,
        std::deque<PointSetANNQueryPtr> nQryVec,
        const Vector3f cen, const float radius
        )
    {
        UICPC* pcI = pcVec[labelI];
        UICPC* pcJ = pcVec[labelJ];
        FastSphereQuerryPtr rQryI = rQryVec[labelI];
        FastSphereQuerryPtr rQryJ = rQryVec[labelJ];
        mpcard *ptsI, *ptsJ;
        mpcard npI, npJ;
        rQryI->querry(cen, radius, &ptsI, npI);
        rQryJ->querry(cen, radius, &ptsJ, npJ);
        return 0;
    }
}

float SingleExpansion::checkBon(
    const int& siteI, const int& siteJ,
    const LabelType& labelI, const LabelType& labelJ
    )
{
    const UnionCell::MapT& cellMapI = unionGrid_->GetCellPtr(siteI)->cellMap;
    const UnionCell::MapT& cellMapJ = unionGrid_->GetCellPtr(siteJ)->cellMap;
    if (cellMapI.at(labelI)->empty() && cellMapJ.at(labelJ)->empty()) return 0;
    const float misCost = 1 * unaryFunc_.max();
    if (cellMapI.at(labelI)->empty()) {
        //return misCost * (float)cellMapJ.at(labelJ)->size() / side_points_;
        return misCost;
    }
    if (cellMapJ.at(labelJ)->empty()) {
        //return misCost * (float)cellMapI.at(labelI)->size() / side_points_;
        return misCost;
    }
    // boundary cost should be handled by unary!
    if (cellMapI.at(labelI)->isBon || cellMapJ.at(labelJ)->isBon)
        return misCost;
    return -1;
}

float SingleExpansion::computeBinary(
    const int& siteI, const int& siteJ,
    const LabelType& labelI, const LabelType& labelJ,
    const int ring
    )
{
    if (labelI == labelJ) return 0;
    const float bonCheckCost = checkBon(siteI, siteJ, labelI, labelJ);
    if (-0.1f < bonCheckCost) return bonCheckCost;

    const GridKeyType key1 = unionGrid_->GetCellIndex(siteI);
    const GridKeyType key2 = unionGrid_->GetCellIndex(siteJ);
    const int xmin = std::min(key1[0] - ring, key2[0] - ring);
    const int ymin = std::min(key1[1] - ring, key2[1] - ring);
    const int zmin = std::min(key1[2] - ring, key2[2] - ring);
    const int xmax = std::max(key1[0] + ring, key2[0] + ring);
    const int ymax = std::max(key1[1] + ring, key2[1] + ring);
    const int zmax = std::max(key1[2] + ring, key2[2] + ring);
    const int vol = (xmax - xmin + 1)*(ymax - ymin + 1)*(zmax - zmin + 1);
    const float sideSize = sub_vol_ / cell_subdiv_;
    //const bool debugShow = (500 > NUM_BINARY) ? true : false;
    const bool debugShow = false;
    //const bool debugShow = (0 == labelI && 1 == labelJ) ? true : false;
    if (debugShow) {
        debugOutput << "\n" << NUM_BINARY++ << " cell pair\n";
    }
    if (debugShow) debugRenderer->beginRenderJob_OneFrame("draw_binary_", DR_FRAME++);
    if (debugShow) {
        //debugRenderer->addBoundingBox(unionGrid_->GetCellBBox(key1),
        //    makeVector3f(1, 1, 0), 1);
        //debugRenderer->addBoundingBox(unionGrid_->GetCellBBox(key2),
        //    makeVector3f(0, 1, 1), 1);
    }
    float cost = 0;
    unsigned numIn = 0;
    const float maxCost = binaryFunc_.max();
    const float misCost = binaryFunc_.eva(0.7f);
    //#pragma omp parallel for schedule(dynamic,1)
    for (int xi = xmin; xi <= xmax; ++xi) {
        for (int yi = ymin; yi <= ymax; ++yi) {
            for (int zi = zmin; zi <= zmax; ++zi) {
                GridKeyType key0;
                key0[0] = xi; key0[1] = yi; key0[2] = zi;
                if (!unionGrid_->Contains(key0)) continue;
                UnionGridCellPtr uniCell = unionGrid_->GetCellPtr(key0);
                const UnionCell::MapT& cellMap = uniCell->cellMap;
                if (cellMap.at(labelI)->empty() && cellMap.at(labelJ)->empty()) continue;
                numIn += 1;
                if (cellMap.at(labelI)->empty() || cellMap.at(labelJ)->empty()) {
                    cost += maxCost;
                }
                else if (cellMap.at(labelI)->isBon || cellMap.at(labelJ)->isBon) {
                    cost += maxCost;
                }
                else {
                    cost += compareCell(
                        cellMap.at(labelI), cellMap.at(labelJ),
                        labelI, labelJ,
                        debugShow
                        );
                    if (debugShow) {
                        //const BoundingBox3f& bb = unionGrid_->GetCellBBox(key0);
                        //debugRenderer->addBoundingBox(bb, makeVector3f(0.4, 0.4, 0.4), 1);
                    }
                }
            }
        }
    }
    //cost /= (float)vol;
    cost /= (float)numIn;
    //if (maxCost < (cost - maxCost * 1e-6)) debugOutput << "computeBinary: " << cost
    //    << " (" << maxCost << ")" << "\n";
    //if (0 == labelI && 1 == labelJ) debugOutput << cost << "\n";
    if (debugShow) debugRenderer->endRenderJob();
    return cost;
}

void SingleExpansion::BuildGraphicalModel(void)
{
    debugOutput << "\nbuilding graphical model ... \n";

    // Build Model
    const IndexType numVar = unionGrid_->size();
    LabelType numLabel = patches_.size();
    debugOutput << numVar << " variables, " << numLabel << " labels.\n";
    const int spaceshape[] = { numVar };
    SpaceType::Ptr space(new SpaceType(spaceshape, spaceshape + 1, numLabel));
    graph_model_ = boost::shared_ptr<GraphModelType>(
        new GraphModelType(space)
        );
    //GraphModelType& graphModel = *graph_model_;

    // 1st-order
    const float costKeepEmp = cost_keep_empty_ * unaryFunc_.max();
    const float costFill = 0.001 * unaryFunc_.max();
    const float costBonEx = 10 * unaryFunc_.max();
    progressWindow->pushStep(true, "1st order");
    dataCost_.resize(numVar * numLabel, 0);
    //const bool drawUnary = (9 == lab) ? true : false;
    const bool drawUnary = false;
    if (drawUnary) debugRenderer->beginRenderJob_OneFrame("draw_unary_", 2);
    for (IndexType var = 0; var < numVar; ++var) {
        const LabelType labIndex[] = { numLabel };
        ExplicitFunctionType::Ptr fptr(new ExplicitFunctionType(labIndex, labIndex + 1));
        ExplicitFunctionType& f = *fptr;

        UnionGridCellPtr uniCell = unionGrid_->GetCellPtr(var);
        const UnionCell::MapT& cellMap = uniCell->cellMap;

        // empty source cell can not take label 0
        if (cellMap.at(0)->empty()) {
            f(0) = 0.f;
            //f(0) = costKeepEmp;
            //bool isBonEx = false;
            ////for (LabelType lab = 1; lab < numLabel; ++lab) {
            ////    if (cellMap.at(lab)->empty()) continue;
            ////    if (cellMap.at(lab)->isBon) {
            ////        f(0) = costBonEx;
            ////        isBonEx = true;
            ////        break;
            ////    }
            ////}
            for (LabelType lab = 1; lab < numLabel; ++lab) {
                // empty cell as transition
                if (cellMap.at(lab)->empty()) {
                    //if (isBonEx) f(lab) = costBonEx;
                    //else f(lab) = costKeepEmp;
                    f(lab) = 0.f;
                }
                else if (cellMap.at(lab)->isBon) f(lab) = cost_bon_;
                // fill as much as possible
                //else f(lab) = costFill * side_points_ / (float)cellMap.at(lab)->size();
                else f(lab) = unaryFunc_.max();
                //else f(lab) = 0;
            }
        }
        else {
            // boundary source cell can not take label 0
            if (cellMap.at(0)->isBon) {
                f(0) = cost_bon_;
                // compute cost by comparing
                for (LabelType lab = 1; lab < numLabel; ++lab) {
                    if (cellMap.at(lab)->empty()) {
                        f(lab) = cost_bon_;
                    }
                    else if (cellMap.at(lab)->isBon) f(lab) = cost_bon_;
                    else {
                        f(lab) = compareCellMajor(
                            cellMap.at(0), cellMap.at(lab),
                            //(500 > NUM_UNARY) ? true : false
                            //false
                            //(9 == lab) ? true : false
                            drawUnary
                            );
                    }
                }
            }
            else {
                f(0) = 0;
                for (LabelType lab = 1; lab < numLabel; ++lab) {
                    f(lab) = cost_bon_;
                }
            }
        }

        for (LabelType lab = 0; lab < numLabel; ++lab) {
            dataCost_[var * numLabel + lab] = f(lab);
        }

        int varIndex[] = { var };
        FactorType::Ptr factorPtr(new FactorType(varIndex, varIndex + 1));
        factorPtr->BindFunction(fptr);
        graph_model_->addFactor(factorPtr);

        progressWindow->progressf((float)var / (float)(numVar - 1));
    }
    if (drawUnary) debugRenderer->endRenderJob();
    progressWindow->popStep();

    // 2nd-order
    const float binaryMult = 100;
    progressWindow->pushStep(true, "2nd order");
    const unsigned numPair = numVar * (numVar - 1) / 2;
    unsigned cntPair = 0;
    const bool drawBinary = false;
    if (drawBinary) debugRenderer->beginRenderJob_OneFrame("draw_binary_", 20);
    for (IndexType varI = 0; varI < numVar; ++varI) {
        for (IndexType varJ = varI + 1; varJ < numVar; ++varJ) {
            ++cntPair;
            progressWindow->progressf((float)cntPair / (float)(numPair - 1));
            const int diff_abs = KeyDist(
                unionGrid_->GetCellIndex(varI),
                unionGrid_->GetCellIndex(varJ)
                );
            if (1 < diff_abs) {
                continue;
            }
            int varIndexL[] = { varI, varJ };
            int varIndexR[] = { varJ, varI };
            {
                FactorType::Ptr tfac;
                bool ret = graph_model_->getFactor(varIndexL, varIndexL + 2, tfac) ||
                    graph_model_->getFactor(varIndexR, varIndexR + 2, tfac);
                if (ret)
                    warning(str(boost::format(
                    "SingleExpansion::BuildGraphicalModel - 2-factor found: %1%, %2%.")
                    % varI % varJ));
            }

            const LabelType labIndex[] = { numLabel, numLabel };
            ExplicitFunctionType::Ptr fptrL(new ExplicitFunctionType(labIndex, labIndex + 2));
            ExplicitFunctionType& fL = *fptrL;
            ExplicitFunctionType::Ptr fptrR(new ExplicitFunctionType(labIndex, labIndex + 2));
            ExplicitFunctionType& fR = *fptrR;
            for (LabelType l1 = 0; l1 < numLabel; ++l1) {
                for (LabelType l2 = 0; l2 < numLabel; ++l2) {
                    //fL(l1, l2) = 0; fR(l2, l1) = 0; continue; // test always no transition cost

                    const float value = computeBinary(varI, varJ, l1, l2, 0);
                    fL(l1, l2) = value * binaryMult;
                    fR(l2, l1) = value * binaryMult;

                    if (0 == l1 && 1 == l2 && drawBinary)
                    {
                        const Vector3f cen1 = unionGrid_->GetCellCenter(varI);
                        const Vector3f cen2 = unionGrid_->GetCellCenter(varJ);
                        const Vector3f cen0 = (cen1 + cen2) / 2;
                        //debugRenderer->addSphere(cen1, cell_size_ / 5,
                        //    ColorSchemer::GetJetColor(l1, numLabel));
                        //debugRenderer->addSphere(cen2, cell_size_ / 5,
                        //    ColorSchemer::GetJetColor(l2, numLabel));
                        const Vector3f color = ColorSchemer::GetJetColor(value, binaryFunc_.max());
                        debugRenderer->addLine(
                            cen1, cen0,
                            color, color,
                            3);
                    }
                    if (1 == l1 && 0 == l2 && drawBinary)
                    {
                        const Vector3f cen1 = unionGrid_->GetCellCenter(varI);
                        const Vector3f cen2 = unionGrid_->GetCellCenter(varJ);
                        const Vector3f cen0 = (cen1 + cen2) / 2;
                        //debugRenderer->addSphere(cen1, cell_size_ / 5,
                        //    ColorSchemer::GetJetColor(l1, numLabel));
                        //debugRenderer->addSphere(cen2, cell_size_ / 5,
                        //    ColorSchemer::GetJetColor(l2, numLabel));
                        const Vector3f color = ColorSchemer::GetJetColor(value, binaryFunc_.max());
                        debugRenderer->addLine(
                            cen2, cen0,
                            color, color,
                            3);
                    }
                }
            }
            {
                FactorType::Ptr factorPtr(new FactorType(varIndexL, varIndexL + 2));
                factorPtr->BindFunction(fptrL);
                graph_model_->addFactor(factorPtr);
            }
             {
                 FactorType::Ptr factorPtr(new FactorType(varIndexR, varIndexR + 2));
                 factorPtr->BindFunction(fptrR);
                 graph_model_->addFactor(factorPtr);
             }
        }
    }
    graph_model_->buildFactorMap();

    if (drawBinary) debugRenderer->endRenderJob();
    progressWindow->popStep();

    //drawUnaryCost();
    //VisualizeCost();

    debugOutput << "done.\n";
}

void SingleExpansion::drawUnaryCost(void)
{
    const float maxCost = unaryFunc_.max();
    const IndexType numVar = unionGrid_->size();
    LabelType numLabel = patches_.size();
    for (LabelType lab = 0; lab < numLabel; ++lab) {
        debugRenderer->beginRenderJob_OneFrame("draw_unary_", lab);
        for (IndexType var = 0; var < numVar; ++var) {
            UnionGridCellPtr uniCell = unionGrid_->GetCellPtr(var);
            const UnionCell::MapT& cellMap = uniCell->cellMap;
            if (cellMap.at(lab)->empty()) continue;
            const float value = dataCost_[var * numLabel + lab] / maxCost;
            const Vector3f color = ColorSchemer::GetJetColor(value);

            GridCellPtr cell = cellMap.at(lab);
            const PointSet& vPS = *cell->GetCellPS(nullptr);
            const AAT& posAAT = vPS.getAAT("position");
            for (unsigned pi = 0; pi < vPS.getNumEntries(); ++pi) {
                const Vector3f& posI = vPS.get3f(pi, posAAT);
                debugRenderer->addPoint(posI, color);
            }

            //const Vector3f cen = unionGrid_->GetCellCenter(var);
            //Matrix3f orientation;
            //orientation.setIdentity();
            //const float scale = (cell_size_ * 0.7f);
            //debugRenderer->addCenteredBox(cen, orientation,
            //    makeVector3f(scale, scale, scale), color);
        }
        debugRenderer->endRenderJob();
    }
}

UICPC* SingleExpansion::CollectPoints(void)
{
    debugOutput << "collecting points ... \n";

    UICPC* synPC = new UICPC;
    synPC->clearAndSetup(0);
    PointSet& synPS = *synPC->getPointSet();
    const AAT& posAAT = synPS.getAAT("position");
    const AAT& nmlAAT = synPS.getAAT("normal");
    const AAT& clrAAT = synPS.getAAT("color");

    LabelType numLabel = patches_.size();
    const IndexType numCell = unionGrid_->size();
    for (IndexType vi = 0; vi < numCell; ++vi) {
        const LabelType& label = labels_[vi];
        UnionGridCellPtr uniCell = unionGrid_->GetCellPtr(vi);

        if (uniCell->cellMap.at(label)->empty()) {
            //debugOutput << "error label: " << vi << "\n";
            continue;
        }

        
        GridCellPtr cell = uniCell->cellMap.at(label);
        PointSet* vPS = cell->GetCellPS(nullptr);
        for (unsigned pi = 0; pi < vPS->getNumEntries(); ++pi) {
            const size_t& numEntries = synPS.getNumEntries();
            synPS.changeHeight(numEntries + 1);
            const Vector3f& pos = vPS->get3f(pi, posAAT);
            synPS.set3f(numEntries, posAAT, pos);
            const Vector3f& nml = vPS->get3f(pi, nmlAAT);
            synPS.set3f(numEntries, nmlAAT, nml);
            synPS.set3f(numEntries, clrAAT, ColorSchemer::GetJetColor(label, numLabel));
        }
    }

    debugOutput << "done.\n";

    return synPC;
}

void SingleExpansion::Infer(void)
{
    debugOutput << "\nsolving ... \n";
    X4_TIMER_START(inference);

    // Infer
    InferGCO();

    {
        const unsigned numTrans = patches_.size();
        debugRenderer->beginRenderJob_OneFrame("show_labels_", 1);
        const IndexType numVar = unionGrid_->size();
        //unionGrid_->DrawWithDR(makeVector3f(0.4, 0.4, 0.4));
        for (IndexType var = 0; var < numVar; var += 1) {
            const LabelType lab = labels_[var];
            if (0 == lab) continue;
            UnionGridCellPtr uniCell = unionGrid_->GetCellPtr(var);
            const UnionCell::MapT& cellMap = uniCell->cellMap;
            const Vector3f color = (cellMap.at(lab)->empty()) ?
                makeVector3f(0.4, 0.4, 0.4) : makeVector3f(0.5, 0, 1);
            //std::ostringstream ss;
            //ss << lab;
            //debugRenderer->addText(
            //    unionGrid_->GetCellCenter(var),
            //    ss.str(),
            //    20, "Verdana",
            //    color);
            //debugRenderer->addBoundingBox(unionGrid_->GetCellBBox(var),
            //    makeVector3f(0.4, 0.4, 0.4), 1);
            Matrix3f orientation;
            orientation.setIdentity();
            const float scale = (cell_size_ * 0.25f);
            debugRenderer->addCenteredBox(
                unionGrid_->GetCellCenter(var), orientation,
                makeVector3f(scale, scale, scale),
                ColorSchemer::GetJetColor(lab, numTrans));
            debugRenderer->addBoundingBox(unionGrid_->GetCellBBox(var),
                color, 1);
        }
        const Vector3f& pos = baseData_->getPointSet()->get3f(
            0, baseData_->getAAT("position")
            );
        debugRenderer->addSphere(pos, inlier_dist_, makeVector3f(1, 0, 1));
        debugRenderer->endRenderJob();
    }

        {
            Matrix3f orientation;
            orientation.setIdentity();
            const float scale = (cell_size_ * 0.25f);
            const float costKeepEmp = cost_keep_empty_ * unaryFunc_.max();
            debugRenderer->beginRenderJob_OneFrame("show_unary_", 2);
            //float cost = 0;
            //debugOutput << "unary: ";
            const IndexType numVar = unionGrid_->size();
            for (IndexType var = 0; var < numVar; var += 1) {
                const LabelType lab = labels_[var];
                const IndexType shape[] = { var };
                FactorType::Ptr factor;
                bool ret = graph_model_->getFactor(shape, shape + 1, factor);
                ExplicitFunctionType& f = *(boost::dynamic_pointer_cast<ExplicitFunctionType>(factor->getFunction()));
                const float value = f(lab);
                if (0.001 > value) continue;
                //cost += value;
                //debugOutput << value << " ";
                if (unaryFunc_.max() - 0.001 < value && unaryFunc_.max() + 0.001 > value)
                {
                    debugRenderer->addCenteredBox(
                        unionGrid_->GetCellCenter(var), orientation,
                        makeVector3f(scale, scale, scale),
                        ColorSchemer::GetJetColor(0.2));
                }
                else if (costKeepEmp - 0.001 < value && costKeepEmp + 0.001 > value)
                {
                    debugRenderer->addCenteredBox(
                        unionGrid_->GetCellCenter(var), orientation,
                        makeVector3f(scale, scale, scale),
                        ColorSchemer::GetJetColor(0.4));
                }
                else if (cost_bon_ - 0.001 < value && cost_bon_ + 0.001 > value)
                {
                    debugRenderer->addCenteredBox(
                        unionGrid_->GetCellCenter(var), orientation,
                        makeVector3f(scale, scale, scale),
                        ColorSchemer::GetJetColor(0.9));
                }
                else {
                    std::ostringstream ss;
                    ss << value;
                    debugRenderer->addText(
                        unionGrid_->GetCellCenter(var),
                        ss.str(),
                        20, "Verdana",
                        makeVector3f(0.8, 1, 0));
                    //debugRenderer->addCenteredBox(
                    //    unionGrid_->GetCellCenter(var), orientation,
                    //    makeVector3f(scale, scale, scale),
                    //    ColorSchemer::GetJetColor(1));
                }
                debugRenderer->addBoundingBox(unionGrid_->GetCellBBox(var),
                    makeVector3f(0.4, 0.4, 0.4), 1);
            }
            debugRenderer->endRenderJob();
            //debugOutput << cost << "\n";
        }
    {
        debugRenderer->beginRenderJob_OneFrame("show_binary_", 3);
        //float cost = 0;
        //debugOutput << "binary: ";
        const IndexType numVar = unionGrid_->size();
        //unionGrid_->DrawWithDR(makeVector3f(0.4, 0.4, 0.4));
        for (size_t fi = 0; fi < graph_model_->numberOfFactors(); ++fi) {
            FactorType& factor = *(*graph_model_)[fi];
            std::vector<IndexType> indices = factor.variableIndices();
            if (indices.size() != 2) continue;
            const LabelType lab0 = labels_[indices[0]];
            const LabelType lab1 = labels_[indices[1]];
            ExplicitFunctionType& f = *(boost::dynamic_pointer_cast<ExplicitFunctionType>(factor.getFunction()));
            const float value = f(lab0, lab1);
            if (0.001 > value) continue;
            //cost += value;
            //debugOutput << value << " ";
            if (50 * unaryFunc_.max() - 0.001 < value && 50 * unaryFunc_.max() + 0.001 > value)
            {
                debugRenderer->addLine(
                    unionGrid_->GetCellCenter(indices[0]),
                    unionGrid_->GetCellCenter(indices[1]),
                    ColorSchemer::GetJetColor(0.4),
                    ColorSchemer::GetJetColor(0.4),
                    5);
            } else if (100 * unaryFunc_.max() - 0.001 < value && 100 * unaryFunc_.max() + 0.001 > value)
            {
                debugRenderer->addLine(
                    unionGrid_->GetCellCenter(indices[0]),
                    unionGrid_->GetCellCenter(indices[1]),
                    ColorSchemer::GetJetColor(0.9),
                    ColorSchemer::GetJetColor(0.9),
                    5);
            }
            else {
                std::ostringstream ss;
                ss << value;
                debugRenderer->addText(
                    (unionGrid_->GetCellCenter(indices[0]) + unionGrid_->GetCellCenter(indices[1])) / 2,
                    ss.str(),
                    20, "Verdana",
                    makeVector3f(0.8, 1, 0));
                debugRenderer->addLine(
                    unionGrid_->GetCellCenter(indices[0]),
                    unionGrid_->GetCellCenter(indices[1]),
                    ColorSchemer::GetJetColor(1),
                    ColorSchemer::GetJetColor(1),
                    5);
            }
            debugRenderer->addBoundingBox(unionGrid_->GetCellBBox(indices[0]),
                makeVector3f(0.4, 0.4, 0.4), 1);
            debugRenderer->addBoundingBox(unionGrid_->GetCellBBox(indices[1]),
                makeVector3f(0.4, 0.4, 0.4), 1);
        }
        debugRenderer->endRenderJob();
        //debugOutput << cost << "\n";
   }

    X4_TIMER_STOP(inference);
    debugOutput << "done.\n\n";
}

void SingleExpansion::InferGCO(void)
{
    //GraphicalModelProxy::Inference(graph_model_, &dataCost_[0]);
    GraphicalModelProxy::Inference(graph_model_);
    const SpaceType& graphSpace = graph_model_->space();
    const IndexType numVar = unionGrid_->size();
    labels_.resize(numVar);
    for (int vi = 0; vi < numVar; ++vi)
        labels_[vi] = graphSpace(vi);
}

void SingleExpansion::VisualizeCost(void)
{
    debugOutput << "drawing cost ... \n";
    const IndexType numVar = unionGrid_->size();
    LabelType numLabel = patches_.size();
    for (LabelType lab = 0; lab < numLabel; ++lab) {
        debugRenderer->beginRenderJob_OneFrame("visualize_cost_", DR_FRAME++);
        unionGrid_->DrawWithDR(makeVector3f(0.4, 0.4, 0.4));
        for (IndexType var = 0; var < numVar; var += 1) {
            const float cost = MRFLibFunctionPointer::DataCostFn(var, lab, graph_model_);
            //const float cost = dataCost_[var * numLabel + lab];
            if (cost < 0.01) continue;
            if (cost > (cost_bon_ - 0.01) && cost < (cost_bon_ + 0.01)) {
                const Vector3f cen = unionGrid_->GetCellCenter(var);
                Matrix3f orientation;
                orientation.setIdentity();
                const float scale = (cell_size_ * 0.25f);
                debugRenderer->addCenteredBox(cen, orientation,
                    makeVector3f(scale, scale, scale), makeVector3f(0.8, 0.4, 0));
                continue;
            }
            std::ostringstream ss;
            ss << cost;
            debugRenderer->addText(
                unionGrid_->GetCellCenter(var),
                ss.str(),
                20, "Verdana",
                makeVector3f(0.5, 0, 1));
        }
        debugRenderer->endRenderJob();
    }
    debugOutput << "done.\n";
}
