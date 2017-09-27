//---------------------------------------------------------------------------
#include "StdAfx.h"
//---------------------------------------------------------------------------
#include <SingleCompletion.h>
#include <omp.h>
#include "Util/ColorSchemer.hpp"
#include "Util\numerical\EigenAdaptor.h"
#include "Util/gco-v3.0/GCoptimization.h"
#include <opengm/inference/messagepassing/messagepassing.hxx>
#include <opengm/inference/external/mrflib.hxx>
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

SingleCompletion::SingleCompletion(
    )
{
    inlier_dist_ = 0.1f;
    cell_size_ = 1.f;
    gauss_truncate_ = 0.01f;
    mean_error_th_ = 0.2f;
    baseData_ = nullptr;
    unionGrid_ = nullptr;
    graph_model_ = nullptr;
}
SingleCompletion::~SingleCompletion()
{
    for (UICPC* pc : shiftPCs_) {
        delete pc;
    }
    shiftPCs_.clear();
}

void SingleCompletion::DrawWithDR(void)
{
}

void SingleCompletion::SetBasePC(UICPC* basePC)
{
    patches_.clear();
    PCGridType::Ptr basePatch =
        boost::shared_ptr<PCGridType>(new PCGridType);
    basePatch->BuideGrid(basePC, cell_size_);
    {
        GridType::Ptr baseGrid = basePatch->grid_;
        UICPC* baseData = basePatch->data_;
        for (int ci = 0; ci < baseGrid->size(); ++ci) {
            GridCellPtr cell = baseGrid->GetCellPtr(ci);
            cell->KNN = boost::shared_ptr<PointSetANNQuery>
                (new PointSetANNQuery(cell->GetCellPS(baseData), 1));
        }
    }
    baseData_ = basePC;
    patches_.push_back(basePatch);
}

void SingleCompletion::ComputeCostMax(const float& cellSize,
    const float& medDist, const float& inRatio)
{
    cell_size_ = cellSize;
    inlier_dist_ = medDist * inRatio;
    side_points_ = cellSize / medDist;
    side_points_ *= side_points_;
}

void SingleCompletion::BuildShiftPatches(const std::deque< PatchTrans::Ptr >& patchTransVec)
{
    for (unsigned ti = 0; ti < patchTransVec.size(); ++ti) {
        //for (unsigned ti = 0; ti < 1; ++ti) {
        PatchTrans::Ptr patchTrans = patchTransVec[ti];
        UICPC* shiftPC = patchTrans->GetPatchPCTransformed();
        PCGridType::Ptr shiftPatch =
            boost::shared_ptr<PCGridType>(new PCGridType);
        shiftPatch->BuideGrid(shiftPC, cell_size_);
        {
            GridType::Ptr patchGrid = shiftPatch->grid_;
            UICPC* patchData = shiftPatch->data_;
            for (int ci = 0; ci < patchGrid->size(); ++ci) {
                GridCellPtr cell = patchGrid->GetCellPtr(ci);
                cell->KNN = boost::shared_ptr<PointSetANNQuery>
                    (new PointSetANNQuery(cell->GetCellPS(patchData), 1));
            }
        }
        shiftPCs_.push_back(shiftPC);
        patches_.push_back(shiftPatch);
    }
}

void SingleCompletion::BuildUnionGrid()
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

    // draw each pathes and the union grid
    {
        //for (unsigned ti = 0; ti < numTrans; ++ti) {
        //    PointSet* PSI = patches_[ti]->data_->getPointSet();
        //    debugRenderer->beginRenderJob_OneFrame("shifted_patches_", DR_FRAME++);
        //    const AAT posAAT = PSI->getAAT("position");
        //    for (unsigned ci = 0; ci < unionGrid_->size(); ++ci) {
        //        UnionGridCellPtr uniCell = unionGrid_->GetCellPtr(ci);
        //        GridCellPtr cell = uniCell->cellMap[ti];
        //        if (nullptr == cell) continue;
        //        const std::deque<unsigned>& points = cell->points;
        //        for (unsigned ii = 0; ii < points.size(); ++ii) {
        //            const Vector3f posI = PSI->get3f(points[ii], posAAT);
        //            debugRenderer->addPoint(posI, ColorSchemer::GetJetColor(ti, numTrans));
        //        }
        //    }
        //    unionGrid_->DrawWithDR(makeVector3f(0.4, 0.4, 0.4));
        //    debugRenderer->endRenderJob();
        //}
    }
    debugOutput << "done.\n";
}

void SingleCompletion::ExtractBoundary()
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

float SingleCompletion::gaussTruncate(const float& value, const float& sigma,
    const float& mult, const float& epsilon)
{
    const float ret = exp(-mult * (value * value) / (sigma * sigma));
    return (ret < epsilon) ? 0 : ret;
}
float SingleCompletion::energyTruncate(const float& value, const float& sigma,
    const float& mult, const float& upper)
{
    float ret = (value < upper) ? value : upper;
    return mult * (ret * ret) / (sigma * sigma);
}

float SingleCompletion::compareCellMajor(
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
    if (debugShow) debugRenderer->beginRenderJob_OneFrame("draw_unary_", DR_FRAME++);
    for (unsigned ii = 0; ii < szPS1; ++ii)
    {
        const Vector3f posI = cellPS1.get3f(ii, posAAT);
        Eigen::Vector3f nmlI = EigenAdaptor::ToEigen(cellPS1.get3f(ii, nmlAAT));
        nmlI.normalize();
        const int32 indexNN = KNN->getNearestPointIndex(posI);
        const Vector3f posN = cellPS2.get3f(indexNN, posAAT);
        Eigen::Vector3f nmlN = EigenAdaptor::ToEigen(cellPS2.get3f(indexNN, nmlAAT));
        nmlN.normalize();

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

        const float nin = abs(nmlI.dot(nmlN));
        const float deg = acos(nin) * 180.0 / PI;
        const float degNorm = std::max<float>(0.0f, std::min<float>(deg / 90, 1.0f));
        //const float value = gaussTruncate(degNorm, 1, 10, gauss_truncate_); // ~60 degrees
        const float value = energyTruncate(degNorm, 1, 10); // -log

        const float weight = gaussTruncate(distNN, inlier_dist_, 1, gauss_truncate_);
        //const float weight = 1.f;

        costIn += weight * value;
        //debugOutput << nin << " - " << degNorm << " - "
        //    << value << " (" << weight << "); ";
    }
    if (debugShow) debugRenderer->endRenderJob();
    //debugOutput << "\n";

    const float matchLimit = (float)szPS1 * 0.3f;
    const float inLimit = energyTruncate(mean_error_th_, 1, 10);
    const float misCost = energyTruncate(0.5f, 1, 10);
    const float maxCost = energyTruncate(1.f, 1, 10);
    const unsigned numMis1 = szPS1 - numIn;
    float cost;
    if (matchLimit > numIn) cost = maxCost;
    else if ((costIn / (float)numIn) < inLimit) cost = 0;
    else {
        cost = ((float)numMis1 * misCost + costIn) / szPS1;
    }

    if (maxCost < cost) debugOutput << cost << "\n";
    if (debugShow) debugOutput << numIn << " (" << szPS1 << ", " << szPS2 << "): "
        << (costIn / (float)numIn) << " --> " << cost;
    //const float empMul = side_points_ / szPS2;
    //cost *= empMul;
    //if (debugShow) debugOutput << " --> " << cost << " @ " << empMul;
    if (debugShow) debugOutput << "\n";

    return cost;
}

float SingleCompletion::compareCell(
    const GridCellPtr& cell1, const GridCellPtr& cell2,
    const bool debugShow)
{
    PointSetANNQueryPtr KNN = cell2->KNN;
    const PointSet& cellPS1 = *cell1->GetCellPS(nullptr);
    const PointSet& cellPS2 = *cell2->GetCellPS(nullptr);
    const unsigned szPS1 = cellPS1.getNumEntries();
    unsigned szPS2 = cellPS2.getNumEntries();
    const AAT posAAT = cellPS2.getAAT("position");

    float costIn = 0;
    unsigned numIn = 0;
    if (debugShow) debugOutput << "\n" << NUM_BINARY++ << " cell pair\n";
    if (debugShow) debugRenderer->beginRenderJob_OneFrame("draw_binary_", DR_FRAME++);
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

        //const float value = gaussTruncate(distNN, inlier_dist_, 4, gauss_truncate_);
        //const float value = 1.f;
        const float value = energyTruncate(distNN, inlier_dist_, 4); // -log

        costIn += value;
    }
    if (debugShow) debugOutput << "\n";
    if (debugShow) debugRenderer->endRenderJob();

    const float matchLimit = (float)szPS1 * 0.3f;
    const float inLimit = energyTruncate(mean_error_th_, 1, 4);
    const float misCost = energyTruncate(0.5f, 1, 4);
    const float maxCost = energyTruncate(1.f, 1, 4);
    const unsigned numMis1 = szPS1 - numIn;
    if (szPS2 < numIn) szPS2 = numIn;
    const unsigned numMis2 = szPS2 - numIn;
    float cost;
    if (matchLimit > numIn) cost = maxCost;
    else if ((costIn / (float)numIn) < inLimit) cost = 0;
    else {
        cost = ((float)(numMis1 + numMis2) * misCost + costIn) / (numMis1 + numMis2 + numIn);
    }

    if (maxCost < cost) debugOutput << cost << "\n";
    if (debugShow) debugOutput << numIn << " (" << szPS1 << ", " << szPS2 << "): "
        << (costIn / (float)numIn) << " --> " << cost;
    //const float empMul = side_points_ / szPS2;
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

float SingleCompletion::computeBinary(
    int siteI, int siteJ, LabelType labelI, LabelType labelJ,
    const int ring
    )
{
    const GridKeyType key1 = unionGrid_->GetCellIndex(siteI);
    const GridKeyType key2 = unionGrid_->GetCellIndex(siteJ);
    const int xmin = std::min(key1[0] - ring, key2[0] - ring);
    const int ymin = std::min(key1[1] - ring, key2[1] - ring);
    const int zmin = std::min(key1[2] - ring, key2[2] - ring);
    const int xmax = std::max(key1[0] + ring, key2[0] + ring);
    const int ymax = std::max(key1[1] + ring, key2[1] + ring);
    const int zmax = std::max(key1[2] + ring, key2[2] + ring);
    //const int vol = (xmax - xmin + 1)*(ymax - ymin + 1)*(zmax - zmin + 1);
    const bool debugShow = false;
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
    float vol = 0;
//#pragma omp parallel for schedule(dynamic,1)
    for (int xi = xmin; xi <= xmax; ++xi) {
        for (int yi = ymin; yi <= ymax; ++yi) {
            for (int zi = zmin; zi <= zmax; ++zi) {
                GridKeyType key0;
                key0[0] = xi; key0[1] = yi; key0[2] = zi;
                if (!unionGrid_->Contains(key0)) continue;
                UnionGridCellPtr uniCell = unionGrid_->GetCellPtr(key0);
                const UnionCell::MapT& cellMap = uniCell->cellMap;
                if (0 == cellMap.count(labelI) || 0 == cellMap.count(labelJ)) {
                    cost += 0.5f;
                }
                else {
                    vol += 1;
                    cost += compareCell(
                        cellMap.at(labelI), cellMap.at(labelJ),
                        false
                        );
                    if (debugShow) {
                        //const BoundingBox3f& bb = unionGrid_->GetCellBBox(key0);
                        //debugRenderer->addBoundingBox(bb, makeVector3f(0.4, 0.4, 0.4), 1);
                    }
                }
            }
        }
    }
    cost /= vol;
    if (debugShow) {
        debugOutput << cost << "\n";
    }
    if (debugShow) debugRenderer->endRenderJob();
    return cost;
}

void SingleCompletion::BuildGraphicalModel(void)
{
    debugOutput << "\nbuilding graphical model ... \n";

    // Build Model
    const IndexType numVar = unionGrid_->size();
    LabelType numLabel = patches_.size();
    debugOutput << numVar << " variables, " << numLabel << " labels.\n";
    std::vector<LabelType> numberOfLabels(numVar, numLabel);
    graph_model_ = boost::shared_ptr<GraphModel>(
        new GraphModel(SpaceType(numberOfLabels.begin(), numberOfLabels.end()))
        );
    //GraphModel graphModel(SpaceType(numberOfLabels.begin(), numberOfLabels.end()));
    GraphModel& graphModel = *graph_model_;

    // 1st-order
    const float maxCost = energyTruncate(1.f, 1, 10);
    const float costBon = 10 * maxCost;
    const float costEmp = energyTruncate(mean_error_th_, 1, 4);
    const float costFill = 0.001f;
    float vol = power(side_points_, 1.5f);
    progressWindow->pushStep(true, "1st order");
    dataCost_.resize(numVar * numLabel, 0);
    for (IndexType var = 0; var < numVar; ++var) {
        const LabelType shape[] = { numLabel };
        ExplicitFunction f(shape, shape + 1);

        UnionGridCellPtr uniCell = unionGrid_->GetCellPtr(var);
        const UnionCell::MapT& cellMap = uniCell->cellMap;

        // empty source cell can not take label 0
        if (0 == cellMap.count(0)) {
            f(0) = 0.f;
            for (LabelType lab = 1; lab < numLabel; ++lab) {
                // never add empty cell
                if (0 == cellMap.count(lab)) {
                    f(lab) = costEmp;
                    continue;
                }
                if (cellMap.at(lab)->isBon) f(lab) = costBon;
                // fill as much as possible
                else f(lab) = (vol - cellMap.at(lab)->size()) * costFill;
                //else f(lab) = 0;
            }
        }
        else {
            // boundary source cell can not take label 0
            if (cellMap.at(0)->isBon) {
                f(0) = costBon;
            }
            else {
                f(0) = 0;
            }

            // compute cost by comparing
            for (LabelType lab = 1; lab < numLabel; ++lab) {
                if (0 == cellMap.count(lab)) {
                    f(lab) = costEmp;
                    continue;
                }
                if (cellMap.at(lab)->isBon) f(lab) = costBon;
                else {
                    f(lab) = compareCellMajor(
                        cellMap.at(0), cellMap.at(lab),
                        //(500 < NUM_UNARY) ? false : true
                        false
                        );
                }
            }
        }

        for (LabelType lab = 0; lab < numLabel; ++lab) {
            dataCost_[var * numLabel + lab] = f(lab);
        }

        FunctionIdentifier fid = graphModel.addFunction(f);
        IndexType varIndex[] = { var };
        graphModel.addFactor(fid, varIndex, varIndex + 1);

        progressWindow->progressf((float)var / (float)(numVar - 1));
    }
    progressWindow->popStep();

    // 2nd-order
    progressWindow->pushStep(true, "2nd order");
    unsigned cntPair = 0;
    for (IndexType vi = 0; vi < numVar; ++vi) {
        for (IndexType vj = vi + 1; vj < numVar; ++vj) {
            ++cntPair;
            const int diff_abs = KeyDist(
                unionGrid_->GetCellIndex(vi),
                unionGrid_->GetCellIndex(vj)
                );
            if (1 < diff_abs) {
                continue;
            }

            const LabelType shape[] = { numLabel, numLabel };
            ExplicitFunction f(shape, shape + 2);
            for (LabelType l1 = 0; l1 < numLabel; ++l1) {
                for (LabelType l2 = 0; l2 < numLabel; ++l2) {
                    f(l1, l2) = computeBinary(vi, vj, l1, l2, 1);
                    //f(l1, l2) = smoothFn(vi, vj, l1, l2);
                    //if (l1 == l2) f(l1, l2) = 0;
                    //else f(l1, l2) = 1;
                }
            }
            FunctionIdentifier fid = graphModel.addFunction(f);
            IndexType varIndex[] = { vi, vj };
            graphModel.addFactor(fid, varIndex, varIndex + 2);

        }
        progressWindow->progressf((float)vi / (float)(numVar - 1));
    }
    progressWindow->popStep();

    debugOutput << "done.\n";
}

UICPC* SingleCompletion::CollectPoints(void)
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

        if (0 == uniCell->cellMap.count(label)) {
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

void SingleCompletion::Infer(void)
{
    debugOutput << "\nsolving ... \n";
    X4_TIMER_START(inference);

    // Infer with LBP
    //InferLBP();
    InferGCO();

    debugRenderer->beginRenderJob_OneFrame("show_labels_", 1);
    const IndexType numVar = unionGrid_->size();
    //unionGrid_->DrawWithDR(makeVector3f(0.4, 0.4, 0.4));
    for (IndexType var = 0; var < numVar; var += 1) {
        if (0 == labels_[var]) continue;
        std::ostringstream ss;
        ss << labels_[var];
        debugRenderer->addText(
            unionGrid_->GetCellCenter(var),
            ss.str(),
            20, "Verdana",
            makeVector3f(0.5, 0, 1));
        debugRenderer->addBoundingBox(unionGrid_->GetCellBBox(var),
            makeVector3f(0.4, 0.4, 0.4), 1);
    }
    debugRenderer->endRenderJob();

    X4_TIMER_STOP(inference);
    debugOutput << "done.\n\n";
}

void SingleCompletion::InferLBP(void)
{
    GraphModel& graphModel = *graph_model_;

    typedef opengm::BeliefPropagationUpdateRules<GraphModel, opengm::Minimizer> UpdateRules;
    typedef opengm::MessagePassing<GraphModel, opengm::Minimizer, UpdateRules, opengm::MaxDistance>  LBP;

    LBP::Parameter parameter(
        unionGrid_->size() * 2, // max number of iterations
        1e-6, // convergence bound
        0 // damping
        );
    LBP lbp(graphModel, parameter);
    lbp.infer();
    lbp.arg(labels_);
}

void SingleCompletion::InferGCO(void)
{
    const IndexType numVar = unionGrid_->size();
    LabelType numLabel = patches_.size();
    GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph(
        numVar, numLabel);
    gc->setDataCost(&dataCost_[0]);
    //gc->setSmoothCost(&pairCost[0]);
    gc->setSmoothCost(&smoothFn);
    for (int vi = 0; vi < numVar; ++vi) {
        for (int vj = vi + 1; vj < numVar; ++vj) {
            const int diff_abs = KeyDist(
                unionGrid_->GetCellIndex(vi),
                unionGrid_->GetCellIndex(vj)
                );
            if (1 == diff_abs) gc->setNeighbors(vi, vj);
        }
    }
    try{
        debugOutput << "Before optimization energy is " << gc->compute_energy() << "\n";
        for (unsigned d = 0; d < 1; ++d)
        {
            gc->expansion(2);
            //gc->swap(2);
            debugOutput << "After optimization energy is " << gc->compute_energy() << "\n";
        }
    }
    catch (GCException e){
        error(e.message);
    }
    labels_.resize(numVar);
    for (int vi = 0; vi < numVar; ++vi)
        labels_[vi] = gc->whatLabel(vi);
    delete gc;
}

void SingleCompletion::InferMRFLBP(void)
{
    GraphModel& graphModel = *graph_model_;

    // MRFLIB only supports graphical models which have a grid structure
    typedef opengm::external::MRFLIB<GraphModel> MRFLIB;

    MRFLIB::Parameter parameter;
    parameter.inferenceType_ = MRFLIB::Parameter::MAXPRODBP;
    parameter.energyType_ = MRFLIB::Parameter::TL2;
    parameter.numberOfIterations_ = 10;

    MRFLIB mrf(graphModel, parameter);
    mrf.infer();
    mrf.arg(labels_);
}

void SingleCompletion::InferMRFTRWS(void)
{
    GraphModel& graphModel = *graph_model_;

    // MRFLIB only supports graphical models which have a grid structure
    typedef opengm::external::MRFLIB<GraphModel> MRFLIB;

    MRFLIB::Parameter parameter;
    parameter.inferenceType_ = MRFLIB::Parameter::TRWS;
    parameter.energyType_ = MRFLIB::Parameter::VIEW;
    parameter.numberOfIterations_ = 10;
    parameter.trwsTolerance_ = 1;

    MRFLIB mrf(graphModel, parameter);
    mrf.infer();
    mrf.arg(labels_);
}

void SingleCompletion::InferMRFGC(void)
{
    GraphModel& graphModel = *graph_model_;

    // MRFLIB only supports graphical models which have a grid structure
    typedef opengm::external::MRFLIB<GraphModel> MRFLIB;
    MRFLIB::Parameter parameter;
    parameter.inferenceType_ = MRFLIB::Parameter::EXPANSION;
    parameter.energyType_ = MRFLIB::Parameter::TL2;
    parameter.numberOfIterations_ = 10;

    MRFLIB mrf(graphModel, parameter);
    mrf.infer();
}

void SingleCompletion::VisualizeCost(void)
{
    debugOutput << "drawing cost ... \n";
    const IndexType numVar = unionGrid_->size();
    LabelType numLabel = patches_.size();
    for (LabelType lab = 0; lab < numLabel; ++lab) {
        debugRenderer->beginRenderJob_OneFrame("visualize_cost_", DR_FRAME++);
        unionGrid_->DrawWithDR(makeVector3f(0.4, 0.4, 0.4));
        for (IndexType var = 0; var < numVar; var += 1) {
            const float cost = dataCost_[var * numLabel + lab];
            if (cost > 0.99 || cost < 0.01) continue;
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
