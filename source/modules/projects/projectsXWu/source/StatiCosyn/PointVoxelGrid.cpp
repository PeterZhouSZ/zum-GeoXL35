//---------------------------------------------------------------------------
#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "PointVoxelGrid.h"
#include "Util\gco-v3.0\GCoptimization.h"
//---------------------------------------------------------------------------
#include "ProgressWindow.h"
#include "AnnSearch.h"
//---------------------------------------------------------------------------
using namespace X4;
//---------------------------------------------------------------------------
namespace {
    int DR_FRAME = 0;
}

SSliceStack::SSliceStack(
    UICPC* dataPC, UICPC* featPC,
    const Vector3f& dir, const float& voxize)
{
    //pairDist_ = nullptr;
    voxize_ = voxize;
    gendir_ = dir;

    // this align generator direction to Z
    tParaZ_ = IDENTITY4F;
    {
        Matrix3f t3s = IDENTITY3F;
        t3s[2][2] = t3s[2][2] * norm(dir) / voxize;
        Matrix3f t3t = IDENTITY3F;
        Vector3f zp = normalize(dir);
        unsigned d = 0;
        for (; d < 3; ++d) if (0.5f > abs(zp[d])) break;
        t3t[0] = t3t[d].crossProduct(zp);
        t3t[1] = zp.crossProduct(t3t[0]);
        t3t[2] = zp;
        tZback_ = expand3To4(t3t * t3s);
        tParaZ_ = invertMatrix(tZback_);
        //t3t[2] = zp * norm(dir);
        //tParaZ_ = expand3To4(t3t * invertMatrix(t3s));
        //tZback_ = expand3To4(t3s * invertMatrix(t3t));
        //const float scale = voxize / norm(dir);
        ////for (; d < 3; ++d) tParaZ_[d][d] = tParaZ_[d][d] * scale;
        //tParaZ_[2][2] = tParaZ_[2][2] * scale;
    }

    // transform points using alignment matrix
    const PointSet& dataPS = *dataPC->getPointSet();
    {
        const unsigned& num_verts = dataPS.getNumEntries();
        zDataPS_ = new PointSet;
        VertexDescriptor vd;
        vd.pushAttrib(dataPS.getDescr()->getAttribute("position"));
        vd.pushAttrib(dataPS.getDescr()->getAttribute("color"));
        vd.pushAttrib(dataPS.getDescr()->getAttribute("normal"));
        zDataPS_->clearAndSetup(1, num_verts, &vd);
        const AAT& posAAT = dataPS.getAAT("position");
        const AAT& colorAAT = dataPS.getAAT("color");
        const AAT& normalAAT = dataPS.getAAT("normal");
        for (unsigned vi = 0; vi < num_verts; ++vi) {
            const Vector3f& pos = dataPS.get3f(vi, posAAT);
            zDataPS_->set3f(vi, posAAT, transformVector3f(tParaZ_, pos));
            zDataPS_->set3f(vi, colorAAT, dataPS.get3f(vi, colorAAT));
            zDataPS_->set3f(vi, normalAAT, dataPS.get3f(vi, normalAAT));
        }
    }
    const PointSet& featPS = *featPC->getPointSet();
    {
        const unsigned& num_verts = featPS.getNumEntries();
        zFeatPS_ = new PointSet;
        VertexDescriptor vd;
        vd.pushAttrib(featPS.getDescr()->getAttribute("position"));
        //vd.pushAttrib(featPS.getDescr()->getAttribute("hoc"));
        zFeatPS_->clearAndSetup(1, num_verts, &vd);
        const AAT& posAAT = featPS.getAAT("position");
        //const AAT& hocAAT_s = featPS.getAAT("hoc");
        //const AAT& hocAAT_t = zFeatPS_->getAAT("hoc");
        //DVectorF hoc;
        //hoc.setDim(1024);
        for (unsigned vi = 0; vi < num_verts; ++vi) {
            {
                const Vector3f& pos = featPS.get3f(vi, posAAT);
                zFeatPS_->set3f(vi, posAAT, transformVector3f(tParaZ_, pos));
            }
            {
                //card8 *pHocS = (card8*)featPS.getDataPointer(vi);
                //card8 *pHocT = (card8*)zFeatPS_->getDataPointer(vi);
                //pHocS += hocAAT_s.getOffset();
                //pHocT += hocAAT_t.getOffset();
                //memcpy((float32*)pHocT, (float32*)pHocS, 1024);
            }
        }
    }

    // compute bounding box
    {
        const unsigned& num_verts = dataPS.getNumEntries();
        const AAT& posAAT = dataPS.getAAT("position");
        BoundingBox3f bb(zDataPS_->get3f(0, posAAT));
        for (unsigned ii = 1; ii < num_verts; ++ii)
            bb.addPoint(zDataPS_->get3f(ii, posAAT));
        //Vector3f sideLength = bb.getSideLength();
        bb.addBorder(bb.getMaxSideLength() * 0.01f);
        spaceLower_ = GetCellIndexFromPoint(bb.lowerCorner);
        spaceUpper_ = GetCellIndexFromPoint(bb.upperCorner);
        spaceVolume_ = 1;
        for (unsigned d = 0; d < 3; ++d) {
            spaceMeasure_[d] = spaceUpper_[d] - spaceLower_[d] + 1;
            //spaceMeasure_[d] = ceil(bb.getSideLength(d) / voxize);
            spaceVolume_ *= spaceMeasure_[d];
        }
        //spaceBB_ = BoundingBox3f(spaceLower_ * voxize_, spaceUpper_ * voxize_);
        //spaceBB_ = bb;
        {
            Vector3f lc, uc;
            for (unsigned d = 0; d < 3; ++d) {
                lc[d] = (float)spaceLower_[d] * voxize_;
                uc[d] = (float)spaceUpper_[d] * voxize_;
            }
            spaceBB_ = BoundingBox3f(lc, uc);
        }
    }

    {
        synthMeasure_ = spaceMeasure_;
        synthLower_ = spaceLower_;
        synthUpper_ = spaceUpper_;
        synthVolume_ = spaceVolume_;
        synthBB_ = spaceBB_;
    }

    {
        // at least 1 coinident voxels
        shiftLower_ = - spaceMeasure_ + makeVector3i(1, 1, 1);
        shiftUpper_ = spaceMeasure_ - makeVector3i(1, 1, 1);
        shiftMeasure_ = shiftUpper_ - shiftLower_ + makeVector3i(1, 1, 1);
        shiftVolume_ = shiftMeasure_[0] * shiftMeasure_[1] * shiftMeasure_[2];
    }

    // initialize storage
    spaceVoxels_.resize(spaceVolume_);
    {
        for (int vi = 0; vi < spaceVolume_; ++vi) {
            spaceVoxels_[vi] = SVoxel::Ptr(new SVoxel);
        }
    }

    Voxelize(zDataPS_, zFeatPS_);
    ComputeShiftHausdorff(featPS, "hoc", 1024);
}
SSliceStack::~SSliceStack()
{
    delete zDataPS_;
    delete zFeatPS_;
    //delete pairDist_;
}

Vector3i SSliceStack::GetCellIndexFromPoint(const Vector3f& pos)
{
    // rounding behevior, so centered at grid point exactly
    Vector3f cen = pos / voxize_;
    cen[0] = (cen[0] > 0.0) ? floor(cen[0] + 0.5) : ceil(cen[0] - 0.5);
    cen[1] = (cen[1] > 0.0) ? floor(cen[1] + 0.5) : ceil(cen[1] - 0.5);
    cen[2] = (cen[2] > 0.0) ? floor(cen[2] + 0.5) : ceil(cen[2] - 0.5);

    return makeVector3i(
        static_cast<int32>(cen[0]),
        static_cast<int32>(cen[1]),
        static_cast<int32>(cen[2])
        );
}
Vector3f SSliceStack::GetCellCenterFromIndex(const Vector3i& idx)
{
    return makeVector3f(idx[0], idx[1], idx[2]) * voxize_;
}
BoundingBox3f SSliceStack::GetCellBBoxFromIndex(const Vector3i& idx)
{
    Vector3f ccen = GetCellCenterFromIndex(idx);
    BoundingBox3f bb;
    bb.lowerCorner = bb.upperCorner = ccen;
    bb.addBorder(voxize_ / 2.0f);
    return bb;
}
int SSliceStack::Index2SN(
    const Vector3i& index,
    const Vector3i& measure, const Vector3i& lower)
{
    Vector3i indexRel = index - lower;
    return
        (indexRel[0] + 
        (indexRel[1] * measure[0]) + 
        (indexRel[2] * measure[0] * measure[1]));
}
Vector3i SSliceStack::SN2Index(
    const int& sn,
    const Vector3i& measure, const Vector3i& lower)
{
    Vector3i result;
    result[0] = sn % measure[0];
    result[1] = (sn / measure[0]) % measure[1];
    result[2] = sn / (measure[0] * measure[1]);
    return result + lower;
}
bool SSliceStack::onBorder(
    const Vector3i& index,
    const Vector3i& lower, const Vector3i& upper)
{
    return
        ((index[0] == lower[0]) || (index[0] == upper[0])) ||
        ((index[1] == lower[1]) || (index[1] == upper[1])) ||
        ((index[2] == lower[2]) || (index[2] == upper[2]));
}
bool SSliceStack::isValid(
    const Vector3i& index,
    const Vector3i& lower, const Vector3i& upper)
{
    return
        ((index[0] >= lower[0]) && (index[0] <= upper[0])) &&
        ((index[1] >= lower[1]) && (index[1] <= upper[1])) &&
        ((index[2] >= lower[2]) && (index[2] <= upper[2]));
}
int SSliceStack::intersectVoxels(
    const Vector3i& lowerL, const Vector3i& upperL,
    const Vector3i& lowerR, const Vector3i& upperR,
    Vector3i& lowerM, Vector3i& upperM)
{
    if (isValid(lowerL, lowerR, upperR)) {
        lowerM = lowerL;
        upperM = upperR;
        return 1;
    } else if (isValid(upperL, lowerR, upperR)) {
        lowerM = lowerR;
        upperM = upperL;
        return 1;
    } else {
        return 0;
    }
}

void SSliceStack::Voxelize(PointSet* dataPS, PointSet* featPS)
{
    {
        const unsigned& num_verts = dataPS->getNumEntries();
        const AAT& posAAT = dataPS->getAAT("position");
        for (unsigned vi = 0; vi < num_verts; ++vi) {
            const Vector3f& pos = dataPS->get3f(vi, posAAT);
            Vector3i idx = GetCellIndexFromPoint(pos);
            spaceVoxels_[Index2SN(idx, spaceMeasure_, spaceLower_)]->
                dataInd_.push_back(vi);
        }
    }
    {
        const unsigned& num_verts = featPS->getNumEntries();
        const AAT& posAAT = featPS->getAAT("position");
        for (unsigned vi = 0; vi < num_verts; ++vi) {
            const Vector3f& pos = featPS->get3f(vi, posAAT);
            Vector3i idx = GetCellIndexFromPoint(pos);
            spaceVoxels_[Index2SN(idx, spaceMeasure_, spaceLower_)]->
                featInd_.push_back(vi);
        }
    }
}

//void SSliceStack::ComputePairHausdorff(
//    const PointSet& fPS, const std::string& aat, const unsigned& fDim)
//{
//    const AAT& fAAT = fPS.getAAT(aat);
//    SparseMatrix<float> pairDistSparse;
//    pairDistSparse.setRows(spaceVolume_);
//
//    std::string msg1 = std::string("Pairwise Hausdorff distance");
//    progressWindow->pushStep(true, msg1, (float)spaceVolume_);
//    distPairMax_ = 0.f; // record max value
//    for (int ii = 0; ii < spaceVolume_; ++ii) {		
//        progressWindow->progress(100 * (double)ii / (double)spaceVolume_);
//        // no storage for empty voxels
//        if (0 == spaceVoxels_[ii]->size()) continue;
//        SVoxel::Ptr voxel_i = spaceVoxels_[ii];
//        // build ANN search structure for this voxel
//        const unsigned num_f_i = voxel_i->size();
//        std::vector<float> fVecVal(num_f_i * fDim);
//        std::vector<float*> fVecPtr(num_f_i);
//        for (unsigned fi = 0; fi < num_f_i; ++fi) {
//            const unsigned& f_id = (*voxel_i)[fi];
//            card8 *ptr = (card8*)fPS.getDataPointer(f_id);
//            ptr += fAAT.getOffset();
//            float32 *fptr = (float32*)ptr;
//            for (mpcard d = 0; d < fDim; ++d) {
//                fVecVal[fi * fDim + d] = fptr[d];
//            }
//            fVecPtr[fi] = &fVecVal[fi * fDim];
//        }
//        AnnSearch anns(0.01f);
//        anns.buildSearchStructure(num_f_i, fDim, &fVecPtr[0]);
//        for (mpcard jj = 0; jj < spaceVolume_; ++jj) {
//            if (ii == jj) continue;
//            if (0 == spaceVoxels_[jj]->size()) continue;
//            {
//                //const Vector3i idx_diff =
//                //    SN2Index(jj, spaceMeasure_, spaceLower_) -
//                //    SN2Index(ii, spaceMeasure_, spaceLower_);
//                //const int diff_abs = abs(idx_diff[0]) + abs(idx_diff[1]) + abs(idx_diff[2]);
//                //if (diff_abs != 1) continue;
//            }
//            if (!pairDistSparse[ii].hasEntry(jj)) {
//                pairDistSparse[ii].setEntryBinary(jj, 0.f);
//                pairDistSparse[jj].setEntryBinary(ii, 0.f);
//            }
//            SVoxel::Ptr voxel_j = spaceVoxels_[jj];
//            const unsigned num_f_j = voxel_j->size();
//            std::vector<float> fVec(fDim);
//            float distMax = 0.f;
//            for (unsigned fj = 0; fj < num_f_j; ++fj) {
//                const unsigned& f_id = (*voxel_j)[fj];
//                card8 *ptr = (card8*)fPS.getDataPointer(f_id);
//                ptr += fAAT.getOffset();
//                float32 *fptr = (float32*)ptr;
//                for (mpcard d = 0; d < fDim; ++d) {
//                    fVec[d] = fptr[d];
//                }
//                float dist = 10;
//                anns.getNearestNeighborAndDistance(&fVec[0], dist);
//                if (distMax < dist) distMax = dist;
//            }
//            if (distMax > pairDistSparse[ii][jj]) {
//                pairDistSparse[ii].setEntryBinary(jj, distMax);
//                pairDistSparse[jj].setEntryBinary(ii, distMax);
//            }
//            if (distPairMax_ < distMax) distPairMax_ = distMax;
//        }
//    }
//    progressWindow->popStep();
//
//    // initialize to 0
//    delete pairDist_;
//    pairDist_ = new float[spaceVolume_ * spaceVolume_];
//    memset(pairDist_, 0, spaceVolume_ * spaceVolume_ * sizeof(float));
//    for (int ii = 0; ii < spaceVolume_; ++ii) {
//        for (int jj = ii + 1; jj < spaceVolume_; ++jj) {
//            const Vector3i idx_diff =
//                SN2Index(jj, spaceMeasure_, spaceLower_) -
//                SN2Index(ii, spaceMeasure_, spaceLower_);
//            const int diff_abs = abs(idx_diff[0]) + abs(idx_diff[1]) + abs(idx_diff[2]);
//            //if (diff_abs != 1) continue;
//            //const float dist = (pairDistSparse[ii].hasEntry(jj)) ?
//            //    pairDistSparse[ii][jj] : (distPairMax_ * 10);
//            //{
//            //    pairDist_[ii * spaceVolume_ + jj] = dist;
//            //    pairDist_[jj * spaceVolume_ + ii] = dist;
//            //}
//            //// self-matching: multiple of largest first neighbor
//            //const float distSelf = dist * 4;
//            //if (pairDist_[ii * spaceVolume_ + ii] < distSelf) {
//            //    pairDist_[ii * spaceVolume_ + ii] = distSelf;
//            //    pairDist_[jj * spaceVolume_ + jj] = distSelf;
//            //}
//
//            //pairDist_[ii * spaceVolume_ + jj] = diff_abs * spaceVoxels_[jj]->size();
//            //continue;
//
//            // increasing cost w.r.t. distance
//            const float w = std::pow(4, abs(diff_abs - 1));
//            float dist = 0.f;
//            if (pairDistSparse[ii].hasEntry(jj)) {
//                dist = pairDistSparse[ii][jj] * w;
//            } else { // prevent adding empty voxels
//                dist = distPairMax_ * 10 * w;
//            }
//            pairDist_[ii * spaceVolume_ + jj] = dist;
//            pairDist_[jj * spaceVolume_ + ii] = dist;
//            if (diff_abs == 1) {
//                // self-matching: multiple of largest first neighbor
//                const float distSelf = dist * 2;
//                if (pairDist_[ii * spaceVolume_ + ii] < distSelf) {
//                    pairDist_[ii * spaceVolume_ + ii] = distSelf;
//                    pairDist_[jj * spaceVolume_ + jj] = distSelf;
//                }
//            }
//            //// first neighbor has lowerest cost
//            //if (diff_abs == 1) {
//            //    pairDist_[ii * spaceVolume_ + jj] = 0;
//            //}
//        }
//    }
//}

void SSliceStack::ComputeShiftHausdorff(
    const PointSet& fPS, const std::string& aat, const unsigned& fDim)
{
    const AAT& fAAT = fPS.getAAT(aat);
    SparseMatrix<float> pairDistSparse;
    pairDistSparse.setRows(spaceVolume_);

    const bool ann_compute = true;
    progressWindow->pushStep(true, "Pairwise Hausdorff distance", (float)spaceVolume_);
    distPairMax_ = 0.f; // record max value
    for (int ii = 0; ii < spaceVolume_; ++ii) {		
        progressWindow->progress(100 * (double)ii / (double)spaceVolume_);
        // no storage for empty voxels
        if (0 == spaceVoxels_[ii]->size()) continue;
        SVoxel::Ptr voxel_i = spaceVoxels_[ii];
        // build ANN search structure for this voxel
        const unsigned num_f_i = voxel_i->size();
        std::vector<float> fVecVal(num_f_i * fDim);
        std::vector<float*> fVecPtr(num_f_i);
        for (unsigned fi = 0; fi < num_f_i; ++fi) {
            const unsigned& f_id = (*voxel_i)[fi];
            card8 *ptr = (card8*)fPS.getDataPointer(f_id);
            ptr += fAAT.getOffset();
            float32 *fptr = (float32*)ptr;
            for (mpcard d = 0; d < fDim; ++d) {
                fVecVal[fi * fDim + d] = fptr[d];
            }
            fVecPtr[fi] = &fVecVal[fi * fDim];
        }
        AnnSearch anns(0.01f);
        if (ann_compute)
            anns.buildSearchStructure(num_f_i, fDim, &fVecPtr[0]);
        for (mpcard jj = 0; jj < spaceVolume_; ++jj) {
            if (ii == jj) continue;
            if (0 == spaceVoxels_[jj]->size()) continue;
            {
                //const Vector3i idx_diff =
                //    SN2Index(jj, spaceMeasure_, spaceLower_) -
                //    SN2Index(ii, spaceMeasure_, spaceLower_);
                //const int diff_abs = abs(idx_diff[0]) + abs(idx_diff[1]) + abs(idx_diff[2]);
                //if (diff_abs != 1) continue;
            }
            if (!pairDistSparse[ii].hasEntry(jj)) {
                pairDistSparse[ii].setEntryBinary(jj, 0.f);
                pairDistSparse[jj].setEntryBinary(ii, 0.f);
            }
            SVoxel::Ptr voxel_j = spaceVoxels_[jj];
            const unsigned num_f_j = voxel_j->size();
            std::vector<float> fVec(fDim);
            float distMax = 0.f;
            for (unsigned fj = 0; fj < num_f_j; ++fj) {
                const unsigned& f_id = (*voxel_j)[fj];
                card8 *ptr = (card8*)fPS.getDataPointer(f_id);
                ptr += fAAT.getOffset();
                float32 *fptr = (float32*)ptr;
                for (mpcard d = 0; d < fDim; ++d) {
                    fVec[d] = fptr[d];
                }
                float dist = 1;
                if (ann_compute)
                    anns.getNearestNeighborAndDistance(&fVec[0], dist);
                //if (distMax < dist) distMax = dist; // Hausdorff: outlier
                distMax += dist / (float)num_f_j; // mean sum of minimum
            }
            if (distMax > pairDistSparse[ii][jj]) {
                pairDistSparse[ii].setEntryBinary(jj, distMax);
                pairDistSparse[jj].setEntryBinary(ii, distMax);
            }
            if (distPairMax_ < distMax) distPairMax_ = distMax;
        }
    }
    progressWindow->popStep();

    const bool debug_show = false;
    // initialize to 0
    //delete pairDist_;
    //pairDist_ = new float[shiftVolume_ * shiftVolume_];
    //memset(pairDist_, 0, shiftVolume_ * shiftVolume_ * sizeof(float));
    pairDist_.resize(shiftVolume_ * shiftVolume_, 0);
    //const float distNonInter = distPairMax_ * spaceVolume_;
    //const float distEmpty = distPairMax_ * shiftMeasure_[0] * shiftMeasure_[1];
    const float distNonInter = distPairMax_ * 2;
    const float distInter = distPairMax_ * 0.1f; // use this for pure counting
    const float distEmpty = distPairMax_ * 1.5f;
    const int radiusInter = 2; // only calculate within this radius
    progressWindow->pushStep(true, "Compare voxels", (float)shiftVolume_);
    for (int ii = 0; ii < shiftVolume_; ++ii) {
        progressWindow->progress(100 * (double)ii / (double)shiftVolume_);
        for (int jj = ii + 1; jj < shiftVolume_; ++jj) {
            if (debug_show) debugRenderer->beginRenderJob_OneFrame("pair_dist_", ii * shiftVolume_ + jj);
            const Vector3i shift_i = SN2Index(ii, shiftMeasure_, shiftLower_);
            const Vector3i shift_j = SN2Index(jj, shiftMeasure_, shiftLower_);
            const Vector3i lowerI = spaceLower_ + shift_i;
            const Vector3i upperI = spaceUpper_ + shift_i;
            const Vector3i lowerJ = spaceLower_ + shift_j;
            const Vector3i upperJ = spaceUpper_ + shift_j;
            Vector3i lowerM, upperM;
            if (!intersectVoxels(lowerI, upperI, lowerJ, upperJ,
                lowerM, upperM))
            {
                pairDist_[ii * shiftVolume_ + jj] = distNonInter * spaceVolume_;
                if (debug_show)
                {
                    std::ostringstream ss;
                    ss << shiftVolume_;
                    debugRenderer->addText(NULL_VECTOR3F,
                        ss.str(),
                        24, "Verdana",
                        makeVector3f(0, 1, 0));
                }
            } else {
                Vector3i measureM = upperM - lowerM + makeVector3i(1, 1, 1);
                int volumeM = measureM[0] * measureM[1] * measureM[2];
                for (unsigned d = 0; d < 3; ++d) {
                    if (2 * radiusInter + 1 >= measureM[d]) continue;
                    const int lb = lowerM[d], ub = upperM[d];
                    if (measureM[d] % 2 == 0) {
                        int mi = (int)floor((float)(lb + ub) / 2);
                        lowerM[d] = mi - radiusInter + 1;
                        upperM[d] = mi + radiusInter;
                    } else {
                        int mi = (int)floor((float)(lb + ub + 1) / 2);
                        lowerM[d] = mi - radiusInter;
                        upperM[d] = mi + radiusInter;
                    }
                }
                measureM = upperM - lowerM + makeVector3i(1, 1, 1);
                volumeM = measureM[0] * measureM[1] * measureM[2];
                // 1. non-intersecting voxel has max cost
                pairDist_[ii * shiftVolume_ + jj] =
                    distNonInter * (spaceVolume_ - volumeM);
                // 2. intersecting voxel has pair distance
                for (int x = lowerM[0]; x <= upperM[0]; ++x) {
                    for (int y = lowerM[1]; y <= upperM[1]; ++y) {
                        for (int z = lowerM[2]; z <= upperM[2]; ++z) {
                            const Vector3i mid = makeVector3i(x, y, z);
                            const int vsn_i = Index2SN(mid,
                                spaceMeasure_, spaceLower_ + shift_i);
                            const int vsn_j = Index2SN(mid,
                                spaceMeasure_, spaceLower_ + shift_j);
                            if (debug_show)
                            {
                                std::ostringstream ss;
                                ss << vsn_i << " -> " << vsn_j;
                                debugRenderer->addText(
                                    GetCellCenterFromIndex(mid),
                                    ss.str(),
                                    20, "Verdana",
                                    makeVector3f(1, 0, 0));
                            }
                            //if (vsn_i == vsn_j) continue; // default 0
                            if (pairDistSparse[vsn_i].hasEntry(vsn_j)) {
                                pairDist_[ii * shiftVolume_ + jj] +=
                                    pairDistSparse[vsn_i][vsn_j];
                            } else {
                                pairDist_[ii * shiftVolume_ + jj] +=
                                    distEmpty;
                            }
                        }
                    }
                }
                if (debug_show)
                {
                    std::ostringstream ss;
                    ss << volumeM;
                    debugRenderer->addText(NULL_VECTOR3F,
                        ss.str(),
                        24, "Verdana",
                        makeVector3f(0, 1, 0));
                }
            }
            //const Vector3i shift_diff = shift_i - shift_j;
            //// square scale factor: nearby is more acceptable
            ////const float ds = (float)
            ////    (shift_diff[0] * shift_diff[0] +
            ////    shift_diff[1] * shift_diff[1] +
            ////    shift_diff[2] * shift_diff[2])
            ////    / shiftVolume_;
            //const float ds = abs(shift_diff[0]) + abs(shift_diff[1]) + abs(shift_diff[2]);
            //pairDist_[ii * shiftVolume_ + jj] *= ds;
            // symmetric distance
            pairDist_[jj * shiftVolume_ + ii] = pairDist_[ii * shiftVolume_ + jj];
            if (debug_show)
            {
                for (int vi = 0; vi < spaceVolume_; ++vi) {
                    {
                        const BoundingBox3f& bb = GetCellBBoxFromIndex(
                            SN2Index(vi, spaceMeasure_, spaceLower_) + shift_i
                            );
                        debugRenderer->addBoundingBox(bb, makeVector3f(1, 1, 0), 2);
                    }
                    {
                        const BoundingBox3f& bb = GetCellBBoxFromIndex(
                            SN2Index(vi, spaceMeasure_, spaceLower_) + shift_j
                            );
                        debugRenderer->addBoundingBox(bb, makeVector3f(0, 1, 1), 2);
                    }
                }
                debugRenderer->endRenderJob();
            }
        }
    }
    progressWindow->popStep();
}

void SSliceStack::Synthesis(void)
{
    // set up new measure
    int syns = (0 < std::rand() % 2 - 0.5f) ? 1 : -1;
    synthMeasure_ = synthMeasure_ + makeVector3i(0, 0, 1);
    synthLower_[2] = (syns < 0) ? (synthLower_[2] - 1) : synthLower_[2];
    synthUpper_[2] = (syns > 0) ? (synthUpper_[2] + 1) : synthUpper_[2];
    synthVolume_ += synthMeasure_[0] * synthMeasure_[1];
    synthLabels_.resize(synthVolume_, 0);
    {
        Vector3f lc, uc;
        for (unsigned d = 0; d < 3; ++d) {
            lc[d] = (float)synthLower_[d] * voxize_;
            uc[d] = (float)synthUpper_[d] * voxize_;
        }
        synthBB_ = BoundingBox3f(lc, uc);
    }
    {
        labelLower_ = synthLower_ - spaceLower_ - spaceMeasure_ + makeVector3i(1, 1, 1);
        labelUpper_ = synthUpper_ - spaceUpper_ + spaceMeasure_ - makeVector3i(1, 1, 1);
        labelMeasure_ = labelUpper_ - labelLower_ + makeVector3i(1, 1, 1);
        labelVolume_ = labelMeasure_[0] * labelMeasure_[1] * labelMeasure_[2];
    }

    const bool debug_show_1 = false;
    //// keep border
    //float *dataCost = new float[synthVolume_ * spaceVolume_];
    //for (int vi = 0; vi < synthVolume_; ++vi) {
    //    for (int li = 0; li < spaceVolume_; ++li) {
    //        const Vector3i vidx = SN2Index(vi, synthMeasure_, synthLower_);
    //        const Vector3i lidx = SN2Index(li, spaceMeasure_, spaceLower_);
    //        const bool onSpaceBorder = onBorder(vidx, synthLower_, synthUpper_);
    //        const bool onSynBorder = onBorder(lidx, spaceLower_, spaceUpper_);
    //        if ((onSpaceBorder && !onSynBorder) || (!onSpaceBorder && onSynBorder)) {
    //            dataCost[vi * spaceVolume_ + li] = distPairMax_;
    //        } else {
    //            dataCost[vi * spaceVolume_ + li] = 0;
    //        }
    //    }
    //}
    std::vector<float> dataCost(synthVolume_ * labelVolume_, 0);
    //float* dataCost = new float[synthVolume_ * labelVolume_];
    //memset(dataCost, 0, synthVolume_ * labelVolume_ * sizeof(float));
    const float distInvalid = distPairMax_ * 10000;
    const float distBorder = distPairMax_ * 1000;
    const float distEmpty = distPairMax_ * 10;
    const size_t voxelMin = 100;
    for (int vi = 0; vi < synthVolume_; ++vi) {
        for (int li = 0; li < labelVolume_; ++li) {
            const Vector3i vidx = SN2Index(vi, synthMeasure_, synthLower_);
            const Vector3i offset = SN2Index(li, labelMeasure_, labelLower_);
            // fixed boundary
            if (vidx[2] == synthLower_[2]) {
                if (offset == makeVector3i(0, 0, 0))
                    dataCost[vi * labelVolume_ + li] = 0;
                else
                    dataCost[vi * labelVolume_ + li] = distInvalid;
                continue;
            } else if (vidx[2] == synthUpper_[2]) {
                if (offset == makeVector3i(0, 0, synthMeasure_[2] - spaceMeasure_[2]))
                    dataCost[vi * labelVolume_ + li] = 0;
                else
                    dataCost[vi * labelVolume_ + li] = distInvalid;
                continue;
            }
            const Vector3i lidx = vidx - offset; // back to space
            if (!isValid(lidx, spaceLower_, spaceUpper_)) {
                dataCost[vi * labelVolume_ + li] = distInvalid;
            } else {
                // border on border
                const bool onSpaceBorder = onBorder(vidx, synthLower_, synthUpper_);
                const bool onSynBorder = onBorder(lidx, spaceLower_, spaceUpper_);
                if ((onSpaceBorder && !onSynBorder) || (!onSpaceBorder && onSynBorder)) {
                    dataCost[vi * labelVolume_ + li] = distBorder;
                } else {
                    dataCost[vi * labelVolume_ + li] = 0;
                }
                // prevent adding empty voxel
                const int sn_s = Index2SN(lidx, spaceMeasure_, spaceLower_);
                if (voxelMin > spaceVoxels_[sn_s]->dataInd_.size())
                    dataCost[vi * labelVolume_ + li] += distEmpty;
            }
            if (debug_show_1) debugRenderer->beginRenderJob_OneFrame("unary_cost_", vi * labelVolume_ + li);
            if (debug_show_1)
            {
                for (int ii = 0; ii < spaceVolume_; ++ii) {
                    {
                        const BoundingBox3f& bb = GetCellBBoxFromIndex(
                            SN2Index(ii, spaceMeasure_, spaceLower_) + offset
                            );
                        debugRenderer->addBoundingBox(bb, makeVector3f(1, 1, 0), 2);
                    }
                }
                for (int ii = 0; ii < synthVolume_; ++ii) {
                    {
                        const BoundingBox3f& bb = GetCellBBoxFromIndex(
                            SN2Index(ii, synthMeasure_, synthLower_)
                            );
                        debugRenderer->addBoundingBox(bb, makeVector3f(0, 1, 1), 2);
                    }
                }
                {
                    std::ostringstream ss;
                    ss << dataCost[vi * labelVolume_ + li];
                    debugRenderer->addText(
                        GetCellCenterFromIndex(vidx),
                        ss.str(),
                        20, "Verdana",
                        makeVector3f(1, 0, 0));
                }
                {
                    std::ostringstream ss;
                    ss << Index2SN(lidx, spaceMeasure_, spaceLower_)
                        << " -> " << vi;
                    debugRenderer->addText(
                        GetCellCenterFromIndex(lidx),
                        ss.str(),
                        20, "Verdana",
                        makeVector3f(1, 0, 0));
                }
            }
            if (debug_show_1) debugRenderer->endRenderJob();
        }
    }

    const bool debug_show_2 = false;
    std::vector<float> pairCost(labelVolume_ * labelVolume_, 0);
    //float* pairCost = new float[labelVolume_, labelVolume_];
    //memset(pairCost, 0, labelVolume_ * labelVolume_ * sizeof(float));
    for (int ii = 0; ii < labelVolume_; ++ii) {
        for (int jj = 0; jj < labelVolume_; ++jj) {
            const Vector3i label_i = SN2Index(ii, labelMeasure_, labelLower_);
            const Vector3i label_j = SN2Index(jj, labelMeasure_, labelLower_);
            const Vector3i shift_min = makeVector3i(
                std::min(label_i[0], label_j[0]),
                std::min(label_i[1], label_j[1]),
                std::min(label_i[2], label_j[2])
                );
            const Vector3i shift_i = label_i - shift_min;
            const Vector3i shift_j = label_j - shift_min;
            if (!isValid(shift_i, shiftLower_, shiftUpper_) ||
                !isValid(shift_j, shiftLower_, shiftUpper_)) {
                    pairCost[ii * labelVolume_ + jj] = distInvalid;
            } else {
                const int sn_i = Index2SN(shift_i, shiftMeasure_, shiftLower_);
                const int sn_j = Index2SN(shift_j, shiftMeasure_, shiftLower_);
                pairCost[ii * labelVolume_ + jj] =
                    pairDist_[sn_i * shiftVolume_ + sn_j];
            }
            if (debug_show_2) debugRenderer->beginRenderJob_OneFrame("binary_cost_", ii * labelVolume_ + jj);
            if (debug_show_2)
            {
                for (int vi = 0; vi < spaceVolume_; ++vi) {
                    {
                        const BoundingBox3f& bb = GetCellBBoxFromIndex(
                            SN2Index(vi, spaceMeasure_, spaceLower_) + label_i
                            );
                        debugRenderer->addBoundingBox(bb, makeVector3f(1, 1, 0), 2);
                    }
                    {
                        const BoundingBox3f& bb = GetCellBBoxFromIndex(
                            SN2Index(vi, spaceMeasure_, spaceLower_) + label_j
                            );
                        debugRenderer->addBoundingBox(bb, makeVector3f(0, 1, 1), 2);
                    }
                }
                {
                    std::ostringstream ss;
                    ss << pairCost[ii * labelVolume_ + jj];
                    debugRenderer->addText(NULL_VECTOR3F,
                        ss.str(),
                        24, "Verdana",
                        makeVector3f(0, 1, 0));
                }
            }
            if (debug_show_2) debugRenderer->endRenderJob();
        }
    }

    {
        GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph(
            synthVolume_, labelVolume_);
        gc->setDataCost(&dataCost[0]);
        gc->setSmoothCost(&pairCost[0]);
        for (int ii = 0; ii < synthVolume_; ++ii) {
            for (int jj = ii + 1; jj < synthVolume_; ++jj) {
                const Vector3i synth_i = SN2Index(ii, synthMeasure_, synthLower_);
                const Vector3i synth_j = SN2Index(jj, synthMeasure_, synthLower_);
                const Vector3i offset = synth_j - synth_i;
                const int diff_abs = abs(offset[0]) + abs(offset[1]) + abs(offset[2]);
                if (diff_abs == 1) gc->setNeighbors(ii, jj);
            }
        }
        try{
            printf("\nBefore optimization energy is %d",
                gc->compute_energy());
            gc->expansion(2);
            //gc->swap(2);
            printf("\nAfter optimization energy is %d",
                gc->compute_energy());
        }
        catch (GCException e){
            error(e.message);
        }
        for (int vi = 0; vi < synthVolume_; ++vi)
            synthLabels_[vi] = gc->whatLabel(vi);
        delete gc;
    }

    //delete dataCost;
    //delete pairCost;
}

UICPC* SSliceStack::CollectSynPoints(void)
{
    Matrix4f tBack = invertMatrix(tParaZ_);
    std::deque<Vector3f> posVec;
    std::deque<Vector3f> colorVec;
    std::deque<Vector3f> normalVec;
    bool errorOnce = false;
    for (int vi = 0; vi < synthVolume_; ++vi) {
        const int sn_l = synthLabels_[vi];
        const Vector3i shift = SN2Index(sn_l, labelMeasure_, labelLower_);
        const Vector3i idx_t = SN2Index(vi, synthMeasure_, synthLower_);
        const Vector3i idx_s = idx_t - shift;
        if (!isValid(idx_s, spaceLower_, spaceUpper_)) {
            if (!errorOnce) {
                error("SSliceStack::CollectSynPoints - wrong label");
                errorOnce = true;
            }
            continue;
        }
        const Vector3f offset = makeVector3f(
            (float)shift[0] * voxize_,
            (float)shift[1] * voxize_,
            (float)shift[2] * voxize_
            );
        const int sn_s = Index2SN(idx_s, spaceMeasure_, spaceLower_);
        //SVoxel::Ptr voxel = spaceVoxels_[sn_s];
        PointSet* vPS = zDataPS_->subset(spaceVoxels_[sn_s]->dataInd_);
        const AAT& posAAT = vPS->getAAT("position");
        const AAT& colorAAT = vPS->getAAT("color");
        const AAT& normalAAT = vPS->getAAT("normal");
        for (unsigned pi = 0; pi < vPS->getNumEntries(); ++pi) {
            const Vector3f& pos = vPS->get3f(pi, posAAT);
            const Vector3f pos_shift = pos + offset;
            posVec.push_back(transformVector3f(tBack, pos_shift));
            colorVec.push_back(vPS->get3f(pi, colorAAT));
            normalVec.push_back(vPS->get3f(pi, normalAAT));
        }
        delete vPS;
    }
    UICPC* synPC = new UICPC;
    VertexDescriptor* vd = createDefaultVertexDescriptor(true,true,true,false,false);
    const unsigned num_p = posVec.size();
    synPC->clearAndSetup(vd, num_p);
    PointSet& synPS = *synPC->getPointSet();
    const AAT& posAAT = synPS.getAAT("position");
    const AAT& colorAAT = synPS.getAAT("color");
    const AAT& normalAAT = synPS.getAAT("normal");
    for (unsigned pi = 0; pi < num_p; ++pi) {
        synPS.set3f(pi, posAAT, posVec[pi]);
        synPS.set3f(pi, colorAAT, colorVec[pi]);
        synPS.set3f(pi, normalAAT, normalVec[pi]);
    }
    return synPC;
}

void SSliceStack::DrawWithDR(const Vector3f& color)
{
    const Vector3f transhift = makeVector3f(5, 0, 0);
    for (int vi = 0; vi < spaceVolume_; ++vi) {
        const BoundingBox3f& bb = GetCellBBoxFromIndex(
            SN2Index(vi, spaceMeasure_, spaceLower_)
            );
        {
            BoundingBox3f bbShift(
                bb.lowerCorner + transhift,
                bb.upperCorner + transhift
                );
            debugRenderer->addBoundingBox(bbShift,
                makeVector3f(0.9f, 0.9f, 0.0f), 2);
        }
        {
            Vector3f lc = transformVector3f(tZback_, bb.lowerCorner);
            Vector3f uc = transformVector3f(tZback_, bb.upperCorner);
            for (unsigned d = 0; d < 3; ++d) {
                if (lc[d] > uc[d]) std::swap(lc[d], uc[d]);
            }
            BoundingBox3f bbShift(lc, uc);
            debugRenderer->addBoundingBox(bbShift,
                color, 2);
        }
    }
    //for (stackMapT::iterator it = spaceVoxels_.begin();
    //    it != spaceVoxels_.end(); ++it) {
    //        const BoundingBox3f& bb = GetCellBBoxFromIndex(
    //            it->first);
    //        debugRenderer->addBoundingBox(bb, color, 2);
    //}
    debugRenderer->addLine(
        //makeVector3f(0, 0, 0),
        //makeVector3f(1, 1, 1) + makeVector3f(0, 0, norm(gendir_)),
        makeVector3f(1, 1, 1),
        makeVector3f(1, 1, 1) + gendir_,
        makeVector3f(1, 0, 0), makeVector3f(0, 0, 1),
        5);
    debugRenderer->addLine(
        NULL_VECTOR3F, normalize(gendir_),
        makeVector3f(1, 0, 0), makeVector3f(0, 0, 1),
        5);
    //debugRenderer->addFineArrow(
    //    NULL_VECTOR3F, normalize(gendir_),
    //    makeVector3f(0.8f, 0.8f, 0), 0.02f, true);
    {
        const unsigned& num_verts = zDataPS_->getNumEntries();
        const AAT& posAAT = zDataPS_->getAAT("position");
        for (unsigned vi = 0; vi < num_verts; ++vi) {
            const Vector3f& pos = zDataPS_->get3f(vi, posAAT);
            const Vector3f posUns = transformVector3f(tZback_, pos);
            debugRenderer->addPoint(posUns,
                makeVector3f(0.9f, 0.9f, 0.9f));
            debugRenderer->addPoint(pos + transhift,
                makeVector3f(0.9f, 0.9f, 0.0f));
        }
    }
    //for (int ii = 0; ii < shiftVolume_; ++ii) {
    //    for (int jj = ii; jj < shiftVolume_; ++jj) {
    //        //for (int jj = 13; jj < 14; ++jj) {
    //        const Vector3i shift_i = SN2Index(ii, shiftMeasure_, shiftLower_);
    //        const Vector3i shift_j = SN2Index(jj, shiftMeasure_, shiftLower_);
    //        const Vector3i shift_diff = shift_j - shift_i;
    //        const int diff_abs = abs(shift_diff[0]) + abs(shift_diff[1]) + abs(shift_diff[2]);
    //        if (diff_abs > 1) continue;
    //        const Vector3f pos = (
    //            GetCellCenterFromIndex(shift_i) +
    //            GetCellCenterFromIndex(shift_j)
    //            ) / 2.f;
    //        std::ostringstream ss;
    //        ss << pairDist_[ii * shiftVolume_ + jj];
    //        const Vector3f color = (diff_abs == 0) ?
    //            makeVector3f(1, 1, 1) :
    //            makeVector3f(1, 1, 0);
    //        debugRenderer->addText(pos,
    //            ss.str(),
    //            16, "Verdana",
    //            color);
    //    }
    //}
}

PointVoxelization::PointVoxelization()
{
    voxize_ = 0.1;
}
PointVoxelization::~PointVoxelization()
{
}

void PointVoxelization::BindPointCloud(UICPC* dataPC, UICPC* featPC)
{
    dataPC_ = dataPC;
    featPC_ = featPC;
}

void PointVoxelization::PushGendir(const Vector3f& dir)
{
    gendirs_.push_back(dir);
    SSliceStack::Ptr stack(new SSliceStack(dataPC_, featPC_, dir, voxize_));
    stacks_.push_back(stack);
}

void PointVoxelization::DrawVoxelsZ(void)
{
    for (unsigned si = 0; si < stacks_.size(); ++si) {
        debugRenderer->beginRenderJob_OneFrame("voxel_grid_", si);
        stacks_[si]->DrawWithDR();
        debugRenderer->endRenderJob();
    }
}

void PointVoxelization::Synthesis(const unsigned& di, Scene* scene)
{
    if (di >= gendirs_.size()) return;
    stacks_[di]->Synthesis();
    UICPC* synPC = stacks_[di]->CollectSynPoints();
    synPC->setMaterialIndex(1);
    std::ostringstream ss;
    ss << "synPC" << di;
    addOrReplacePointCloud(scene, ss.str(), synPC);
}

//void VoxelJointGrid::compareJointVolume(
//    const Vector3i& lowerM, const Vector3i& upperM,
//    const Vector3i& shift,
//    const PointSet& fPS, const AAT& fAAT,
//    float& pairdist
//    )
//{
//    bool debug_show = false;
//    //if (shift[0] == 0 && shift[1] == 0 && shift[2] == 1) debug_show = true;
//    const bool ann_compute = true;
//    const unsigned ann_num = 1000;
//    const Vector3f shiftRel = makeVector3f(
//        (float)shift[0] * voxize_,
//        (float)shift[1] * voxize_,
//        (float)shift[2] * voxize_
//        );
//    //omp_set_num_threads(8);
//    #pragma omp parallel for schedule(dynamic,1)
//    for (int x = lowerM[0]; x <= upperM[0]; ++x) {
//        for (int y = lowerM[1]; y <= upperM[1]; ++y) {
//            for (int z = lowerM[2]; z <= upperM[2]; ++z) {
//                const Vector3i mid = makeVector3i(x, y, z);
//                for (unsigned d = 0; d < 3; ++d) {
//                    Vector3i m0 = mid;
//                    m0[d] -= 1;
//                    if (!SGridDesc::isValid(m0, lowerM, upperM)) continue;
//                    Vector3i m1 = mid - shift;
//                    const int sn0 = spaceDesc_.Index2SN(m0);
//                    const int sn1 = spaceDesc_.Index2SN(m1);
//                    if (voxels_[sn0]->countEmpty || voxels_[sn1]->countEmpty) {
//                        pairdist += 10 * distAmplifier(2 * voxize_, voxize_);
//                        continue;
//                    }
//                    SJoint::Ptr sj0 = voxels_[sn0]->upper[d];
//                    SJoint::Ptr sj1 = voxels_[sn1]->lower[d];
//                    const size_t nump0 = sj0->size();
//                    const size_t nump1 = sj1->size();
//                    if (0 == nump0 || 0 == nump1) {
//                        pairdist += 10 * distAmplifier(2 * voxize_, voxize_);
//                        continue;
//                    }
//                    if (!ann_compute) {
//                        pairdist += distPairMax_ * 0.1;
//                        continue;
//                    }
//                    if (debug_show) debugRenderer->beginRenderJob_OneFrame("joint_", DR_FRAME++);
//                    {
//                        std::deque<unsigned> indice(nump0);
//                        std::copy(sj0->indice.begin(), sj0->indice.end(), indice.begin());
//                        //std::deque<unsigned>& indice = sj0->indice;
//                        unsigned step = 1;
//                        if (nump0 > ann_num) {
//                            std::random_shuffle(indice.begin(), indice.end());
//                            step = floor((float)nump0 / (float)ann_num);
//                        }
//                        PointSetANNQueryPtr knn = sj1->psKNN;
//                        for (unsigned vi = 0; vi < nump0; vi += step) {
//                            const Vector3f pos = fPS.get3f(indice[vi], fAAT) - shiftRel;
//                            float distSqr;
//                            knn->getNearestPointIndexAndSqrDistance(pos, distSqr);
//                            distSqr = sqrt(distSqr);
//                            pairdist += (distAmplifier(distSqr, voxize_) / (float)nump0);
//                            if (debug_show)
//                            {
//                                debugRenderer->addPoint(
//                                    pos + makeVector3f(1, 0, 0),
//                                    makeVector3f(1, 1, 0)
//                                    );
//                                debugRenderer->addPoint(
//                                    transformVector3f(tZback_, pos),
//                                    makeVector3f(1, 1, 0)
//                                    );
//                            }
//                        }
//                    }
//                    {
//                        std::deque<unsigned> indice(nump1);
//                        std::copy(sj1->indice.begin(), sj1->indice.end(), indice.begin());
//                        //const std::deque<unsigned> indice = sj1->indice;
//                        unsigned step = 1;
//                        if (nump1 > ann_num) {
//                            std::random_shuffle(indice.begin(), indice.end());
//                            step = floor((float)nump1 / (float)ann_num);
//                        }
//                        PointSetANNQueryPtr knn = sj0->psKNN;
//                        for (unsigned vi = 0; vi < nump1; ++vi) {
//                            const Vector3f pos = fPS.get3f(indice[vi], fAAT) + shiftRel;
//                            float distSqr;
//                            knn->getNearestPointIndexAndSqrDistance(pos, distSqr);
//                            distSqr = sqrt(distSqr);
//                            pairdist += (distAmplifier(distSqr, voxize_) / (float)nump1);
//                            if (debug_show)
//                            {
//                                debugRenderer->addPoint(
//                                    pos + makeVector3f(1, 0, 0),
//                                    makeVector3f(0, 1, 1)
//                                    );
//                                debugRenderer->addPoint(
//                                    transformVector3f(tZback_, pos),
//                                    makeVector3f(0, 1, 1)
//                                    );
//                            }
//                        }
//                    }
//                    if (debug_show)
//                    {
//                        {
//                            const BoundingBox3f& bb = GetCellBBoxFromIndex(m0);
//                            {
//                                Vector3f lc = bb.lowerCorner + makeVector3f(1, 0, 0);
//                                Vector3f uc = bb.upperCorner + makeVector3f(1, 0, 0);
//                                BoundingBox3f bbTb(lc, uc);
//                                debugRenderer->addBoundingBox(bbTb, makeVector3f(1, 1, 0), 2);
//                            }
//                            {
//                                Vector3f lc = transformVector3f(tZback_, bb.lowerCorner);
//                                Vector3f uc = transformVector3f(tZback_, bb.upperCorner);
//                                for (unsigned d = 0; d < 3; ++d) {
//                                    if (lc[d] > uc[d]) std::swap(lc[d], uc[d]);
//                                }
//                                BoundingBox3f bbTb(lc, uc);
//                                debugRenderer->addBoundingBox(bbTb, makeVector3f(1, 1, 0), 2);
//                            }
//                        }
//                        {
//                            const BoundingBox3f& bb = GetCellBBoxFromIndex(m1);
//                            {
//                                Vector3f lc = bb.lowerCorner + makeVector3f(1, 0, 0);
//                                Vector3f uc = bb.upperCorner + makeVector3f(1, 0, 0);
//                                BoundingBox3f bbTb(lc, uc);
//                                debugRenderer->addBoundingBox(bbTb, makeVector3f(1, 1, 0), 2);
//                            }
//                            {
//                                Vector3f lc = transformVector3f(tZback_, bb.lowerCorner);
//                                Vector3f uc = transformVector3f(tZback_, bb.upperCorner);
//                                for (unsigned d = 0; d < 3; ++d) {
//                                    if (lc[d] > uc[d]) std::swap(lc[d], uc[d]);
//                                }
//                                BoundingBox3f bbTb(lc, uc);
//                                debugRenderer->addBoundingBox(bbTb, makeVector3f(1, 1, 0), 2);
//                            }
//                        }
//                        std::ostringstream ss;
//                        ss << pairdist;
//                        debugRenderer->addText(
//                            NULL_VECTOR3F,
//                            ss.str(),
//                            20, "Verdana",
//                            makeVector3f(0, 1, 0));
//                    }
//                    if (debug_show) debugRenderer->endRenderJob();
//                }
//            }
//        }
//    }
//}
//
//void VoxelJointGrid::ComputePairDist(
//    const PointSet& fPS, const std::string& aat, const unsigned& fDim)
//{
//    const AAT& fAAT = fPS.getAAT(aat);
//
//    bool debug_show = false;
//    DR_FRAME = 0;
//    //const Vector3i& msShift = shiftDesc_.ms;
//    //const Vector3i& lcShift = shiftDesc_.lc;
//    //const Vector3i& ucShift = shiftDesc_.uc;
//    const int volShift = shiftDesc_.vol;
//    const float distNonInter = distAmplifier(2 * voxize_, voxize_) * 6 * 10;
//    const int radiusInter = volShift; // only calculate within this radius
//    pairDist_.resize(volShift, 0);
//    progressWindow->pushStep(true, "Pairwise distance", (float)volShift);
//    for (int ii = 0; ii < volShift; ++ii) {
//        progressWindow->progress(100 * (double)ii / (double)volShift);
//        const Vector3i shift_i = shiftDesc_.SN2Index(ii);
//        const Vector3i spacelc = spaceDesc_.lc;
//        const Vector3i spaceuc = spaceDesc_.uc;
//        const Vector3i shiftlc = spaceDesc_.lc + shift_i;
//        const Vector3i shiftuc = spaceDesc_.uc + shift_i;
//        const bool ds_condition = true
//            && (shift_i[0] == 0 && shift_i[1] == 0)
//            ;
//        if (debug_show && ds_condition) debugRenderer->beginRenderJob_OneFrame("pair_dist_", DR_FRAME++);
//        Vector3i lowerM, upperM;
//        if (!SGridDesc::intersectVoxels(spacelc, spaceuc, shiftlc, shiftuc,
//            lowerM, upperM))
//        {
//            error("VoxelJointGrid::ComputePairDist - wrong shift");
//            continue;
//        }
//        Vector3i measureM = upperM - lowerM + makeVector3i(1, 1, 1);
//        for (unsigned d = 0; d < 3; ++d) {
//            if (2 * radiusInter + 1 >= measureM[d]) continue;
//            const int lb = lowerM[d], ub = upperM[d];
//            if (measureM[d] % 2 == 0) {
//                int mi = (int)floor((float)(lb + ub) / 2);
//                lowerM[d] = mi - radiusInter + 1;
//                upperM[d] = mi + radiusInter;
//            } else {
//                int mi = (int)floor((float)(lb + ub + 1) / 2);
//                lowerM[d] = mi - radiusInter;
//                upperM[d] = mi + radiusInter;
//            }
//        }
//        measureM = upperM - lowerM + makeVector3i(1, 1, 1);
//        int volumeM = measureM[0] * measureM[1] * measureM[2];
//        // 1. non-intersecting voxel has max cost
//        pairDist_[ii] = distNonInter * 2 * (spaceDesc_.vol - volumeM);
//        // 2. intersecting voxel has pair distance
//        compareJointVolume(
//            lowerM, upperM,
//            shift_i,
//            fPS, fAAT,
//            pairDist_[ii]);
//        if (debug_show && ds_condition)
//        {
//            std::ostringstream ss;
//            ss << shift_i[0] << ", " << shift_i[1] << ", " << shift_i[2]
//            << ": " << pairDist_[ii];
//            debugRenderer->addText(
//                makeVector3f(0, 0, 2),
//                ss.str(),
//                20, "Verdana",
//                makeVector3f(0, 1, 0));
//        }
//        if (debug_show && ds_condition)
//        {
//            for (int vi = 0; vi < spaceDesc_.vol; ++vi) {
//                {
//                    const BoundingBox3f& bb = GetCellBBoxFromIndex(
//                        spaceDesc_.SN2Index(vi)
//                        );
//                    Vector3f lc = transformVector3f(tZback_, bb.lowerCorner);
//                    Vector3f uc = transformVector3f(tZback_, bb.upperCorner);
//                    for (unsigned d = 0; d < 3; ++d) {
//                        if (lc[d] > uc[d]) std::swap(lc[d], uc[d]);
//                    }
//                    BoundingBox3f bbTb(lc, uc);
//                    debugRenderer->addBoundingBox(bbTb, makeVector3f(1, 1, 0), 2);
//                }
//                {
//                    const BoundingBox3f& bb = GetCellBBoxFromIndex(
//                        spaceDesc_.SN2Index(vi) + shift_i
//                        );
//                    Vector3f lc = transformVector3f(tZback_, bb.lowerCorner);
//                    Vector3f uc = transformVector3f(tZback_, bb.upperCorner);
//                    for (unsigned d = 0; d < 3; ++d) {
//                        if (lc[d] > uc[d]) std::swap(lc[d], uc[d]);
//                    }
//                    BoundingBox3f bbTb(lc, uc);
//                    debugRenderer->addBoundingBox(bbTb, makeVector3f(0, 1, 1), 2);
//                }
//                {
//                    const Vector3i vidx = spaceDesc_.SN2Index(vi);
//                    SVoxelJoint::Ptr voxel = voxels_[spaceDesc_.Index2SN(vidx)];
//                    const std::deque<unsigned>& indice = voxel->dataInd;
//                    for (unsigned pi = 0; pi < indice.size(); ++pi) {
//                        const Vector3f& pos = fPS.get3f(indice[pi], fAAT);
//                        Vector3f posShift = makeVector3f(
//                            pos[0] + shift_i[0] * voxize_,
//                            pos[1] + shift_i[1] * voxize_,
//                            pos[2] + shift_i[2] * voxize_
//                            );
//                        debugRenderer->addPoint(
//                            transformVector3f(tZback_, posShift),
//                            makeVector3f(0, 0.9f, 0.9f)
//                            );
//                    }
//                }
//            }
//        }
//        if (debug_show && ds_condition) debugRenderer->endRenderJob();
//    }
//    progressWindow->popStep();
//}

    //// binary
    //const bool debug_show_2 = false;
    //if (debug_show_2) DR_FRAME = 0;
    //std::vector<float> pairCost(labelDesc_.vol * labelDesc_.vol, 0);
    //for (int ii = 0; ii < labelDesc_.vol; ++ii) {
    //    for (int jj = 0; jj < labelDesc_.vol; ++jj) {
    //        const Vector3i label_i = labelDesc_.SN2Index(ii);
    //        const Vector3i label_j = labelDesc_.SN2Index(jj);
    //        const Vector3i shift = label_j - label_i;
    //        //if (shift[0] == 0 && shift[1] == 0 && shift[1] == 0) {
    //        //    pairCost[ii * labelDesc_.vol + jj] = 0;
    //        //} else
    //            if (!shiftDesc_.isValid(shift)) {
    //            pairCost[ii * labelDesc_.vol + jj] = distInvalid;
    //        } else {
    //            pairCost[ii * labelDesc_.vol + jj] =
    //                pairDist_[shiftDesc_.Index2SN(shift)];
    //        }
    //        //if (shift[0] != 0 || shift[1] != 0) continue;
    //        if (debug_show_2) debugRenderer->beginRenderJob_OneFrame("binary_cost_", DR_FRAME++);
    //        if (debug_show_2)
    //        {
    //            {
    //                std::ostringstream ss;
    //                ss << shift[0] << ", " << shift[1] << ", " << shift[2]
    //                << ": " << pairCost[ii * labelDesc_.vol + jj];
    //                debugRenderer->addText(
    //                    makeVector3f(0, 0, 2),
    //                    ss.str(),
    //                    20, "Verdana",
    //                    makeVector3f(0, 1, 0));
    //            }
    //            for (int vi = 0; vi < spaceDesc_.vol; ++vi) {
    //                {
    //                    const BoundingBox3f& bb = GetCellBBoxFromIndex(
    //                        spaceDesc_.SN2Index(vi)
    //                        );
    //                    Vector3f lc = transformVector3f(tZback_, bb.lowerCorner);
    //                    Vector3f uc = transformVector3f(tZback_, bb.upperCorner);
    //                    for (unsigned d = 0; d < 3; ++d) {
    //                        if (lc[d] > uc[d]) std::swap(lc[d], uc[d]);
    //                    }
    //                    BoundingBox3f bbTb(lc, uc);
    //                    debugRenderer->addBoundingBox(bbTb, makeVector3f(1, 1, 0), 2);
    //                }
    //                {
    //                    const BoundingBox3f& bb = GetCellBBoxFromIndex(
    //                        spaceDesc_.SN2Index(vi) + shift
    //                        );
    //                    Vector3f lc = transformVector3f(tZback_, bb.lowerCorner);
    //                    Vector3f uc = transformVector3f(tZback_, bb.upperCorner);
    //                    for (unsigned d = 0; d < 3; ++d) {
    //                        if (lc[d] > uc[d]) std::swap(lc[d], uc[d]);
    //                    }
    //                    BoundingBox3f bbTb(lc, uc);
    //                    debugRenderer->addBoundingBox(bbTb, makeVector3f(0, 1, 1), 2);
    //                }
    //                {
    //                    //const Vector3i vidx = spaceDesc_.SN2Index(vi);
    //                    //SVoxelJoint::Ptr voxel = voxels_[spaceDesc_.Index2SN(vidx)];
    //                    //const std::deque<unsigned>& indice = voxel->dataInd;
    //                    //const AAT& posAAT = zDataPS_->getAAT("position");
    //                    //for (unsigned pi = 0; pi < indice.size(); ++pi) {
    //                    //    const Vector3f& pos = zDataPS_->get3f(indice[pi], posAAT);
    //                    //    Vector3f posShift = makeVector3f(
    //                    //        pos[0] + shift[0] * voxize_,
    //                    //        pos[1] + shift[1] * voxize_,
    //                    //        pos[2] + shift[2] * voxize_
    //                    //        );
    //                    //    debugRenderer->addPoint(
    //                    //        transformVector3f(tZback_, posShift),
    //                    //        makeVector3f(0, 0.9f, 0.9f)
    //                    //        );
    //                    //}
    //                }
    //            }
    //        }
    //        if (debug_show_2) debugRenderer->endRenderJob();
    //    }
    //}
    ////// smooth check
    ////DR_FRAME = 0;
    ////for (int vi = 0; vi < synthDesc_.vol; ++vi) {
    ////    for (int vj = 0; vj < synthDesc_.vol; ++vj) {
    ////        const Vector3i synth_i = synthDesc_.SN2Index(vi);
    ////        const Vector3i synth_j = synthDesc_.SN2Index(vj);
    ////        const Vector3i offset = synth_j - synth_i;
    ////        const int diff_abs = abs(offset[0]) + abs(offset[1]) + abs(offset[2]);
    ////        if (diff_abs != 1) continue;
    ////        for (int ii = 0; ii < labelDesc_.vol; ++ii) {
    ////            for (int jj = 0; jj < labelDesc_.vol; ++jj) {
    ////                const float& e00 = pairCost[ii * labelDesc_.vol + ii];
    ////                const float& e01 = pairCost[ii * labelDesc_.vol + jj];
    ////                const float& e10 = pairCost[jj * labelDesc_.vol + ii];
    ////                const float& e11 = pairCost[jj * labelDesc_.vol + jj];
    ////                if (e00+e11 <= e01+e10) continue;
    ////                debugRenderer->beginRenderJob_OneFrame("smooth_check_", DR_FRAME++);
    ////                {
    ////                    const BoundingBox3f& bb = GetCellBBoxFromIndex(
    ////                        spaceDesc_.SN2Index(vi)
    ////                        );
    ////                    Vector3f lc = transformVector3f(tZback_, bb.lowerCorner);
    ////                    Vector3f uc = transformVector3f(tZback_, bb.upperCorner);
    ////                    for (unsigned d = 0; d < 3; ++d) {
    ////                        if (lc[d] > uc[d]) std::swap(lc[d], uc[d]);
    ////                    }
    ////                    BoundingBox3f bbTb(lc, uc);
    ////                    debugRenderer->addBoundingBox(bbTb, makeVector3f(1, 1, 0), 2);
    ////                    {
    ////                        std::ostringstream ss;
    ////                        ss << e00;
    ////                        debugRenderer->addText(
    ////                            transformVector3f(tZback_,
    ////                            GetCellCenterFromIndex(spaceDesc_.SN2Index(vi))),
    ////                            ss.str(),
    ////                            24, "Verdana",
    ////                            makeVector3f(1, 1, 0));
    ////                    }
    ////                    {
    ////                        std::ostringstream ss;
    ////                        ss << e01;
    ////                        debugRenderer->addText(
    ////                            transformVector3f(tZback_,
    ////                            GetCellCenterFromIndex(spaceDesc_.SN2Index(vi))) + makeVector3f(0, 0, 1),
    ////                            ss.str(),
    ////                            24, "Verdana",
    ////                            makeVector3f(1, 0, 0));
    ////                    }
    ////                }
    ////                {
    ////                    const BoundingBox3f& bb = GetCellBBoxFromIndex(
    ////                        spaceDesc_.SN2Index(vj)
    ////                        );
    ////                    Vector3f lc = transformVector3f(tZback_, bb.lowerCorner);
    ////                    Vector3f uc = transformVector3f(tZback_, bb.upperCorner);
    ////                    for (unsigned d = 0; d < 3; ++d) {
    ////                        if (lc[d] > uc[d]) std::swap(lc[d], uc[d]);
    ////                    }
    ////                    BoundingBox3f bbTb(lc, uc);
    ////                    debugRenderer->addBoundingBox(bbTb, makeVector3f(0, 1, 1), 2);
    ////                    {
    ////                        std::ostringstream ss;
    ////                        ss << e11;
    ////                        debugRenderer->addText(
    ////                            transformVector3f(tZback_,
    ////                            GetCellCenterFromIndex(spaceDesc_.SN2Index(vj))) + makeVector3f(0, 0, -1),
    ////                            ss.str(),
    ////                            24, "Verdana",
    ////                            makeVector3f(0, 1, 1));
    ////                    }
    ////                    {
    ////                        std::ostringstream ss;
    ////                        ss << e10;
    ////                        debugRenderer->addText(
    ////                            transformVector3f(tZback_,
    ////                            GetCellCenterFromIndex(spaceDesc_.SN2Index(vj))),
    ////                            ss.str(),
    ////                            24, "Verdana",
    ////                            makeVector3f(0, 0, 1));
    ////                    }
    ////                }
    ////            }
    ////        }
    ////        debugRenderer->endRenderJob();
    ////    }
    ////}

