//---------------------------------------------------------------------------
#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "VoxelJointGrid.h"
#include "Util/gco-v3.0/GCoptimization.h"
#include <opengm/opengm.hxx>
#include <opengm/graphicalmodel/graphicalmodel.hxx>
#include <opengm/inference/messagepassing/messagepassing.hxx>
#include <omp.h>
//---------------------------------------------------------------------------
#include "ProgressWindow.h"
#include "AnnSearch.h"
#include "SGRelativeTimeAnimationNode.h"
//---------------------------------------------------------------------------
using namespace X4;
//---------------------------------------------------------------------------
namespace {
    int DR_FRAME = 0;
}

namespace {
    SGridDesc spaceDesc_;
    SGridDesc shiftDesc_;
    SGridDesc synthDesc_;
    SGridDesc labelDesc_;
    std::deque< float > pairDist_;
    float distPairMax_ = -1000;
    float voxize_ = -0.1;
    float jointRatio_ = -0.01;
    float distInvalid = -1;
}

int Vec2Direc(const Vector3i& vec)
{
    if (vec[0] == -1 && vec[1] == 0 && vec[2] == 0) return 0;
    if (vec[0] == 0 && vec[1] == -1 && vec[2] == 0) return 1;
    if (vec[0] == 0 && vec[1] == 0 && vec[2] == -1) return 2;
    if (vec[0] == 1 && vec[1] == 0 && vec[2] == 0) return 3;
    if (vec[0] == 0 && vec[1] == 1 && vec[2] == 0) return 4;
    if (vec[0] == 0 && vec[1] == 0 && vec[2] == 1) return 5;
    return -1;
}
Vector3i Direc2Vec(const int& direc)
{
    switch(direc) {
    case 0: return makeVector3i(-1, 0, 0); break;
    case 1: return makeVector3i(0, -1, 0); break;
    case 2: return makeVector3i(0, 0, -1); break;
    case 3: return makeVector3i(1, 0, 0); break;
    case 4: return makeVector3i(0, 1, 0); break;
    case 5: return makeVector3i(0, 0, 1); break;
    default: return makeVector3i(0, 0, 0);
    }
}

int SGridDesc::Index2SN(const Vector3i& index) const
{
    Vector3i indexRel = index - lc;
    return
        (indexRel[0] + 
        (indexRel[1] * ms[0]) + 
        (indexRel[2] * ms[0] * ms[1]));
}
Vector3i SGridDesc::SN2Index(const int& sn) const
{
    Vector3i result;
    result[0] = sn % ms[0];
    result[1] = (sn / ms[0]) % ms[1];
    result[2] = sn / (ms[0] * ms[1]);
    return result + lc;
}
bool SGridDesc::isValid(const Vector3i& index) const
{
    return SGridDesc::isValid(index, lc, uc);
}
bool SGridDesc::onBorder(const Vector3i& index) const
{
    return SGridDesc::onBorder(index, lc, uc);
}
bool SGridDesc::onBorder(const Vector3i& index,
                         const Vector3i& lower, const Vector3i& upper)
{
    return
        ((index[0] == lower[0]) || (index[0] == upper[0])) ||
        ((index[1] == lower[1]) || (index[1] == upper[1])) ||
        ((index[2] == lower[2]) || (index[2] == upper[2]));
}
bool SGridDesc::isValid(const Vector3i& index,
                        const Vector3i& lower, const Vector3i& upper)
{
    return
        ((index[0] >= lower[0]) && (index[0] <= upper[0])) &&
        ((index[1] >= lower[1]) && (index[1] <= upper[1])) &&
        ((index[2] >= lower[2]) && (index[2] <= upper[2]));
}
int SGridDesc::intersectVoxels(
    const Vector3i& lowerL, const Vector3i& upperL,
    const Vector3i& lowerR, const Vector3i& upperR,
    Vector3i& lowerM, Vector3i& upperM)
{
    for (unsigned d = 0; d < 3; ++d) {
        lowerM[d] = std::max(lowerL[d], lowerR[d]);
        upperM[d] = std::min(upperL[d], upperR[d]);
        if (lowerM[d] > upperM[d]) return false;
    }
    return true;
}

VoxelJointGrid::VoxelJointGrid(
    UICPC* dataPC,
    const Vector3f& dir, const Vector3f& delta, const float& voxize)
{
    jointRatio_ = 0.01;
    voxize_ = voxize;
    gendir_ = dir;
    const PointSet& dataPS = *dataPC->getPointSet();

    {
        const unsigned& num_verts = dataPS.getNumEntries();
        const AAT& posAAT = dataPS.getAAT("position");
        PCA3f pointPCA;
        for (unsigned vi = 1; vi < num_verts; ++vi)
            pointPCA.addPoint(dataPS.get3f(vi, posAAT));
        Vector3f eigenValues;
        Matrix3f frame;
        Vector3f center;
        pointPCA.analyze(eigenValues, frame, center);
        {
            Matrix3f t3t = IDENTITY3F;
            Vector3f zp = normalize(dir);
            std::deque<float> ip3(3);
            for (unsigned d = 0; d < 3; ++d) ip3[d] = abs(zp * frame[d]);
            auto maxi = std::distance(ip3.begin(), std::max_element(ip3.begin(), ip3.end()));
            t3t[1] = (0 == maxi) ? frame[1] : frame[0];
            t3t[0] = t3t[1].crossProduct(zp);
            t3t[1] = zp.crossProduct(t3t[0]);
            t3t[2] = dir;

            Matrix4f t4s = IDENTITY4F;
            t4s[2][2] = t4s[2][2] * voxize;

            tParaZ_ = t4s * invertMatrix(expand3To4(t3t));
            tZback_ = invertMatrix(tParaZ_);

            //debugRenderer->beginRenderJob_OneFrame("frame_trans_", 0);
            //for (unsigned d = 0; d < 3; ++d) {
            //    Vector3f color = NULL_VECTOR3F;
            //    color[d] = 1;
            //    debugRenderer->addLine(
            //        NULL_VECTOR3F,
            //        t3t[d],
            //        makeVector3f(1, 1, 1), color,
            //        3);
            //    const Vector3f shift = makeVector3f(2, 0, 0);
            //    debugRenderer->addLine(
            //        NULL_VECTOR3F + shift,
            //        transformVector3f(tParaZ_, t3t[d]) + shift,
            //        makeVector3f(1, 1, 1), color,
            //        3);
            //}
            //debugRenderer->endRenderJob();
        }
    }

    //// this align generator direction to Z
    //tParaZ_ = IDENTITY4F;
    //{
    //    Matrix3f t3s = IDENTITY3F;
    //    t3s[2][2] = t3s[2][2] * norm(dir) / voxize;
    //    Matrix3f t3t = IDENTITY3F;
    //    Vector3f zp = normalize(dir);
    //    unsigned d = 0;
    //    for (; d < 3; ++d) if (0.5f > abs(zp[d])) break;
    //    t3t[0] = t3t[d].crossProduct(zp);
    //    t3t[1] = zp.crossProduct(t3t[0]);
    //    t3t[2] = zp;
    //    tZback_ = expand3To4(t3t * t3s);
    //    tParaZ_ = invertMatrix(tZback_);
    //}

    // transform points using alignment matrix
    {
        const unsigned& num_verts = dataPS.getNumEntries();
        zDataPS_ = new PointSet;
        VertexDescriptor vd;
        vd.pushAttrib(dataPS.getDescr()->getAttribute("position"));
        zDataPS_->clearAndSetup(1, num_verts, &vd);
        const AAT& posAAT = dataPS.getAAT("position");
        for (unsigned vi = 0; vi < num_verts; ++vi) {
            const Vector3f& pos = dataPS.get3f(vi, posAAT) + delta;
            zDataPS_->set3f(vi, posAAT, transformVector3f(tParaZ_, pos));
        }
    }

    // compute bounding box
    {
        const unsigned& num_verts = dataPS.getNumEntries();
        const AAT& posAAT = dataPS.getAAT("position");
        BoundingBox3f bb(zDataPS_->get3f(0, posAAT));
        for (unsigned ii = 1; ii < num_verts; ++ii)
            bb.addPoint(zDataPS_->get3f(ii, posAAT));
        bb.addBorder(bb.getMaxSideLength() * 0.01f);
        spaceDesc_.lc = GetCellIndexFromPoint(bb.lowerCorner);
        spaceDesc_.uc = GetCellIndexFromPoint(bb.upperCorner);
        spaceDesc_.vol = 1;
        for (unsigned d = 0; d < 3; ++d) {
            spaceDesc_.ms[d] = spaceDesc_.uc[d] - spaceDesc_.lc[d] + 1;
            spaceDesc_.vol *= spaceDesc_.ms[d];
        }
    }

    {
        synthDesc_ = spaceDesc_;
    }

    {
        // at least 1 coinident voxels
        shiftDesc_.lc = - spaceDesc_.ms + makeVector3i(1, 1, 1);
        shiftDesc_.uc = spaceDesc_.ms - makeVector3i(1, 1, 1);
        shiftDesc_.ms = shiftDesc_.uc - shiftDesc_.lc + makeVector3i(1, 1, 1);
        shiftDesc_.vol = shiftDesc_.ms[0] * shiftDesc_.ms[1] * shiftDesc_.ms[2];
    }

    // initialize storage
    voxels_.resize(spaceDesc_.vol);
    {
        for (int vi = 0; vi < spaceDesc_.vol; ++vi) {
            voxels_[vi] = SVoxelJoint::Ptr(new SVoxelJoint);
        }
    }

    Voxelize(zDataPS_);
    //ComputePairDist(*zDataPS_, "position", 3);
    ComputeVoxelPairDist(*zDataPS_, "position", 3);
}
VoxelJointGrid::~VoxelJointGrid()
{
    delete zDataPS_;
}

Vector3i VoxelJointGrid::GetCellIndexFromPoint(const Vector3f& pos)
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
Vector3f VoxelJointGrid::GetCellCenterFromIndex(const Vector3i& idx)
{
    return makeVector3f(idx[0], idx[1], idx[2]) * voxize_;
}
BoundingBox3f VoxelJointGrid::GetCellBBoxFromIndex(const Vector3i& idx)
{
    Vector3f ccen = GetCellCenterFromIndex(idx);
    BoundingBox3f bb;
    bb.lowerCorner = bb.upperCorner = ccen;
    bb.addBorder(voxize_ / 2.0f);
    return bb;
}

void VoxelJointGrid::Voxelize(PointSet* dataPS)
{
    // twice joint size
    distPairMax_ = 2 * voxize_* jointRatio_;
    {
        const unsigned& num_verts = dataPS->getNumEntries();
        const AAT& posAAT = dataPS->getAAT("position");
        for (unsigned vi = 0; vi < num_verts; ++vi) {
            const Vector3f& pos = dataPS->get3f(vi, posAAT);
            const Vector3i& idx = GetCellIndexFromPoint(pos);
            SVoxelJoint::Ptr voxel = voxels_[spaceDesc_.Index2SN(idx)];
            voxel->dataInd.push_back(vi);
            const BoundingBox3f& bb = GetCellBBoxFromIndex(idx);
            const Vector3f& lc = bb.lowerCorner, uc = bb.upperCorner;
            Vector3u cid;
            for (unsigned d = 0; d < 3; ++d) {
                const float& len = bb.getSideLength(d);
                //const float sl = len * 0.5;
                const float sl = len * jointRatio_;
                if (pos[d] > lc[d] && pos[d] < lc[d] + sl) {
                    voxel->lower[d]->indice.push_back(vi);
                } else if (pos[d] < uc[d] && pos[d] > uc[d] - sl) {
                    voxel->upper[d]->indice.push_back(vi);
                }
                cid[d] = floor(10 * (pos[d] - lc[d]) / voxize_);
            }
            voxel->count[cid[0] * 100 + cid[1] * 10 + cid[2]] ++;
        }
    }
    for (int x = spaceDesc_.lc[0]; x <= spaceDesc_.uc[0]; ++x) {
        for (int y = spaceDesc_.lc[1]; y <= spaceDesc_.uc[1]; ++y) {
            for (int z = spaceDesc_.lc[2]; z <= spaceDesc_.uc[2]; ++z) {
                SVoxelJoint::Ptr voxel = voxels_[spaceDesc_.Index2SN(makeVector3i(x, y, z))];
                for (unsigned d = 0; d < 3; ++d) {
                    voxel->lower[d]->psKNN = PointSetANNQueryPtr(
                        //new PointSetANNQuery(dataPS->subset(voxel->dataInd), 1)
                        new PointSetANNQuery(dataPS->subset(voxel->lower[d]->indice), 1)
                        );
                    voxel->upper[d]->psKNN = PointSetANNQueryPtr(
                        //new PointSetANNQuery(dataPS->subset(voxel->dataInd), 1)
                        new PointSetANNQuery(dataPS->subset(voxel->upper[d]->indice), 1)
                        );
                }
                // lower than average is counted as empty
                const size_t szCem = floor(0.01 * (float)voxel->dataInd.size());
                unsigned cem = 0;
                for (unsigned ii = 0; ii < 1000; ++ii) {
                    if (voxel->count[ii] < szCem) ++cem;
                }
                voxel->countEmpty = (cem < 100) ? true : false;
            }
        }
    }
}

inline float distAmplifier(const float& value, const float& sigma)
{
    return value * exp(2 * value / sigma);
    //return value;
}

void VoxelJointGrid::compareJoint(
    SJoint::Ptr sj0, SJoint::Ptr sj1,
    const Vector3i& spaceIvec, const Vector3i& spaceJvec,
    const Vector3f& sideShift,
    const PointSet& fPS, const AAT& fAAT,
    const float& distEmpty,
    float& pairdist
    )
{
    bool debug_show = false;
    //if (shift[0] == 0 && shift[1] == 0 && shift[2] == 1) debug_show = true;
    const unsigned ann_num = 1000; // maximal query count for each voxel
    const size_t nump0 = sj0->size();
    const size_t nump1 = sj1->size();
    if (0 == nump0 && 0 == nump1) {
        pairdist = 0.1 * distEmpty;
        return;
    } else if ((0 == nump0 && 0 != nump1) || (0 != nump0 && 0 == nump1)) {
        pairdist = distEmpty;
        return;
    }
    const int msum = spaceDesc_.ms[0] + spaceDesc_.ms[1] + spaceDesc_.ms[2];
    const Vector3i spaceDiffInt = spaceJvec - spaceIvec;
    const int delta =
        abs(spaceDiffInt[0]) +
        abs(spaceDiffInt[1]) +
        abs(spaceDiffInt[2]);
    const float dmult = exp(5 * (float)delta / (float)msum);
    const Vector3f spaceDiff = GetCellCenterFromIndex(spaceDiffInt);
    //const Vector3f spaceDiff = makeVector3f(
    //    (float)spaceDiffInt[0] * voxize_,
    //    (float)spaceDiffInt[1] * voxize_,
    //    (float)spaceDiffInt[2] * voxize_
    //    );
    const Vector3f spaceShift = spaceDiff + sideShift;
    if (debug_show) debugRenderer->beginRenderJob_OneFrame("joint_", DR_FRAME++);
    {
        std::deque<unsigned> indice(nump0);
        std::copy(sj0->indice.begin(), sj0->indice.end(), indice.begin());
        //std::deque<unsigned>& indice = sj0->indice;
        unsigned step = 1;
        if (nump0 > ann_num) {
            // make sure to take a randomly selected subset
            std::random_shuffle(indice.begin(), indice.end());
            step = floor((float)nump0 / (float)ann_num);
        }
        PointSetANNQueryPtr knn = sj1->psKNN;
        for (unsigned vi = 0; vi < nump0; vi += step) {
            const Vector3f pos = fPS.get3f(indice[vi], fAAT) + spaceShift;
            float distSqr;
            knn->getNearestPointIndexAndSqrDistance(pos, distSqr);
            distSqr = sqrt(distSqr);
            pairdist += (distAmplifier(distSqr, voxize_) / (float)nump0);
            if (debug_show)
            {
                debugRenderer->addPoint(
                    pos + makeVector3f(5, 0, 0),
                    makeVector3f(1, 1, 0)
                    );
                debugRenderer->addPoint(
                    transformVector3f(tZback_, pos),
                    makeVector3f(1, 1, 0)
                    );
            }
        }
    }
    {
        std::deque<unsigned> indice(nump1);
        std::copy(sj1->indice.begin(), sj1->indice.end(), indice.begin());
        //const std::deque<unsigned> indice = sj1->indice;
        unsigned step = 1;
        if (nump1 > ann_num) {
            std::random_shuffle(indice.begin(), indice.end());
            step = floor((float)nump1 / (float)ann_num);
        }
        PointSetANNQueryPtr knn = sj0->psKNN;
        for (unsigned vi = 0; vi < nump1; ++vi) {
            const Vector3f pos = fPS.get3f(indice[vi], fAAT);
            float distSqr;
            knn->getNearestPointIndexAndSqrDistance(pos, distSqr);
            distSqr = sqrt(distSqr);
            pairdist += (distAmplifier(distSqr, voxize_) / (float)nump1);
            if (debug_show)
            {
                debugRenderer->addPoint(
                    pos + makeVector3f(5, 0, 0),
                    makeVector3f(0, 1, 1)
                    );
                debugRenderer->addPoint(
                    transformVector3f(tZback_, pos),
                    makeVector3f(0, 1, 1)
                    );
            }
        }
    }
    pairdist = pairdist * dmult; // heavier penalty on remote connections
    if (debug_show)
    {
        {
            const BoundingBox3f& bb = GetCellBBoxFromIndex(spaceIvec);
            debugRenderer->addBoundingBox(
                tZback_, bb,
                makeVector3f(1, 1, 0), 2);
        }
        {
            const BoundingBox3f& bb = GetCellBBoxFromIndex(spaceJvec);
            debugRenderer->addBoundingBox(
                tZback_, bb,
                makeVector3f(0, 1, 1), 2);
        }
        const Vector3f spaceIpos = GetCellCenterFromIndex(spaceIvec);
        //const Vector3f spaceIpos = makeVector3f(
        //    (float)spaceIvec[0] * voxize_,
        //    (float)spaceIvec[1] * voxize_,
        //    (float)spaceIvec[2] * voxize_
        //    );
        const Vector3f spaceIposShift = spaceIpos + spaceShift;
        debugRenderer->addLine(
            transformVector3f(tZback_, spaceIpos),
            transformVector3f(tZback_, spaceIposShift),
            makeVector3f(1, 0, 0), makeVector3f(0, 0, 1),
            5);
        std::ostringstream ss;
        ss << pairdist;
        debugRenderer->addText(
            NULL_VECTOR3F,
            ss.str(),
            20, "Verdana",
            makeVector3f(0, 1, 0));
    }
    if (debug_show) debugRenderer->endRenderJob();
}
void VoxelJointGrid::ComputeVoxelPairDist(
    const PointSet& fPS, const std::string& aat, const unsigned& fDim)
{
    const AAT& fAAT = fPS.getAAT(aat);

    DR_FRAME = 0;
    const bool ann_compute = true;
    const int volSpace = spaceDesc_.vol;
    const float distEmpty = distAmplifier(2 * voxize_, voxize_);
    const int radiusInter = volSpace; // only calculate within this radius
    pairDist_.resize(volSpace * volSpace * 6, 0);
    progressWindow->pushStep(true, "Pairwise distance", (float)volSpace);
    for (int ii = 0; ii < volSpace; ++ii) {
        progressWindow->progress(100 * (double)ii / (double)volSpace);
        const Vector3i spaceIvec = spaceDesc_.SN2Index(ii);
        for (int jj = 0; jj < volSpace; ++jj) {
            const Vector3i spaceJvec = spaceDesc_.SN2Index(jj);
            //if (voxels_[ii]->countEmpty || voxels_[jj]->countEmpty) {
            //    for (int d = 0; d < 6; ++d) {
            //        pairDist_[ii * volSpace * 6 + jj * 6 + d] = distEmpty;
            //    }
            //    continue;
            //}
            if (!ann_compute) {
                for (int d = 0; d < 6; ++d) {
                    pairDist_[ii * volSpace * 6 + jj * 6 + d] = distPairMax_ * 0.1;
                }
                continue;
            }
            {
                SJoint::Ptr sj0 = voxels_[ii]->lower[0];
                SJoint::Ptr sj1 = voxels_[jj]->upper[0];
                const Vector3f sideShift = makeVector3f(1, 0, 0) * voxize_;
                //const Vector3f sideShift = makeVector3f(
                //    1 * voxize_, 0 * voxize_, 0 * voxize_
                //    );
                float pairdist = 0;
                compareJoint(
                    sj0, sj1,
                    spaceIvec, spaceJvec,
                    sideShift,
                    fPS, fAAT,
                    distEmpty,
                    pairdist
                    );
                pairDist_[ii * volSpace * 6 + jj * 6 + 0] = pairdist;
            }
            {
                SJoint::Ptr sj0 = voxels_[ii]->lower[1];
                SJoint::Ptr sj1 = voxels_[jj]->upper[1];
                const Vector3f sideShift = makeVector3f(0, 1, 0) * voxize_;
                //const Vector3f sideShift = makeVector3f(
                //    0 * voxize_, 1 * voxize_, 0 * voxize_
                //    );
                float pairdist = 0;
                compareJoint(
                    sj0, sj1,
                    spaceIvec, spaceJvec,
                    sideShift,
                    fPS, fAAT,
                    distEmpty,
                    pairdist
                    );
                pairDist_[ii * volSpace * 6 + jj * 6 + 1] = pairdist;
            }
            {
                SJoint::Ptr sj0 = voxels_[ii]->lower[2];
                SJoint::Ptr sj1 = voxels_[jj]->upper[2];
                const Vector3f sideShift = makeVector3f(0, 0, 1) * voxize_;
                //const Vector3f sideShift = makeVector3f(
                //    0 * voxize_, 0 * voxize_, 1 * voxize_
                //    );
                float pairdist = 0;
                compareJoint(
                    sj0, sj1,
                    spaceIvec, spaceJvec,
                    sideShift,
                    fPS, fAAT,
                    distEmpty,
                    pairdist
                    );
                pairDist_[ii * volSpace * 6 + jj * 6 + 2] = pairdist;
            }
            {
                SJoint::Ptr sj0 = voxels_[ii]->upper[0];
                SJoint::Ptr sj1 = voxels_[jj]->lower[0];
                const Vector3f sideShift = makeVector3f(-1, 0, 0) * voxize_;
                //const Vector3f sideShift = makeVector3f(
                //    -1 * voxize_, 0 * voxize_, 0 * voxize_
                //    );
                float pairdist = 0;
                compareJoint(
                    sj0, sj1,
                    spaceIvec, spaceJvec,
                    sideShift,
                    fPS, fAAT,
                    distEmpty,
                    pairdist
                    );
                pairDist_[ii * volSpace * 6 + jj * 6 + 3] = pairdist;
            }
            {
                SJoint::Ptr sj0 = voxels_[ii]->upper[1];
                SJoint::Ptr sj1 = voxels_[jj]->lower[1];
                const Vector3f sideShift = makeVector3f(0, -1, 0) * voxize_;
                //const Vector3f sideShift = makeVector3f(
                //    0 * voxize_, -1 * voxize_, 0 * voxize_
                //    );
                float pairdist = 0;
                compareJoint(
                    sj0, sj1,
                    spaceIvec, spaceJvec,
                    sideShift,
                    fPS, fAAT,
                    distEmpty,
                    pairdist
                    );
                pairDist_[ii * volSpace * 6 + jj * 6 + 4] = pairdist;
            }
            {
                SJoint::Ptr sj0 = voxels_[ii]->upper[2];
                SJoint::Ptr sj1 = voxels_[jj]->lower[2];
                const Vector3f sideShift = makeVector3f(0, 0, -1) * voxize_;
                //const Vector3f sideShift = makeVector3f(
                //    0 * voxize_, 0 * voxize_, -1 * voxize_
                //    );
                float pairdist = 0;
                compareJoint(
                    sj0, sj1,
                    spaceIvec, spaceJvec,
                    sideShift,
                    fPS, fAAT,
                    distEmpty,
                    pairdist
                    );
                pairDist_[ii * volSpace * 6 + jj * 6 + 5] = pairdist;
            }
        }
    }
    progressWindow->popStep();
}

namespace {
    float smoothFn(int siteI, int siteJ, int labelI, int labelJ)
    {
        const Vector3i synthIvec = synthDesc_.SN2Index(siteI);
        const Vector3i synthJvec = synthDesc_.SN2Index(siteJ);
        const Vector3i offset = synthJvec - synthIvec;
        if (1 != abs(offset[0]) + abs(offset[1]) + abs(offset[2]))
            return distInvalid;
        const Vector3i labelIvec = labelDesc_.SN2Index(labelI);
        const Vector3i labelJvec = labelDesc_.SN2Index(labelJ);
        const Vector3i spaceIvec = synthIvec - labelIvec;
        const Vector3i spaceJvec = synthJvec - labelJvec;
        if (!spaceDesc_.isValid(spaceIvec) || !spaceDesc_.isValid(spaceJvec)) {
            return distInvalid;
        }
        const int spaceI = spaceDesc_.Index2SN(spaceIvec);
        const int spaceJ = spaceDesc_.Index2SN(spaceJvec);
        const float& dist = pairDist_[spaceI * spaceDesc_.vol * 6 + spaceJ * 6 + Vec2Direc(offset)];
        if (labelI == labelJ) return dist;

        {
            const Vector3i sidii = synthIvec - labelIvec;
            const Vector3i sidji = synthJvec - labelIvec;
            if (!spaceDesc_.isValid(sidii) ||
                !spaceDesc_.isValid(sidji)) return dist + distInvalid;
            const int sii = spaceDesc_.Index2SN(sidii);
            const int sji = spaceDesc_.Index2SN(sidji);
            const float dii = pairDist_[sii * spaceDesc_.vol * 6 + sji * 6 + Vec2Direc(offset)];
            if (dii > dist) return dist + dii;
        }
    {
        const Vector3i sidij = synthIvec - labelJvec;
        const Vector3i sidjj = synthJvec - labelJvec;
        if (!spaceDesc_.isValid(sidij) ||
            !spaceDesc_.isValid(sidjj)) return dist + distInvalid;
        const int sij = spaceDesc_.Index2SN(sidij);
        const int sjj = spaceDesc_.Index2SN(sidjj);
        const float djj = pairDist_[sij * spaceDesc_.vol * 6 + sjj * 6 + Vec2Direc(offset)];
        if (djj > dist) return dist + djj;
    }
    return dist;
    }
}

void VoxelJointGrid::Synthesis(
    int lowerB, int upperB)
{
    // set up new measure
    {
        if (lowerB >= upperB) { lowerB = upperB - 1; upperB += 1; }
        const int& lc2 = spaceDesc_.lc[2], uc2 = spaceDesc_.uc[2];
        lowerB = lowerB - lc2;
        upperB = uc2 - upperB;
        if (0 > lowerB) lowerB = 0;
        if (0 > upperB) upperB = 0;
    }
    {
        //int syns = (0 < std::rand() % 2 - 0.5f) ? 1 : -1;
        int syns = 1;
        synthDesc_.ms = synthDesc_.ms + makeVector3i(0, 0, 1);
        synthDesc_.lc[2] = (syns < 0) ? (synthDesc_.lc[2] - 1) : synthDesc_.lc[2];
        synthDesc_.uc[2] = (syns > 0) ? (synthDesc_.uc[2] + 1) : synthDesc_.uc[2];
        synthDesc_.vol += synthDesc_.ms[0] * synthDesc_.ms[1];
        synthLabels_.resize(synthDesc_.vol, 0);
    }
    {
        // one side is fixed
        labelDesc_.lc = synthDesc_.lc - spaceDesc_.lc
            - makeVector3i(spaceDesc_.ms[0], spaceDesc_.ms[1], 0)
            + makeVector3i(1, 1, 0);
        // at least 2 overlap, to eliminate non-metric caused by invalid move
        labelDesc_.uc = synthDesc_.uc - spaceDesc_.uc
            + spaceDesc_.ms
            - makeVector3i(1, 1, 1);
        labelDesc_.ms = labelDesc_.uc - labelDesc_.lc + makeVector3i(1, 1, 1);
        labelDesc_.vol = labelDesc_.ms[0] * labelDesc_.ms[1] * labelDesc_.ms[2];
    }

    // unary: keep border and prevent adding empty voxel
    std::vector<float> dataCost(synthDesc_.vol * labelDesc_.vol, 0);
    distInvalid = distAmplifier(2 * voxize_, voxize_) * 6 * labelDesc_.vol * 1000000;
    const float distBorder = distAmplifier(2 * voxize_, voxize_) * 6 * labelDesc_.vol * 1000000;
    const float distEmpty = distAmplifier(2 * voxize_, voxize_) * 6 * labelDesc_.vol * 1;
    for (int vi = 0; vi < synthDesc_.vol; ++vi) {
        for (int li = 0; li < labelDesc_.vol; ++li) {
            const Vector3i vidx = synthDesc_.SN2Index(vi);
            const Vector3i offset = labelDesc_.SN2Index(li);
            // fixed margin
            {
                const int d = 2;
                const int ld = vidx[d] - synthDesc_.lc[d];
                const int ud = synthDesc_.uc[d] - vidx[d];
                const int lo = synthDesc_.lc[d] - spaceDesc_.lc[d];
                const int uo = synthDesc_.uc[d] - spaceDesc_.uc[d];
                Vector3i lom = NULL_VECTOR3I, uom = NULL_VECTOR3I;
                lom[d] = lo; uom[d] = uo;
                if (ld <= lowerB) {
                    if (offset == lom)
                        dataCost[vi * labelDesc_.vol + li] = 0;
                    else
                        dataCost[vi * labelDesc_.vol + li] = distInvalid;
                    continue;
                } else if (ud <= upperB) {
                    if (offset == uom)
                        dataCost[vi * labelDesc_.vol + li] = 0;
                    else
                        dataCost[vi * labelDesc_.vol + li] = distInvalid;
                    continue;
                }
            }
            const Vector3i lidx = vidx - offset; // back to space
            if (!spaceDesc_.isValid(lidx)) {
                dataCost[vi * labelDesc_.vol + li] = distInvalid;
            } else {
                // border on border
                const bool onSpaceBorder = synthDesc_.onBorder(vidx);
                const bool onSynBorder = spaceDesc_.onBorder(lidx);
                if ((onSpaceBorder && !onSynBorder) || (!onSpaceBorder && onSynBorder)) {
                    dataCost[vi * labelDesc_.vol + li] = distBorder;
                } else {
                    dataCost[vi * labelDesc_.vol + li] = 0;
                }
                //// prevent adding empty voxel
                //if (voxels_[spaceDesc_.Index2SN(lidx)]->countEmpty)
                //    dataCost[vi * labelDesc_.vol + li] += distEmpty;
            }
        }
    }

    //{ // smooth check
    //    DR_FRAME = 0;
    //    debugOutput << "\ndistInvalid: " << distInvalid << "\n";
    //    for (int vi = 0; vi < synthDesc_.vol; ++vi) {
    //        for (int vj = 0; vj < synthDesc_.vol; ++vj) {
    //            const Vector3i synth_i = synthDesc_.SN2Index(vi);
    //            const Vector3i synth_j = synthDesc_.SN2Index(vj);
    //            const Vector3i offset = synth_j - synth_i;
    //            const int diff_abs = abs(offset[0]) + abs(offset[1]) + abs(offset[2]);
    //            if (diff_abs != 1) continue;
    //            for (int ii = 0; ii < labelDesc_.vol; ++ii) {
    //                for (int jj = 0; jj < labelDesc_.vol; ++jj) {
    //                    const float& e00 = smoothFn(vi, vj, ii, ii);
    //                    const float& e01 = smoothFn(vi, vj, ii, jj);
    //                    const float& e10 = smoothFn(vi, vj, jj, ii);
    //                    const float& e11 = smoothFn(vi, vj, jj, jj);
    //                    if (e00+e11 <= e01+e10) continue;
    //                    debugOutput << DR_FRAME << ": " << e00 << ", " << e11 << ", " << e01 << ", " << e10 << "\n";
    //                    //debugRenderer->beginRenderJob_OneFrame("smooth_check_", DR_FRAME++);
    //                    //{
    //                    //    const BoundingBox3f& bb = GetCellBBoxFromIndex(
    //                    //        spaceDesc_.SN2Index(vi)
//                    //        );
//                    //    debugRenderer->addBoundingBox(
//                    //        tZback_, bb,
//                    //        makeVector3f(1, 1, 0), 2);
//                    //    {
//                    //        std::ostringstream ss;
//                    //        ss << str(boost::format(
//                    //            "(%1%, %2%) - (%3%, %4%): %5%")
//                    //            % vi % vj % ii % ii % e00);
//                    //        debugRenderer->addText(
//                    //            transformVector3f(tZback_,
//                    //            GetCellCenterFromIndex(spaceDesc_.SN2Index(vi))),
//                    //            ss.str(),
//                    //            24, "Verdana",
//                    //            makeVector3f(1, 1, 0));
//                    //    }
//                    //    {
//                    //        std::ostringstream ss;
//                    //        ss << str(boost::format(
//                    //            "(%1%, %2%) - (%3%, %4%): %5%")
//                    //            % vi % vj % ii % jj % e01);
//                    //        debugRenderer->addText(
//                    //            transformVector3f(tZback_,
//                    //            GetCellCenterFromIndex(spaceDesc_.SN2Index(vi))) + makeVector3f(1, 1, 1),
//                    //            ss.str(),
//                    //            24, "Verdana",
//                    //            makeVector3f(1, 0, 0));
//                    //    }
//                    //}
//                    //{
//                    //    const BoundingBox3f& bb = GetCellBBoxFromIndex(
//                    //        spaceDesc_.SN2Index(vj)
//                    //        );
//                    //    debugRenderer->addBoundingBox(
//                    //        tZback_, bb,
//                    //        makeVector3f(0, 1, 1), 2);
//                    //    {
//                    //        std::ostringstream ss;
//                    //        ss << str(boost::format(
//                    //            "(%1%, %2%) - (%3%, %4%): %5%")
//                    //            % vi % vj % jj % jj % e11);
//                    //        debugRenderer->addText(
//                    //            transformVector3f(tZback_,
//                    //            GetCellCenterFromIndex(spaceDesc_.SN2Index(vj))),
//                    //            ss.str(),
//                    //            24, "Verdana",
//                    //            makeVector3f(0, 1, 1));
//                    //    }
//                    //    {
//                    //        std::ostringstream ss;
//                    //        ss << str(boost::format(
//                    //            "(%1%, %2%) - (%3%, %4%): %5%")
//                    //            % vi % vj % jj % ii % e10);
//                    //        debugRenderer->addText(
//                    //            transformVector3f(tZback_,
//                    //            GetCellCenterFromIndex(spaceDesc_.SN2Index(vj))) + makeVector3f(-1, -1, -1),
//                    //            ss.str(),
//                    //            24, "Verdana",
//                    //            makeVector3f(0, 0, 1));
//                    //    }
//                    //}
//                    //debugRenderer->endRenderJob();
//                }
//            }
//        }
//    }
//}

{
    GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph(
        synthDesc_.vol, labelDesc_.vol);
    gc->setDataCost(&dataCost[0]);
    //gc->setSmoothCost(&pairCost[0]);
    gc->setSmoothCost(&smoothFn);
    for (int vi = 0; vi < synthDesc_.vol; ++vi) {
        for (int vj = vi + 1; vj < synthDesc_.vol; ++vj) {
            const Vector3i synth_i = synthDesc_.SN2Index(vi);
            const Vector3i synth_j = synthDesc_.SN2Index(vj);
            const Vector3i offset = synth_j - synth_i;
            const int diff_abs = abs(offset[0]) + abs(offset[1]) + abs(offset[2]);
            if (diff_abs == 1) gc->setNeighbors(vi, vj);
        }
    }
    try{
        debugOutput << "\nBefore optimization energy is " << gc->compute_energy() << "\n";
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
    for (int vi = 0; vi < synthDesc_.vol; ++vi)
        synthLabels_[vi] = gc->whatLabel(vi);
    delete gc;
}
}

void VoxelJointGrid::SynthesisGM(int lowerB, int upperB)
{
	// set up new measure
	{
		if (lowerB >= upperB) { lowerB = upperB - 1; upperB += 1; }
		const int& lc2 = spaceDesc_.lc[2], uc2 = spaceDesc_.uc[2];
		lowerB = lowerB - lc2;
		upperB = uc2 - upperB;
		if (0 > lowerB) lowerB = 0;
		if (0 > upperB) upperB = 0;
	}
	{
		//int syns = (0 < std::rand() % 2 - 0.5f) ? 1 : -1;
		int syns = 1;
		synthDesc_.ms = synthDesc_.ms + makeVector3i(0, 0, 1);
		synthDesc_.lc[2] = (syns < 0) ? (synthDesc_.lc[2] - 1) : synthDesc_.lc[2];
		synthDesc_.uc[2] = (syns > 0) ? (synthDesc_.uc[2] + 1) : synthDesc_.uc[2];
		synthDesc_.vol += synthDesc_.ms[0] * synthDesc_.ms[1];
		synthLabels_.resize(synthDesc_.vol, 0);
	}
	{
		// one side is fixed
		labelDesc_.lc = synthDesc_.lc - spaceDesc_.lc
			- makeVector3i(spaceDesc_.ms[0], spaceDesc_.ms[1], 0)
			+ makeVector3i(1, 1, 0);
		// at least 2 overlap, to eliminate non-metric caused by invalid move
		labelDesc_.uc = synthDesc_.uc - spaceDesc_.uc
			+ spaceDesc_.ms
			- makeVector3i(1, 1, 1);
		labelDesc_.ms = labelDesc_.uc - labelDesc_.lc + makeVector3i(1, 1, 1);
		labelDesc_.vol = labelDesc_.ms[0] * labelDesc_.ms[1] * labelDesc_.ms[2];
	}

	distInvalid = distAmplifier(2 * voxize_, voxize_) * 6 * labelDesc_.vol * 1000000;
	const float distBorder = distAmplifier(2 * voxize_, voxize_) * 6 * labelDesc_.vol * 1000000;
	const float distEmpty = distAmplifier(2 * voxize_, voxize_) * 6 * labelDesc_.vol * 1;

	// Build Model
	typedef double ValueType;
	typedef size_t IndexType;
	typedef size_t LabelType;
	typedef opengm::Adder OpType;
	typedef opengm::ExplicitFunction<ValueType, IndexType, LabelType> ExplicitFunction;
	typedef opengm::meta::TypeListGenerator<ExplicitFunction>::type FunctionTypeList;
	typedef opengm::DiscreteSpace<IndexType, LabelType> SpaceType;
	typedef opengm::GraphicalModel<ValueType, OpType, FunctionTypeList, SpaceType> Model;
	typedef Model::FunctionIdentifier FunctionIdentifier;
	LabelType numLabel = labelDesc_.vol;
	std::vector<LabelType> numberOfLabels(synthDesc_.vol, numLabel);
	Model gm(SpaceType(numberOfLabels.begin(), numberOfLabels.end()));

	// 1st-order
	for (IndexType var = 0; var < gm.numberOfVariables(); ++var) {
		const LabelType shape[] = { numLabel };
		ExplicitFunction f(shape, shape + 1);
		for (LabelType lab = 0; lab < numLabel; ++lab) {
			const Vector3i vidx = synthDesc_.SN2Index(var);
			const Vector3i offset = labelDesc_.SN2Index(lab);
			// fixed margin
			{
				const int d = 2;
				const int ld = vidx[d] - synthDesc_.lc[d];
				const int ud = synthDesc_.uc[d] - vidx[d];
				const int lo = synthDesc_.lc[d] - spaceDesc_.lc[d];
				const int uo = synthDesc_.uc[d] - spaceDesc_.uc[d];
				Vector3i lom = NULL_VECTOR3I, uom = NULL_VECTOR3I;
				lom[d] = lo; uom[d] = uo;
				if (ld <= lowerB) {
					if (offset == lom)
						f(lab) = 0;
					else
						f(lab) = distInvalid;
					continue;
				}
				else if (ud <= upperB) {
					if (offset == uom)
						f(lab) = 0;
					else
						f(lab) = distInvalid;
					continue;
				}
			}
			const Vector3i lidx = vidx - offset; // back to space
			if (!spaceDesc_.isValid(lidx)) {
				f(lab) = distInvalid;
			}
			else {
				// border on border
				const bool onSpaceBorder = synthDesc_.onBorder(vidx);
				const bool onSynBorder = spaceDesc_.onBorder(lidx);
				if ((onSpaceBorder && !onSynBorder) || (!onSpaceBorder && onSynBorder)) {
					f(lab) = distBorder;
				}
				else {
					f(lab) = 0;
				}
				//// prevent adding empty voxel
				//if (voxels_[spaceDesc_.Index2SN(lidx)]->countEmpty)
				//    f(lab) += distEmpty;
			}
		}
		FunctionIdentifier fid = gm.addFunction(f);
		IndexType varIndex[] = { var };
		gm.addFactor(fid, varIndex, varIndex + 1);
	}

	// 2nd-order
	for (int vi = 0; vi < synthDesc_.vol; ++vi) {
		for (int vj = vi + 1; vj < synthDesc_.vol; ++vj) {
			const Vector3i synth_i = synthDesc_.SN2Index(vi);
			const Vector3i synth_j = synthDesc_.SN2Index(vj);
			const Vector3i offset = synth_j - synth_i;
			const int diff_abs = abs(offset[0]) + abs(offset[1]) + abs(offset[2]);
			if (diff_abs != 1) continue;

			const LabelType shape[] = { numLabel, numLabel };
			ExplicitFunction f(shape, shape + 2);
			for (LabelType l1 = 0; l1 < numLabel; ++l1) {
				for (LabelType l2 = 0; l2 < numLabel; ++l2) {
					f(l1, l2) = smoothFn(vi, vj, l1, l2);
				}
			}
			FunctionIdentifier fid = gm.addFunction(f);
			IndexType varIndex[] = { vi, vj };
			gm.addFactor(fid, varIndex, varIndex + 2);
		}
	}

	// Infer with LBP
	typedef opengm::BeliefPropagationUpdateRules<Model, opengm::Maximizer> UpdateRules;
	typedef opengm::MessagePassing<Model, opengm::Maximizer, UpdateRules, opengm::MaxDistance>  LBP;

	LBP::Parameter parameter(100, 0.01, 0.8); //maximal number of iterations=0, minimal message distance=0.01, damping=0.8
	LBP lbp(gm, parameter);

	lbp.infer();
	std::vector<LabelType> l_LBP;
	lbp.arg(l_LBP);
	for (size_t i = 0; i < l_LBP.size(); ++i) synthLabels_[i] = l_LBP[i];
}

UICPC* VoxelJointGrid::CollectSynPoints(void)
{
    Matrix4f tBack = invertMatrix(tParaZ_);
    std::deque<Vector3f> posVec;
    bool errorOnce = false;
    const bool debug_show = false;
    for (int vi = 0; vi < synthDesc_.vol; ++vi) {
        const int sn_l = synthLabels_[vi];
        const Vector3i shift = labelDesc_.SN2Index(sn_l);
        const Vector3i idx_t = synthDesc_.SN2Index(vi);
        const Vector3i idx_s = idx_t - shift;
        if (!spaceDesc_.isValid(idx_s)) {
            if (!errorOnce) {
                error("VoxelJointGrid::CollectSynPoints - wrong label");
                errorOnce = true;
            }
            debugRenderer->beginRenderJob_OneFrame("wrong_label_", DR_FRAME++);
            {
                const BoundingBox3f& bb = GetCellBBoxFromIndex(idx_t);
                debugRenderer->addBoundingBox(
                    tZback_, bb,
                    makeVector3f(1, 1, 0), 2);
            }
            {
                const BoundingBox3f& bb = GetCellBBoxFromIndex(idx_s);
                debugRenderer->addBoundingBox(
                    tZback_, bb,
                    makeVector3f(1, 0, 1), 2);
                {
                    std::ostringstream ss;
                    ss << idx_t[0] << ", " << idx_t[1] << ", " << idx_t[2];
                    debugRenderer->addText(
                        transformVector3f(tZback_, GetCellCenterFromIndex(idx_t)),
                        ss.str(),
                        24, "Verdana",
                        makeVector3f(1, 1, 0));
                }
                {
                    std::ostringstream ss;
                    ss << idx_s[0] << ", " << idx_s[1] << ", " << idx_s[2];
                    debugRenderer->addText(
                        transformVector3f(tZback_, GetCellCenterFromIndex(idx_s)),
                        ss.str(),
                        24, "Verdana",
                        makeVector3f(1, 0, 1));
                }
            }
            for (int vi = 0; vi < spaceDesc_.vol; ++vi) {
                const Vector3i vidx = spaceDesc_.SN2Index(vi);
                const BoundingBox3f& bb = GetCellBBoxFromIndex(vidx);
                Vector3f lc = transformVector3f(tZback_, bb.lowerCorner);
                Vector3f uc = transformVector3f(tZback_, bb.upperCorner);
                for (unsigned d = 0; d < 3; ++d) {
                    if (lc[d] > uc[d]) std::swap(lc[d], uc[d]);
                }
                BoundingBox3f bbTb(lc, uc);
                const float scale = bbTb.getMinSideLength() * 0.8;
                const Vector3f cen = GetCellCenterFromIndex(vidx);
                //const Vector3f cen = makeVector3f(
                //    vidx[0] * voxize_,
                //    vidx[1] * voxize_,
                //    vidx[2] * voxize_);
                debugRenderer->addCenteredBox(
                    transformVector3f(tZback_, cen),
                    IDENTITY3F,
                    makeVector3f(scale,scale,scale),
                    makeVector3f(1, 1, 0));
            }

            debugRenderer->endRenderJob();
            continue;
        }
        if (debug_show)
        {
            debugRenderer->beginRenderJob_OneFrame("label_", DR_FRAME++);
            {
                const BoundingBox3f& bb = GetCellBBoxFromIndex(idx_t);
                debugRenderer->addBoundingBox(
                    tZback_, bb,
                    makeVector3f(1, 1, 0), 2);
            }
            {
                const BoundingBox3f& bb = GetCellBBoxFromIndex(idx_s);
                debugRenderer->addBoundingBox(
                    tZback_, bb,
                    makeVector3f(0, 1, 1), 2);
                std::ostringstream ss;
                ss << idx_t[0] << ", " << idx_t[1] << ", " << idx_t[2] << " << "
                    << idx_s[0] << ", " << idx_s[1] << ", " << idx_s[2];
                debugRenderer->addText(
                    transformVector3f(tZback_, GetCellCenterFromIndex(idx_s)),
                    ss.str(),
                    24, "Verdana",
                    makeVector3f(0, 1, 0));
            }
            debugRenderer->endRenderJob();
        }
        const Vector3f offset = GetCellCenterFromIndex(shift);
        //const Vector3f offset = makeVector3f(
        //    (float)shift[0] * voxize_,
        //    (float)shift[1] * voxize_,
        //    (float)shift[2] * voxize_
        //    );
        SVoxelJoint::Ptr voxel = voxels_[spaceDesc_.Index2SN(idx_s)];
        PointSet* vPS = zDataPS_->subset(voxel->dataInd);
        const AAT& posAAT = vPS->getAAT("position");
        for (unsigned pi = 0; pi < vPS->getNumEntries(); ++pi) {
            const Vector3f& pos = vPS->get3f(pi, posAAT);
            const Vector3f pos_shift = pos + offset;
            posVec.push_back(transformVector3f(tBack, pos_shift));
        }
        delete vPS;
    }
    UICPC* synPC = new UICPC;
    //VertexDescriptor* vd = createDefaultVertexDescriptor(true,true,true,false,false);
    const unsigned num_p = posVec.size();
    synPC->clearAndSetup(zDataPS_->getDescr(), num_p);
    PointSet& synPS = *synPC->getPointSet();
    const AAT& posAAT = synPS.getAAT("position");
    for (unsigned pi = 0; pi < num_p; ++pi) {
        synPS.set3f(pi, posAAT, posVec[pi]);
    }
    return synPC;
}

void VoxelJointGrid::DrawWithDR(void)
{
    const Vector3f transhift = makeVector3f(5, 0, 0);
    for (int vi = 0; vi < spaceDesc_.vol; ++vi)
    {
        const Vector3i vidx = spaceDesc_.SN2Index(vi);
        const BoundingBox3f& bb = GetCellBBoxFromIndex(vidx);
        {
            BoundingBox3f bbShift(
                bb.lowerCorner + transhift,
                bb.upperCorner + transhift
                );
            debugRenderer->addBoundingBox(bbShift,
                makeVector3f(0.9f, 0.9f, 0.0f), 2);
            debugRenderer->addBoundingBox(
                tZback_, bb,
                makeVector3f(0.0f, 0.9f, 0.9f), 2);
        }
        int vm = -1;
        for (unsigned d = 0; d < 3; ++d) {
            unsigned d1 = (d + 1) % 3;
            unsigned d2 = (d + 2) % 3;
            if (vidx[d1] == spaceDesc_.lc[d1] &&
                vidx[d2] == spaceDesc_.lc[d2]) {
                    vm = d;
                    break;
            }
        }
        if (-1 != vm)
        {
            std::ostringstream ss;
            ss << vidx[vm];
            //ss << vi << ": " << vidx[vm];
            debugRenderer->addText(
                transformVector3f(tZback_, GetCellCenterFromIndex(vidx)),
                ss.str(),
                24, "Verdana",
                makeVector3f(0.9f, 0, 0.0f));
        }
    }
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
}

PointDistributor::PointDistributor()
{
    voxize_ = 0.1;
}
PointDistributor::~PointDistributor()
{
}

void PointDistributor::BindPointCloud(UICPC* dataPC)
{
    dataPC_ = dataPC;
}

void PointDistributor::PushGendir(const Vector3f& dir, const Vector3f& delta)
{
    gendirs_.push_back(dir);
    VoxelJointGrid::Ptr grid(new VoxelJointGrid(dataPC_, dir, delta, voxize_));
    grids_.push_back(grid);
}

void PointDistributor::DrawVoxelsZ(void)
{
    for (unsigned si = 0; si < grids_.size(); ++si) {
        debugRenderer->beginRenderJob_OneFrame("voxel_grid_", si);
        grids_[si]->DrawWithDR();
        debugRenderer->endRenderJob();
    }
}

void PointDistributor::Synthesis(
    const unsigned& di, Scene* scene,
    int lowerB, int upperB)
{
    if (di >= gendirs_.size()) return;
	//grids_[di]->Synthesis(lowerB, upperB);
	grids_[di]->SynthesisGM(lowerB, upperB);
	UICPC* synPC = grids_[di]->CollectSynPoints();
    synPC->setMaterialIndex(1);
    addOrReplacePointCloud(scene, str(boost::format("root/synPC_%1%") % di), synPC);

    SGRelativeTimeAnimationNode* groupList = dynamic_cast<SGRelativeTimeAnimationNode*>(
        getSceneGraphNodeByName(scene, "root/syn_steps"));
    if (!groupList)
    {
        groupList = new SGRelativeTimeAnimationNode;
        scene->getRootNode()->addChildNode(groupList);
        groupList->setName("syn_steps");
        groupList->setVisible(false);
    }
    addPointCloud(scene, dynamic_cast<PointCloud*>(synPC->copy()),
        "root/syn_steps/synPC");
}