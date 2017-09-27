#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "KNNPaternGentor.h"
#include "Util/ColorSchemer.hpp"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

KNNPaternGentor::KNNPaternGentor(const unsigned& num) : num_search(num)
{
}

void KNNPaternGentor::Generate(PointSet* PS)
{
    knnVec_.clear();
    const size_t num_points = PS->getNumEntries();
    const AAT posAAT = PS->getAAT("position");
    PointSetANNQuery knn(PS, num_search);
    int32* indices;
    card32 numEnt;
    for (size_t pi = 0; pi < num_points; ++pi) {
        const Vector3f pos = PS->get3f(pi, posAAT);
        knn.setTargetPoint(pos);
        indices = knn.getAllIndices(numEnt); // self included
        /**
        std::deque<unsigned> vec(indices, indices + numEnt);
        /**/
        std::deque<unsigned> vec;
        for (unsigned ii = 0; ii < numEnt; ++ii) {
            if (pi == indices[ii]) continue;
            vec.push_back(indices[ii]);
        }
        //**/
        knnVec_.push_back(vec);
    }
}

template< typename FT = float >
FT floor0(const FT& val)
{
    if ((FT)0.0 > val) return ceil(val);
    else return floor(val);
}

void KNNPaternGentor::Discretize(PointSet* PS, const float& discale)
{
    discale_ = discale;
    offVec_.clear();
    const size_t num_points = PS->getNumEntries();
    const AAT posAAT = PS->getAAT("position");
    for (size_t pi = 0; pi < num_points; ++pi) {
        std::deque< key_type > off;
        const Vector3f pos = PS->get3f(pi, posAAT);
        const std::deque<unsigned>& knn = knnVec_[pi];
        for (const unsigned& nindex : knn) {
            const Vector3f pos_n = PS->get3f(nindex, posAAT);
            key_type arr = {
                floor(pos_n[0] / discale) - floor(pos[0] / discale),
                floor(pos_n[1] / discale) - floor(pos[1] / discale),
                floor(pos_n[2] / discale) - floor(pos[2] / discale)
            };
            off.push_back(arr);
        }
        offVec_.push_back(off);
    }
}

BoundingBox3f KNNPaternGentor::makeBoundingBox(const key_type& key)
{
    Vector3f cen;
    for (unsigned ii = 0; ii < 3; ++ii) cen[ii] = static_cast<float>(key[ii]) + .5f;
    cen *= discale_;
    BoundingBox3f bb;
    bb.lowerCorner = bb.upperCorner = cen;
    bb.addBorder(discale_ / 2.0f);
    return bb;
}

void KNNPaternGentor::ShowPattern(PointSet* PS)
{
    const size_t num_points = PS->getNumEntries();
    const AAT posAAT = PS->getAAT("position");
    bool disOK = (offVec_.size() == knnVec_.size()) ? true : false;
    for (size_t pi = 0; pi < num_points; ++pi) {
        debugRenderer->beginRenderJob_OneFrame("show_pattern_", DR_FRAME++);
        const Vector3f pos = PS->get3f(pi, posAAT);
        const key_type key = {
            floor(pos[0] / discale_),
            floor(pos[1] / discale_),
            floor(pos[2] / discale_)
        };
        debugRenderer->addPoint(pos, StandardColors<100>::color_red);
        const std::deque<unsigned>& knn = knnVec_[pi];
        for (size_t nn = 0; nn < knn.size(); ++nn) {
            const Vector3f pos_n = PS->get3f(knn[nn], posAAT);
            debugRenderer->addLine(
                pos, pos_n,
                StandardColors<100>::color_red,
                StandardColors<100>::color_cyan, 2.0f);

            if (!disOK) continue;
            const key_type disp = offVec_[pi][nn];
            const key_type key_n = {
                key[0] + disp[0],
                key[1] + disp[1],
                key[2] + disp[2]
            };
            debugRenderer->addBoundingBox(makeBoundingBox(key_n),
                StandardColors<100>::color_cyan, 1);
        }
        debugRenderer->endRenderJob();
    }
}
