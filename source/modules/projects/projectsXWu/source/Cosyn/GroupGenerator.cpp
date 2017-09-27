#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "GroupGenerator.h"
#include "Util/ColorSchemer.hpp"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

GroupGenerator::GroupGenerator(void)
{
}

Eigen::Matrix4f ExtractFrame(
    RepBox::Ptr box)
{
    Eigen::Matrix4f frame = Eigen::Matrix4f::Identity();
    {
        frame.col(0).head<3>() = EigenAdaptor::ToEigen(box->basis[1]);
        frame.col(0).head<3>().normalize();
        frame.col(1).head<3>() = EigenAdaptor::ToEigen(box->basis[2]);
        frame.col(1).head<3>().normalize();
        frame.col(2).head<3>() = EigenAdaptor::ToEigen(box->basis[0]);
        frame.col(2).head<3>().normalize();
        frame.col(3).head<3>() = EigenAdaptor::ToEigen(box->centroid);
    }
    return frame;
}

Eigen::Matrix4f GroupGenerator::ComputeTransformation(
    RepBox::Ptr source, RepBox::Ptr target
    )
{
    Eigen::Matrix4f sorcF = ExtractFrame(source);
    Eigen::Matrix4f destF = ExtractFrame(target);
    return destF * sorcF.inverse();
}

void GroupGenerator::Generate(std::deque<RepBox::Ptr>* boxList)
{
    const size_t numBox = boxList->size();
    if (3 > numBox) return;

    debugOutput << "generating ... \n";

    std::deque<Eigen::Vector3f> axis0(3);
    std::deque<float> dist0(3);
    std::deque<SynthLattice1D::Ptr> latVec;
    std::deque<bool> checkMark(numBox, false);
    for (size_t sorcI = 0; sorcI < numBox; ++sorcI) {
        if (checkMark[sorcI]) continue;
        RepBox::Ptr boxSorc = boxList->at(sorcI);
        const Eigen::Vector3f cenSorc = EigenAdaptor::ToEigen(boxSorc->centroid);
        { // build axis info
            for (unsigned d = 0; d < 3; ++d) {
                axis0[d] = EigenAdaptor::ToEigen(boxSorc->basis[d] * abs(boxSorc->coeffs[d]));
            }
            dist0[0] = max(abs(boxSorc->coeffs[1]), abs(boxSorc->coeffs[2])) * 0.1;
            dist0[1] = max(abs(boxSorc->coeffs[2]), abs(boxSorc->coeffs[0])) * 0.1;
            dist0[2] = max(abs(boxSorc->coeffs[0]), abs(boxSorc->coeffs[1])) * 0.1;
        }
        for (size_t destI = sorcI + 1; destI < numBox; ++destI) {
            if (checkMark[destI]) continue;

            RepBox::Ptr boxDest = boxList->at(destI);
            const Eigen::Vector3f cenDest = EigenAdaptor::ToEigen(boxDest->centroid);
            size_t startI = sorcI;
            Eigen::Vector3f dirUnit = Eigen::Vector3f::Identity();
            float lenVec = 1;
            std::deque<float> serPos;
            std::deque<size_t> serInd;
            {
                dirUnit = cenDest - cenSorc;
                lenVec = dirUnit.norm();
                dirUnit.normalize();

                Eigen::Vector3f dist01 = cenDest - cenSorc;
                for (size_t bi = 0; bi < numBox; ++bi) {
                    Eigen::Vector3f cen = EigenAdaptor::ToEigen(boxList->at(bi)->centroid);
                    const float dist = abs(((cen - cenSorc).cross(dist01)).norm()) / dist01.norm();
                    if (0.01 < dist) continue; // not on this line
                    serPos.push_back(cen.dot(dirUnit));
                    serInd.push_back(bi);
                }
            }
            //if (sqrt(numBox) > serPos.size()
            //    && 3 > serPos.size()
            //    ) continue; // generator too small
            std::deque<size_t> sortInd(serInd.size());
            for (size_t ii = 0; ii < serInd.size(); ++ii) sortInd[ii] = ii;
            std::sort(sortInd.begin(), sortInd.end(),
                [&serPos](size_t i0, size_t i1) { return serPos[i0] < serPos[i1]; }
            );

            for (size_t bi = 0, bj = 1; bj < serPos.size(); ++bi, ++bj) {
                const float dist = abs(serPos[sortInd[bj]] - serPos[sortInd[bi]]);
                const float mult = dist / lenVec;
                if (0.01 > mult || 0.99 < mult) continue; // same level or larger distance
                const float revMult = lenVec / dist;
                const float multAve = revMult / round(revMult);
                if (0.1 < abs(multAve - 1)) continue; // not integer multiplier
                lenVec = dist;
                startI = serInd[bi];
            }

            SynthLattice1D::Ptr lat(new SynthLattice1D);
            lat->base_ = EigenAdaptor::ToEigen(boxList->at(startI)->centroid);
            float sumLen = 0;
            lat->elem_.push_back(0);
            std::deque<size_t> elemVec;
            elemVec.push_back(startI);
            for (size_t bi = 0; bi < serPos.size(); ++bi) {
                if (serInd[bi] == startI) continue;
                Eigen::Vector3f cen = EigenAdaptor::ToEigen(boxList->at(serInd[bi])->centroid);
                Eigen::Vector3f vec = cen - lat->base_;
                const float mult = vec.norm() / lenVec;
                const float multAve = mult / round(mult);
                if (0.1 < abs(multAve - 1)) continue; // not integer multiplier
                int sign = (0 < vec.dot(dirUnit)) ? 1 : -1;
                lat->elem_.push_back(sign * round(mult));
                sumLen += multAve;
                elemVec.push_back(serInd[bi]);
            }
            if (sqrt(numBox) > elemVec.size()
                && 3 > elemVec.size()
                ) // continue; // generator too small
            { // constraint onto the axis directions
                bool bGood = false;
                for (unsigned d = 0; d < 3; ++d) {
                    const float dist = abs(((cenDest - cenSorc).cross(axis0[d])).norm()) /
                        axis0[d].norm();
                    if (dist0[d] > dist) {
                        bGood = true;
                        break;
                    }
                }
                if (!bGood) continue;
            }
            for (size_t ei = 0; ei < elemVec.size(); ++ei) checkMark[elemVec[ei]] = true;
            // take more robust average
            lenVec = lenVec * sumLen / (lat->elem_.size() - 1);
            Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
            trans.col(3).head<3>() = dirUnit * lenVec;
            lat->SetTrans(trans);
            std::sort(lat->elem_.begin(), lat->elem_.end());
            latVec.push_back(lat);

            break; // source checked
        }
    }

    latVec_.push_back(latVec);

    //debugRenderer->beginRenderJob_OneFrame("lattice_1d_", DR_FRAME++);
    //for (size_t li = 0; li < latVec.size(); ++li) {
    //    latVec[li]->DebugDraw(ColorSchemer::GetJetColor((float)li / latVec.size()), 0.02);
    //}
    //RepBoxLoader::DrawBoxes((*boxList), ColorSchemer::GetJetColor(0));
    //debugRenderer->endRenderJob();

    debugOutput << "done.\n";
}

void GroupGenerator::Generate(std::deque< std::deque<RepBox::Ptr> >* boxList)
{
    debugOutput << "\n";
    debugOutput << "generating transformation groups ... \n";

    size_t numList = (*boxList).size();
    debugOutput << numList << " in total\n";

    for (size_t ll = 0; ll < numList; ++ll) {
        if ((*boxList)[ll].empty()) continue;
        debugOutput << "########################################\n";
        debugOutput << "# " << ll << "\n";
        Generate(&boxList->at(ll));
        debugOutput << "\n";
    }

    for (size_t li = 0; li < latVec_.size(); ++li) {
        const Vector3f color = ColorSchemer::GetJetColor((float)li / latVec_.size());
        const std::deque<SynthLattice1D::Ptr>& latVec = latVec_[li];
        debugRenderer->beginRenderJob_OneFrame("lattice_1d_", DR_FRAME++);
        for (size_t lj = 0; lj < latVec.size(); ++lj) {
            latVec[lj]->DebugDraw(color, 0.02);
        }
        RepBoxLoader::DrawBoxes(boxList->at(li), color);
        debugRenderer->endRenderJob();
    }

    debugOutput << "all done.\n";
}
