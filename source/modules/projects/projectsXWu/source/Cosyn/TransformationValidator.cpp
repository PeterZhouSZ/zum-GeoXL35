#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "TransformationValidator.h"
#include "Util/ColorSchemer.hpp"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

TransformationValidator::TransformationValidator(void)
{
    examplePC_ = nullptr;
    match_ratio_ = 10;
    outlier_ratio_ = 4.f;
    inlier_ratio_ = .8f;
    num_icp_ = 20;
}

void TransformationValidator::SetPC(UICPC* pc)
{
    examplePC_ = pc;
}

Eigen::Matrix4f TransformationValidator::ComputeTransformation(
    RepBox::Ptr source, RepBox::Ptr target
    )
{
    Eigen::Matrix4f sorcF = Eigen::Matrix4f::Identity();
    {
        sorcF.col(0).head<3>() = EigenAdaptor::ToEigen(source->basis[1]);
        sorcF.col(0).head<3>().normalize();
        sorcF.col(1).head<3>() = EigenAdaptor::ToEigen(source->basis[2]);
        sorcF.col(1).head<3>().normalize();
        sorcF.col(2).head<3>() = EigenAdaptor::ToEigen(source->basis[0]);
        sorcF.col(2).head<3>().normalize();
        sorcF.col(3).head<3>() = EigenAdaptor::ToEigen(source->centroid);
    }
    Eigen::Matrix4f destF = Eigen::Matrix4f::Identity();
    {
        destF.col(0).head<3>() = EigenAdaptor::ToEigen(target->basis[1]);
        destF.col(0).head<3>().normalize();
        destF.col(1).head<3>() = EigenAdaptor::ToEigen(target->basis[2]);
        destF.col(1).head<3>().normalize();
        destF.col(2).head<3>() = EigenAdaptor::ToEigen(target->basis[0]);
        destF.col(2).head<3>().normalize();
        destF.col(3).head<3>() = EigenAdaptor::ToEigen(target->centroid);
    }
    return destF * sorcF.inverse();
}

bool TransformationValidator::dampingSnap(
    const Eigen::Matrix4f& trans,
    const Vector3f& centroid, const float& searchRadius,
    FastSphereQuerryPtr rQuery, SubspaceICP::Ptr icp
    )
{
    icp->setupParameters(
        match_ratio_,
        outlier_ratio_,     // outlier distance factor
        inlier_ratio_,                               // minimum inlier percentage
        0.8f,              // allowed move distance
        num_icp_,
        1e-5,                       // maximal residual tolerance
        1e-5,                // convergence difference threshold
        SubspaceICP::Graphical            // show_icl_icp
        );

    const PointSet& examplePS = *examplePC_->getPointSet();
    const float maxRadius = 0.6 * examplePS.getBoundingBox().getDiagonalLength();

    // damping with larger radius often introduce too much noise
    for (unsigned mr = 1; mr < 3; mr *= 2) {
        const float currRadius = searchRadius * (float)mr;
        if (currRadius > maxRadius) break;

        PointSet* patchPS = nullptr;
        {
            mpcard numIndices;
            mpcard* indices;
            rQuery->querry(centroid, currRadius, &indices, numIndices);
            std::deque<mpcard> points(numIndices);
            std::copy(indices, indices + numIndices, points.begin());
            patchPS = examplePS.subset(points);
        }
        float residual = icp->CalculateResidualAfterICP(
            patchPS, EigenAdaptor::ToEigen(centroid), trans
            );
        delete patchPS;

        if (SubspaceICP::NotAligned == icp->GetAlignStatus())
            if (1 == mr) return false;
            else return true;
    }

    return true;
}

//#include "Util/ColorSchemer.hpp"
void TransformationValidator::Validate(std::deque<RepBox::Ptr>* boxList)
{
    debugOutput << "validating transformations ... \n";

    debugRenderer->beginRenderJob_OneFrame("before_validation", DR_FRAME++, true);
    RepBoxLoader::DrawBoxes((*boxList), ColorSchemer::GetJetColor(0));
    debugRenderer->endRenderJob();

    const RepBox::Ptr tempBox = (*boxList)[0];
    const Eigen::Vector3f tempCen = EigenAdaptor::ToEigen(tempBox->centroid);
    Eigen::Matrix4f tempFrame = Eigen::Matrix4f::Identity();
    for (unsigned d = 0; d < 3; ++d) {
        tempFrame.col(d).head<3>() = EigenAdaptor::ToEigen(tempBox->basis[d]).normalized();
    }
    tempFrame.col(3).head<3>() = tempCen;

    debugOutput << "building searching struct ...\n";
    FastSphereQuerryPtr rQuery(new FastSphereQuerry(examplePC_));
    debugOutput << "building icp ...\n";
    SubspaceICP::Ptr icp(new SubspaceICP(examplePC_, tempFrame));
    //icp->SetAxisLocked(0);
    debugOutput << "done.\n";

    prefixOutput.clear();
    progressWindow->pushStep(true, "icp validation");
    size_t validJ = 1;
    const size_t numBox = (*boxList).size();
    for (size_t ti = 1; ti < numBox; ++ti) { // skip the first template
        RepBox::Ptr boxI = (*boxList)[ti];

        float searchRadius = 0;
        {
            for (unsigned d = 0; d < 3; ++d) {
                searchRadius += boxI->coeffs[d] * boxI->coeffs[d];
            }
            searchRadius = 1.05 * sqrt(searchRadius);
        }

        prefixOutput.push(str(boost::format("0-%1%>  ") % ti));
        prefixOutput.write("icp validating ... \n");
        prefixOutput.push("  ");

        Eigen::Matrix4f initTrans = ComputeTransformation(tempBox, boxI);

        const bool bValid = dampingSnap(
            initTrans,
            tempBox->centroid, searchRadius,
            rQuery, icp
            );

        // erase invalid box pointer, and release automatically
        if (bValid) {
            (*boxList)[validJ] = (*boxList)[ti];
            ++validJ;
        }

        const Eigen::Matrix4f Trec = icp->GetUpdatedTransformation();
        const Eigen::Matrix3f Trot = Trec.block<3, 3>(0, 0);
        {
            const Eigen::Vector3f newCen = (Trec * tempCen.homogeneous()).head<3>();
            boxI->centroid = EigenAdaptor::FromEigen(newCen);
            boxI->cenref = boxI->centroid;
        }
        for (unsigned d = 0; d < 3; ++d) {
            const Eigen::Vector3f oa = EigenAdaptor::ToEigen(tempBox->basis[d]);
            const Eigen::Vector3f na = Trot * oa;
            boxI->basis[d] = EigenAdaptor::FromEigen(na);
        }

        prefixOutput.pop();
        prefixOutput.write("done.\n");
        prefixOutput.pop();
        progressWindow->progressf((float)ti / (float)(numBox - 1));
    }
    progressWindow->popStep();

    // erase/remove all invalid boxes at the end of the list
    for (; validJ < numBox; ++validJ) {
        (*boxList)[validJ] = nullptr;
    }
    (*boxList).erase(std::remove((*boxList).begin(), (*boxList).end(), nullptr), (*boxList).end());

    debugRenderer->beginRenderJob_OneFrame("after_validation", DR_FRAME++, true);
    RepBoxLoader::DrawBoxes((*boxList), ColorSchemer::GetJetColor(0.8));
    debugRenderer->endRenderJob();

    debugOutput << "done.\n";
}

void TransformationValidator::Validate(std::deque< std::deque<RepBox::Ptr> >* boxList)
{
    debugOutput << "\n";
    debugOutput << "validating a list of transformations ... \n";

    size_t numList = (*boxList).size();
    debugOutput << numList << " in total\n";

    for (size_t ll = 0; ll < numList; ++ll) {
        if ((*boxList)[ll].empty()) continue;
        debugOutput << "########################################\n";
        debugOutput << "# " << ll << "\n";
        Validate(&boxList->at(ll));
        debugOutput << "\n";
    }

    debugOutput << "all done.\n";
}
