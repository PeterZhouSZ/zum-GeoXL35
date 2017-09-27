#ifndef PatchTransGenerator_inline_H
#define PatchTransGenerator_inline_H
//---------------------------------------------------------------------------
#include "GridAligner.h"
#include "SubspaceICP.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

template<typename CellT>
PatchTransGenerator<CellT>::PatchTransGenerator(void)
{
    example_ = nullptr;
    guide_ = nullptr;
}

template<typename CellT>
void PatchTransGenerator<CellT>::SetGrid(
    typename PCwithGrid<CellT>::Ptr example,
    typename PCwithGrid<CellT>::Ptr guide)
{
    example_ = example;
    guide_ = guide;
}

template<typename CellT>
Matrix4f PatchTransGenerator<CellT>::ComputeTransformation(
    const RepBox::Ptr source, const RepBox::Ptr target)
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
    Eigen::Matrix4f trans = destF * sorcF.inverse();
    return EigenAdaptor::FromEigen(trans);
}

template<typename CellT>
void PatchTransGenerator<CellT>::GenerateTempMatch(RepBoxLoader::Ptr repBoxLoader)
{
    debugOutput << "generating transformations ... \n";

    std::deque< RepBox::Ptr > tempList;
    repBoxLoader->GetTemplateList(&tempList);

    prefixOutput.clear();
    //const unsigned numTemp = tempList.size();
    const unsigned numTemp = 1;
    for (unsigned ti = 0; ti < numTemp; ++ti) {
        prefixOutput.push(str(boost::format("%1%>  ") % ti));
        RepBox::Ptr temp = tempList[ti];
        GridAligner<CellT>::Ptr gridAligner(new GridAligner<CellT>);
        gridAligner->SetGrid(example_, guide_);
        gridAligner->SetTemplate(temp);
        prefixOutput.write("aligning grid ... \n");
        gridAligner->Align();
        prefixOutput.write("done.\n");
        {
            std::ostringstream ss;
            ss << "eliminating high cost candidates ... "
                << gridAligner->patch_trans.size();
            prefixOutput.write(ss.str());
        }
        gridAligner->ScoreFilter(1.f);
        {
            std::ostringstream ss;
            ss << " --> "
                << gridAligner->patch_trans.size() << "\n";
            prefixOutput << ss.str();
        }
        prefixOutput.write("done.\n");

        Eigen::Matrix4f startFrame = Eigen::Matrix4f::Identity();
        SubspaceICP::Ptr icp(new SubspaceICP(guide_->data_, startFrame));
        const unsigned& numICP = 5;
        icp->setupParameters(
            10.f,     // outlier distance factor
            0.4f,                               // minimum inlier percentage
             //0.01f,           // minimum match percentage
            0.8f,              // allowed move distance
            numICP,
            0.08f,                       // maximal residual tolerance
            1e-4,                // convergence difference threshold
            SubspaceICP::Graphical            // show_icl_icp
            );
        std::deque< PatchTrans::Ptr >& patchTransVec = gridAligner->patch_trans;
        const size_t numTrans = patchTransVec.size();
        progressWindow->pushStep(true, "icp validation");
        for (size_t pi = 0; pi < numTrans; ++pi) {
            PatchTrans::Ptr patchTrans = patchTransVec[pi];
            PointSet* queryPS = patchTrans->GetPatchPS();
            {
                std::ostringstream ss;
                ss << "icp validating ... @" << pi << "\n";
                prefixOutput.write(ss.str());
            }
            prefixOutput.push("  ");
            //startFrame.block<3, 3>(0, 0) = EigenAdaptor::ToEigen(patchTrans->frame);
            //startFrame.col(3).head<3>() = EigenAdaptor::ToEigen(patchTrans->center);
            float residual = icp->CalculateResidualAfterICP(
                queryPS, EigenAdaptor::ToEigen(patchTrans->center),
                EigenAdaptor::ToEigen(patchTrans->trans));
            prefixOutput.pop();
            prefixOutput.write("done.\n");
            patchTrans->trans = EigenAdaptor::FromEigen(icp->GetUpdatedTransformation());
            patchTrans->score = residual;
            patchTrans->points = icp->GetInlier();
            progressWindow->progressf((float)pi / (float)(numTrans - 1));
        }
        progressWindow->popStep();
        gridAligner->ScoreFilter();
        prefixOutput.pop();
    }
    debugOutput << "done.\n";
}

template<typename CellT>
void PatchTransGenerator<CellT>::GenerateInClass(const std::deque<RepBox::Ptr>& boxList)
{
    debugOutput << "generating transformations ... \n";

    const unsigned numBox = boxList.size();
    const RepBox::Ptr tempBox = boxList[0];
    for (unsigned ti = 1; ti < numBox; ++ti) { // skip the first template
        const RepBox::Ptr boxI = boxList[ti];
        PatchTrans::Ptr patchTrans(new PatchTrans);
        patchTrans->trans = PatchTransGenerator::ComputeTransformation(
            tempBox, boxI
            );
        patchTrans->data = example_->data_;
        const unsigned numPoints = example_->data_->getNumPoints();
        patchTrans->points.resize(numPoints);
        for (unsigned i = 0; i < numPoints; ++i) {
            patchTrans->points[i] = i;
        }
        patch_trans.push_back(patchTrans);
    }

    debugOutput << "done.\n";
}

template<typename CellT>
void PatchTransGenerator<CellT>::GenerateInClass(const std::deque< Eigen::Matrix4f >& symmetries)
{
    debugOutput << "generating transformations ... \n";

    const unsigned numTrans = symmetries.size();
    for (unsigned ti = 0; ti < numTrans; ++ti) {
        PatchTrans::Ptr patchTrans(new PatchTrans);
        patchTrans->trans = EigenAdaptor::FromEigen(symmetries[ti]);
        patchTrans->data = example_->data_;
        const unsigned numPoints = example_->data_->getNumPoints();
        patchTrans->points.resize(numPoints);
        for (unsigned i = 0; i < numPoints; ++i) {
            patchTrans->points[i] = i;
        }
        patch_trans.push_back(patchTrans);
    }

    debugOutput << "done.\n";
}

#endif
