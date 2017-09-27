#ifndef GridAligner_inline_H
#define GridAligner_inline_H
//---------------------------------------------------------------------------
#include "Util\numerical\EigenAdaptor.h"
#include "ProjectSettings.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

template<typename CellT>
GridAligner<CellT>::GridAligner(void)
{
    example_ = nullptr;
    guide_ = nullptr;
}

template<typename CellT>
void GridAligner<CellT>::SetGrid(
    typename PCwithGrid<CellT>::Ptr example,
    typename PCwithGrid<CellT>::Ptr guide)
{
    example_ = example;
    guide_ = guide;
}

template<typename CellT>
void GridAligner<CellT>::SetTemplate(RepBox::Ptr tempBox)
{
    temp_box = tempBox;
}

template<typename CellT>
float GridAligner<CellT>::estimateCost(
    PointSet* startPS, PointSet* targetPS,
    const Matrix4f& trans, PointSetANNQueryPtr KNN,
    const float& outlierDist
    )
{
    const Eigen::Matrix4f transE = EigenAdaptor::ToEigen(trans);
    const unsigned numPoints = startPS->getNumEntries();
    const AAT NORMAL = targetPS->getAAT("normal");
    const AAT POSITION = targetPS->getAAT("position");

    float residual = 0;
    unsigned numInliers = 0;

    // calculate corresponding points
    for (unsigned ii = 0; ii < numPoints; ++ii) {
        const Eigen::Vector3f posI = EigenAdaptor::ToEigen(startPS->get3f(ii, POSITION));
        const Eigen::Vector3f posI_trans = EigenAdaptor::Transform(transE, posI);

        // seek closest point
        const int32 indexNN = KNN->getNearestPointIndex(EigenAdaptor::FromEigen(posI_trans));
        const Eigen::Vector3f posInn = EigenAdaptor::ToEigen(targetPS->get3f(indexNN, POSITION));
        const Eigen::Vector3f nmlInn = EigenAdaptor::ToEigen(targetPS->get3f(indexNN, NORMAL));

        const float distNN = (posInn - posI_trans).norm();
        if (distNN > outlierDist) {
            continue;
        }

        // project onto surfel
        Eigen::Vector3f projectedPoint = posI_trans - nmlInn * (nmlInn.dot(posI_trans - posInn));
        float projectedDistance = (posI_trans - projectedPoint).squaredNorm();

        // current residual
        residual += projectedDistance;
        ++numInliers;
    }
    // root mean square
    residual /= (float)numInliers;
    residual = sqrt(residual);
    return residual;
}

template<typename CellT>
void GridAligner<CellT>::Align(void)
{
    Eigen::Vector3f upDir = EigenAdaptor::ToEigen(temp_box->basis[2]);
    upDir.normalize();
    GridType::Ptr guideGrid = guide_->grid_;

    UICPC* guidePC = guide_->data_;
    PointSet* guidePS = guidePC->getPointSet();
    const AAT posAAT = guidePC->getAAT("position");
    const AAT nmlAAT = guidePC->getAAT("normal");
    PointSetANNQueryPtr knn(new PointSetANNQuery(guidePS, 1));

    UICPC* examplePC = example_->data_;
    PointSet* examplePS = examplePC->getPointSet();
    FastSphereQuerry rQuery(examplePC);

    progressWindow->pushStep(true, "grid align");
    const int numCell = guideGrid->size();
    patch_trans.resize(numCell);
//#pragma omp parallel for schedule(dynamic,1)
    for (int ci = 0; ci < numCell; ++ci) {
        Eigen::Matrix4f transCell = Eigen::Matrix4f::Zero();
        {
            const Vector3f cenCell = guideGrid->GetCellCenter(ci);
            const int32 indexPS = knn->getNearestPointIndex(cenCell);
            const Eigen::Vector3f cenPS = EigenAdaptor::ToEigen(guidePS->get3f(indexPS, posAAT));
            Eigen::Vector3f nmlPS = EigenAdaptor::ToEigen(guidePS->get3f(indexPS, nmlAAT));
            nmlPS.normalize();
            if (projectSettings.bFlipNormal) nmlPS = -nmlPS;
            const Eigen::Vector3f lftPS = nmlPS.cross(upDir);
            transCell.col(0).head<3>() = upDir.cross(lftPS);
            transCell.col(1).head<3>() = upDir;
            transCell.col(2).head<3>() = lftPS;
            transCell.col(3).head<3>() = cenPS;
            transCell(3, 3) = 1.f;
        }
        Eigen::Matrix4f transTemp = Eigen::Matrix4f::Zero();
        {
            transTemp.col(0).head<3>() = EigenAdaptor::ToEigen(temp_box->basis[1]);
            transTemp.col(0).head<3>().normalize();
            transTemp.col(1).head<3>() = upDir;
            transTemp.col(2).head<3>() = EigenAdaptor::ToEigen(temp_box->basis[0]);
            transTemp.col(2).head<3>().normalize();
            transTemp.col(3).head<3>() = EigenAdaptor::ToEigen(temp_box->centroid);
            transTemp(3, 3) = 1.f;
        }
        Eigen::Matrix4f temp2cell = transCell * transTemp.inverse();

        PatchTrans::Ptr patchTrans = patch_trans[ci];
        {
            float searchRadius;
            {
                float maxt = std::max(norm(temp_box->basis[0]), norm(temp_box->basis[1]));
                searchRadius = std::max(maxt, norm(temp_box->basis[2]));
            }
            searchRadius *= 1.5f;
            patchTrans->data = examplePC;
            patchTrans->center = temp_box->centroid;
            patchTrans->frame = EigenAdaptor::FromEigen(
                static_cast<Eigen::Matrix3f>(transTemp.block(0, 0, 3, 3))
                );
            patchTrans->radius = searchRadius;
            patchTrans->trans = EigenAdaptor::FromEigen(temp2cell);

            mpcard numIndices;
            mpcard* indices;
            rQuery.querry(temp_box->centroid, searchRadius, &indices, numIndices);

            patchTrans->points.resize(numIndices);
            for (mpcard ii = 0; ii < numIndices; ++ii) {
                patchTrans->points[ii] = (unsigned)*(indices + ii);
            }
            //std::copy(indices, indices + numIndices, patchTrans->points.begin());

            PointSet* patchPS = patchTrans->GetPatchPS();
            const float outlierDist = searchRadius;
            patchTrans->score = estimateCost(patchPS, guidePS, patchTrans->trans,
                knn, outlierDist);
        }

        const bool bDebugDraw = false;
        if (bDebugDraw)
        {
            debugRenderer->beginRenderJob_OneFrame("grid_align_", DebugRenderer::DR_FRAME++);
            guide_->VisualizeGrid(makeVector3f(0.4f, 0.4f, 0.4f));
            patchTrans->Draw();
            debugRenderer->endRenderJob();
        }

        progressWindow->progressf((float)ci / (float)(numCell - 1));
    }
    progressWindow->popStep();
}

template<typename CellT>
void GridAligner<CellT>::ScoreFilter(const float& epsilon)
{
    struct IsHighCost
    {
        IsHighCost(const float& epsilon) : epsilon_(epsilon) {}
        int epsilon_;
        bool operator()(const PatchTrans::Ptr patchTrans) const
        {
            return epsilon_ < patchTrans->score;
        }
    };
    patch_trans.erase(
        std::remove_if(patch_trans.begin(), patch_trans.end(), IsHighCost(epsilon)),
        patch_trans.end()
        );
}

#endif
