#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Symmetry/SymmDetHiPrec.h"
#include "Util/TrimeshUnorderedGrid.h"
//---------------------------------------------------------------------------
#include "UnstructuredInCoreTriangleMesh.h"
#include "BoundingBox.h"
#include "StringHelper.h"
#include "Timer.h"
//---------------------------------------------------------------------------

SymmDetHiPrec::SymmDetHiPrec()
{
}

void SymmDetHiPrec::Clear(void)
{
    lineFeatures.clear();
    equivalentLines.clear();
    lineClassID.clear();

    refsymmvec.clear();
    rotsymmvec.clear();
    trasymmvec.clear();
    symmvec.clear();
}

void SymmDetHiPrec::DetectSymmtry(TrimeshStatic::Ptr smesh)
{
    strimesh = smesh;

    // setup global properties	
    BoundingBox3f meshBoundingBox = getPCBBox(&*smesh->GetMesh());
    symmDetCont->diagonal_length = meshBoundingBox.getDiagonalLength();
    symmDetCont->spatial_tolerance = meshBoundingBox.getDiagonalLength()
        * symmDetCont->relative_spatial_tolerance;
    symmDetCont->grid_size = meshBoundingBox.getDiagonalLength()
        * symmDetCont->relative_grid_size;
    symmDetCont->sameline_th = 6 * symmDetCont->grid_size;
    symmDetCont->min_feat_length = 10 * symmDetCont->grid_size;

    //TrimeshUnorderedGrid::Ptr meshgrid;
    //meshgrid->Setup(&*smesh->GetMesh(), mSymmDetCont->grid_size);

    // construct a proxy
    SymmFeatProxyLine::Ptr symmFeatProxyLine (new SymmFeatProxyLine);
    symmDetCont->smesh = smesh;
    symmFeatProxyLine->SetContext(symmDetCont);

    // compute lines features
    //X4_TIMER_START(ExtractFeature);
    symmFeatProxyLine->ExtractFeature();
    //X4_TIMER_STOP(ExtractFeature);
    lineFeatures = symmDetCont->featArr;
    debugOutput << "feature line number: " << lineFeatures.size() << "\n";

    // build traversal tree for lines features
    //X4_TIMER_START(BuildFeatureTree);
    symmFeatProxyLine->BuildFeatureTree();
    //X4_TIMER_STOP(BuildFeatureTree);

    // classify lines
    //X4_TIMER_START(ClusterSimilarLine);
    symmFeatProxyLine->ClusterSimilarLine();
    //X4_TIMER_STOP(ClusterSimilarLine);
    equivalentLines = symmDetCont->equivalentLines;
    lineClassID = symmDetCont->lineClassID;
    debugOutput << "feature cluster number: " << equivalentLines.size() << "\n";

    // search for co-planarity
    //X4_TIMER_START(DetectCoPlanarity);
    symmFeatProxyLine->DetectCoPlanarity(copsymmvec);
    for (size_t ii = 0; ii < copsymmvec.size(); ++ii) {
        symmvec.push_back(copsymmvec[ii]);
        //copsymmvec[ii]->show_ = false;
    }
    if (0 < copsymmvec.size()) { copsymmvec[0]->show_ = true; }
    //X4_TIMER_STOP(DetectCoPlanarity);
    debugOutput << "co-planarity: " << copsymmvec.size() << "\n";

    // search for high precision reflections
    //X4_TIMER_START(DetectReflection);
    symmFeatProxyLine->DetectReflection(refsymmvec);
    //X4_TIMER_STOP(DetectReflection);
    //X4_TIMER_START(PropagateReflectionPoints);
    symmFeatProxyLine->PropagateReflectionPoints(refsymmvec);
    //X4_TIMER_STOP(PropagateReflectionPoints);
    for (size_t ii = 0; ii < refsymmvec.size(); ++ii) {
        symmvec.push_back(refsymmvec[ii]);
        //refsymmvec[ii]->show_ = false;
    }
    if (0 < refsymmvec.size()) { refsymmvec[0]->show_ = true; }
    debugOutput << "reflective symmetry: " << refsymmvec.size() << "\n";

    // search for high precision rotations
    //X4_TIMER_START(DetectRotation);
    symmFeatProxyLine->DetectRotation(rotsymmvec);
    for (size_t ii = 0; ii < rotsymmvec.size(); ++ii) {
        symmvec.push_back(rotsymmvec[ii]);
        //rotsymmvec[ii]->show_ = false;
    }
    if (0 < rotsymmvec.size()) { rotsymmvec[0]->show_ = true; }
    //X4_TIMER_STOP(DetectRotation);
    debugOutput << "rotational symmetry: " << rotsymmvec.size() << "\n";

    // search for high precision translations
    //X4_TIMER_START(DetectTranslation);
    symmFeatProxyLine->DetectTranslation(trasymmvec);
    for (size_t ii = 0; ii < trasymmvec.size(); ++ii) {
        symmvec.push_back(trasymmvec[ii]);
        //trasymmvec[ii]->show_ = false;
    }
    if (0 < trasymmvec.size()) { trasymmvec[0]->show_ = true; }
    //X4_TIMER_STOP(DetectTranslation);
    debugOutput << "translational symmetry: " << trasymmvec.size() << "\n";
}

int SymmDetHiPrec::ShowFeatureLines(int sframe, bool const& clear)
{
    std::string const id = "feature_lines_";
    debugRenderer->clearRenderJob_AllFrames(id);
    if (clear || 0 > sframe) return sframe;

    debugRenderer->beginRenderJob_OneFrame(id, sframe++);
    for (size_t ii = 0; ii < lineFeatures.size(); ++ii) {
        debugRenderer->addLine(
            lineFeatures[ii]->pos0, lineFeatures[ii]->pos1,
            makeVector3f(1, 0, 0),
            makeVector3f(0, 0, 1),
            3);
        debugRenderer->addPoint(
            lineFeatures[ii]->pos0,
            makeVector3f(1, 1, 0)
            );
            debugRenderer->addPoint(
            lineFeatures[ii]->pos1,
            makeVector3f(1, 1, 0)
            );
    }
    debugRenderer->endRenderJob();
    return sframe;
}

int SymmDetHiPrec::ShowFeatureLinesCluster(int sframe, bool const& clear)
{
    std::string const id = "line_cluster_";
    debugRenderer->clearRenderJob_AllFrames(id);
    if (clear || 0 > sframe) return sframe;

    for (size_t ii = 0; ii < equivalentLines.size(); ++ii) {
        debugRenderer->beginRenderJob_OneFrame(id, sframe++);
        debugRenderer->addLine(
            equivalentLines[ii]->elem->pos0,
            equivalentLines[ii]->elem->pos1,
            makeVector3f(0, 1, 0),
            makeVector3f(0, 1, 0),
            3);
        for (size_t jj = 0; jj < equivalentLines[ii]->size(); ++jj) {
            debugRenderer->addLine(
                equivalentLines[ii]->elem->cen,
                equivalentLines[ii]->lset[jj]->cen,
                makeVector3f(1, 0, 0),
                makeVector3f(0, 0, 1),
                3);
        }
        debugRenderer->endRenderJob();
    }
    return sframe;
}

int SymmDetHiPrec::ShowCoPlanarity(int sframe, bool const& usefeature, bool const& clear)
{
    std::string const id = "coplanarity_";
    debugRenderer->clearRenderJob_AllFrames(id);
    if (clear || 0 > sframe) return sframe;

    for (size_t ii = 0; ii < copsymmvec.size(); ++ii) {
        if (!copsymmvec[ii]->show_) continue;
        debugRenderer->beginRenderJob_OneFrame(id, sframe++);
        copsymmvec[ii]->DrawWithDR(makeVector3f(1,1,0));
        debugRenderer->endRenderJob();
    }
    return sframe;
}

int SymmDetHiPrec::ShowReflections(int sframe, bool const& usefeature, bool const& clear)
{
    std::string const id = "reflection_";
    debugRenderer->clearRenderJob_AllFrames(id);
    if (clear || 0 > sframe) return sframe;

    for (size_t ii = 0; ii < refsymmvec.size(); ++ii) {
        if (!refsymmvec[ii]->show_) continue;

        debugRenderer->beginRenderJob_OneFrame(id, sframe++);
        if (usefeature) {
            refsymmvec[ii]->DrawWithDR(makeVector3f(1,1,0));
        } else {
            refsymmvec[ii]->DrawPlane(makeVector3f(1,1,0));
            std::vector<SymmFeatVertexPair>& vertpairs = refsymmvec[ii]->vertPairs_;
            for (size_t pi = 0; pi < vertpairs.size(); ++pi) {
                mpcard vindx0 = vertpairs[pi].first;
                mpcard vindx1 = vertpairs[pi].second;
                Vector3f point0 = strimesh->GetVertPosition(vindx0);
                Vector3f point1 = strimesh->GetVertPosition(vindx1);
                debugRenderer->addLine(
                    point0, point1,
                    makeVector3f(1, 0, 0),
                    makeVector3f(0, 0, 1),
                    3);
            }
        }
        debugRenderer->endRenderJob();

        ////debugOutput << refsymmvec[ii]->mTrans << "\n";
    }
    return sframe;
}

int SymmDetHiPrec::ShowRotations(int sframe, bool const& usefeature, bool const& clear)
{
    std::string const id = "rotation_";
    debugRenderer->clearRenderJob_AllFrames(id);
    if (clear || 0 > sframe) return sframe;

    for (size_t ii = 0; ii < rotsymmvec.size(); ++ii) {
        //OrbitPosMap::iterator jt = rotsymmvec[ii]->patPair_.first->pmap_.begin();
        //for (; jt != rotsymmvec[ii]->patPair_.first->pmap_.end(); ++jt) {
        //    debugOutput << jt->first;
        //}
        //debugOutput << "\n";
        //jt = rotsymmvec[ii]->patPair_.second->pmap_.begin();
        //for (; jt != rotsymmvec[ii]->patPair_.second->pmap_.end(); ++jt) {
        //    debugOutput << jt->first;
        //}
        //debugOutput << "\n";

        if (!rotsymmvec[ii]->show_) continue;
        debugRenderer->beginRenderJob_OneFrame(id, sframe++);
        rotsymmvec[ii]->DrawWithDR();
        debugRenderer->endRenderJob();
    }
    return sframe;
}

int SymmDetHiPrec::ShowTranslations(int sframe, bool const& usefeature, bool const& clear)
{
    std::string const id = "translation_";
    
    debugRenderer->clearRenderJob_AllFrames(id);
    
    if (clear || 0 > sframe) return sframe;

    for (size_t ii = 0; ii < trasymmvec.size(); ++ii) {
        if (!trasymmvec[ii]->show_) continue;
        debugRenderer->beginRenderJob_OneFrame(id, sframe++);
        trasymmvec[ii]->DrawWithDR();
        debugRenderer->endRenderJob();
    }
    return sframe;
}
