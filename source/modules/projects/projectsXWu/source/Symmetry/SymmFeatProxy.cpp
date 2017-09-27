#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Symmetry/SymmFeatProxy.h"
#include "Util/TrimeshUnorderedGrid.h"
#include "Util/NoUse.h"
#include "Util/TupleSampler.h"
#include "Util/IsometryGrid.h"
//---------------------------------------------------------------------------
#include "VertexArray.h"
#include "RigidMotion.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

template<class T>
bool size_greater(const T& l, const T& r)
{
    return l->size() > r->size();
}

template<class T>
bool size_less_value (const T& l, const float& sz, const float& th)
{
    return th > l->size() / sz;
}

void SymmFeatProxyLine::ExtractFeature(void)
{
    TrimeshStatic::Ptr smesh = symmDetCont->smesh;
    std::vector<SymmFeatLine::Ptr>& featArr = symmDetCont->featArr;

    boost::unordered_map< size_t, set<size_t> > vertShared;
    std::vector<SymmFeatLine::Ptr> lineCandi;

    const TSEdgeV& edges = smesh->GetEdges();
    for (unsigned ei = 0; ei < edges.size(); ++ei) {
        if (!smesh->IsEdgeBorder(ei) && !smesh->IsEdgeCrease(ei))
            continue;
        //float l = norm(
        //    smesh->GetVertPosition(smesh->GetEdgeVert0(ei)) -
        //    smesh->GetVertPosition(smesh->GetEdgeVert1(ei))
        //    );
        //if (l < symmDetCont->min_feat_length) continue;

        vertShared[smesh->GetEdgeVert0(ei)].insert(lineCandi.size());
        vertShared[smesh->GetEdgeVert1(ei)].insert(lineCandi.size());

        SymmFeatLine::Ptr line (new SymmFeatLine);
        line->Setup(smesh, ei);
        lineCandi.push_back(line);
    }
    //debugRenderer->beginRenderJob_OneFrame("debug_", DR_FRAME++);
    //for (unsigned ii = 0; ii < lineCandi.size(); ++ii) {
    //    debugRenderer->addLine(
    //        lineCandi[ii]->pos0, lineCandi[ii]->pos1,
    //        makeVector3f(1, 0, 0),
    //        makeVector3f(0, 0, 1),
    //        3);
    //}
    //debugRenderer->endRenderJob();

    std::vector<unsigned> mlc;
    for (unsigned ii = 0; ii < lineCandi.size(); ++ii)
        mlc.push_back(ii);
    boost::unordered_map< size_t, set<size_t> >::iterator
        it = vertShared.begin();
    for (; it != vertShared.end(); ++it) {
        set<size_t>& nedges = it->second;
        //{
        //    debugRenderer->beginRenderJob_OneFrame("debug_", DR_FRAME++);
        //    debugRenderer->addPoint(
        //        smesh->GetVertexPosition(it->first),
        //        makeVector3f(1, 1, 0));
        //    set<unsigned>::iterator it3 = nedges.begin();
        //    for (; it3 != nedges.end(); ++it3) {
        //        debugRenderer->addLine(
        //            lineCandi[(*it3)]->pos0, lineCandi[(*it3)]->pos1);
        //    }
        //    debugRenderer->endRenderJob();
        //}
        set<size_t>::iterator it0 = nedges.begin();
        for (; it0 != nedges.end(); ++it0) {
            set<size_t>::iterator it1 = it0;
            for (++it1; it1 != nedges.end(); ++it1) {
                if (!lineCandi[(*it0)]->Join(smesh, lineCandi[(*it1)]))
                    continue;
                unsigned tt0 = mlc[(*it0)], tt1 = mlc[(*it1)];
                for (unsigned ii = 0; ii < mlc.size(); ++ii) {
                    if (mlc[ii] != tt1) continue;
                    lineCandi[ii] = lineCandi[(*it0)];
                    mlc[ii] = tt0;
                }
            }
        }
    }

    set<unsigned> outLines;
    copy(mlc.begin(), mlc.end(), inserter(outLines, outLines.begin()));
    for (set<unsigned>::iterator it = outLines.begin(); it != outLines.end(); ++it) {
        featArr.push_back(lineCandi[*it]);
    }

    for (int ii = 0; ii < featArr.size(); ++ii) {
        SymmFeatLine::Ptr line = featArr[ii];
        line->UniformDirection();

        //debugRenderer->beginRenderJob_OneFrame("debug_", ii);
        //line->DrawWithDR();
        //debugRenderer->endRenderJob();
    }
}

void SymmFeatProxyLine::BuildFeatureTree(void)
{
    std::vector<SymmFeatLine::Ptr>& featArr = symmDetCont->featArr;

    UnstructuredInCorePointCloudPtr lineFeatureUPC (new UnstructuredInCorePointCloud);
    lineFeatureUPC->clearAndSetup(featArr.size(), true, false, false, true);
    AAT posAAT = lineFeatureUPC->getAAT("position");
    for (mpcard ii = 0; ii < featArr.size(); ++ii) {
        // use line center as feature position
        lineFeatureUPC->getPointSet()->set3f(ii,posAAT, featArr[ii]->cen);
    }
    HierarchicalKNNIteratorPtr hItFeatures 
        (new HierarchicalKNNIterator(&*lineFeatureUPC, 32, nullptr));
    hItFeatures->setMaxDistanceToSeekPoint(symmDetCont->sameline_th);

    symmDetCont->hItFeatures = hItFeatures;
    symmDetCont->lineFeatureUPC = lineFeatureUPC;
}

void SymmFeatProxyLine::DetectCoPlanarity(const std::vector<SymmFeatLine::Ptr>& featArr,
                       std::vector<SymmPlane::Ptr>& copsymmvec)
{
    std::vector<bool> featChecked(featArr.size(), false);
    boost::array<size_t, 2> indx2;

    /*  random sampling <> enumerate
    TupleSampler<mpcard, 2> sampler(featArr.size()-1);
    for (mpcard ii = 0; ii < symmDetCont->num_iter; ++ii) {
        if (!sampler.SampleUniqSort(indx2)) continue;
    /*/
    for (indx2[0] = 0; indx2[0] < featArr.size(); ++indx2[0]) {
        for (indx2[1] = indx2[0]+1; indx2[1] < featArr.size(); ++indx2[1]) {
    //*/
            if (featChecked[indx2[0]] || featChecked[indx2[1]]) continue;
        }
    }
}

void SymmFeatProxyLine::DetectCoPlanarity(std::vector<SymmPlane::Ptr>& copsymmvec)
{

    //std::vector<SymmFeatSetLine::Ptr>& equivalentLines = symmDetCont->equivalentLines;

    //copsymmvec.clear();

    //for (int ci = 0; ci < equivalentLines.size(); ++ci) {
    //    DetectCoPlanarity(equivalentLines[ci]->lset, copsymmvec);
    //}

    //std::sort(copsymmvec.begin(), copsymmvec.end(),
    //    size_greater<SymmPlane::Ptr>);

    //if (0 < copsymmvec.size()) {
    //    float max_sz = copsymmvec[0]->size();
    //    copsymmvec.erase(
    //        std::remove_if(copsymmvec.begin(), copsymmvec.end(),
    //        boost::bind(size_less_value<SymmPlane::Ptr>,
    //        _1, max_sz, 0.6f)),
    //        copsymmvec.end()
    //        );
    //}

    //debugOutput << "co-planarity: " << copsymmvec.size() << "\n";


    TrimeshStatic::Ptr smesh = symmDetCont->smesh;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    card32 const sizeThreshold =
        smesh->GetNumFaces() * symmDetCont->coplanarity_size;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    std::set<mpcard> overall;

    for (mpcard i = 0; i < smesh->GetNumFaces(); ++i)
    {
        if (overall.find(i) != overall.end()) { continue; }

        Vector3f const normal = smesh->GetFaceNormal(i);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        std::set<mpcard> current;
        std::set<mpcard> processed;

        std::deque<mpcard> stack;

        stack.push_back(i);

        //debugRenderer->beginRenderJob_OneFrame("debug_", DR_FRAME++);
        //Vector3i vi = smesh->GetFaceVertices(i);
        //debugRenderer->addTriangle(
        //    smesh->GetVertPosition(vi[0]),
        //    smesh->GetVertPosition(vi[1]),
        //    smesh->GetVertPosition(vi[2]),
        //    makeVector3f(1, 1, 0)
        //    );
        //debugRenderer->endRenderJob();

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        while (!stack.empty())
        {
            mpcard const face = stack.back();

            stack.pop_back();

            if (  overall.find(face) !=   overall.end()) { continue; }
            if (processed.find(face) != processed.end()) { continue; }

            processed.insert(face);

            // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            Vector3f const faceNormal = smesh->GetFaceNormal(face);

            float32 angle  = (normal           * faceNormal          );
            angle /= (normal.getNorm() * faceNormal.getNorm());
            angle  = acos(angle);
            angle *= 180.0f / M_PI;

            if (symmDetCont->coplanarity_angle < angle)                  { continue; }

            // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            current.insert(face);

            std::vector<mpcard> const adjacent =
                smesh->GetFaceAdjacentFaces(face);

            stack.insert(stack.end(), adjacent.begin(), adjacent.end());

            //debugRenderer->beginRenderJob_OneFrame("debug_", DR_FRAME++);
            //for (std::set<mpcard>::iterator it = current.begin();
            //    it != current.end(); ++it)
            //{
            //    Vector3i const vi = smesh->GetFaceVertices(*it);
            //    debugRenderer->addTriangle(
            //        smesh->GetVertPosition(vi[0]),
            //        smesh->GetVertPosition(vi[1]),
            //        smesh->GetVertPosition(vi[2]),
            //        makeVector3f(1, 1, 0)
            //        );
            //}
            //debugRenderer->endRenderJob();

        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        if (current.size() < sizeThreshold) { continue; }

        overall.insert(current.begin(), current.end());

        std::set<mpcard> vertices;

        for (std::set<mpcard>::iterator it = current.begin();
            it != current.end(); ++it)
        {
            Vector3i const faceVertices =
                smesh->GetFaceVertices(*it);

            vertices.insert(faceVertices[0]);
            vertices.insert(faceVertices[1]);
            vertices.insert(faceVertices[2]);
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        std::vector<Vector3f> verts;
        std::vector<mpcard> vindx;
        std::set<mpcard>::iterator it;
        for (it = vertices.begin(); it != vertices.end(); ++it) {
            verts.push_back(smesh->GetVertPosition(*it));
            vindx.push_back(*it);
        }

        SymmPlane::Ptr tsymm (new SymmPlane(verts, vindx));
        copsymmvec.push_back(tsymm);
    }

    std::sort(copsymmvec.begin(), copsymmvec.end(),
        size_greater<SymmPlane::Ptr>);

    if (0 < copsymmvec.size()) {
        float max_sz = copsymmvec[0]->size();
        copsymmvec.erase(
            std::remove_if(copsymmvec.begin(), copsymmvec.end(),
            boost::bind(size_less_value<SymmPlane::Ptr>,
            _1, max_sz, 0.6f)),
            copsymmvec.end()
            );
    }
}

void SymmFeatProxyLine::
CheckSymmetryTransformation(const Matrix4f& trans, float& score,
                            std::map< pair<size_t, size_t>, LineDir >& symmetricLines
                            )
{
    std::vector<SymmFeatLine::Ptr>& featArr = symmDetCont->featArr;
    HierarchicalKNNIteratorPtr hItFeatures = symmDetCont->hItFeatures;

    for (mpcard lcp0 = 0; lcp0 < featArr.size(); ++lcp0) {
        SymmFeatLine::Ptr fl0 = featArr[lcp0];
        SymmFeatLine::Ptr fl1;
        Vector3f cen0 = fl0->cen;
        Vector3f cen0_t = transformVector3f(trans, cen0);
        hItFeatures->setSeekPointAndReset(cen0_t);
        LineDir foundCounterPart = E_DIR;
        mpcard lcp1;
        while (!hItFeatures->atEnd() && E_DIR == foundCounterPart) {
            lcp1 = hItFeatures->getCurrentPointIndex();
            hItFeatures->next();
            if (lcp0 == lcp1) continue;

            fl1 = featArr[lcp1];
            SymmFeatLine::Ptr fl0_t (new SymmFeatLine);
            fl0_t->Setup(
                transformVector3f(trans, fl0->pos0),
                transformVector3f(trans, fl0->pos1),
                fl1->surfaceN0, fl1->surfaceN1,
                fl1->isBder);
            fl0_t->edges = fl0->edges;
            fl0_t->vertice = fl0->vertice;
            foundCounterPart = fl0_t->SameAs(fl1, symmDetCont->sameline_th);
        }
        if (E_DIR == foundCounterPart) continue;

        symmetricLines[make_pair(lcp0, lcp1)] = foundCounterPart;
        score += fl0->leng;
        score += fl1->leng;
    }
}

void SymmFeatProxyLine::
ClusterSimilarLine(void)
{
    std::vector<SymmFeatLine::Ptr>& featArr = symmDetCont->featArr;
    std::vector<SymmFeatSetLine::Ptr>& equivalentLines = symmDetCont->equivalentLines;
    std::vector<unsigned>& lineClassID = symmDetCont->lineClassID;

    lineClassID.resize(featArr.size());
    for (unsigned fi = 0; fi < featArr.size(); ++fi) {
        SymmFeatLine::Ptr sf = featArr[fi];

        // search for existing representative element with similar properties 
        bool classFound = false;
        for (unsigned ei = 0; ei < equivalentLines.size(); ++ei) {
            SymmFeatLine::Ptr rep_sf = equivalentLines[ei]->elem;
            if (sf->SimilarTo(rep_sf, 1 * symmDetCont->grid_size)) {
                classFound = true;
                equivalentLines[ei]->Insert(sf, fi);
                lineClassID[fi] = ei;
                break;
            }
        }

        // no representative element, so we create a new class 
        if (!classFound) {
            lineClassID[fi] = (unsigned)equivalentLines.size();
            SymmFeatSetLine::Ptr pls (new SymmFeatSetLine(sf));
            equivalentLines.push_back(pls);
        }
    }
}

void SymmFeatProxyLine::
detectTranslation2P(const std::vector<SymmFeatLine::Ptr> &featArr,
                    const boost::array<size_t, 2>& indx2,
                    std::vector<bool>& featChecked,
                    std::vector<SymmTranslation::Ptr>& trasymmvec)
{
    // setup from end point
    PatternTranslation::Ptr tpat0 (new PatternTranslation);
    {
        array2vec3f pvec;
        for (mpcard j = 0; j < 2; ++j) {
            pvec[j] = featArr[indx2[j]]->pos0;
        }
        Vector2i ivec;
        if (!tpat0->SetupFrom2P(pvec, ivec)) return;
    }
    PatternTranslation::Ptr tpat1 (new PatternTranslation);
    {
        array2vec3f pvec;
        for (mpcard j = 0; j < 2; ++j) {
            pvec[j] = featArr[indx2[j]]->pos1;
        }
        Vector2i ivec;
        if (!tpat1->SetupFrom2P(pvec, ivec)) return;
    }
    if (!tpat0->ConsistentWith(tpat1) || tpat0->SameAs(tpat1)) return;

    // self-similar checking
    SymmTranslation::Ptr tsymm (new SymmTranslation(tpat0, tpat1));
    float score;
    std::map< pair<size_t, size_t>, LineDir > symmetricLines;
    CheckSymmetryTransformation(
        tsymm->Get1StepTransformation(),
        score, symmetricLines);
    //if (0.8f * featArr.size() / symmDetCont->featArr.size()
    //    > symmetricLines.size() / symmDetCont->featArr.size()
    //    ) return;
    if (0.8f * featArr.size() > symmetricLines.size())
        return;

    // expand and threshold checking
    std::vector<bool> tchecked = featChecked;
    for (size_t ii = 0; ii < featArr.size(); ++ii) {
        int ix;
        if (tsymm->AddVertice(featArr[ii], ix)) {
            tchecked[ii] = true;
        }
    }
    if (false
        || 4 > tsymm->size()
        //|| tsymm->size() < 0.05f*featArr.size()
        || !tsymm->IsSeries()
        ) return;
    if (true
        ) {
            trasymmvec.push_back(tsymm);
            //featChecked = tchecked;
    }
}

void SymmFeatProxyLine::
DetectTranslation(std::vector<SymmTranslation::Ptr>& trasymmvec)
{
    std::vector<SymmFeatSetLine::Ptr>& equivalentLines = symmDetCont->equivalentLines;

    trasymmvec.clear();

    for (int ci = 0; ci < equivalentLines.size(); ++ci) {
        DetectTranslation(equivalentLines[ci]->lset, trasymmvec);
    }

    std::sort(trasymmvec.begin(), trasymmvec.end(),
        size_greater<SymmTranslation::Ptr>);

    if (0 < trasymmvec.size()) {
        float max_sz = trasymmvec[0]->size();
        trasymmvec.erase(
            std::remove_if(trasymmvec.begin(), trasymmvec.end(),
            boost::bind(size_less_value<SymmTranslation::Ptr>,
            _1, max_sz, 0.6f)),
            trasymmvec.end()
            );
    }
}

void SymmFeatProxyLine::
DetectTranslation(const std::vector<SymmFeatLine::Ptr>& featArr,
                  std::vector<SymmTranslation::Ptr>& trasymmvec)
{
    std::vector<bool> featChecked(featArr.size(), false);
    boost::array<size_t, 2> indx2;
    size_t featsize = featArr.size();

    if (symmDetCont->num_sample < featsize) {
        TupleSampler<mpcard, 2> sampler(featArr.size()-1);
        for (mpcard ii = 0; ii < symmDetCont->num_iter; ++ii) {
            if (!sampler.SampleUniqSort(indx2)) continue;
            if (featChecked[indx2[0]] || featChecked[indx2[1]]) continue;
            detectTranslation2P(featArr, indx2, featChecked, trasymmvec);
        }
    } else {
        for (indx2[0] = 0; indx2[0] < featArr.size(); ++indx2[0]) {
            for (indx2[1] = indx2[0]+1; indx2[1] < featArr.size(); ++indx2[1]) {
                if (featChecked[indx2[0]] || featChecked[indx2[1]]) continue;
                detectTranslation2P(featArr, indx2, featChecked, trasymmvec);
            }
        }
    }
}

void SymmFeatProxyLine::
detectRotation3P(const std::vector<SymmFeatLine::Ptr> &featArr,
                 const boost::array<size_t, 3>& indx3,
                 std::vector<bool>& featChecked,
                 std::vector<SymmRotation::Ptr>& rotsymmvec)
{
    // setup from end point
    PatternRotation::Ptr tpat0 (new PatternRotation);
    {
        array3vec3f pvec;
        for (mpcard j = 0; j < 3; ++j) {
            pvec[j] = featArr[indx3[j]]->pos0;
        }
        Vector3i ivec;
        if (!tpat0->SetupFrom3P(pvec, ivec)) return;
    }
    PatternRotation::Ptr tpat1 (new PatternRotation);
    {
        array3vec3f pvec;
        for (mpcard j = 0; j < 3; ++j) {
            pvec[j] = featArr[indx3[j]]->pos1;
        }
        Vector3i ivec;
        if (!tpat1->SetupFrom3P(pvec, ivec)) return;
    }
    if (4 > tpat0->capacity() || 4 > tpat1->capacity()) return;
    if (!tpat0->ConsistentWith(tpat1) || tpat0->SameAs(tpat1)) return;

    SymmRotation::Ptr tsymm (new SymmRotation(tpat0, tpat1));
    for (mpcard j = 0; j < 3; ++j) {
        int ix;
        if (!tsymm->AddVertice(featArr[indx3[j]], ix)) {
            warning("SymmFeatProxyLine::detectRotation3P: "
                "AddVertice");

            //debugRenderer->beginRenderJob_OneFrame("debug_", DR_FRAME++);
            //tsymm->DrawWithDR();
            //debugRenderer->endRenderJob();

            return;
        }
    }

    // self-similar checking
    float score;
    std::map< pair<size_t, size_t>, LineDir > symmetricLines;
    CheckSymmetryTransformation(
        tsymm->Get1StepTransformation(),
        score, symmetricLines);
    if (0.2f * featArr.size() > symmetricLines.size())
        return;

    // expand and threshold checking
    std::vector<bool> tchecked = featChecked;
    for (size_t ii = 0; ii < featArr.size(); ++ii) {
        int ix;
        if (tsymm->AddVertice(featArr[ii], ix)) {
            tchecked[ii] = true;
        }
    }

    if (tpat0->size() != tpat1->size()) {
        warning("SymmFeatProxyLine::detectRotation3P: "
            "first - second");
        debugOutput << "SymmFeatProxyLine::detectRotation3P: "
            << tpat0->size() << " " << tpat1->size() << "\n";
        OrbitPosMap::iterator it;
        for (it = tpat0->pmap_.begin(); it != tpat0->pmap_.end(); ++it) {
            debugOutput << it->first;
        }
        debugOutput << "\n";
        for (it = tpat1->pmap_.begin(); it != tpat1->pmap_.end(); ++it) {
            debugOutput << it->first;
        }
        debugOutput << "\n";
        return;
    }
    if (tsymm->overt_.size() != tpat0->size()) {
        warning("SymmFeatProxyLine::detectRotation3P: "
            "pattern - first");
        debugOutput << "SymmFeatProxyLine::detectRotation3P: "
            << tsymm->overt_.size() << " " << tpat0->size() << "\n";
        OrbitVerticeMap::iterator it;
        for (it = tsymm->overt_.begin(); it != tsymm->overt_.end(); ++it) {
            debugOutput << it->first;
        }
        debugOutput << "\n";
        OrbitPosMap::iterator jt;
        for (jt = tpat0->pmap_.begin(); jt != tpat0->pmap_.end(); ++jt) {
            debugOutput << jt->first;
        }
        debugOutput << "\n";
        return;
    }

    if (!tsymm->Finalize()) return;

    if (false
        || 4 > tsymm->size()
        //|| tsymm->size() < 0.4f*tsymm->capacity()
        //|| tsymm->capacity() < 0.1f*featArr.size()
        || !tsymm->IsSeries()
        ) return;
    if (true
        ) {
            rotsymmvec.push_back(tsymm);
            //featChecked = tchecked;
    }
}

void SymmFeatProxyLine::
DetectRotation(std::vector<SymmRotation::Ptr>& rotsymmvec)
{
    std::vector<SymmFeatSetLine::Ptr>& equivalentLines = symmDetCont->equivalentLines;

    rotsymmvec.clear();

    for (int ci = 0; ci < equivalentLines.size(); ++ci) {
        DetectRotation(equivalentLines[ci]->lset, rotsymmvec);
    }

    std::sort(rotsymmvec.begin(), rotsymmvec.end(),
        size_greater<SymmRotation::Ptr>);

    if (0 < rotsymmvec.size()) {
        float max_sz = rotsymmvec[0]->size();
        rotsymmvec.erase(
            std::remove_if(rotsymmvec.begin(), rotsymmvec.end(),
            boost::bind(size_less_value<SymmRotation::Ptr>,
            _1, max_sz, 0.6f)),
            rotsymmvec.end()
            );
    }
}

void SymmFeatProxyLine::
DetectRotation(const std::vector<SymmFeatLine::Ptr>& featArr,
               std::vector<SymmRotation::Ptr>& rotsymmvec)
{
    std::vector<bool> featChecked(featArr.size(), false);
    boost::array<size_t, 3> indx3;
    size_t featsize = featArr.size();

    if (symmDetCont->num_sample < featsize) {
        TupleSampler<mpcard, 3> sampler(featArr.size()-1);
        for (mpcard ii = 0; ii < symmDetCont->num_iter; ++ii) {
            if (!sampler.SampleUniqSort(indx3)) continue;
            if (featChecked[indx3[0]] || featChecked[indx3[1]] || featChecked[indx3[2]]) continue;
            detectRotation3P(featArr, indx3, featChecked, rotsymmvec);
        }
    } else {
        for (indx3[0] = 0; indx3[0] < featArr.size(); ++indx3[0]) {
            for (indx3[1] = indx3[0]+1; indx3[1] < featArr.size(); ++indx3[1]) {
                for (indx3[2] = indx3[1]+1; indx3[2] < featArr.size(); ++indx3[2]) {
                    if (featChecked[indx3[0]] || featChecked[indx3[1]] || featChecked[indx3[2]]) continue;
                    detectRotation3P(featArr, indx3, featChecked, rotsymmvec);
                }
            }
        }
    }
}

void SymmFeatProxyLine::
DetectReflection(std::vector<SymmReflective::Ptr>& refsymmvec)
{
    std::vector<SymmFeatLine::Ptr>& featArr = symmDetCont->featArr;
    std::vector<SymmFeatSetLine::Ptr>& equivalentLines = symmDetCont->equivalentLines;
    UnstructuredInCorePointCloudPtr lineFeatureUPC = symmDetCont->lineFeatureUPC;
    HierarchicalKNNIteratorPtr hItFeatures = symmDetCont->hItFeatures;

    refsymmvec.clear();

    std::vector<bool> linechecked(featArr.size(), false);
    PlaneGrid testedPlanes(4 * symmDetCont->grid_size, 1e-2);

    float sameline_th = symmDetCont->sameline_th;
    // start searching for reflective symmetries
    for (mpcard ci = 0; ci < equivalentLines.size(); ++ci) {
        std::vector<SymmFeatLine::Ptr>& eqLines = equivalentLines[ci]->lset;
        if (eqLines.size() < 2)
            continue;
        linechecked.assign(featArr.size(), false);

        for (size_t eql0 = 0; eql0 < eqLines.size(); ++eql0) {
            if (linechecked[eql0]) continue;
            for (size_t eql1 = eql0+1; eql1 < eqLines.size(); ++eql1) {
                if (linechecked[eql1]) continue;

                // construct reflective transformation
                Matrix3f T3; Vector3f N, C;
                {
                    if (!SymmReflective::GenerateReflection(
                        eqLines[eql0]->cen, eqLines[eql1]->cen, T3, N, C))
                        continue;
                    // skip tested transformations
                    if (!testedPlanes.Insert(N, C))
                        continue;
                }
                Matrix4f T4 = makeTranslation4f(C) * expand3To4(T3) * makeTranslation4f(-C);

                // score reflection
                float score = 0;
                std::map< pair<size_t, size_t>, LineDir > symmetricLines;
                for (mpcard lcp0 = 0; lcp0 < lineFeatureUPC->getNumPoints(); ++lcp0) {
                    SymmFeatLine::Ptr fl0 = featArr[lcp0];
                    SymmFeatLine::Ptr fl1;
                    Vector3f cen0 = fl0->cen;
                    Vector3f cen0_t = transformVector3f(T4, cen0);
                    hItFeatures->setSeekPointAndReset(cen0_t);
                    LineDir foundCounterPart = E_DIR;
                    mpcard lcp1;
                    while (!hItFeatures->atEnd() && E_DIR == foundCounterPart) {
                        lcp1 = hItFeatures->getCurrentPointIndex();
                        hItFeatures->next();
                        if (lcp0 == lcp1) continue;

                        fl1 = featArr[lcp1];
                        Vector3f fl1_pos0_t = transformVector3f(T4, fl1->pos0);
                        Vector3f fl1_pos1_t = transformVector3f(T4, fl1->pos1);
                        SymmFeatLine::Ptr fl1_t (new SymmFeatLine);
                        fl1_t->Setup(fl1_pos0_t, fl1_pos1_t,
                            fl1->surfaceN0, fl1->surfaceN1,
                            fl1->isBder);
                        fl1_t->edges = fl0->edges;
                        fl1_t->vertice = fl0->vertice;
                        foundCounterPart = fl0->SameAs(fl1_t, sameline_th);

                        //if (!fl0->isBder) continue;
                        //debugRenderer->beginRenderJob_OneFrame("border_", DR_FRAME++);
                        //debugRenderer->addLine(
                        //    fl0->cen, fl1->cen,
                        //    makeVector3f(1, 0, 0),
                        //    makeVector3f(0, 0, 1),
                        //    3);
                        //debugRenderer->addLine(
                        //    fl1_t->pos0, fl1_t->pos1,
                        //    makeVector3f(1, 0, 0),
                        //    makeVector3f(0, 0, 1),
                        //    3);
                        //debugRenderer->endRenderJob();
                        //debugOutput << fl1_t->isBder << "\n";
                    }
                    if (E_DIR == foundCounterPart) continue;

                    Vector3f d0 = fl0->cen - C;
                    (0 > d0 * N) ?
                        symmetricLines[make_pair(lcp0, lcp1)] = foundCounterPart :
                        symmetricLines[make_pair(lcp1, lcp0)] = foundCounterPart;
                    score += fl0->leng;
                    score += fl1->leng;
                }

                if (symmetricLines.size() >= 0.2f*eqLines.size()
                    || 20*symmDetCont->diagonal_length < score
                    ) {
                        //debugRenderer->beginRenderJob_OneFrame("reflection_", DR_FRAME++);
                        //for (size_t ii = 0; ii < lineFeatureUPC->getNumPoints(); ++ii) {
                        //    debugRenderer->addPoint(
                        //        transformVector3f(T4, featArr[ii]->cen),
                        //        makeVector3f(1, 1, 0)
                        //        );
                        //}
                        //debugRenderer->endRenderJob();
                        //debugOutput << symmetricLines.size() << "\n";

                        // compute oobb
                        SymmReflective::Ptr sym (new SymmReflective);
                        std::map< pair<size_t, size_t>, LineDir >::iterator ii = symmetricLines.begin();
                        for (; ii != symmetricLines.end(); ++ii) {
                            SymmFeatLinePair sflp = make_pair(
                                featArr[ii->first.first], featArr[ii->first.second]);
                            sym->Insert(sflp, ii->second);
                            //linechecked[ii->first] = true;
                            //linechecked[ii->second] = true;
                        }
                        sym->Initialize(T4, N);
                        refsymmvec.push_back(sym);
                }
            }
        }
    }

	std::sort(refsymmvec.begin(), refsymmvec.end(),
        size_greater<SymmReflective::Ptr>);

    if (0 < refsymmvec.size()) {
        float max_sz = refsymmvec[0]->size();
        refsymmvec.erase(
            std::remove_if(refsymmvec.begin(), refsymmvec.end(),
            boost::bind(size_less_value<SymmReflective::Ptr>,
            _1, max_sz, 0.6f)),
            refsymmvec.end()
            );
    }
}

void SymmFeatProxyLine::
PropagateReflectionPoints(std::vector<SymmReflective::Ptr>& refsymmvec)
{
    TrimeshStatic::Ptr smesh = symmDetCont->smesh;
    VertexArray va_ver(smesh->GetMesh()->getPointSet());
    UnstructuredInCorePointCloudPtr posUPC (new UnstructuredInCorePointCloud);
    posUPC->clearAndSetup(va_ver.getNumElements(), true, false, false, true);
    AAT posAAT = posUPC->getAAT("position");
    for (mpcard ii = 0; ii < va_ver.getNumElements(); ++ii) {
        posUPC->getPointSet()->set3f(ii,posAAT, va_ver.getPosition3f(ii));
    }
    HierarchicalKNNIteratorPtr hItFeatures 
        (new HierarchicalKNNIterator(&*posUPC, 32, nullptr));
    hItFeatures->setMaxDistanceToSeekPoint(symmDetCont->sameline_th);

    std::deque<mpcard> vertQue;
    std::vector<int> processed;
    std::vector<SymmFeatVertexPair> vertpairs;
    for (size_t ri = 0; ri < refsymmvec.size(); ++ri) {
        vertQue.clear();
        processed.assign(smesh->GetNumVerts(), -1);
        vertpairs.clear();
        std::vector<SymmFeatLinePair>& symmFeat = refsymmvec[ri]->featPairs_;
        for (size_t pi = 0; pi < symmFeat.size(); ++pi) {
            {
                mpcard vindx = symmFeat[pi].first->v0;
                if (0 > processed[vindx]) vertQue.push_back(vindx);
                processed[vindx] = 0;
                while (!vertQue.empty()) {
                    mpcard v1, v0 = vertQue.front();
                    vertQue.pop_front();
                    std::vector<mpcard> nverts = smesh->GetVert1RingVerts(v0);
                    for (size_t ii = 0; ii < nverts.size(); ++ii) {
                        if (0 <= processed[nverts[ii]]) continue;
                        vertQue.push_back(nverts[ii]);
                        processed[nverts[ii]] = 0;
                    }
                    if (0 < processed[v0]) continue;
                    Vector3f pos0 = smesh->GetVertPosition(v0);
                    if (refsymmvec[ri]->OnFrontSide(pos0)) {
                        Vector3f pos1 = refsymmvec[ri]->GetReflectedPos(pos0);
                        hItFeatures->setSeekPointAndReset(pos1);
                        if (!hItFeatures->atEnd()) {
                            v1 = hItFeatures->getCurrentPointIndex();
                            vertpairs.push_back(make_pair(v0, v1));
                            processed[v1] = 1;
                            for (hItFeatures->next(); !hItFeatures->atEnd(); hItFeatures->next()){
                                processed[hItFeatures->getCurrentPointIndex()] = 1;
                            }
                        }
                    }
                    processed[v0] = 1;
                }
            }
            {
                mpcard vindx = symmFeat[pi].first->v1;
                if (0 > processed[vindx]) vertQue.push_back(vindx);
                processed[vindx] = 0;
                while (!vertQue.empty()) {
                    mpcard v1, v0 = vertQue.front();
                    vertQue.pop_front();
                    std::vector<mpcard> nverts = smesh->GetVert1RingVerts(v0);
                    for (size_t ii = 0; ii < nverts.size(); ++ii) {
                        if (0 <= processed[nverts[ii]]) continue;
                        vertQue.push_back(nverts[ii]);
                        processed[nverts[ii]] = 0;
                    }
                    if (0 < processed[v0]) continue;
                    Vector3f pos0 = smesh->GetVertPosition(v0);
                    if (refsymmvec[ri]->OnFrontSide(pos0)) {
                        Vector3f pos1 = refsymmvec[ri]->GetReflectedPos(pos0);
                        hItFeatures->setSeekPointAndReset(pos1);
                        if (!hItFeatures->atEnd()) {
                            v1 = hItFeatures->getCurrentPointIndex();
                            vertpairs.push_back(make_pair(v0, v1));
                            processed[v1] = 1;
                            for (hItFeatures->next(); !hItFeatures->atEnd(); hItFeatures->next()){
                                processed[hItFeatures->getCurrentPointIndex()] = 1;
                            }
                        }
                    }
                    processed[v0] = 1;
                }
            }
        }
        refsymmvec[ri]->SetVertPairs(vertpairs);
    }
}

void SymmFeatProxyLine::
DetectSymmtry(
              std::vector<SymmRotation::Ptr>& rotsymmvec,
              std::vector<SymmTranslation::Ptr>& trasymmvec,
              std::vector<SymmReflective::Ptr>& refsymmvec)
{
}

std::vector<Matrix4f> SymmFeatProxyLine::generateTransformations(
    SymmFeatLine::Ptr l0, SymmFeatLine::Ptr l1)
{
    std::vector<Matrix4f> ret;

    Matrix3f frame0, frame1;
    Vector3f origin0, origin1;
    l0->GetFrame(frame0, origin0);
    l1->GetFrame(frame1, origin1);

    ret.push_back(
        makeTranslation4f(origin1) *
        expand3To4(frame1) * expand3To4(frame0).transpose() *
        makeTranslation4f(-origin0) );

    frame1[2] *= -1.0f;
    ret.push_back(
        makeTranslation4f(origin1) *
        expand3To4(frame1) * expand3To4(frame0).transpose() *
        makeTranslation4f(-origin0) );

    frame1[0] *= -1.0f;
    ret.push_back(
        makeTranslation4f(origin1) *
        expand3To4(frame1) * expand3To4(frame0).transpose() *
        makeTranslation4f(-origin0) );

    frame1[2] *= -1.0f;
    ret.push_back(
        makeTranslation4f(origin1) *
        expand3To4(frame1) * expand3To4(frame0).transpose() *
        makeTranslation4f(-origin0) );

    const float threshold = 0.01f;
    for (unsigned ii = 0; ii < ret.size(); ++ii) {
        unsigned numNearlyOnes = 0;
        for (unsigned j = 0; j < 3; ++j) {
            for (unsigned k = 0; k < 3; ++k) {
                if (fabs(fabs(ret[ii][j][k]) - 1.f) < threshold)
                    ++numNearlyOnes;
            }
        }
        if (3 != numNearlyOnes) continue;
        for (unsigned j = 0; j < 3; ++j) {
            for (unsigned k = 0; k < 3; ++k) {
                if (fabs(fabs(ret[ii][j][k]) - 1.f) < threshold)
                    ret[ii][j][k] = (ret[ii][j][k] < 0) ? -1 : 1;
                else ret[ii][j][k] = 0;
            }
        }
    }

    return ret;
}
