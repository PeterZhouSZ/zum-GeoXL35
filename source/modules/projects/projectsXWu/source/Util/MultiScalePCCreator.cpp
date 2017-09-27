//---------------------------------------------------------------------------
#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "MultiScalePCCreator.h"
//---------------------------------------------------------------------------
#include "PCCTriangleMeshSampler.h"
#include "ProgressWindow.h"
#include "TopologyRangeSearch.h"
#include "Monomials2ndOrder.h"
#include <omp.h>
#include "FastSphereQuerry.h"
#include "PCCComputeTopology.h"
#include "InCorePCTopologyGraph.h"
#include "PCCResampler.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

IMPLEMENT_CLASS( MultiScalePCCreator , 0 )
{
    BEGIN_CLASS_INIT( MultiScalePCCreator );
    ADD_OBJECT_PROP(params, 0, MultiScalePCParams::getClass(), true);
    ADD_OBJECT_PROP(smoother, 0, PointCloudSmoother::getClass(), true);

    ADD_FLOAT32_PROP(grid_spacing_factor, 0);
    ADD_FLOAT32_PROP(top_ring_factor, 0);
    ADD_BOOLEAN_PROP(use_top_expansion, 0);
    ADD_BOOLEAN_PROP(debugdraw, 0);
}

IMPLEMENT_CLASS( SimpleFeaturePointCloudSmoother, 0 )
{
    BEGIN_CLASS_INIT( SimpleFeaturePointCloudSmoother )
}

MultiScalePCCreator::MultiScalePCCreator()
{
    grid_spacing_factor = 0.001f;
    top_ring_factor = 4;
    params = new MultiScalePCParams;
    params->numLevels = 1;
    smoother = new SimpleFeaturePointCloudSmoother();
    use_top_expansion = true;
    debugdraw = false;
}

MultiScalePCCreator::~MultiScalePCCreator()
{
    delete params;
    delete smoother;
}

MultiScaleInCorePointCloud* MultiScalePCCreator::convert(PointCloud *pc)
{
    UnstructuredInCoreTriangleMesh *mesh = dynamic_cast<UnstructuredInCoreTriangleMesh*>(pc);
    UnstructuredInCorePointCloud   *upc  = dynamic_cast<UnstructuredInCorePointCloud*>(pc);
    MultiScaleInCorePointCloud     *resultPC = nullptr;
    if (mesh) {
        const float diag_length = mesh->getPointSet()->getBoundingBox().getDiagonalLength();
        params->baseGridSpacing = diag_length * grid_spacing_factor;
        debugOutput << str( boost::format("base grid spacing changed to %1%, which is %2%%% diagonal length (%3%).\n")
            % params->baseGridSpacing % (grid_spacing_factor*100) % diag_length
            );
        resultPC = createPointCloudFromMesh(mesh);
    } else if (upc) {
        const float diag_length = upc->getPointSet()->getBoundingBox().getDiagonalLength();
        params->baseGridSpacing = diag_length * grid_spacing_factor;
        debugOutput << str( boost::format("base grid spacing changed to %1%, which is %2%%% diagonal length (%3%).\n")
            % params->baseGridSpacing % (grid_spacing_factor*100) % diag_length
            );
        resultPC = createPointCloudFromPointCloud(upc);
    }
    return resultPC;
}

MultiScaleInCorePointCloud* MultiScalePCCreator::createPointCloudFromMesh( UnstructuredInCoreTriangleMesh *mesh )
{
    debugOutput << "MultiScalePCCreator::createPointCloudFromMesh - converting triangle mesh.\n";
    UnstructuredInCorePointCloud *pc = PCCTriangleMeshSampler::sampleMeshPoisson(mesh, params->baseGridSpacing, -1);
    MultiScaleInCorePointCloud *result = createPointCloudFromPointCloud(pc);
    delete pc;
    return result;
}

MultiScaleInCorePointCloud* MultiScalePCCreator::createPointCloudFromPointCloud( UnstructuredInCorePointCloud* pc )
{
    const float medpd = getMedianPointDistance(pc);
    float& base_grid_spacing = params->baseGridSpacing;
    if (base_grid_spacing < medpd - std::numeric_limits<float>::epsilon()) {
        base_grid_spacing = medpd;
        debugOutput << str( boost::format("base grid spacing changed to %1%, which is the median point distance.\n")
            % params->baseGridSpacing
            );
    } else if (base_grid_spacing > medpd + std::numeric_limits<float>::epsilon()) {
        PCCResampler resampler;
        resampler.resamplePC_ar(pc, base_grid_spacing);
        debugOutput << str( boost::format("point cloud downsampled to base grid spacing %1%.\n")
            % params->baseGridSpacing
            );
    }

    MultiScaleInCorePointCloud *result = new MultiScaleInCorePointCloud();
    const PointSet *ps = pc->getPointSet();
    UnstructuredInCorePointCloud *tmpPC = new UnstructuredInCorePointCloud();
    tmpPC->setPointSet((PointSet*)ps->copy());
    computeCurv2forPC(tmpPC, params->baseGridSpacing, top_ring_factor, use_top_expansion, debugdraw);	
    result->clearAndSetup(tmpPC->getDescr());
    result->addPointSet(tmpPC->getPointSet());
    delete tmpPC;

    result->build(params, smoother);
    return result;
}

std::vector<mpcard> ExpandKdepNN(
    //FastSphereQuerry &query, Vector3f pos, const float query_radius, const PointSet* ps,
    InCorePCTopologyGraph* tpg, const unsigned& pindex,
    const unsigned& depnn)
{
    //const unsigned num_pts = ps->getNumEntries();
    //AAT posAAT = ps->getAAT( "position" );
    //mpcard numPoints;
    //mpcard *pts;
    std::vector<mpcard> output;
    std::set<mpcard> nnset;
    const unsigned num_pts = tpg->getNumVertices();
    for (unsigned ni = 0; ni < tpg->getNumAdjacentVertices(pindex); ++ni) {
        nnset.insert(tpg->getAdjacentVertex(pindex, ni));
    }
    //query.querry(pos, query_radius, &pts, numPoints);
    //for (unsigned ii = 0; ii < numPoints; ++ii) {
    //    nnset.insert(pts[ii]);
    //}
    std::set<mpcard> next_stack(nnset);
    std::set<mpcard> expansion_stack(nnset);
    for (unsigned step = 0; step < depnn; ++step) {
        if (nnset.size() == num_pts) break;
        for (mpcard ei : expansion_stack) {
            //const Vector3f& pos = ps->get3f(ei, posAAT);
            //query.querry(pos, query_radius, &pts, numPoints);
            //for (size_t ii = 0; ii < numPoints; ++ii) {
            //    next_stack.insert(pts[ii]);
            //}
            for (unsigned ni = 0; ni < tpg->getNumAdjacentVertices(ei); ++ni) {
                nnset.insert(tpg->getAdjacentVertex(ei, ni));
            }
        }
        expansion_stack.clear();
        std::set_difference(
            next_stack.begin(), next_stack.end(),
            nnset.begin(), nnset.end(),
            std::inserter(expansion_stack, expansion_stack.end()));
        for (mpcard ii : expansion_stack) nnset.insert(ii);
    }
    std::copy(nnset.begin(), nnset.end(), std::back_inserter(output));
    return output;
}

void MultiScalePCCreator::computeCurv2forPC(PointCloud* pc, float base_sigma, float top_ring_factor, bool use_top_expansion, bool debugdraw)
{
    //X4_TIMER_START(compute_curvature);

    checkAttribute( pc, "curv2", 2, VAD::DATA_FORMAT_FLOAT32 );
    checkAttribute( pc, "tangent_u", 3, VAD::DATA_FORMAT_FLOAT32 );
    checkAttribute( pc, "tangent_v", 3, VAD::DATA_FORMAT_FLOAT32 );

    AAT posAAT = pc->getAAT( "position" );
    AAT normalAAT = pc->getAAT( "normal" );
    AAT curv2AAT = pc->getAAT( "curv2" );
    AAT tangent_uAAT = pc->getAAT( "tangent_u" );
    AAT tangent_vAAT = pc->getAAT( "tangent_v" );

    SceneObjectIterator *iter = pc->createIterator( SOIT::CAP_BASIC_PC | SOIT::CAP_MODIFYABLE_PC );
    pAssert( iter != nullptr );
    BasicPointCloudIterator *bIt = dynamic_cast<BasicPointCloudIterator*>(iter);
    ModifyablePointCloudIterator *mIt = dynamic_cast<ModifyablePointCloudIterator*>(iter);
    IndexedPointCloudIterator *indIt = dynamic_cast<IndexedPointCloudIterator*>(iter);
    const PointSet *ps = bIt->lockBlockRead();
    const float top_radius = base_sigma * 2.1f; // sqrt(2.f);
    const float query_radius = top_radius * top_ring_factor;
    debugOutput << str( boost::format("computing curvature: base sigma: %1%, top radius %2%, query radius %3%.\n")
        % base_sigma % top_radius % query_radius
        );

    InCorePCTopologyGraph* tpg = nullptr;
    TopNeighborhoodSphereSearch* query_top = nullptr;
    FastSphereQuerry* query = nullptr;
    if (use_top_expansion) {
        PCCComputeTopology cmd;
        cmd.setup(PCCComputeTopology::TOPTYPE_EPS, top_radius);
        tpg = cmd.computeEpsTopology(ps, pc);
        query_top = new TopNeighborhoodSphereSearch;
    } else {
        query = new FastSphereQuerry(pc);
    }

    card32 index = 0;
    progressWindow->pushStep(true, "curv2");
    std::vector<Vector3f> posBuffer;
    while( !bIt->atEnd() )
    {
        Vector3f pos = bIt->get3f( posAAT );
        mpcard numPoints;
        mpcard *pts;
        std::vector<mpcard> nnvec;
        if (use_top_expansion) {
            //unsigned depnn = 3;
            //nnvec = ExpandKdepNN(query, pos, query_radius, ps, depnn);
            //nnvec = ExpandKdepNN(tpg, index, depnn);
            query_top->setup(tpg, ps, (card32)indIt->getCurrentPointIndex(), pos, query_radius, posAAT);
            nnvec.clear();
            while (!query_top->atEnd()) {
                nnvec.push_back(query_top->getCurrentIndex());
                query_top->next();
            }
            numPoints = nnvec.size();
            pts = &nnvec[0];
        } else {
            query->querry(pos, query_radius, &pts, numPoints);
        }
        if (numPoints == 0) {
            bIt->next();
            index++;
            continue;
        }

        Vector3f pcaNormal;
        //if (usePCANormal) {
        PCA<float,3> pca;
        for (card32 i=0; i<numPoints; i++) {
            Vector3f sample_pos = ps->get3f(pts[i], posAAT);
            float dist_2 = normQuad(sample_pos - pos);
            float w = exp(-dist_2/(2*base_sigma*base_sigma));
            pca.addPoint(sample_pos, w);
        }
        Vector3f eigenValues,centroid;
        Matrix3f eigenVectors;
        {
            pca.analyze(eigenValues,eigenVectors,centroid);
            if( eigenVectors[2] * bIt->get3f(normalAAT) < 0.0f )
                eigenVectors *= -1.0f;
            pcaNormal = normalize(eigenVectors[2]);
            mIt->set3f(normalAAT, pcaNormal);
        }
        //} else {
        //   pcaNormal = bIt->get3f(normalAAT);
        //}
        Matrix3f tangentSystem = calcTangentSystem( pcaNormal );
        Matrix3f invTangentSystem = tangentSystem.transpose();

        if( posBuffer.size() < numPoints )
        {
            posBuffer.resize(numPoints);
        }

        // transform to a canonical frame
        for( card32 i=0;i<numPoints;i++ )
        {
            Vector3f tmp = ps->get3f(pts[i], posAAT) - pos;
            //posBuffer[i] = invTangentSystem * tmp / (sqrt(2.0f)*baseSigma);
            posBuffer[i] = invTangentSystem * tmp;
        }

        // compute second fundamental form
        StaticVector<float32, 6> coeff;
        Monomials2ndOrder::computeCoeffs( &posBuffer[0], (card32)numPoints, coeff, 1.0f );

        float b_u = coeff[1];
        float b_v = coeff[2];
        float A_uu = coeff[3];
        float A_uv = coeff[4];
        float A_vv = coeff[5];

        // eigen values corresponds to principal curvature
        Matrix2f A = makeMatrix2f(A_uu, A_uv,
            A_uv, A_vv);
        Vector2f lambda;
        Matrix2f eVects;
        A.computeEigenStructure(lambda, eVects);

        float32 lambda1 = lambda[0];
        float32 lambda2 = lambda[1];

        Vector2f v1 = eVects[0];
        Vector2f v2 = eVects[1];

        if (fabs(lambda1) < fabs(lambda2)) {
            std::swap(lambda1, lambda2);
            std::swap(v1, v2);
        }

        // make right handed coordinate system to preserve sign of curvature
        if (determinant(v1, v2) < 0) {
            v2 = -v2;
        }

        // transform back to tangential frame
        Vector3f newTangentU = tangentSystem[0] * v1[0] + tangentSystem[1] * v1[1];
        Vector3f newTangentV = tangentSystem[0] * v2[0] + tangentSystem[1] * v2[1];

        //// take eigen std::vector for pencil neighborhood shape
        //if (eigenValues[0] > 2.5f*eigenValues[1] && eigenValues[1] < 2.f*eigenValues[2]) {
        //    newTangentV = eigenVectors[0];
        //    lambda1 = eigenValues[0]*5000.f; // empirical value
        //}

        mIt->set2f(curv2AAT, makeVector2f(lambda1, lambda2));
        mIt->set3f(tangent_uAAT, newTangentU);
        mIt->set3f(tangent_vAAT, newTangentV);
        index++;
        bIt->next();

        if (debugdraw
            && abs(lambda1) > 0.1f
            && abs(lambda1) / abs(lambda2) > 2
            ) {
                debugRenderer->beginRenderJob_OneFrame("curv2_query_", DR_FRAME++);
                debugRenderer->addPoint(pos, makeVector3f(1, 0, 0));
                for (card32 i=0; i<numPoints; i++) {
                    Vector3f sample_pos = ps->get3f(pts[i], posAAT);
                    debugRenderer->addPoint(sample_pos, makeVector3f(0, 1, 0));
                }
                debugRenderer->addLine(
                    centroid, centroid+5000.f*eigenVectors[0]*eigenValues[0],
                    makeVector3f(1, 0, 0), makeVector3f(1, 0, 0),
                    2);
                debugRenderer->addLine(
                    centroid, centroid+5000.f*eigenVectors[1]*eigenValues[1],
                    makeVector3f(0, 1, 0), makeVector3f(0, 1, 0),
                    2);
                debugRenderer->addLine(
                    centroid, centroid+5000.f*eigenVectors[2]*eigenValues[2],
                    makeVector3f(0, 0, 1), makeVector3f(0, 0, 1),
                    2);
                //debugOutput << eigenValues[0] << "\t" << eigenValues[1] << "\t";
                debugRenderer->addLine(
                    pos, pos+.5f*newTangentU*lambda1,
                    makeVector3f(1, 1, 0), makeVector3f(1, 1, 0),
                    2);
                debugRenderer->addLine(
                    pos, pos+.5f*newTangentV*lambda2,
                    makeVector3f(0, 1, 1), makeVector3f(0, 1, 1),
                    2);
                //debugOutput << lambda1 << "\t" << lambda2 << "\n";
                debugRenderer->endRenderJob();
        }

        if( index % 1000 == 0) {
            // TODO (daniel): Canceling the process from the progressWindow doesn't work with OpenMP...
            progressWindow->progress((float)index/(float)pc->getNumPoints()*100.0f);
        }
    }
    progressWindow->popStep();

    bIt->unlock();
    delete iter;
    delete tpg;
    delete query_top;
    delete query;
    //X4_TIMER_STOP(compute_curvature);
}

UnstructuredInCorePointCloud * SimpleFeaturePointCloudSmoother::createSmoothed( 
    UnstructuredInCorePointCloud *input, float32 currentGridSpacing, float32 smoothingFactor ) const
{
    UnstructuredInCorePointCloud *result = GaussianPointCloudSmoother::createSmoothed(input, currentGridSpacing, smoothingFactor);
    PointSet *ps = result->getPointSet();
    if (ps->providesAttribute("normal", 3, VAD::DATA_FORMAT_FLOAT32)) {
        AAT normalsAAT = ps->getAAT("normal");
        mpcard n=ps->getNumEntries();
        for (mpcard i=0; i<n; i++) {

            ps->set3f(i, normalsAAT, normalize(ps->get3f(i,normalsAAT)));
        }
    } else {
        warning("SimpleFeaturePointCloudSmoother::createSmoothed - no normals found()");
    }
    MultiScalePCCreator::computeCurv2forPC(result, currentGridSpacing*smoothingFactor);
    return result;
}
