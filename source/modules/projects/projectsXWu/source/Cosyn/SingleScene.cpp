#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "SingleScene.h"
#include "Util\ColorSchemer.hpp"
#include "Util/SceneAndPointCloudTools.h"
#include <boost/math/special_functions/fpclassify.hpp>
//---------------------------------------------------------------------------
#include "VertexArray.h"
#include "ProgressWindow.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

SingleScene::SingleScene(const float& cellSize, const bool& verbose) : scan_data_(nullptr), scan_data_query_(nullptr),
    data_grid_(new GridType(cellSize)), curr_grid_(new GridType(cellSize)), verbose_(verbose)
{
    gr_model_.reset();
    empty_cell_energy_ = 10;

    addNullCandi();
}

SingleScene::~SingleScene()
{
    delete scan_data_;
    delete candi_pcs_[0];
    //for (UnstructuredInCorePointCloud* it : candi_pcs_) delete it;
    //delete scan_data_query_;
    for (HierarchicalKNNIterator* it : candi_pcs_query_) delete it;
    candi_pcs_query_.clear();
}

void SingleScene::addNullCandi(void)
{
    UnstructuredInCorePointCloud *tmpPC = new UnstructuredInCorePointCloud();
    tmpPC->clearAndSetup(0);
    candi_pcs_.push_back(tmpPC);
}

void SingleScene::SetScanData(PointSet* ps)
{
    delete scan_data_;
    scan_data_ = new UnstructuredInCorePointCloud;
    PointSet* tmpPS = (PointSet*)ps->copy();
    scan_data_->setPointSet(tmpPS);
    //candi_pcs_.push_back(pc);
}

void SingleScene::AddCandidate(UnstructuredInCorePointCloud* pc, const std::string& name)
{
    candi_pcs_.push_back(pc);
    candi_name_.push_back(name);
}

void SingleScene::BuildGrid(void)
{
    const float query_radius = curr_grid_->cell_size * sqrt(3) / 2;
    {
        VertexArray points(scan_data_);
        const unsigned num_points = points.getNumElements();
        for (unsigned jj = 0; jj < num_points; ++jj) {
            const Vector3f& pos = points.getPosition3f(jj);
            {
                GridKeyType idx;
                GridType::CellPtr node = data_grid_->AddPoint(pos, idx);
                node->points[0].push_back(jj);
            }
            //{
            //    GridKeyType idx;
            //    GridType::CellPtr node = curr_grid_->AddPoint(pos, idx);
            //    node->points[0].push_back(jj);
            //}
        }
    }
    if (scan_data_query_) delete scan_data_query_;
    scan_data_query_ = new HierarchicalKNNIterator(scan_data_, 32, nullptr);
    scan_data_query_->setMaxDistanceToSeekPoint(query_radius);

    for (HierarchicalKNNIterator* it : candi_pcs_query_) delete it;
    candi_pcs_query_.clear();
    for (unsigned pci = 0; pci < candi_pcs_.size(); ++pci) {
        UnstructuredInCorePointCloud* upc = candi_pcs_[pci];
        VertexArray points(upc);
        const unsigned num_points = points.getNumElements();
        for (unsigned jj = 0; jj < num_points; ++jj) {
            const Vector3f& pos = points.getPosition3f(jj);
            GridKeyType idx;
            GridType::CellPtr node = curr_grid_->AddPoint(pos, idx);
            node->points[pci].push_back(jj); // reserve label 0 to null data
        }
        HierarchicalKNNIterator* qit = new HierarchicalKNNIterator(upc, 32, nullptr);
        qit->setMaxDistanceToSeekPoint(query_radius);
        candi_pcs_query_.push_back(qit);
    }
    debugOutput << str( boost::format("constructed grid with %1% cells.\n")
        % curr_grid_->size()
        );
}

void SingleScene::VisualizeGrid(void)
{
    debugRenderer->beginRenderJob_OneFrame("candidate_patches_", DebugRenderer::DR_FRAME++);
    unsigned num_pc = candi_pcs_.size();
    for (unsigned ii = 0; ii < num_pc; ++ii) {
        UnstructuredInCorePointCloud* upc = candi_pcs_[ii];
        AAT posAAT = upc->getAAT( "position" );
        VertexArray points(upc);
        const unsigned num_points = points.getNumElements();
        for (unsigned jj = 0; jj < num_points; ++jj) {
            const Vector3f& pos = points.getPosition3f(jj);
            debugRenderer->addPoint(pos, ColorSchemer::GetColor(ii));
        }
    }
    debugRenderer->endRenderJob();

    debugRenderer->beginRenderJob_OneFrame("visualize_grid_", DebugRenderer::DR_FRAME++);
    //data_grid_->DrawWithDR(makeVector3f(1.f, 0.f, 0.f));
    curr_grid_->DrawWithDR(makeVector3f(0.2f, 0.2f, 0.2f));

    if (!gr_model_) return;
    SpaceType& gr_space = gr_model_->space();
    const size_t num_var = gr_space.numberOfVariables();
    for (size_t variable = 0; variable < num_var; ++variable) {
        const LabelType label = gr_space(variable);
        const Vector3f cell_cen = curr_grid_->GetCellCenter(variable);
        Matrix3f orientation; 
        orientation.setIdentity();
        const float scale = (curr_grid_->cell_size * 0.5f);
        const Vector3f color = (label == 0) ?
            makeVector3f(0.f, 0.f, 0.f) :
            ColorSchemer::GetColor(label-1);

        debugRenderer->addCenteredBox(cell_cen, orientation,
            makeVector3f(scale,scale,scale), color);		
    }
    debugRenderer->endRenderJob();
}

GraphModelPtr SingleScene::BuildGraphicalModel(void)
{
    const grid_map_type& cell_grid = curr_grid_->cell_grid;

    const size_t num_var = curr_grid_->size();
    const size_t spaceshape[] = { num_var };
    const size_t num_label = candi_pcs_.size();
    SpaceType::Ptr space (new SpaceType(spaceshape, spaceshape+1, num_label));
    GraphModelPtr gramodel(new GraphModelType(space));

    size_t num_factor_1 = 0, num_factor_2 = 0;

    progressWindow->pushStep(true, "building 1-factor ...");
    for (size_t variable = 0; variable < num_var; ++variable) {
        const size_t shape[] = { num_label };
        ExplicitFunctionType::Ptr fptr (new ExplicitFunctionType(shape, shape + 1));
        ExplicitFunctionType& f = *fptr;
        for (size_t state = 0; state < num_label; ++state) {
            f(state) = ComputeUnaryCost(variable, state);
        }
        size_t variableIndex[] = { variable };
        FactorType::Ptr factorPtr(new FactorType(variableIndex, variableIndex + 1));
        factorPtr->BindFunction(fptr);
        gramodel->addFactor(factorPtr);
        ++num_factor_1;
        progressWindow->progressf((float)(variable)/(float)(num_var-1));
    }
    progressWindow->popStep();

    progressWindow->pushStep(true, "building 2-factor ...");
    for (size_t variable = 0; variable < num_var; ++variable) {
        const GridKeyType& key = curr_grid_->GetCellIndex(variable);
        for (size_t j = 0; j < GridType::Dim; ++j) { // find neighbor connections
            GridKeyType key_n = key;
            ++key_n[j];
            if (cell_grid.count(key_n) == 0) continue;
            const size_t& variable_n = curr_grid_->GetSeqNumber(key_n);

            const size_t shape[] = {
                num_label,
                num_label,
            };
            ExplicitFunctionType::Ptr fptr (new ExplicitFunctionType(shape, shape + 2));
            ExplicitFunctionType& f = *fptr;
            for (size_t state1 = 0; state1 < num_label; ++state1) {
                for (size_t state2 = state1; state2 < num_label; ++state2) { // assume symmetric
                    f(state1, state2) = f(state2, state1) =
                        ComputeBinaryCost(variable, state1,
                        variable_n, state2);
                }
            }
            size_t variableIndex[] = { variable, variable_n };
            FactorType::Ptr factorPtr(new FactorType(variableIndex, variableIndex + 2));
            factorPtr->BindFunction(fptr);
            gramodel->addFactor(factorPtr);
            ++num_factor_2;
        }
        progressWindow->progressf((float)(variable)/(float)(num_var-1));
    }
    progressWindow->popStep();

    gramodel->buildFactorMap();
    debugOutput << str( boost::format("Built a graphical model with %1% 1-factors and %2% 2-factors.\n")
        % num_factor_1 % num_factor_2
        );

    gr_model_ = gramodel;
    return gramodel;
}		

ValueType SingleScene::DistDiff2Norm(
    UnstructuredInCorePointCloud* point_cloud,
    const GridType::Ptr& query_grid,
    const size_t& state,
    HierarchicalKNNIterator* qit,
    const GridType::Ptr& variable_grid,
    const size_t& variable
    )
{
    ValueType ret_value = 0;

    AAT posAAT = point_cloud->getAAT( "position" );
    VertexArray points(point_cloud);
    const GridKeyType& key = variable_grid->GetCellIndex(variable);
    if (!query_grid->Contains(key)) { // null cell, possible for scan data grid
        if (verbose_) {
            debugRenderer->addBoundingBox(variable_grid->GetCellBBox(variable), StandardColors<100>::color_red, 2);
        }
        return variable_grid->size(variable) * empty_cell_energy_; // high penulty for null space fill-in, or missing data
    } else {
        if (verbose_) {
            debugRenderer->addBoundingBox(variable_grid->GetCellBBox(variable), StandardColors<100>::color_green, 2);
        }
    }
    const std::deque<unsigned>& indices = query_grid->GetCellPtr(key)->points[state];
    for (unsigned ii : indices) {
        const Vector3f& pos = points.getPosition3f(ii);
        qit->setSeekPointAndReset(pos);
        while (!qit->atEnd()) {
            const Vector3f& pos_n = qit->get3f(posAAT);
            const GridKeyType& key_n = variable_grid->GetCellIndex(pos_n);
            if (key == key_n) {
                ret_value += normQuad(pos - pos_n);
                if (verbose_) {
                    debugRenderer->addLine(
                        pos, pos_n,
                        StandardColors<100>::color_yellow, StandardColors<100>::color_green,
                        3.0f);
                }
                break;
            } else {
                if (verbose_) {
                    debugRenderer->addLine(
                        pos, pos_n,
                        StandardColors<100>::color_yellow, StandardColors<100>::color_red,
                        3.0f);
                }
            }
            qit->next();
        }
        if (qit->atEnd()) {
            ret_value += empty_cell_energy_;
            if (verbose_) {
                debugRenderer->addPoint(pos, StandardColors<100>::color_magenta);
            }
        } else {
            if (verbose_) {
                debugRenderer->addPoint(pos, StandardColors<100>::color_yellow);
            }
        }
    }

    return ret_value;
}

ValueType SingleScene::ComputeUnaryCost(
    const size_t& variable, const size_t& state
    )
{
    ValueType ret_value = 0;

    UnstructuredInCorePointCloud* upc0 = scan_data_;
    UnstructuredInCorePointCloud* upc1 = candi_pcs_[state];

    if (verbose_) {
        //debugRenderer->beginRenderJob_OneFrame("1-factor_", DebugRenderer::DR_FRAME++);
        //{
        //    AAT posAAT = upc0->getAAT( "position" );
        //    VertexArray points(upc0);
        //    const unsigned num_points = points.getNumElements();
        //    for (unsigned jj = 0; jj < num_points; ++jj) {
        //        const Vector3f& pos = points.getPosition3f(jj);
        //        debugRenderer->addPoint(pos, StandardColors<100>::color_grey);
        //    }
        //}
        //{
        //    AAT posAAT = upc1->getAAT( "position" );
        //    VertexArray points(upc1);
        //    const unsigned num_points = points.getNumElements();
        //    for (unsigned jj = 0; jj < num_points; ++jj) {
        //        const Vector3f& pos = points.getPosition3f(jj);
        //        debugRenderer->addPoint(pos, StandardColors<100>::color_cyan);
        //    }
        //}
        //debugRenderer->endRenderJob();
        debugRenderer->beginRenderJob_OneFrame("1-factor_", DebugRenderer::DR_FRAME++);
    }

    { // in this cell, for each point in the candidate, query the scan data
        ret_value += DistDiff2Norm(
            upc1,
            curr_grid_,
            state,
            scan_data_query_,
            curr_grid_,
            variable
            );
    }

    { // in this cell, for each point in the scan data, query the candidate
        ret_value += DistDiff2Norm(
            upc0,
            data_grid_,
            0,
            candi_pcs_query_[state],
            curr_grid_,
            variable
            );
    }

    ret_value = sqrt(ret_value);

    if (verbose_) {
        debugOutput << str( boost::format("1-factor { %1% [%2%] }: %3%\n")
            % variable % state % ret_value
            );
    }
    if (verbose_) {
        debugRenderer->endRenderJob();
    }

    return ret_value;
}

ValueType SingleScene::ComputeBinaryCost(
    const size_t& variable1, const size_t& state1,
    const size_t& variable2, const size_t& state2
    )
{
    ValueType ret_value = 0;

    UnstructuredInCorePointCloud* upc1 = candi_pcs_[state1];
    UnstructuredInCorePointCloud* upc2 = candi_pcs_[state2];

    if (verbose_) {
        //debugRenderer->beginRenderJob_OneFrame("2-factor_", DebugRenderer::DR_FRAME++);
        //{
        //    AAT posAAT = upc1->getAAT( "position" );
        //    VertexArray points(upc1);
        //    const unsigned num_points = points.getNumElements();
        //    for (unsigned jj = 0; jj < num_points; ++jj) {
        //        const Vector3f& pos = points.getPosition3f(jj);
        //        debugRenderer->addPoint(pos, StandardColors<100>::color_yellow);
        //    }
        //}
        //{
        //    AAT posAAT = upc2->getAAT( "position" );
        //    VertexArray points(upc2);
        //    const unsigned num_points = points.getNumElements();
        //    for (unsigned jj = 0; jj < num_points; ++jj) {
        //        const Vector3f& pos = points.getPosition3f(jj);
        //        debugRenderer->addPoint(pos, StandardColors<100>::color_cyan);
        //    }
        //}
        //debugRenderer->endRenderJob();
        debugRenderer->beginRenderJob_OneFrame("1-factor_", DebugRenderer::DR_FRAME++);
    }

    if (state1 != state2) {
        /*************************************************************************/
        //ret_value = empty_cell_energy_; // Potts
        ret_value = empty_cell_energy_ *
            (curr_grid_->GetCellPtr(variable1)->size(state2) + curr_grid_->GetCellPtr(variable2)->size(state1));
        /*************************************************************************/

        //{ // in the 1st cell, for each point in the 2nd candidate, query the 1st candidate
        //    ret_value += DistDiff2Norm(
        //        upc2,
        //        curr_grid_,
        //        state2,
        //        candi_pcs_query_[state1],
        //        curr_grid_,
        //        variable1
        //        );
        //}

        //{ // in the 2nd cell, for each point in the 1st candidate, query the 2nd candidate
        //    ret_value += DistDiff2Norm(
        //        upc1,
        //        curr_grid_,
        //        state1,
        //        candi_pcs_query_[state2],
        //        curr_grid_,
        //        variable2
        //        );
        //}

        //ret_value = sqrt(ret_value);
    }

    if (verbose_) {
        debugOutput << str( boost::format("2-factor { %1% [%2%] - %3% [%4%] }: %5%\n")
            % variable1 % state1 % variable2 % state2 % ret_value
            );
    }
    if (verbose_) {
        debugRenderer->endRenderJob();
    }

    return ret_value;
}

UnstructuredInCorePointCloud* SingleScene::ExtractCurrentPC(void)
{
    // extract all points belong to a candidate, according to the result label
    const unsigned num_pc = candi_pcs_.size();
    std::vector< std::deque < unsigned > > cell_pc(num_pc);
    SpaceType& gr_space = gr_model_->space();
    const size_t num_var = gr_space.numberOfVariables();
    const size_t num_label = gr_space.numberOfLabels();
    unsigned num_points = 0;
    for (size_t variable = 0; variable < num_var; ++variable) {
        const LabelType label = gr_space(variable);
        const GridType::CellPtr& cell_ptr = curr_grid_->GetCellPtr(variable);
        const std::deque < unsigned >& source = cell_ptr->points[label];
        std::deque < unsigned >& target = cell_pc[label];
        std::copy(source.begin(), source.end(), std::inserter(target, target.end()));
        num_points += source.size();
    }

    // fill-in scan data
    const unsigned num_scan_cell = data_grid_->size();
    VertexArray data_points(scan_data_);
    for (unsigned ci = 0; ci < num_scan_cell; ++ci) {
        const GridKeyType& key = data_grid_->GetCellIndex(ci);
        if (curr_grid_->Contains(key)) { // do not fill repeated data
            const LabelType label = gr_space(curr_grid_->GetSeqNumber(key));
            if (curr_grid_->GetCellPtr(key)->size(label) >= data_grid_->size(ci)) continue;
        }
        const std::deque < unsigned >& source = data_grid_->GetCellPtr(key)->points[0];
        std::deque < unsigned >& target = cell_pc[0];
        std::copy(source.begin(), source.end(), std::inserter(target, target.end()));
        num_points += source.size();
    }

    UnstructuredInCorePointCloud * ret_upc = new UnstructuredInCorePointCloud;
    ret_upc->clearAndSetup(scan_data_->getDescr(), num_points);
    checkAttribute( ret_upc, "color", 3, VAD::DATA_FORMAT_FLOAT32 ) ;
    AAT posAAT = ret_upc->getAAT("position");
    AAT normalAAT = ret_upc->getAAT("normal");
    AAT colorAAT = ret_upc->getAAT("color");
    VertexArray target_points(ret_upc);

    // mix all the candidate
    unsigned pindex = 0;
    for (unsigned ii = 1; ii < num_pc; ++ii) {
        const std::deque < unsigned >& indices = cell_pc[ii];
        if (indices.empty()) continue;

        const Vector3f color = (ii == 0) ?
            makeVector3f(0.f, 0.f, 0.f) :
            ColorSchemer::GetColor(ii-1);
        UnstructuredInCorePointCloud* source_upc = candi_pcs_[ii];
        VertexArray source_points(source_upc);
        for (unsigned jj : indices) {
            target_points.setPosition3f(pindex, source_points.getPosition3f(jj));
            target_points.setNormal3f(pindex, source_points.getNormal3f(jj));
            target_points.setColor3f(pindex, color);
            ++ pindex;
        }
    }
    const std::deque < unsigned >& indices = cell_pc[0];
    if (!indices.empty())
    {
        const Vector3f color = StandardColors<100>::color_grey;
        for (unsigned jj : indices) {
            target_points.setPosition3f(pindex, data_points.getPosition3f(jj));
            target_points.setNormal3f(pindex, data_points.getNormal3f(jj));
            target_points.setColor3f(pindex, color);
            ++ pindex;
        }
    }

    return ret_upc;
}
