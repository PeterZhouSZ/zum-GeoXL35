#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "GraphicalModelProxy.h"
#include "Util/gco-v3.0/GCoptimization.h"
//---------------------------------------------------------------------------
#include "ProgressWindow.h"
//---------------------------------------------------------------------------

ValueType MRFLibFunctionPointer::DataCostFn(IndexType site, LabelType lab, GraphModelPtr graphModel)
{
    const IndexType shape[] = {
        site
    };
    FactorType::Ptr factor;
    bool ret = graphModel->getFactor(shape, shape + 1, factor);
    if (!ret)
        error(str(boost::format("MRFLibFunctionPointer - 1-factor not found: %1%.") % site));
    ExplicitFunctionType& f = *(boost::dynamic_pointer_cast<ExplicitFunctionType>(factor->getFunction()));
    return f(lab);
}

ValueType MRFLibFunctionPointer::SmoothCostFn(IndexType site1, IndexType site2, LabelType l1, LabelType l2, GraphModelPtr graphModel)
{
    const IndexType shape[] = {
        site1, site2
    };
    FactorType::Ptr factor;
    bool ret = graphModel->getFactor(shape, shape + 2, factor);
    if (!ret)
        error(str(boost::format("MRFLibFunctionPointer - 2-factor not found: %1%, %2%.") % site1 % site2));
    ExplicitFunctionType& f = *(boost::dynamic_pointer_cast<ExplicitFunctionType>(factor->getFunction()));
    return f(l1, l2);
}

ValueType MRFLibFunctionPointer::DataCostFn(IndexType site, LabelType lab, void* pGM)
{
    GraphModelType* graphModel = static_cast<GraphModelType*>(pGM);

    const IndexType shape[] = {
        site
    };
    FactorType::Ptr factor;
    bool ret = graphModel->getFactor(shape, shape + 1, factor);
    if (!ret)
        error(str(boost::format("MRFLibFunctionPointer - 1-factor not found: %1%.") % site));
    ExplicitFunctionType& f = *(boost::dynamic_pointer_cast<ExplicitFunctionType>(factor->getFunction()));
    return f(lab);
}

ValueType MRFLibFunctionPointer::SmoothCostFn(IndexType site1, IndexType site2, LabelType l1, LabelType l2, void* pGM)
{
    GraphModelType* graphModel = static_cast<GraphModelType*>(pGM);

    const IndexType shape[] = {
        site1, site2
    };
    FactorType::Ptr factor;
    bool ret = graphModel->getFactor(shape, shape + 2, factor);
    if (!ret)
        error(str(boost::format("MRFLibFunctionPointer - 2-factor not found: %1%, %2%.") % site1 % site2));
    ExplicitFunctionType& f = *(boost::dynamic_pointer_cast<ExplicitFunctionType>(factor->getFunction()));
    return f(l1, l2);
}

ValueType MRFLibFunctionPointer::SmoothCostPotts(IndexType site1, IndexType site2, LabelType l1, LabelType l2)
{
    if (l1 == l2) return 0;
    else return 10;
}

//============================================================================

GraphicalModelProxy::GraphicalModelProxy(void)
{

}

GraphicalModelProxy::~GraphicalModelProxy(void)
{

}

// NOTE: graph-cut code may swallow error message, and terminate silently!
// If it terminates quickly without changing anything, take special care!
// Example: energy value is too large and might overflow.
void GraphicalModelProxy::Inference(GraphModelPtr& graphModel, ValueType* dataCost)
{
    progressWindow->pushStep(true, "optimizing ...");

    size_t numVar = graphModel->numberOfVariables();
    size_t numLabel = graphModel->numberOfLabels(0);
    int numIter = 20;

    GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph(
        numVar, numLabel);
    for (unsigned var = 0; var < numVar; ++var) gc->setLabel(var, 0);
    //gc->setDataCost(&dataCost_[0]);
    //gc->setSmoothCost(&pairCost[0]);
    if (nullptr != dataCost) gc->setDataCost(dataCost);
    else gc->setDataCost(MRFLibFunctionPointer::DataCostFn, (void*)(&*graphModel));
    gc->setSmoothCost(MRFLibFunctionPointer::SmoothCostFn, (void*)(&*graphModel));
    //gc->setSmoothCost(MRFLibFunctionPointer::SmoothCostPotts);
    for (size_t fi = 0; fi < graphModel->numberOfFactors(); ++fi) {
        FactorType& factor = *(*graphModel)[fi];
        std::vector<IndexType> indices = factor.variableIndices();
        if (indices.size() != 2) continue;
        gc->setNeighbors(indices[0], indices[1]);

        if (0)
        {
            ExplicitFunctionType& f = *(boost::dynamic_pointer_cast<ExplicitFunctionType>(factor.getFunction()));
            debugOutput << indices[0] << " - " << indices[1] << ": ";
            debugOutput << graphModel->buildFactorKey(indices.begin(), indices.end());
            //for (LabelType l1 = 0; l1 < numLabel; ++l1) {
            //    for (LabelType l2 = 0; l2 < numLabel; ++l2) {
            //        debugOutput << f(l1, l2) << " ";
            //    }
            //}
            debugOutput << "\n";
        }
    }
    //gc->setVerbosity(0);
    ValueType E_smooth, E_data;
    E_smooth = gc->giveSmoothEnergy();
    E_data = gc->giveDataEnergy();
    debugOutput << str(boost::format("Total Energy (before optimization) = %1% (Smoothness energy %2%, Data Energy %3%)\n")
        % (double)(E_smooth + E_data) % (double)E_smooth % (double)E_data
        );
    try {
        gc->swap(numIter);
    }
    catch (const std::string& ex) {
        throw PException("GraphicalModelProxy::Inference - " + ex);
    }
    catch (...) {
        throw PException("GraphicalModelProxy::Inference - unknown exception in the mrf optimization process");
    }
    //try {
    //    gc->expansion(numIter);
    //}
    //catch (const std::string& ex) {
    //    debugOutput << ex << "\n";
    //    throw;
    //}
    //catch (...) {
    //    try {
    //        debugOutput << "NOTICE: expansion failed, switching to swap";
    //        for (unsigned var = 0; var < numVar; ++var) gc->setLabel(var, 0);
    //        gc->swap(numIter);
    //    }
    //    catch (const std::string& ex) {
    //        throw PException("GraphicalModelProxy::Inference - " + ex);
    //    }
    //    catch (...) {
    //        throw PException("GraphicalModelProxy::Inference - unknown exception in the mrf optimization process");
    //    }
    //}
    E_smooth = gc->giveSmoothEnergy();
    E_data = gc->giveDataEnergy();
    debugOutput << str(boost::format("Total Energy (after optimization) = %1% (Smoothness energy %2%, Data Energy %3%)\n")
        % (double)(E_smooth + E_data) % (double)E_smooth % (double)E_data
        );
    SpaceType& gr_space = graphModel->space();
    for (int var = 0; var < numVar; ++var) {
        gr_space(var) = gc->whatLabel(var);
        //debugOutput << str( boost::format("Label of site %1% is %2%\n")
        //    % var % gc->whatLabel(var)
        //    );
    }
    delete gc;
}
