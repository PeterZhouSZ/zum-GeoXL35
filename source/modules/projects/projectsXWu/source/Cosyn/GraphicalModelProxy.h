#ifndef GraphicalModelProxy_H
#define GraphicalModelProxy_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "SparseNGrid.h"
#include "GraphicalModel/GraphicalModel.hpp"
//---------------------------------------------------------------------------

class GraphicalModelProxy
{
public:
    typedef boost::shared_ptr< GraphicalModelProxy > Ptr;
    typedef boost::shared_ptr< const GraphicalModelProxy > ConstPtr;

public:
    typedef float ValueType;
    typedef int IndexType;
    typedef int LabelType;
    typedef GraphicalSpace<IndexType, LabelType> SpaceType;
    typedef GraphicalModel<ValueType, SpaceType > GraphModelType;
    typedef boost::shared_ptr< GraphModelType > GraphModelPtr;
    typedef GraphicalFactor<GraphModelType> FactorType;
    typedef ExplicitFunction<ValueType, IndexType, LabelType> ExplicitFunctionType;

    typedef LabelGrid GridType;

public:
    GraphicalModelProxy(void);
    ~GraphicalModelProxy(void);

    static void Inference(GraphModelPtr& graphModel, ValueType* dataCost = nullptr);

public:
};

typedef GraphicalModelProxy::ValueType ValueType;
typedef GraphicalModelProxy::IndexType IndexType;
typedef GraphicalModelProxy::LabelType LabelType;
typedef GraphicalModelProxy::SpaceType SpaceType;
typedef GraphicalModelProxy::GraphModelType GraphModelType;
typedef GraphicalModelProxy::GraphModelPtr GraphModelPtr;
typedef GraphicalModelProxy::FactorType FactorType;
typedef GraphicalModelProxy::ExplicitFunctionType ExplicitFunctionType;

//============================================================================

class MRFLibFunctionPointer
{
    //typedef boost::function< ValueType(int site, LabelType lab) > data_cost_fn;
    //data_cost_fn data_fn = boost::bind(&MRFLibFunctionPointer::DataCostFn, _1, _2, graphModel);
    //typedef boost::function< ValueType(int site1, int site2, LabelType l1, LabelType l2) > smooth_cost_fn;
    //smooth_cost_fn smooth_fn = boost::bind(&MRFLibFunctionPointer::SmoothCostFn, _1, _2, _3, _4, graphModel);

public:
    MRFLibFunctionPointer(GraphModelPtr graphModel) : graphModel_(graphModel) {}

    static ValueType DataCostFn(IndexType site, LabelType lab, GraphModelPtr graphModel);

    static ValueType SmoothCostFn(IndexType site1, IndexType site2, LabelType l1, LabelType l2, GraphModelPtr graphModel);

    static ValueType DataCostFn(IndexType site, LabelType lab, void* pGM);

    static ValueType SmoothCostFn(IndexType site1, IndexType site2, LabelType l1, LabelType l2, void* pGM);

    static ValueType SmoothCostPotts(IndexType site1, IndexType site2, LabelType l1, LabelType l2);

public:
    GraphModelPtr graphModel_;
};

#endif
