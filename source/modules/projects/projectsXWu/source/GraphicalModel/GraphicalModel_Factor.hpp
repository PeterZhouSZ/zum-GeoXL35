#ifndef GRAPHICALMODEL_FACTOR_HPP
#define GRAPHICALMODEL_FACTOR_HPP
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "GraphicalModel_Function.hpp"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

template<class MODEL>
class GraphicalFactor {
public:
    typedef boost::shared_ptr< GraphicalFactor > Ptr;
    typedef boost::shared_ptr< const GraphicalFactor > ConstPtr;

    typedef MODEL GraphicalModelType;
    typedef boost::shared_ptr< MODEL > GraphicalModelTypePtr;

    typedef typename GraphicalModelType::ValueType ValueType;
    typedef typename GraphicalModelType::IndexType IndexType;
    typedef typename GraphicalModelType::LabelType LabelType;

    typedef GraphicalFunction<ValueType, IndexType, LabelType> FunctionType;
    typedef typename FunctionType::Ptr FunctionTypePtr;

    //typedef std::string FactorKey;
    typedef typename std::deque<typename IndexType>::iterator Iterator;

public:
    template<class ITERATOR>
    GraphicalFactor(ITERATOR shapeBegin, ITERATOR shapeEnd);

    void BindFunction(FunctionTypePtr f);
    FunctionTypePtr getFunction();

    size_t size() const;
    Iterator begin() const;
    Iterator end() const;
    std::vector<IndexType> variableIndices() const;

    //FactorKey BuildKey(void) const;

private:
    FunctionTypePtr funtion_;
    std::deque<IndexType> variableIndices_; // indices of corresponding variables
    //FactorKey key_;
};

template<class MODEL>
template<class ITERATOR>
GraphicalFactor<MODEL>::GraphicalFactor(
    ITERATOR shapeBegin, ITERATOR shapeEnd
    )
{
    while (shapeBegin != shapeEnd){
        variableIndices_.push_back(*shapeBegin);
        ++shapeBegin;
    }
    //key_ = BuildKey();
}

template<class MODEL>
size_t GraphicalFactor<MODEL>::size() const
{
    return variableIndices_.size();
}

template<class MODEL>
typename GraphicalFactor<MODEL>::Iterator
    GraphicalFactor<MODEL>::begin() const
{
    return variableIndices_.begin();
}

template<class MODEL>
typename GraphicalFactor<MODEL>::Iterator
    GraphicalFactor<MODEL>::end() const
{
    return variableIndices_.end();
}

template<class MODEL>
std::vector<typename GraphicalFactor<MODEL>::IndexType>
    GraphicalFactor<MODEL>::variableIndices() const
{
    std::vector<typename GraphicalFactor<MODEL>::IndexType> vec(
        variableIndices_.begin(), variableIndices_.end());
    return vec;
}

//template<class MODEL>
//typename GraphicalFactor<MODEL>::FactorKey
//    GraphicalFactor<MODEL>::BuildKey(void) const
//{
//    //std::sort(shapeBegin, shapeEnd);
//    std::ostringstream oss;
//    oss << variableIndices_.size() << ":";
//    for (IndexType var : variableIndices_) {
//        oss << var << ",";
//    }
//    return oss.str();
//}

template<class MODEL>
void GraphicalFactor<MODEL>::BindFunction(
    FunctionTypePtr f
    )
{
    funtion_ = f;
}

template<class MODEL>
typename GraphicalFactor<MODEL>::FunctionTypePtr
    GraphicalFactor<MODEL>::getFunction()
{
    return funtion_;
}

#endif
