#ifndef GRAPHICALMODEL_HPP
#define GRAPHICALMODEL_HPP
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "GraphicalModel_Space.hpp"
#include "GraphicalModel_Factor.hpp"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

template<
    class VALUE,
    class SPACE = GraphicalSpace< >
>
class GraphicalModel
{
public:
    typedef boost::shared_ptr< GraphicalModel > Ptr;
    typedef boost::shared_ptr< const GraphicalModel > ConstPtr;

    typedef GraphicalModel<VALUE, SPACE> GraphicalModelType;
    typedef GraphicalFactor<GraphicalModel<VALUE, SPACE> > FactorType;
    typedef boost::shared_ptr< FactorType > FactorTypePtr;
    typedef SPACE SpaceType;
    typedef boost::shared_ptr< SpaceType > SpaceTypePtr;
    typedef VALUE ValueType;
    typedef typename SpaceType::IndexType IndexType;
    typedef typename SpaceType::LabelType LabelType;

    typedef std::deque<FactorTypePtr> factor_vec_type;
    //typedef typename FactorType::FactorKey FactorKey;
    typedef std::string FactorKey;
    typedef boost::bimap< FactorKey, FactorTypePtr> factor_map_type;

public:
    GraphicalModel(const SpaceTypePtr& space);

    const SpaceType& space() const;
    SpaceType& space();
    size_t numberOfVariables() const;
    size_t numberOfVariables(const IndexType& factorIndex) const;
    size_t numberOfLabels(const IndexType& variableIndex) const;

    size_t numberOfFactors() const;
    size_t addFactor(const FactorTypePtr& factor);
    FactorTypePtr operator[] (const IndexType& factorIndex);

    template<class ITERATOR>
    FactorKey buildFactorKey(ITERATOR shapeBegin, ITERATOR shapeEnd) const;
    bool getFactor(const FactorKey& factorKey, FactorTypePtr& factor) const;
    template<class ITERATOR>
    bool getFactor(ITERATOR shapeBegin, ITERATOR shapeEnd, FactorTypePtr& factor) const;
    void buildFactorMap(void);

private:
    SpaceTypePtr space_;
    factor_vec_type factor_vec_;
    factor_map_type factor_map_;
};

template<class VALUE, class SPACE>
GraphicalModel<VALUE, SPACE>::GraphicalModel
    (
    const SpaceTypePtr& space
    )
    : space_(space)
{  
}

template<class VALUE, class SPACE>
const typename GraphicalModel<VALUE, SPACE>::SpaceType&
    GraphicalModel<VALUE, SPACE>::space() const
{
    return *space_;
}

template<class VALUE, class SPACE>
typename GraphicalModel<VALUE, SPACE>::SpaceType&
    GraphicalModel<VALUE, SPACE>::space()
{
    return *space_;
}

template<class VALUE, class SPACE>
size_t GraphicalModel<VALUE, SPACE>::numberOfVariables() const
{
    return space_->numberOfVariables();
}

template<class VALUE, class SPACE>
size_t GraphicalModel<VALUE, SPACE>::numberOfLabels(
    const IndexType& variableIndex
    ) const
{
    return space_->numberOfLabels(variableIndex);
}

template<class VALUE, class SPACE>
size_t GraphicalModel<VALUE, SPACE>::numberOfFactors() const
{
    return factor_vec_.size();
}

template<class VALUE, class SPACE>
size_t GraphicalModel<VALUE, SPACE>::addFactor(
    const FactorTypePtr& factor
    )
{
    size_t index = factor_vec_.size();
    factor_vec_.push_back(factor);
    return index;
}

template<class VALUE, class SPACE>
typename GraphicalModel<VALUE, SPACE>::FactorTypePtr
    GraphicalModel<VALUE, SPACE>::operator[](
    const typename GraphicalModel<VALUE, SPACE>::IndexType& factorIndex
    )
{
    return factor_vec_[factorIndex];
}

template<class VALUE, class SPACE>
template<class ITERATOR>
typename GraphicalModel<VALUE, SPACE>::FactorKey
    GraphicalModel<VALUE, SPACE>::buildFactorKey(
    ITERATOR shapeBegin, ITERATOR shapeEnd
    ) const
{
    //std::sort(shapeBegin, shapeEnd);
    std::ostringstream oss;
    oss << std::distance(shapeBegin, shapeEnd) << ":";
    for (ITERATOR it = shapeBegin; it != shapeEnd; ++it) {
        oss << *it << ",";

    }
    return oss.str();
}

template<class VALUE, class SPACE>
bool GraphicalModel<VALUE, SPACE>::getFactor(
    const FactorKey& factorKey, FactorTypePtr& factor
    ) const
{
    factor_map_type::left_const_iterator it = factor_map_.left.find(factorKey);
    if (it == factor_map_.left.end()) return false;
    factor = it->second;
    return true;
}

template<class VALUE, class SPACE>
template<class ITERATOR>
bool GraphicalModel<VALUE, SPACE>::getFactor(
    ITERATOR shapeBegin, ITERATOR shapeEnd, FactorTypePtr& factor
    ) const
{
    return getFactor(buildFactorKey(shapeBegin, shapeEnd), factor);
}

template<class VALUE, class SPACE>
void GraphicalModel<VALUE, SPACE>::buildFactorMap(void)
{
    factor_vec_type::const_iterator it = factor_vec_.begin();
    for (; it != factor_vec_.end(); ++it) {
        FactorTypePtr factor = *it;
        std::vector<IndexType> shape = factor->variableIndices();
        FactorKey factor_key = buildFactorKey(shape.begin(), shape.end());
        //FactorKey factor_key = factor->BuildKey();
        factor_map_.insert(factor_map_type::value_type(factor_key, factor));
    }
}

#endif
