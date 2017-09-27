#ifndef GRAPHICALMODEL_SPACE_HPP
#define GRAPHICALMODEL_SPACE_HPP
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "Util/marray.hpp"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

template<class INDEX = std::size_t, class LABEL = std::size_t>
class GraphicalSpace
    : public marray::Marray<LABEL>
{
public:
    typedef boost::shared_ptr< GraphicalSpace > Ptr;
    typedef boost::shared_ptr< const GraphicalSpace > ConstPtr;

    typedef INDEX IndexType;
    typedef LABEL LabelType;

public:
    template <class ITERATOR>
    GraphicalSpace()
        : marray::Marray<LABEL>(), numberOfLabels((LABEL)0)
    {}

    template <class ITERATOR>
    GraphicalSpace(
        ITERATOR shapeBegin, ITERATOR shapeEnd,
        const LABEL& numberOfLabels, const LABEL& label = (LABEL)0
        )
        : marray::Marray<LABEL>(shapeBegin, shapeEnd, label),
        numberOfStates_(numberOfLabels)
    {}

    size_t numberOfVariables() const;
    size_t numberOfLabels(const IndexType& variableIndex) const;
    size_t numberOfLabels() const;

public:
    LabelType numberOfStates_;
};

template<class INDEX, class LABEL>
size_t GraphicalSpace<INDEX, LABEL>::numberOfVariables() const
{
    return size();
}

template<class INDEX, class LABEL>
size_t GraphicalSpace<INDEX, LABEL>::numberOfLabels(
    const IndexType& variableIndex
    ) const
{
    return numberOfStates_;
}

template<class INDEX, class LABEL>
size_t GraphicalSpace<INDEX, LABEL>::numberOfLabels() const
{
    return numberOfStates_;
}

#endif
