#ifndef GRAPHICALMODEL_FUNCTION_HPP
#define GRAPHICALMODEL_FUNCTION_HPP
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "marray.hpp"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

template</*class FUNCTION, */class VALUE, class INDEX = size_t, class LABEL = size_t>
class GraphicalFunction
{
public:
    typedef boost::shared_ptr< GraphicalFunction > Ptr;
    typedef boost::shared_ptr< const GraphicalFunction > ConstPtr;

    //typedef FUNCTION FunctionType;
    typedef VALUE ValueType;
    typedef INDEX IndexType;
    typedef LABEL LabelType;

public:
    virtual ~GraphicalFunction() {};

    //template <class ITERATOR>
    //ValueType& operator()(ITERATOR shapeThis)
    //{ return (ValueType)0; }

    //template <class ITERATOR>
    //ValueType& operator()(ITERATOR shapeBegin, ITERATOR shapeEnd)
    //{ return (ValueType)0; }
};

template<class VALUE, class INDEX = size_t, class LABEL = size_t>
class ExplicitFunction
    : public marray::Marray<VALUE>,
    public GraphicalFunction</*ExplicitFunction<VALUE, INDEX, LABEL>, */VALUE, INDEX, LABEL>
{
public:
    typedef boost::shared_ptr< ExplicitFunction > Ptr;
    typedef boost::shared_ptr< const ExplicitFunction > ConstPtr;

    typedef VALUE ValueType;
    typedef INDEX IndexType;
    typedef LABEL LabelType;

public:
    ExplicitFunction(const VALUE& value)
        : marray::Marray<VALUE>(value)
    {}

    template <class ITERATOR>
    ExplicitFunction(ITERATOR shapeBegin, ITERATOR shapeEnd)
        : marray::Marray<VALUE>(shapeBegin, shapeEnd)
    {}

    template <class ITERATOR>
    ExplicitFunction(ITERATOR shapeBegin, ITERATOR shapeEnd, const VALUE& value)
        : marray::Marray<VALUE>(shapeBegin, shapeEnd, value)
    {}

    //template <class ITERATOR>
    //ValueType& operator()(ITERATOR shapeThis)
    //{ return marray::Marray<VALUE>::operator ()(shapeThis); }

    //template <class ITERATOR>
    //ValueType& operator()(ITERATOR shapeBegin, ITERATOR shapeEnd)
    //{ return marray::Marray<VALUE>::operator ()(shapeBegin, shapeEnd); }
};

#endif
