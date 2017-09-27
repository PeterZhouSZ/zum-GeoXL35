#ifndef FunctionContainer_H
#define FunctionContainer_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

template< class FuncPtr >
class FunctionContainer
{
public:
    typedef boost::shared_ptr< FunctionContainer > Ptr;
    typedef boost::shared_ptr< const FunctionContainer > ConstPtr;

    typedef unsigned index_type;
    typedef FuncPtr value_type;

public:
    size_t size() { return func_list.size(); }
    void push_back(const value_type& fp) { func_list.push_back(fp); }
    value_type operator[](const index_type& index) { return func_list[index]; }
    const value_type operator[](const index_type& index) const { return func_list[index]; }

private:
    std::deque< value_type > func_list;
};

typedef void (*VoidFuncPtr)(void);
typedef FunctionContainer< VoidFuncPtr > VoidFuncList;

#endif
