#pragma once
#ifndef TYPE_LIST_HPP
#define TYPE_LIST_HPP

#include <limits>
#include <vector>

#define MAKE_TYPELIST_1(T1) \
    meta::TypeList<T1, meta::ListEnd >

#define MAKE_TYPELIST_2(T1, T2) \
    meta::TypeList<T1, MAKE_TYPELIST_1(T2) >

#define MAKE_TYPELIST_3(T1, T2, T3) \
    meta::TypeList<T1, MAKE_TYPELIST_2(T2, T3) >

#define MAKE_TYPELIST_4(T1, T2, T3, T4) \
    meta::TypeList<T1, MAKE_TYPELIST_3(T2, T3, T4) >

#define MAKE_TYPELIST_5(T1, T2, T3, T4, T5) \
    meta::TypeList<T1, MAKE_TYPELIST_4(T2, T3, T4, T5) >

#define MAKE_TYPELIST_6(T1, T2, T3, T4, T5, T6) \
    meta::TypeList<T1, MAKE_TYPELIST_5(T2, T3, T4, T5, T6) >

#define MAKE_TYPELIST_7(T1, T2, T3, T4, T5, T6, T7) \
    meta::TypeList<T1, MAKE_TYPELIST_6(T2, T3, T4, T5, T6, T7) >

#define MAKE_TYPELIST_8(T1, T2, T3, T4, T5, T6, T7, T8) \
    meta::TypeList<T1, MAKE_TYPELIST_7(T2, T3, T4, T5, T6, T7, T8) >

#define MAKE_TYPELIST_9(T1, T2, T3, T4, T5, T6, T7, T8, T9) \
    meta::TypeList<T1, MAKE_TYPELIST_8(T2, T3, T4, T5, T6, T7, T8, T9) >

#define MAKE_TYPELIST_10(T1, T2, T3, T4, T5, T6, T7, T8, T9, T10) \
    meta::TypeList<T1, MAKE_TYPELIST_9(T2, T3, T4, T5, T6, T7, T8, T9, T10) >

/// namespace for meta-programming
namespace meta {

    /// metaprogramming identity metafunction
    template<class T>
    struct Self {
        typedef T type;
    };
    /// metaprogramming Empty type
    struct EmptyType {
    };
    /// metaprogramming Null type
    struct NullType {
    };
    /// end of a typelist type
    struct ListEnd {
    };

    /// metaprogramming length of typelist metafunction
    template<class T_List>
    struct LengthOfTypeList {
        typedef meta::Int < 1 + LengthOfTypeList<typename T_List::TailType>::type::value> type;
        enum {
            value = type::value
        };
    };
    /// metaprogramming length of typelist metafunction
    template< >
    struct LengthOfTypeList<meta::ListEnd> {
        typedef meta::Int < 0 > type;
        enum {
            value = 0
        };
    };
    /// metaprogramming type at typelist metafunction
    template<class T_List, unsigned int Index>
    struct TypeAtTypeList {
        typedef typename TypeAtTypeList<typename T_List::TailType, Index - 1 > ::type type;
    };
    /// metaprogramming type at typelist metafunction
    template<class T_List>
    struct TypeAtTypeList<T_List, 0 > {
        typedef typename T_List::HeadType type;
    };
    /// metaprogramming type at typelist save metafunction
    template<class T_List, unsigned int Index, class T_DefaultType>
    struct TypeAtTypeListSave
        : meta::EvalIf<
        meta::LengthOfTypeList<T_List>::value >= Index ? true : false,
        meta::TypeAtTypeList<T_List, Index>,
        meta::Self<T_DefaultType>
        >::type {
    };

    /// metaprogramming typelist
    template<class T_Head, class T_Tail>
    struct TypeList {
        typedef meta::ListEnd ListEnd;
        typedef T_Head HeadType;
        typedef T_Tail TailType;
    };
    /// metaprogramming typelist generator  metafunction
    template
        <
            class T1,
            class T2 = meta::ListEnd,
            class T3 = meta::ListEnd,
            class T4 = meta::ListEnd,
            class T5 = meta::ListEnd,
            class T6 = meta::ListEnd,
            class T7 = meta::ListEnd,
            class T8 = meta::ListEnd,
            class T9 = meta::ListEnd,
            class T10 = meta::ListEnd,
            class T11 = meta::ListEnd,
            class T12 = meta::ListEnd,
            class T13 = meta::ListEnd,
            class T14 = meta::ListEnd,
            class T15 = meta::ListEnd
        >
    struct TypeListGenerator {
        typedef meta::TypeList<T1, typename TypeListGenerator<T2, T3, T4, T5, T6, T7, T8, T9, T10, T11, T12, T13, T14, T15>::type > type;
    };
    /// metaprogramming typelist generator  metafunction
    template< >
    struct TypeListGenerator<meta::ListEnd> {
        typedef meta::ListEnd type;
    };

#endif
