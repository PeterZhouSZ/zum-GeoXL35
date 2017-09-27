//----------------------------------------------------------------------
#ifndef TAssert_h_
#define TAssert_h_
//----------------------------------------------------------------------

//======================================================================
// Preprocessor macros
//----------------------------------------------------------------------
#define TASSERT(expression)                       \
{                                                 \
  bool const value = (expression) ? true : false; \
  TAssert<value>::Assert();            \
}

//======================================================================
// Compile-time assertion template
// see http://www.flipcode.com/archives/Compile-Time_Assertions.shtml
//----------------------------------------------------------------------
template <bool B>
struct TAssert
{};

template <>
struct TAssert<true>
{
  static void Assert() {};
};

//----------------------------------------------------------------------
#endif //TAssert_h_
//----------------------------------------------------------------------
