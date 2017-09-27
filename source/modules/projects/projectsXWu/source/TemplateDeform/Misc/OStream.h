#ifndef OStream_h_
#define OStream_h_

//----------------------------------------------------------------------
#include <iostream>
#include <iomanip>
//----------------------------------------------------------------------

namespace X4
{
//----------------------------------------------------------------------
static card32 const FPPrecision =  6; // floating point output precision
static card32 const FPWidth     = 10; // floating point output width
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
static bool const UseFPPrecision = true;
static bool const UseFPWidth     = true;
//----------------------------------------------------------------------
static card32 const IWidth      =  4; // integer output width
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
static bool const UseIWidth      = true;
//----------------------------------------------------------------------
static card32 const MatrixRowSeparator = 3; // blank line every x rows
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
static bool const UseMatrixRowSeparator = true;
//----------------------------------------------------------------------

//----------------------------------------------------------------------
#ifdef  LinearAlgebraH
#ifndef LinearAlgebraH_OStream_h_
#define LinearAlgebraH_OStream_h_
//----------------------------------------------------------------------

//----------------------------------------------------------------------
template <card32 SIZE>
inline std::ostream& operator<<(
  std::ostream                     & stream,
  StaticVector<float32, SIZE> const& vector)
{
  stream << "StaticVectorF: ";

  if (UseIWidth) { stream << std::setw(IWidth); }

  stream << SIZE << " entries "
         << "[ ";
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //obtain stream format flags to later reset original stream state
  std::ios_base::fmtflags flags = stream.flags();

  if (UseFPPrecision) { stream.setf(std::ios::fixed); }
         
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (mpcard i = 0; i < SIZE; ++i)
  {
    if (UseFPWidth    ) { stream << std::setw(FPWidth);             }
    if (UseFPPrecision) { stream << std::setprecision(FPPrecision); }

    stream << vector[i] << " ";
  }
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  stream.flags(flags);
  
  stream << "]";

  return stream;
}

//----------------------------------------------------------------------
template <card32 ROWS, card32 COLS>
inline std::ostream& operator<<(
  std::ostream                           & stream,
  StaticMatrix<float32, COLS, ROWS> const& matrix)
{
  //stream << "StaticMatrixF: ";

  if (UseIWidth) { stream << std::setw(IWidth); }

  stream << ROWS << " rows, ";

  if (UseIWidth) { stream << std::setw(IWidth); }
  
  stream << COLS << " columns";
  stream << std::endl
         << "[" << std::endl;
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //obtain stream format flags to later reset original stream state
  std::ios_base::fmtflags flags = stream.flags();
  
  if (UseFPPrecision) { stream.setf(std::ios::fixed); }
         
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (card32 j = 0; j < ROWS; ++j)
  {
    if (UseMatrixRowSeparator && j && !(j % MatrixRowSeparator))
    {
      stream << std::endl;
    }
    
    for (card32 i = 0; i < COLS; ++i)
    {
      if (UseFPWidth    ) { stream << std::setw(FPWidth);             }
      if (UseFPPrecision) { stream << std::setprecision(FPPrecision); }

      stream << matrix[i][j] << " ";
    }

    stream << std::endl;
  }
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  stream.flags(flags);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  stream << "]" << std::endl;

  return stream;
}

//----------------------------------------------------------------------
#endif //LinearAlgebraH_OStream_h_
#endif //LinearAlgebraH
//----------------------------------------------------------------------

//----------------------------------------------------------------------
#ifdef  DynamicLinearAlgebraH
#ifndef DynamicLinearAlgebraH_OStream_h_
#define DynamicLinearAlgebraH_OStream_h_
//----------------------------------------------------------------------

inline std::ostream& operator<<(std::ostream  & stream,
                                DVectorI const& vector)
{
  stream << "DVectorI: ";

  if (UseIWidth) { stream << std::setw(IWidth); }

  stream << vector.size() << " entries "
         << "[ ";
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //obtain stream format flags to later reset original stream state
  std::ios_base::fmtflags flags = stream.flags();

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (mpcard i = 0; i < vector.size(); ++i)
  {
    stream << vector[i] << " ";
  }
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  stream.flags(flags);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  stream << "]";

  return stream;
}

//----------------------------------------------------------------------
inline std::ostream& operator<<(std::ostream  & stream,
                                DVectorF const& vector)
{
  stream << "DVectorF: ";

  if (UseIWidth) { stream << std::setw(IWidth); }

  stream << vector.size() << " entries "
         << "[ ";
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //obtain stream format flags to later reset original stream state
  std::ios_base::fmtflags flags = stream.flags();

  if (UseFPPrecision) { stream.setf(std::ios::fixed); }
         
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (mpcard i = 0; i < vector.size(); ++i)
  {
    if (UseFPWidth    ) { stream << std::setw(FPWidth);             }
    if (UseFPPrecision) { stream << std::setprecision(FPPrecision); }

    stream << vector[i] << " ";
  }
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  stream.flags(flags);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  stream << "]";

  return stream;
}

//----------------------------------------------------------------------
inline std::ostream& operator<<(std::ostream  & stream,
                                DMatrixF const& matrix)
{
  //stream << "DMatrixF: ";

  if (UseIWidth) { stream << std::setw(IWidth); }

  stream << matrix.getRows() << " rows, ";

  if (UseIWidth) { stream << std::setw(IWidth); }
  
  stream << matrix.getColumns() << " columns";
  stream << std::endl
         << "[" << std::endl;
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //obtain stream format flags to later reset original stream state
  std::ios_base::fmtflags flags = stream.flags();
  
  if (UseFPPrecision) { stream.setf(std::ios::fixed); }
         
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (card32 j = 0; j < matrix.getRows(); ++j)
  {
    if (UseMatrixRowSeparator && j && !(j % MatrixRowSeparator))
    {
      stream << std::endl;
    }
    
    for (card32 i = 0; i < matrix.getColumns(); ++i)
    {
      if (UseFPWidth    ) { stream << std::setw(FPWidth);             }
      if (UseFPPrecision) { stream << std::setprecision(FPPrecision); }

      stream << matrix[i][j] << " ";
    }

    stream << std::endl;
  }
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  stream.flags(flags);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  stream << "]" << std::endl;

  return stream;
}

//----------------------------------------------------------------------
#endif //DynamicLinearAlgebraH_OStream_h_
#endif //DynamicLinearAlgebraH
//----------------------------------------------------------------------

//----------------------------------------------------------------------
#ifdef  SparseLinearAlgebraH
#ifndef SparseLinearAlgebraH_OStream_h_
#define SparseLinearAlgebraH_OStream_h_
//----------------------------------------------------------------------

//----------------------------------------------------------------------
inline std::ostream& operator<<(std::ostream       & stream,
                                SparseVectorF const& vector)
{
  stream << "SparseVectorF: ";

  if (UseIWidth) { stream << std::setw(IWidth); }

  stream << vector.entries.size() << " entries "
         << "[ ";
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //obtain stream format flags to later reset original stream state
  std::ios_base::fmtflags flags = stream.flags();

  if (UseFPPrecision) { stream.setf(std::ios::fixed); }
         
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (mpcard i = 0; i < vector.entries.size(); ++i)
  {
    if (UseIWidth     ) { stream << std::setw(IWidth);              }

    stream << vector.entries[i].index << ":";

    if (UseFPWidth    ) { stream << std::setw(FPWidth);             }
    if (UseFPPrecision) { stream << std::setprecision(FPPrecision); }

    stream << vector.entries[i].value << " ";
  }
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  stream.flags(flags);
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  stream << "]";

  return stream;
}

//----------------------------------------------------------------------
inline std::ostream& operator<<(std::ostream       & stream,
                                SparseMatrixF const& matrix)
{
  stream << "SparseMatrixF: ";

  if (UseIWidth) { stream << std::setw(IWidth); }

  stream << matrix.getNumRows() << " rows, ";

  if (UseIWidth) { stream << std::setw(IWidth); }
  
  stream << matrix.getNumCols() << " columns";
  stream << std::endl
         << "[" << std::endl;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  mpcard const rows = matrix.getRows();

  for (mpcard i = 0; i < rows; ++i)
  {
    if (UseMatrixRowSeparator && i && !(i % MatrixRowSeparator))
    {
      stream << std::endl;
    }
    
    stream << "Row ";
    
    if (UseIWidth) { stream << std::setw(IWidth); }
    
    stream << i << ": ";

    stream << matrix[i] << std::endl;
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  stream << "]" << std::endl;

  return stream;
}

//----------------------------------------------------------------------
#endif //SparseLinearAlgebraH_OStream_h_
#endif //SparseLinearAlgebraH
//----------------------------------------------------------------------

} //namespace X4

#endif
