#ifndef CICPHandle_h_
#define CICPHandle_h_
//----------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//----------------------------------------------------------------------
#include "userInterface/handles3d/Handle3D.h"
//----------------------------------------------------------------------

namespace X4
{
//======================================================================
// class CICPHandle
//----------------------------------------------------------------------
class CICPHandle : public Handle3D
{
  X4_CLASS(CICPHandle)

public:
  //====================================================================
  // Constructors
  //--------------------------------------------------------------------
  CICPHandle();
  CICPHandle(Vector3f const& origin,
             Vector3f const& target, 
             mpint    const  id);

  //====================================================================
  // Member functions
  //--------------------------------------------------------------------
  virtual void assign(Object const* obj, X4CopyContext* context = NULL);

  //====================================================================
  // Member variables
  //--------------------------------------------------------------------
  mpint id_;
};

} //namespace X4

#endif //CICPHandle_h_
