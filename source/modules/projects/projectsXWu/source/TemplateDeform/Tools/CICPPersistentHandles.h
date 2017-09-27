#ifndef CICPPersistentHandles_h_
#define CICPPersistentHandles_h_
//----------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//----------------------------------------------------------------------
#include "userInterface/handles3d/Handle3D.h"
//----------------------------------------------------------------------
#include "ObjectListProperty.h"
//----------------------------------------------------------------------
#include <vector>
//----------------------------------------------------------------------

namespace X4
{
//======================================================================
// class CICPPersistentHandles
//----------------------------------------------------------------------
class CICPPersistentHandles : public Persistent
{
  //====================================================================
  // X4 stuff
  //--------------------------------------------------------------------
  X4_CLASS(CICPPersistentHandles);

  IMPLEMENT_OBJECT_LIST_METHODS_STL(handles, Handle3D);

public:
  //====================================================================
  // Constructors
  //--------------------------------------------------------------------
  CICPPersistentHandles();
  CICPPersistentHandles(std::vector<Handle3D*> const& handles);
  
  //====================================================================
  // Destructor
  //--------------------------------------------------------------------
  virtual ~CICPPersistentHandles();

  //====================================================================
  // Member functions
  //--------------------------------------------------------------------
  virtual void assign(Object const* obj, X4CopyContext* context = NULL);
  
private:
  //====================================================================
  // Member variables
  //--------------------------------------------------------------------
  std::vector<Handle3D*> m_handles; //IMPLEMENT_OBJECT_LIST_METHODS_STL
};

} //namespace X4

#endif //CICPPersistentHandles_h_
