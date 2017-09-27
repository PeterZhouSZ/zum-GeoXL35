//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "CICPPersistentHandles.h"
//----------------------------------------------------------------------
#include "CICPHandle.h"
//----------------------------------------------------------------------
#include "CopyObjectProperties.h"
//----------------------------------------------------------------------

namespace X4
{

IMPLEMENT_X4_CLASS(CICPPersistentHandles, 0)
{
  BEGIN_CLASS_INIT(CICPPersistentHandles);

  ADD_OBJECT_LIST_PROP(handles, 0, CICPHandle::getClass());
}

//======================================================================
// Constructors
//----------------------------------------------------------------------
CICPPersistentHandles::CICPPersistentHandles()
{}

//----------------------------------------------------------------------
CICPPersistentHandles::CICPPersistentHandles(
  std::vector<Handle3D*> const& handles)
{
  for (mpcard i = 0; i < handles.size(); ++i)
  {
    m_handles.push_back(dynamic_cast<Handle3D*>(handles.at(i)->copy()));
  }
}

//======================================================================
// Destructor
//----------------------------------------------------------------------
CICPPersistentHandles::~CICPPersistentHandles()
{
  for (mpcard i = 0; i < m_handles.size(); ++i)
  {
    delete m_handles.at(i);
  }
}

//======================================================================
// Member functions
//----------------------------------------------------------------------
void CICPPersistentHandles::assign(
  Object const* obj, X4CopyContext* context)
{
  if (!dynamic_cast<CICPPersistentHandles const*>(obj))
  {
    throw PException("CICPPersistentHandles::assign() - Wrong type.");
  }

  copyObjectProperties(obj, this);
}

} //namespace X4
