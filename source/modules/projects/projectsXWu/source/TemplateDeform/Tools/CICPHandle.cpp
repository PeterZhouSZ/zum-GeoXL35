//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "CICPHandle.h"
//----------------------------------------------------------------------
#include "CopyObjectProperties.h"
//----------------------------------------------------------------------

namespace X4
{

IMPLEMENT_X4_CLASS(CICPHandle, 0)
{
  BEGIN_CLASS_INIT(CICPHandle);

  ADD_CARD32_PROP(id_, 0);
}

//======================================================================
// Constructors
//----------------------------------------------------------------------
CICPHandle::CICPHandle()
: Handle3D(),
  id_(0)
{}

//----------------------------------------------------------------------
CICPHandle::CICPHandle(Vector3f const& origin,
                       Vector3f const& target,
                       mpint    const  id)
: id_(id)
{
  Vector4f const color = makeVector4f(0.5f, 1.0f, 0.4f, 1.0f);

  points.push_back(Point(target, color));
  points.push_back(Point(origin, NULL_VECTOR4F, false));

  lines.push_back(Line(0, 1, makeVector4f(1.0f, 1.0f, 1.0f, 0.3f)));
}

//======================================================================
// Member functions
//----------------------------------------------------------------------
void CICPHandle::assign(Object const* obj, X4CopyContext* context)
{
  Handle3D::assign(obj, context);

  if (!dynamic_cast<CICPHandle const*>(obj))
  {
    throw PException("CICPHandle::assign() - Wrong type.");
  }

  copyObjectProperties(obj, this);
}

} //namespace X4
