//---------------------------------------------------------------------------
#ifndef EditSynHandleH
#define EditSynHandleH
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "Handle3DGizmo.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class PROJECTSXWU_API EditSynHandle : public Handle3DGizmo
{
public:
    typedef boost::shared_ptr< EditSynHandle > Ptr;
    typedef boost::shared_ptr< const EditSynHandle > ConstPtr;

public:
    explicit EditSynHandle(const std::deque<unsigned>& points, const card32& width, const card32& height);

protected:
};

#endif
