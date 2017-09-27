#ifndef PatternSymmetry_H
#define PatternSymmetry_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class PROJECTSXWU_API PatternSymmetry
{
public:
    typedef boost::shared_ptr< PatternSymmetry > Ptr;
    typedef boost::shared_ptr< const PatternSymmetry > ConstPtr;

public:
    virtual ~PatternSymmetry() {}

    virtual bool ConsistentWith(const PatternSymmetry::Ptr& ps, const float& th = 1e-4) = 0;
    virtual bool SameAs(const PatternSymmetry::Ptr& ps) = 0;

    virtual bool PointOnOrbit(const Vector3f& p, int& ix, const float& th = 1e-2) = 0;
    virtual bool AddPoint(const Vector3f& p, int& ix, const float& th = 1e-2) = 0;

    virtual void DrawWithDR(void) const = 0;

public:
    inline size_t size(void) { return pmap_.size(); }
    inline size_t size(void) const { return pmap_.size(); }

public:
    OrbitPosMap pmap_;
};

#endif
