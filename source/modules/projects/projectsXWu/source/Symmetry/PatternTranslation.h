#ifndef PatternTranslation_H
#define PatternTranslation_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "Symmetry/GroupOrbit.h"
#include "Symmetry/PatternSymmetry.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class PROJECTSXWU_API PatternTranslation : public PatternSymmetry
{
public:
    typedef boost::shared_ptr< PatternTranslation > Ptr;
    typedef boost::shared_ptr< const PatternTranslation > ConstPtr;

public:
    bool SetupFrom2P(const array2vec3f& pvec, Vector2i& ivec,
        const float& th = 1e-4);

    bool ParallelTo(const PatternTranslation::Ptr& pt);

public:
    virtual bool ConsistentWith(const PatternSymmetry::Ptr& ps, const float& th = 1e-4);
    virtual bool SameAs(const PatternSymmetry::Ptr& ps);

    virtual bool PointOnOrbit(const Vector3f& p, int& ix, const float& th = 1e-2);
    virtual bool AddPoint(const Vector3f& p, int& ix, const float& th = 1e-2);

    virtual void DrawWithDR(void) const;

public:
    OrbitLine orbit_;
    Vector3f direction_;
};

#endif
