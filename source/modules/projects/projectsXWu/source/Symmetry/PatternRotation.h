#ifndef PatternRotation_H
#define PatternRotation_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "Symmetry/GroupOrbit.h"
#include "Symmetry/PatternSymmetry.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class PROJECTSXWU_API PatternRotation : public PatternSymmetry
{
public:
    typedef boost::shared_ptr< PatternRotation > Ptr;
    typedef boost::shared_ptr< const PatternRotation > ConstPtr;

public:
    bool SetupFrom3P(const array3vec3f& pvec, Vector3i& ivec);
    bool Finalize(void);

    bool ParallelTo(const PatternRotation::Ptr& pr);
    bool CoAxisTo(const PatternRotation::Ptr& pr, const float& th = 1e-4);
    bool Concentric(const PatternRotation::Ptr& pr, const float& th = 1e-4);

    inline size_t capacity(void) { return num_; }
    inline size_t capacity(void) const { return num_; }

    bool GetRotatedPos(const Vector3f& pin, const int& step,
        Vector3f& pout, Matrix4f& tout);

public:
    virtual bool ConsistentWith(const PatternSymmetry::Ptr& ps, const float& th = 1e-4);
    virtual bool SameAs(const PatternSymmetry::Ptr& ps);

    virtual bool PointOnOrbit(const Vector3f& p, int& ix, const float& th = 1e-2);
    virtual bool AddPoint(const Vector3f& p, int& ix, const float& th = 1e-2);

    virtual void DrawWithDR(void) const;

private:
    bool generateFrame(const array3vec3f& pvec, Matrix3f& frame,
        float& step, float& radius,
        int& maxi, Vector3f& v1, Vector3f& v2);

public:
    OrbitCircle orbit_;
    Vector3f normal_;
    Vector3f center_;

    Matrix4f toOYZ_;
    Matrix4f toWorld_;

    int num_;
    OrbitPosMap pproj_;
    //std::vector<Vector3f> pproj_vec_;
};

#endif
