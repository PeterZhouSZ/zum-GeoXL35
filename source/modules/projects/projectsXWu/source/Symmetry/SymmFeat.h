#ifndef SymmFeat_H
#define SymmFeat_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "Util/TrimeshStatic.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class PROJECTSXWU_API SymmFeat
{
public:
    typedef boost::shared_ptr< SymmFeat > Ptr;
    typedef boost::shared_ptr< const SymmFeat > ConstPtr;

public:
    virtual ~SymmFeat() {}
    virtual bool SimilarTo(SymmFeat::Ptr osf, const float& th = 1e-6) const = 0;
};

enum LineDir { SAME_DIR = 0, DIFF_DIR, E_DIR = -1 };
class PROJECTSXWU_API SymmFeatLine : public SymmFeat
{
public:
    typedef boost::shared_ptr< SymmFeatLine > Ptr;
    typedef boost::shared_ptr< const SymmFeatLine > ConstPtr;

public:
    enum { UNPROCESSED = 0, NO_BORDER, IS_BORDER };
    void Setup(TrimeshStatic::Ptr smesh, const unsigned& indx);
    void Setup(
        const Vector3f& p0, const Vector3f& p1,
        const Vector3f& n0, const Vector3f& n1,
        bool bb);
    SymmFeatLine::Ptr CopyTo(void);
    void UniformDirection(void);
    void Reverse(void);

    bool Join(TrimeshStatic::Ptr smesh, SymmFeatLine::Ptr ol);
    void GetFrame(Matrix3f& frame, Vector3f& origin) const;
    void DrawWithDR(void);

public:
    virtual bool SimilarTo(SymmFeat::Ptr osf, const float& th = 1e-6) const;
    LineDir SameAs(SymmFeatLine::Ptr osf, const float& th = 1e-6) const;

public:
    std::deque<mpcard> edges;
    std::deque<mpcard> vertice;
    mpcard v0, v1;
    Vector3f pos0, pos1;
    Vector3f cen;
    Vector3f dir;
    float leng;
    bool isBder;
    Vector3f surfaceN0, surfaceN1;
    Vector3f normal;
    float32 normalAngle;
};

class PROJECTSXWU_API SymmFeatSetLine
{
public:
    typedef boost::shared_ptr< SymmFeatSetLine > Ptr;
    typedef boost::shared_ptr< const SymmFeatSetLine > ConstPtr;

public:
    SymmFeatSetLine(const SymmFeatLine::Ptr& sfl);
    void Insert(const SymmFeatLine::Ptr& sfl, const int& indx);
    size_t size() { return lset.size(); }
    typedef std::vector<SymmFeatLine::Ptr> SFLSet;
    SFLSet lset;
    //std::vector<int> indx;
    SymmFeatLine::Ptr elem;
    Vector3f cen;

private:
    bool near_center(const Vector3f& cen,
        const SymmFeatLine::Ptr& e0, const SymmFeatLine::Ptr& e1)
    {
        return norm(e0->normal - cen) < norm(e1->normal - cen);
    }
};

#endif
