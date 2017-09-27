#ifndef SymmContainer_H
#define SymmContainer_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class PROJECTSXWU_API SymmContainer
{
public:
    typedef boost::shared_ptr< SymmContainer > Ptr;
    typedef boost::shared_ptr< const SymmContainer > ConstPtr;
    typedef std::map< int, std::deque< mpcard > > OrbitVerticeMap;

public:
    SymmContainer() : show_(true) {}

    virtual ~SymmContainer() {}

    virtual std::string GetName(void) = 0;
    virtual std::string GetDescription(void) = 0;

    virtual Matrix4f Get1StepTransformation(void) = 0;

    virtual bool GetUpdatedTransformation(
        PointSet  const& deformed,
        AAT       const& defAAT,
        Matrix4f& Tf, Matrix4f& Tb
        );

    virtual bool GetCorrespondence(
        std::vector<Vector3f>& points0,
        std::vector<Vector3f>& points1
        );

    virtual bool IsSeries(void);

    virtual void DrawWithDR(
        PointSet  const& deformed,
        AAT       const& defAAT
        );

public:
    OrbitVerticeMap overt_;
    bool show_;
};

typedef SymmContainer::OrbitVerticeMap OrbitVerticeMap;

#endif
