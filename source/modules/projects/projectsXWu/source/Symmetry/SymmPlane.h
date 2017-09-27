#ifndef SymmPlane_H
#define SymmPlane_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "Symmetry/SymmContainer.h"
#include "Symmetry/SymmFeat.h"
#include "Symmetry/PatternTranslation.h"
//---------------------------------------------------------------------------
#include "OOBBox3f.h"
//---------------------------------------------------------------------------

class PROJECTSXWU_API SymmPlane : public SymmContainer
{
public:
    typedef boost::shared_ptr< SymmPlane > Ptr;
    typedef boost::shared_ptr< const SymmPlane > ConstPtr;

public:
    SymmPlane(const std::vector<Vector3f>& vertices,
        const std::vector<mpcard>& vindx);

    void DrawWithDR(const Vector3f& color) const;
    inline size_t size(void) { return vertices_.size(); }
    inline size_t size(void) const { return vertices_.size(); }

public:
    virtual Matrix4f Get1StepTransformation(void);

    virtual std::string GetName(void);
    virtual std::string GetDescription(void);

public:
    std::vector<Vector3f> vertices_;
    std::vector<mpcard> vindx_;

    Vector3f cen_;
    Vector3f end_[3];
    mpcard nn_indx;
};

#endif
