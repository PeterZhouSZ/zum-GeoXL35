#ifndef SymmRotation_H
#define SymmRotation_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "Symmetry/SymmContainer.h"
#include "Symmetry/SymmFeat.h"
#include "Symmetry/PatternRotation.h"
//---------------------------------------------------------------------------
#include "OOBBox3f.h"
//---------------------------------------------------------------------------

class PROJECTSXWU_API SymmRotation : public SymmContainer
{
public:
    typedef boost::shared_ptr< SymmRotation > Ptr;
    typedef boost::shared_ptr< const SymmRotation > ConstPtr;
    typedef pair<PatternRotation::Ptr, PatternRotation::Ptr> RotationPair;

public:
    SymmRotation(PatternRotation::Ptr p0, PatternRotation::Ptr p1);

    void SetupVertice(const SymmFeatLine::Ptr& sfl, const int& idx);
    bool AddVertice(const SymmFeatLine::Ptr& sfl, int& idx);

    bool Finalize(void);

    void DrawWithDR(void) const;
    inline size_t size(void) { return patPair_.first->size(); }
    inline size_t size(void) const { return patPair_.first->size(); }
    inline size_t capacity(void) { return patPair_.first->capacity(); }
    inline size_t capacity(void) const { return patPair_.first->capacity(); }

public:
    virtual Matrix4f Get1StepTransformation(void);

    virtual std::string GetName(void);
    virtual std::string GetDescription(void);

    virtual bool IsSeries(void);

public:
    RotationPair patPair_;
};
typedef SymmRotation::RotationPair RotationPair;

#endif
