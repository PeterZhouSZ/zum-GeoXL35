#ifndef SymmTranslation_H
#define SymmTranslation_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "Symmetry/SymmContainer.h"
#include "Symmetry/SymmFeat.h"
#include "Symmetry/PatternTranslation.h"
//---------------------------------------------------------------------------
#include "OOBBox3f.h"
//---------------------------------------------------------------------------

class PROJECTSXWU_API SymmTranslation : public SymmContainer
{
public:
    typedef boost::shared_ptr< SymmTranslation > Ptr;
    typedef boost::shared_ptr< const SymmTranslation > ConstPtr;
    typedef pair<PatternTranslation::Ptr, PatternTranslation::Ptr> TranslationPair;

public:
    SymmTranslation(PatternTranslation::Ptr p0, PatternTranslation::Ptr p1);

    void SetupVertice(const SymmFeatLine::Ptr& sfl, const int& idx);
    bool AddVertice(const SymmFeatLine::Ptr& sfl, int& idx);

    void DrawWithDR(void) const;
    inline size_t size(void) { return patPair_.first->size(); }
    inline size_t size(void) const { return patPair_.first->size(); }

public:
    virtual Matrix4f Get1StepTransformation(void);

    virtual std::string GetName(void);
    virtual std::string GetDescription(void);

public:
    TranslationPair patPair_;
};
typedef SymmTranslation::TranslationPair TranslationPair;

#endif
