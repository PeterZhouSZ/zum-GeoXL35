#ifndef SymmDet_H
#define SymmDet_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "Symmetry/SymmDetCont.h"
#include "Symmetry/SymmFeatProxy.h"
#include "Util/TrimeshStatic.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class PROJECTSXWU_API SymmDet
{
public:
    virtual void DetectSymmtry(TrimeshStatic::Ptr smesh) = 0;
};

class PROJECTSXWU_API SymmDetHiPrec : public SymmDet
{
public:
    typedef boost::shared_ptr< SymmDetHiPrec > Ptr;
    typedef boost::shared_ptr< const SymmDetHiPrec > ConstPtr;

    SymmDetHiPrec();
    void SetContext(SymmDetCont::Ptr cont) { symmDetCont = cont; }
    void Clear(void);

    int ShowFeatureLines(int sframe = 0, bool const& clear = false);
    int ShowFeatureLinesCluster(int sframe = 0, bool const& clear = false);
    int ShowCoPlanarity(int sframe = 0, bool const& usefeature = true, bool const& clear = false);
    int ShowReflections(int sframe = 0, bool const& usefeature = true, bool const& clear = false);
    int ShowRotations(int sframe = 0, bool const& usefeature = true, bool const& clear = false);
    int ShowTranslations(int sframe = 0, bool const& usefeature = true, bool const& clear = false);

public:
    virtual void DetectSymmtry(TrimeshStatic::Ptr smesh);

public:
    TrimeshStatic::Ptr strimesh;
    SymmDetCont::Ptr symmDetCont;

    std::vector<SymmFeatLine::Ptr> lineFeatures;
    std::vector<SymmFeatSetLine::Ptr> equivalentLines;
    std::vector<unsigned> lineClassID;

    std::vector<SymmPlane::Ptr> copsymmvec;
    std::vector<SymmReflective::Ptr> refsymmvec;
    std::vector<SymmRotation::Ptr> rotsymmvec;
    std::vector<SymmTranslation::Ptr> trasymmvec;
    std::vector<SymmContainer::Ptr> symmvec;
};

#endif
