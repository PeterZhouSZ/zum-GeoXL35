#ifndef SymmFeatProxy_H
#define SymmFeatProxy_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "Symmetry/SymmDetCont.h"
#include "Symmetry/SymmFeat.h"
#include "Symmetry/SymmReflective.h"
#include "Symmetry/SymmRotation.h"
#include "Symmetry/SymmTranslation.h"
#include "Symmetry/SymmPlane.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class PROJECTSXWU_API SymmFeatProxy
{
public:
    typedef boost::shared_ptr< SymmFeatProxy > Ptr;
    typedef boost::shared_ptr< const SymmFeatProxy > ConstPtr;

public:
    virtual ~SymmFeatProxy() {}
};

class PROJECTSXWU_API SymmFeatProxyLine : public SymmFeatProxy
{
public:
    typedef boost::shared_ptr< SymmFeatProxyLine > Ptr;
    typedef boost::shared_ptr< const SymmFeatProxyLine > ConstPtr;

    void SetContext(SymmDetCont::Ptr cont) { symmDetCont = cont; }

    void DetectSymmtry(
        std::vector<SymmRotation::Ptr>& rotsymmvec,
        std::vector<SymmTranslation::Ptr>& trasymmvec,
        std::vector<SymmReflective::Ptr>& refsymmvec);

public:
    void ExtractFeature(void);

    void ClusterSimilarLine(void);

    void BuildFeatureTree(void);

    void DetectCoPlanarity(std::vector<SymmPlane::Ptr>& copsymmvec);
    void DetectCoPlanarity(const std::vector<SymmFeatLine::Ptr>& featArr,
        std::vector<SymmPlane::Ptr>& copsymmvec);


    void CheckSymmetryTransformation(const Matrix4f& trans, float& score,
        std::map< pair<size_t, size_t>, LineDir >& symmetricLines
        );

    void DetectRotation(std::vector<SymmRotation::Ptr>& rotsymmvec);
    void DetectRotation(const std::vector<SymmFeatLine::Ptr>& featArr,
        std::vector<SymmRotation::Ptr>& rotsymmvec);

    void DetectTranslation(std::vector<SymmTranslation::Ptr>& trasymmvec);
    void DetectTranslation(const std::vector<SymmFeatLine::Ptr>& featArr,
        std::vector<SymmTranslation::Ptr>& trasymmvec);

    void DetectReflection(std::vector<SymmReflective::Ptr>& refsymmvec);
    void PropagateReflectionPoints(std::vector<SymmReflective::Ptr>& refsymmvec);

private:
    std::vector<Matrix4f> generateTransformations(
        SymmFeatLine::Ptr l0, SymmFeatLine::Ptr l1);

    void detectRotation3P(const std::vector<SymmFeatLine::Ptr> &featArr,
        const boost::array<size_t, 3>& indx3,
        std::vector<bool>& featChecked,
        std::vector<SymmRotation::Ptr>& rotsymmvec);

    void detectTranslation2P(const std::vector<SymmFeatLine::Ptr> &featArr,
        const boost::array<size_t, 2>& indx2,
        std::vector<bool>& featChecked,
        std::vector<SymmTranslation::Ptr>& trasymmvec);

private:
    SymmDetCont::Ptr symmDetCont;
};

#endif
