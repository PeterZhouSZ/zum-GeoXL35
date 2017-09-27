#ifndef SymmReflective_H
#define SymmReflective_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "Symmetry/SymmContainer.h"
#include "Symmetry/SymmFeat.h"
//---------------------------------------------------------------------------
#include "OOBBox3f.h"
//---------------------------------------------------------------------------

class PROJECTSXWU_API SymmReflective : public SymmContainer
{
public:
    typedef boost::shared_ptr< SymmReflective > Ptr;
    typedef boost::shared_ptr< const SymmReflective > ConstPtr;
    typedef std::pair<SymmFeatLine::Ptr, SymmFeatLine::Ptr> SymmFeatLinePair;
    typedef std::pair<mpcard, mpcard> SymmFeatVertexPair;

public:
    SymmReflective(void) {}
    //SymmReflective(const Matrix4f& T_, const Vector3f& N_) :
    //  trans4_(T_), normal_(N_) {}
      size_t size(void) { return featPairs_.size(); }
    void Insert(const SymmFeatLinePair& sflp, const LineDir& dir);
    void SetVertPairs(const std::vector<SymmFeatVertexPair>& vlist) { vertPairs_ = vlist; }
    void Initialize(const Matrix4f& T_, const Vector3f& N_);

    void DrawPlane(const Vector3f& color);
    void DrawWithDR(const Vector3f& color);

public:
    static bool GenerateReflection(
        const Vector3f& p0, const Vector3f& p1,
        Matrix3f& T3, Vector3f& N, Vector3f& C);
    Vector3f GetReflectedPos(const Vector3f& p);
    bool OnFrontSide(const Vector3f& p);

public:
    virtual Matrix4f Get1StepTransformation(void);

    virtual bool GetUpdatedTransformation(
        PointSet  const& deformed,
        AAT       const& defAAT,
        Matrix4f& Tf, Matrix4f& Tb
        );

    virtual std::string GetName(void);
    virtual std::string GetDescription(void);

    virtual void DrawWithDR(
        PointSet  const& deformed,
        AAT       const& defAAT
        );

private:
    void update(const std::vector<Vector3f>& symPoints);

public:
    Matrix4f trans4_;
    Vector3f normal_;

    std::vector<SymmFeatLinePair> featPairs_;
    std::vector<LineDir> lineDir_;
    std::vector<SymmFeatVertexPair> vertPairs_;

    Vector3f cen_proj;
    Vector3f end_proj[3];
    mpcard nn_indx;
};
typedef SymmReflective::SymmFeatLinePair SymmFeatLinePair;
typedef SymmReflective::SymmFeatVertexPair SymmFeatVertexPair;

#endif
