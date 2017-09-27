#ifndef DefieldSymm_H
#define DefieldSymm_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
#include "ObjectListProperty.h"
#include "Plane3f.h"
//---------------------------------------------------------------------------

namespace NAMESPACE_VERSION {
    namespace sym {
        class SymmetryGroup;
    }
}

struct OrbitPairTransformation // sequentially connected point pair in a orbit
{
    std::deque<unsigned> source;
    std::deque<unsigned> target;
    Matrix4f transformation;
};

class PROJECTSXWU_API DefieldSymm : public AttachedData
{
    DEFINE_ABSTRACT_CLASS( DefieldSymm )
public:
    typedef boost::shared_ptr< DefieldSymm > Ptr;
    typedef boost::shared_ptr< const DefieldSymm > ConstPtr;
    typedef std::map< int32, std::deque< card32 > > OrbitVerticeMap; // sn on the orbit --> indice in this location
    typedef std::deque< std::deque< unsigned > > OrbitStack; // indice in all the orbits

    enum SymmRank { REFLECTION, DIHEDRAL, ROTATION_V, ROTATION_H, DIHEDRAL_H, ROTATION };

public:
    DefieldSymm() : show_(true), numtrans_(0), trans_(IDENTITY4F) {}
    virtual ~DefieldSymm() {}

    virtual inline std::string GetClassName(void) = 0;
    virtual inline std::string GetDescription(void);

    virtual void assign(const Object* obj, COPY_CONTEXT *context /* = nullptr */);
    virtual void read(InputObjectStream *s);
    virtual void write(OutputObjectStream *s) const;

public:
    virtual inline unsigned GetNumSubSymm(void) const;
    virtual DefieldSymm* GetSubSymm(unsigned subs);

    virtual Vector3f GetNormalDirection(const Vector3f& pos, const Vector3f& ref) const = 0; // "canonical" direction for corresponding group
    virtual Vector3f GetNormalPosition(const Vector3f& pos, const Vector3f& ref); // projected onto "canonical" direction
    virtual float GetNormalDelta(const Vector3f& pos, const Vector3f& ref); // displacement on "canonical" direction

    virtual inline unsigned GetNumTransformation(void) const { return numtrans_; }
    virtual inline unsigned GetNumBaseTransformation(void) const { return numtrans_; }
    Matrix4f GetTransformation(void) const { return trans_; }
    virtual Matrix4f GetTransformation(const int& ii) const;
    virtual Matrix4f GetTransformation(int source, int target) const;

    virtual int AddOrbitIndice(const std::deque<unsigned>& orbitIndice, const std::deque<Vector3f>& orbitPosition);
    unsigned GetNumOrbits(void) const { return stack_.size(); }
    bool empty(void) const { return (0 == GetNumOrbits()); }
    virtual std::deque<OrbitPairTransformation> GetOrbitPairTransformations(const unsigned& orx) const;
    virtual std::deque<OrbitPairTransformation> GetOrbitPairTransformations(void) const;

    const OrbitVerticeMap& GetOrbitVerticeMap(void) const { return overt_; }
    const OrbitStack& GetOrbitStack(void) const  { return stack_; }

    virtual bool UpdatedTransformation(
        PointSet  const& deformed,
        AAT       const& defAAT
        ) = 0;

    virtual bool GetCorrespondence(
        std::vector<card32>& points0,
        std::vector<card32>& points1
        );

    virtual bool IsSeries(void);

    virtual void DrawWithDR(
        PointSet  const& deformed,
        AAT       const& defAAT
        );

public:
    static int CreateFromSG(sym::SymmetryGroup* group, DefieldSymm::Ptr& symm);
    static int CreateFromSG(sym::SymmetryGroup* group, DefieldSymm** symm);

    bool GetUpdatedTransformation(
        PointSet  const& deformed,
        AAT       const& defAAT,
        Matrix4f& Tf, Matrix4f& Tb
        );

public:
    int rank_;
    Vector3f color_;

protected:
    OrbitVerticeMap overt_; // used for compute updated transformation by correspondence
    OrbitStack stack_; // used for generate pair-wise transformation for neighboring orbit elements
    card32 numtrans_;
    Matrix4f trans_;
    bool show_;
};

struct DefieldSymmVec : public AttachedData
{
    DEFINE_CLASS( DefieldSymmVec )
public:
    ~DefieldSymmVec() {
        for (unsigned ii = 0; ii < symmvec_.size(); ++ii) delete symmvec_[ii];
        symmvec_.clear();
    }
    static std::string getDefaultName() { return "DefieldSymmetryAttachment"; }
    std::deque<DefieldSymm*> symmvec_;
    //IMPLEMENT_OBJECT_LIST_METHODS_STL_NO_PREFIX_M(symmvec_, DefieldSymm);
    virtual void assign(const Object* obj, COPY_CONTEXT *context /* = nullptr */);
    virtual void read(InputObjectStream *s);
    virtual void write(OutputObjectStream *s) const;
};

class PROJECTSXWU_API DefieldSymmReflection : public DefieldSymm
{
    DEFINE_CLASS( DefieldSymmReflection )
public:
    typedef boost::shared_ptr< DefieldSymmReflection > Ptr;
    typedef boost::shared_ptr< const DefieldSymmReflection > ConstPtr;

public:
    DefieldSymmReflection(void);

    virtual void assign(const Object* obj, COPY_CONTEXT *context /* = nullptr */);
    virtual void read(InputObjectStream *s);
    virtual void write(OutputObjectStream *s) const;

    virtual inline std::string GetClassName(void) { return "Reflection"; }
    virtual Vector3f GetNormalDirection(const Vector3f& pos, const Vector3f& ref) const;
    virtual std::deque<OrbitPairTransformation> GetOrbitPairTransformations(const unsigned& orx) const;
    virtual std::deque<OrbitPairTransformation> GetOrbitPairTransformations(void) const;
    virtual bool UpdatedTransformation(
        PointSet  const& deformed,
        AAT       const& defAAT
        );

public:
    Plane3f plane_;
};

#endif
