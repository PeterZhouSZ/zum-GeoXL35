#ifndef DefieldSymmRotation_H
#define DefieldSymmRotation_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "SToolBox/DefieldSymm.h"
//---------------------------------------------------------------------------
#include "ObjectListProperty.h"
#include "Plane3f.h"
//---------------------------------------------------------------------------

class PROJECTSXWU_API DefieldSymmRotation : public DefieldSymm
{
    DEFINE_CLASS( DefieldSymmRotation )
public:
    typedef boost::shared_ptr< DefieldSymmRotation > Ptr;
    typedef boost::shared_ptr< const DefieldSymmRotation > ConstPtr;

public:
    DefieldSymmRotation(void);
    void Sort(const PointSet* samPS);

    virtual void assign(const Object* obj, COPY_CONTEXT *context /* = nullptr */);
    virtual void read(InputObjectStream *s);
    virtual void write(OutputObjectStream *s) const;

    virtual inline std::string GetClassName(void) { return "Rotation"; }
    virtual Vector3f GetNormalDirection(const Vector3f& pos, const Vector3f& ref) const;
    virtual void DrawWithDR(
        PointSet  const& deformed,
        AAT       const& defAAT
        );
    virtual bool UpdatedTransformation(
        PointSet  const& deformed,
        AAT       const& defAAT
        );

public:
    Matrix4f trans_b_;
    Vector3f center_;
    Vector3f axis_;
    std::deque<unsigned> fixedRigion;
};

class PROJECTSXWU_API DefieldSymmRotationV : public DefieldSymmRotation
{
    DEFINE_CLASS( DefieldSymmRotationV )
public:
    typedef boost::shared_ptr< DefieldSymmRotationV > Ptr;
    typedef boost::shared_ptr< const DefieldSymmRotationV > ConstPtr;

public:
    DefieldSymmRotationV(void);
    ~DefieldSymmRotationV(void);

    virtual void assign(const Object* obj, COPY_CONTEXT *context /* = nullptr */);
    virtual void read(InputObjectStream *s);
    virtual void write(OutputObjectStream *s) const;

    virtual inline std::string GetClassName(void) { return "RotationV"; }
    virtual inline unsigned GetNumSubSymm(void) const;
    virtual DefieldSymm* GetSubSymm(unsigned subs);
    virtual inline unsigned GetNumTransformation(void) const;
    virtual Matrix4f GetTransformation(const int& ii) const;
    virtual int AddOrbitIndice(const std::deque<unsigned>& orbitIndice, const std::deque<Vector3f>& orbitPosition);
    virtual std::deque<OrbitPairTransformation> GetOrbitPairTransformations(void) const;
    virtual void DrawWithDR(
        PointSet  const& deformed,
        AAT       const& defAAT
        );
    virtual bool UpdatedTransformation(
        PointSet  const& deformed,
        AAT       const& defAAT
        );

protected:
    int RefMatching(const std::deque<unsigned>& orbitIndice, const std::deque<Vector3f>& orbitPosition);

public:
    std::vector<DefieldSymmReflection*> reflections_;
};

class PROJECTSXWU_API DefieldSymmRotationH : public DefieldSymmRotation
{
    DEFINE_CLASS( DefieldSymmRotationH )
public:
    typedef boost::shared_ptr< DefieldSymmRotationH > Ptr;
    typedef boost::shared_ptr< const DefieldSymmRotationH > ConstPtr;

public:
    DefieldSymmRotationH(void);
    ~DefieldSymmRotationH(void);

    virtual void assign(const Object* obj, COPY_CONTEXT *context /* = nullptr */);
    virtual void read(InputObjectStream *s);
    virtual void write(OutputObjectStream *s) const;

    virtual inline std::string GetClassName(void) { return "RotationH"; }
    virtual inline unsigned GetNumSubSymm(void) const;
    virtual DefieldSymm* GetSubSymm(unsigned subs);
    virtual inline unsigned GetNumTransformation(void) const;
    virtual Matrix4f GetTransformation(const int& ii) const;
    virtual int AddOrbitIndice(const std::deque<unsigned>& orbitIndice, const std::deque<Vector3f>& orbitPosition);
    virtual std::deque<OrbitPairTransformation> GetOrbitPairTransformations(void) const;
    virtual void DrawWithDR(
        PointSet  const& deformed,
        AAT       const& defAAT
        );
    virtual bool UpdatedTransformation(
        PointSet  const& deformed,
        AAT       const& defAAT
        );

public:
    DefieldSymmReflection* reflection_h_;
};

#endif
