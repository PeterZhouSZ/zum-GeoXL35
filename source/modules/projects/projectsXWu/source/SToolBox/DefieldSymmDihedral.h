#ifndef DefieldSymmDihedral_H
#define DefieldSymmDihedral_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "SToolBox/DefieldSymmRotation.h"
//---------------------------------------------------------------------------
#include "ObjectListProperty.h"
#include "Plane3f.h"
//---------------------------------------------------------------------------

class PROJECTSXWU_API DefieldSymmDihedral : public DefieldSymmRotation
{
    DEFINE_CLASS( DefieldSymmDihedral )
public:
    typedef boost::shared_ptr< DefieldSymmDihedral > Ptr;
    typedef boost::shared_ptr< const DefieldSymmDihedral > ConstPtr;

public:
    DefieldSymmDihedral(void);
    ~DefieldSymmDihedral(void);

    virtual void assign(const Object* obj, COPY_CONTEXT *context /* = nullptr */);
    virtual void read(InputObjectStream *s);
    virtual void write(OutputObjectStream *s) const;

    virtual inline std::string GetClassName(void) { return "Dihedral"; }
    virtual inline unsigned GetNumSubSymm(void) const;
    virtual DefieldSymm* GetSubSymm(unsigned subs);
    virtual Vector3f GetNormalPosition(const Vector3f& pos, const Vector3f& ref);
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
    int RotMatching(const std::deque<unsigned>& orbitIndice, const std::deque<Vector3f>& orbitPosition);

public:
    std::vector<DefieldSymmReflection*> reflections_;
};

class PROJECTSXWU_API DefieldSymmDihedralH : public DefieldSymmDihedral
{
    DEFINE_CLASS( DefieldSymmDihedralH )
public:
    typedef boost::shared_ptr< DefieldSymmDihedralH > Ptr;
    typedef boost::shared_ptr< const DefieldSymmDihedralH > ConstPtr;

public:
    DefieldSymmDihedralH(void);
    ~DefieldSymmDihedralH(void);

    virtual void assign(const Object* obj, COPY_CONTEXT *context /* = nullptr */);
    virtual void read(InputObjectStream *s);
    virtual void write(OutputObjectStream *s) const;

    virtual inline std::string GetClassName(void) { return "DihedralH"; }
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
