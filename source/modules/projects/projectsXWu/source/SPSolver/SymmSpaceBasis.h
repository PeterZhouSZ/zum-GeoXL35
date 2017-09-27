//----------------------------------------------------------------------
#ifndef SymmSpaceBasis_h_
#define SymmSpaceBasis_h_
//----------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//----------------------------------------------------------------------
//----------------------------------------------------------------------

struct SymmSpaceBasisAxis
{
    SymmSpaceBasisAxis() : axis(NULL_VECTOR3F), delta(0) {}
    float operator*(const Vector3f& v) const { return axis * v; }
    Vector3f evaluate(const float& s) const { return s * axis; }
    Vector3f evaluate(void) const { return delta * axis; }
    Vector3f axis; // basis axis direction
    float delta; // displacement on the basis axis
};

template<unsigned DIM = 3>
class SymmSpaceBasis
{
public:
    typedef boost::shared_ptr< SymmSpaceBasis > Ptr;
    typedef boost::shared_ptr< const SymmSpaceBasis > ConstPtr;

    enum { Dim = DIM };
    typedef unsigned index_type;
    typedef SymmSpaceBasisAxis value_type;
    typedef std::array<value_type, DIM> basis_type;

public:
    SymmSpaceBasis() : oringin(NULL_VECTOR3F) {}

    SymmSpaceBasis(const basis_type& B, const Vector3f& O) : basis(B), oringin(O)
    {
    }

    value_type& operator[](const index_type& index) { return basis[index]; }
    const value_type& operator[](const index_type& index) const { return basis[index]; }

    StaticVector<float, DIM> operator*(const Vector3f& v) const
    {
        StaticVector<float, DIM> ret;
        ret.setZero();
        for (index_type bi = 0; bi < DIM; ++bi) {
            ret[bi] = basis[bi] * v;
        }
        return ret;
    }
    Vector3f evaluate(const StaticVector<float, DIM>& v) const
    {
        Vector3f ret;
        ret.setZero();
        for (index_type bi = 0; bi < DIM; ++bi) {
            ret += basis[bi].evaluate(v[bi]);
        }
        return ret;
    }

    Vector3f evaluate(void) const
    {
        Vector3f ret;
        ret.setZero();
        for (index_type bi = 0; bi < DIM; ++bi) {
            ret += basis[bi].evaluate();
        }
        return ret;
    }

    void DebugDraw(void) const
    {
        for (unsigned d = 0; d < DIM; ++d) {
            Vector3f color = NULL_VECTOR3F;
            color[d % 3] = 1.f;
            debugRenderer->addLine(
                oringin, oringin + basis[d].axis,
                color, color,
                1);
        }
        debugRenderer->addPoint(
            oringin, makeVector3f(1, 1, 0)
            );
    }

    void DebugDraw(const Vector3f& pos) const
    {
        for (unsigned d = 0; d < DIM; ++d) {
            Vector3f color = NULL_VECTOR3F;
            color[d % 3] = 1.f;
            debugRenderer->addLine(
                pos, pos + basis[d].axis,
                color, color,
                1);
        }
        debugRenderer->addPoint(
            pos, makeVector3f(1, 1, 0)
            );
    }

    //static int BuildBasis(
    //    SceneObject* samobj,
    //    std::deque<SparseMatrixD>& basisVec
    //    );

private:
    basis_type basis;
    Vector3f oringin;
};

#endif
