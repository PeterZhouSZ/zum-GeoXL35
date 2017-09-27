#ifndef SamplerSymmBlock_H
#define SamplerSymmBlock_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------

struct PointSymmBlock : public AttachedData
{
    DEFINE_CLASS( PointSymmBlock );
    virtual void assign(const Object* obj, COPY_CONTEXT *context /* = nullptr */);
    virtual void read(InputObjectStream *s);
    virtual void write(OutputObjectStream *s) const;

    card32 elem_; // index of representative element
    std::deque<card32> pindx_; // indice (in the sample set) of all elements on the this orbit
    struct PointSymmIndex { card32 symm_; card32 ordx_; }; // corresponding symmetry group index, and transformation index
    std::deque<PointSymmIndex> knots_;
};
struct PointSymmBlockVec : public AttachedData
{
    DEFINE_CLASS( PointSymmBlockVec );
    ~PointSymmBlockVec()
    {
        for (unsigned ii = 0; ii < blockvec_.size(); ++ii) delete blockvec_[ii];
        blockvec_.clear();
    }
    static std::string getDefaultName() { return "PointSymmetryBlockAttachment"; }
    std::vector< PointSymmBlock* > blockvec_;
    IMPLEMENT_OBJECT_LIST_METHODS_STL_NO_PREFIX_M(blockvec_, PointSymmBlock);
};

//--------------------------------------------------------------------------------------

struct LineSymmBlock : public AttachedData
{
    DEFINE_CLASS( LineSymmBlock );
    virtual void assign(const Object* obj, COPY_CONTEXT *context /* = nullptr */);
    virtual void read(InputObjectStream *s);
    virtual void write(OutputObjectStream *s) const;

    card32 elem_; // index of representative element
    std::deque<card32> pindx_; // indice of all elements on the this orbit
    std::deque< std::deque<card32> > knots_; // corresponding sample indice for all lines
};
struct LineSymmBlockVec : public AttachedData
{
    DEFINE_CLASS( LineSymmBlockVec );
    ~LineSymmBlockVec()
    {
        for (unsigned ii = 0; ii < blockvec_.size(); ++ii) delete blockvec_[ii];
        blockvec_.clear();
    }
    static std::string getDefaultName() { return "LineSymmetryBlockAttachment"; }
    std::vector< LineSymmBlock* > blockvec_;
    IMPLEMENT_OBJECT_LIST_METHODS_STL_NO_PREFIX_M(blockvec_, LineSymmBlock);
};

#endif
