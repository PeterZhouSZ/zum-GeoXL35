#ifndef SymmSampler_H
#define SymmSampler_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
#include "SToolBox/SymmSampleGraph.h"
//---------------------------------------------------------------------------

class UnorderedGridUniq;

class PROJECTSXWU_API SymmSampler
{
public:
    typedef boost::shared_ptr< SymmSampler > Ptr;
    typedef boost::shared_ptr< const SymmSampler > ConstPtr;

    // otherwise group index
    enum { UNTOUCHED = -1, FEATURE_LINE = -2, FILL_IN = -3, PRE_ALLOCATED = -4, OVERLAPPING = -5, PART_SEP = 100 };

public:
    virtual ~SymmSampler() {}
    virtual std::string TName(void) { return "SymmSampler"; }

public:
    SymmSampler();
    static void DoSample(
        Scene* scene,
        const std::string& dataFolder,
        const std::string& symmWorkNameBase,
        const std::string& inputSymmName,
        const std::string& symmGroupNameBase,
        UnorderedGridUniq* p_pointGrid,
        const float& sampleth = 0.1
        );

    static int32 GroupCode(const Vector2i& groupId);
    static Vector3f MapGroupColor(const int32& groupId);
    static Vector3f MapGroupColor(const Vector2i& groupId);
    static int PartIndex(const int groupID);
    static int GroupSN(const int groupID);

public:
    //typedef int (*GenerateGroupHandler) (
    //    sym::SymmetryGroup* group, DefieldSymm* symm,
    //    Vector3f pos, HierarchicalKNNIterator* hItDet,
    //    HierarchicalKNNIterator* hItSam, std::deque<int> const& mark,
    //    std::deque<Vector3f>& pcand
    //    );
    //std::map<std::string, GenerateGroupHandler> GenerateGroupHandlerMap;

    //typedef int (*ConstructGroupHandler) (
    //    DefieldSymm* symm, const card8& groupIndex, UICPC* outpoisson,
    //    const std::deque<Vector3f>& pcand, std::deque<unsigned>& pindx,
    //    UnorderedGridUniq& pointgrid
    //    );
    //std::map<std::string, ConstructGroupHandler> ConstructGroupHandlerMap;
};

class PROJECTSXWU_API SymmSamplerDihedral
{
public:
    typedef boost::shared_ptr< SymmSamplerDihedral > Ptr;
    typedef boost::shared_ptr< const SymmSamplerDihedral > ConstPtr;

public:
    virtual ~SymmSamplerDihedral() {}
    virtual std::string TName(void) { return "SymmSamplerDihedral"; }
};

#endif
