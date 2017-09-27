#ifndef SymmLineSampler_H
#define SymmLineSampler_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
#include "SToolBox/SamplerSymmBlock.h"
#include "Util/TrimeshUnorderedGrid.h"
//---------------------------------------------------------------------------

class PROJECTSXWU_API SymmLineSampler
{
public:
    typedef boost::shared_ptr< SymmLineSampler > Ptr;
    typedef boost::shared_ptr< const SymmLineSampler > ConstPtr;

public:
    virtual ~SymmLineSampler() {}
    virtual std::string TName(void) { return "SymmLineSampler"; }

public:
    SymmLineSampler();
    static void DoSample(
        Scene* scene,
        const std::string& dataFolder,
        const std::string& symmWorkNameBase,
        const std::string& inputSymmName,
        const std::string& symmGroupNameBase,
        UnorderedGridUniq* p_pointGrid,
        const float& sampleth = 0.1
        );
};

#endif
