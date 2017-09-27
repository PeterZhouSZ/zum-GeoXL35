#ifndef DummySampler_H
#define DummySampler_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
#include "SToolBox/SamplerSymmBlock.h"
#include "SToolBox/DefieldSymm.h"
#include "Util/TrimeshUnorderedGrid.h"
//---------------------------------------------------------------------------

class PROJECTSXWU_API DummySampler
{
public:
    typedef boost::shared_ptr< DummySampler > Ptr;
    typedef boost::shared_ptr< const DummySampler > ConstPtr;

public:
    virtual ~DummySampler() {}
    virtual std::string TName(void) { return "DummySampler"; }

public:
    static void DoSample(
        Scene* scene,
        const std::string& symmWorkName,
        const std::string& inputSymmName,
        UnorderedGridUniq* pPointgrid,
        const float& sampleth = 0.1
        );
};

#endif
