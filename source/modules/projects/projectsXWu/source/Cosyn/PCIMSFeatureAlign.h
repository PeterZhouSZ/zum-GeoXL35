//---------------------------------------------------------------------------
#ifndef PCIMSFeatureAlignH
#define PCIMSFeatureAlignH
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "Util\MultiScalePCCreator.h"
#include "Cosyn\C13MultiScaleFeatureDetector.h"
#include "Cosyn\C13MultiScaleFeatureDescriptor.h"
//---------------------------------------------------------------------------
#include "PCInteractionTool.h"
#include "SGRelativeTimeAnimationNode.h"
//---------------------------------------------------------------------------

class PROJECTSXWU_API PCIMSFeatureAlignSettings : public AttachedData
{
    DEFINE_CLASS(PCIMSFeatureAlignSettings);
public:
    card32 knnTopology;

    bool show_icl_icp;
    bool use_region_growing;
    float icl_dist_ratio;
    float icl_min_score;
    float support_ratio;
    float icp_init_radius_radio;
    float outlier_dist_factor;

    PCIMSFeatureAlignSettings();
    ~PCIMSFeatureAlignSettings();
    static const char* getDefaultName() {return "PCIMSFeatureAlignSettings";}
};

class PROJECTSXWU_API PCIMSFeatureAlign : public PCInteractionTool
{
    DEFINE_CLASS( PCIMSFeatureAlign )

public:
    PCIMSFeatureAlign(void);
    ~PCIMSFeatureAlign(void);
    void execute(void);

    void createFolders(void);
    void convert2MSPC(void);
    void CoarseSegmentation(void);
    void detectFeatures(void);
    void extractSupports(void);
    void extractCrossPoints(void);
    void computeDescriptors(void);
    void computeCrossPointTpg(void);
    void alignLBase(void);
    void geometricValidation(void);
    void CleanTransformations(void);

    void showBoundaryLines(void);
    void showPlanes(void);
    void showFeatureLineClusters(void);
    void showLBaseAlignment(void);
    void showLBases(void);

public:
    virtual void keyDown(GeneralKey key) { keyDownPassToCam(key); }
    virtual void keyUp(GeneralKey key) { keyUpPassToCam(key); }
    virtual void mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState) { mouseDownPassToCam(x, y, buttonsState, modifiersState); }
    virtual void mouseMoved(int32 x, int32 y) { mouseMovedPassToCam(x, y); }
    virtual void mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState) { mouseUpPassToCam(x, y, buttonsState, modifiersState); }
    virtual void areaResize(card32 width, card32 height) { areaResizePassToCam(width, height); }
    virtual void mouseWheelRotated(int32 rotatedDelta, Modifiers modifiersState) { mouseWheelRotatedPassToCam(rotatedDelta, modifiersState); }

private:
    static MultiScaleInCorePointCloud* getMSPCFromListNode(SGListNode* node, const unsigned& id);
    static InCorePCTopologyGraph* buildKNN(PointSet* ups, const card32& nnnum = 4);

private:
    std::string dataFolder;
    std::string muscFolder;
    std::string featFolder;
    std::string crosFolder;

    card32 minLevel;
    card32 maxLevel;
    PCIMSFeatureAlignSettings      *settings_;

    MultiScalePCCreator            *multiScalePCCreator;
    C13MultiScaleFeatureDetector   *featureDetector;
    C13MultiScaleFeatureDescriptor *featureDescriptor;
};

#endif
