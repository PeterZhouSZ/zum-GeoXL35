#ifndef PCICosyn_H
#define PCICosyn_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "SingleScene.h"
//---------------------------------------------------------------------------
#include "PCInteractionTool.h"
//---------------------------------------------------------------------------

class PROJECTSXWU_API PCICosyn : public PCInteractionTool
{
    DEFINE_CLASS( PCICosyn )

public:
    PCICosyn(void);
    ~PCICosyn(void);
    virtual void assign(const Object* obj, COPY_CONTEXT *context = nullptr);

    void GenerateRandomRangeScans(void);
    void StitchScans(void);
    void InferPatches(void);

    void ExtractCurrentResult(void);
    void VisualizeResult(void);

public:
    virtual void keyDown(GeneralKey key) { keyDownPassToCam(key); }
    virtual void keyUp(GeneralKey key) { keyUpPassToCam(key); }
    virtual void mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState) { mouseDownPassToCam(x, y, buttonsState, modifiersState); }
    virtual void mouseMoved(int32 x, int32 y) { mouseMovedPassToCam(x, y); }
    virtual void mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState) { mouseUpPassToCam(x, y, buttonsState, modifiersState); }
    virtual void areaResize(card32 width, card32 height) { areaResizePassToCam(width, height); }
    virtual void mouseWheelRotated(int32 rotatedDelta, Modifiers modifiersState) { mouseWheelRotatedPassToCam(rotatedDelta, modifiersState); }

private:
    std::string data_folder;
    std::string data_source;
    std::string range_scan_folder;
    std::string range_scan_prefix;
    std::string musc_folder;
    std::string feature_folder;
    std::string result_folder;
    std::string result_prefix;

    unsigned range_scan_num;
    unsigned num_infer_trans;
    float cell_size;
    unsigned verbose;

    std::deque< SingleScene::Ptr > single_scene_vec_;
};

#endif
