#ifndef PCISymmDet_H
#define PCISymmDet_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
#include "Symmetry/SymmDetCont.h"
//---------------------------------------------------------------------------
#include "PCInteractionTool.h"
//#include "CCSGSymmetryGroup.h"
//---------------------------------------------------------------------------

//namespace NAMESPACE_VERSION {
//    class FeatureSet;
//    class DVectorUObjectList;
//}
//class PROJECTSXWU_API CCSGClassification : public AttachedData
//{
//    DEFINE_CLASS(CCSGClassification);
//public:
//    CCSGClassification() {}
//    virtual ~CCSGClassification();
//
//    static std::string getDefaultName() { return "CCSGClassification"; }
//
//    std::map<std::string, std::vector<int> > shapeMap;
//
//    std::map<unsigned, std::vector<Matrix4f> > symmetryT;
//    std::map<unsigned, std::vector<unsigned> > symmetryId;
//
//    std::vector<CCSGSymmetryGroup*> m_Group;
//    IMPLEMENT_OBJECT_LIST_METHODS_STL(Group, CCSGSymmetryGroup);
//
//    virtual void write(OutputObjectStream *s) const;
//    virtual void read(InputObjectStream *s);
//};

class PROJECTSXWU_API PCISymmDet : public PCInteractionTool
{
    DEFINE_CLASS( PCISymmDet );
public:
    void DetectHiPrecSymmtry(void);
    //void DetectCCSGSymmtry(void);

public:
    PCISymmDet(void);
    ~PCISymmDet(void);

    virtual void mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState) {}
    virtual void mouseMoved(int32 x, int32 y) {}
    virtual void mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState) {}
    virtual void areaResize(card32 width, card32 height) {}
    virtual void keyDown(GeneralKey key) {}
    virtual void keyUp(GeneralKey key) {}
    virtual void glDrawTool(GLContext *glContext) {}

private:
    //std::string selectSymmetryGroupName;
    //FeatureSet* detectFeatureLines(UnstructuredInCoreTriangleMesh* selcloud);
    //DVectorUObjectList* clusterFeatureLines(FeatureSet* fs);
    //pair<UnstructuredInCorePointCloud*, CCSGClassification*> findPreliminarySymmetryGroups(
    //    const std::string& shapeName, UnstructuredInCoreTriangleMesh* selcloud,
    //    FeatureSet* fs, DVectorUObjectList* clusters);
    //pair<UnstructuredInCorePointCloud*, std::vector<UnstructuredInCorePointCloud*>>
    //    extractFinalSymmetricGroups(UnstructuredInCorePointCloud* shape, FeatureSet* _fs,
    //    DVectorUObjectList* _clusters, CCSGClassification* ccsg, InCorePCTopologyGraph* tpg);

    //std::vector<CCSGSymmetryGroup*> findLatticeGroups(FeatureSet* _fs, DVectorUObjectList* _clusters, ProgressWindow* progress);
    //std::vector<CCSGSymmetryGroup*> findOtherGroups(FeatureSet* fs, DVectorUObjectList* clusters, ProgressWindow* progress);
    //std::vector<UnstructuredInCorePointCloud*> extractLatticeGroupElements(std::vector<CCSGSymmetryGroup*>& groups, UnstructuredInCorePointCloud* shape, InCorePCTopologyGraph* tpg, FeatureSet* fs, DVectorUObjectList* clusters, std::vector<char>& assigned);
    //std::vector<UnstructuredInCorePointCloud*> extractOtherGroupElements(std::vector<CCSGSymmetryGroup*>& groups, UnstructuredInCorePointCloud* shape, InCorePCTopologyGraph* tpg, FeatureSet* fs, DVectorUObjectList* clusters, std::vector<char>& assigned);

private:
    SymmDetCont::Ptr symmDetCont;
    //SymmDetContCCSG::Ptr symmDetContCCSG;
};

#endif
