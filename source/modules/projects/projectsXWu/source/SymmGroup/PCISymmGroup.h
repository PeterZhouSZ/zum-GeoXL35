//---------------------------------------------------------------------------
#ifndef PCI_SYMM_GROUP_H
#define PCI_SYMM_GROUP_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
#include "PCInteractionTool.h"
#include "PointCloud.h"
#include "ObjectListProperty.h"
//---------------------------------------------------------------------------
#include <RenderingTool.h>
#include <InCorePCTopologyGraph.h>
#include <LocalDescriptor.h>
#include <omp.h>
#include <QMutex>
#include <QWaitCondition>

#include <SymmetryGroup.h>
#include <detector/FeatureLine.h>
#include "SymmetryGroupAttachment.h"

namespace NAMESPACE_VERSION {


    /**
    TODO after Siggraph deadline:

    - a Dihedral group can be created by combining Cv and Ch. Check for this case also. It would mean that in order to build Dh we not only
    search for a 2fold rotation but rather also search for Ch in case if we search for a complimentary group of Cv or vice versa.

    - add additional spatial tolerance when combining groups. The spatial tolerance for detecting groups ius sometimes better be different then this one

    - improve group combination by somehow comparing the regions in a better way. maybe using the detectBaseElement() to combine groups, in order to remove 
    geometry which is over continuous regions

    - try to deal with 2fold and reflections symmetries. They sometimes cause to clutter the solution with a lot of other small stuff. This is 
    might be related to the previous step of improvement

    - there are sometime similar (or even subgroups) symmetries identified in the same region but wiht lower number of points. in particular they
    are covered by the larger symmetries, but this isn't identified. need to investigate why. i suppose this is the matter of thresholds
    and might be already solved by first two improvements

    - same as with Lattice structure in case of a rotational group we might be interested in removing the symmetric part and keeping the generator only
    however there would be a trouble if part of this symmetric geometry is used in another group. here some kind of group region consolidation is required
    which checks, which groups are actually the result of a larger group and removes them. This is less a symmetry detection problem rather then
    the main idea if we want to keep them or not.

    - in case of sphere objects we find an explosion of symmetries, this need to be treated or assumed no spheres

    - sometimes lattice generator starts from another position as its generator placed in
    */

    class SceneEditorWidget;
    class DVectorUObjectList;
    class FeatureSet;
    class SGListNode;
    class Statistics;

    /**
    * Settings 
    **/
    class PROJECTSXWU_API SymmGroupSettings : public AttachedData
    {
        DEFINE_CLASS(SymmGroupSettings);
    public:
        SymmGroupSettings();
        ~SymmGroupSettings();

        static const char* getDefaultName() {return "SymmGroupSettings";}

        int numCPUCores;

        float32 adjMaxDistance;
        float32 descriptorRadiusBBFactor;

        float32 proximityRelationFactor;

        float32 shapeSymmThreshold;

        int32 maxIcpIterations;
        float32 icpOutlierDistanceFactor;
        float32 icpMinPercentInliers;
        card32 minConnectedComponent;
        float32 distCompareSubpartThreshold;
        float32 symGroupCompareThreshold;
        float32 smallPartMinimalSizeFactor;

        int getNumAvailableThreads();

        float32 latticeMinGeneratorLength;
        float32 spatialTolerance;
        float32 angleTolerance;
        float32 detectorAngleTolerance;
        float32 symmetryCoveredPercentage;
        float32 symMeshSubsampling;
        float32 gridAngleTolerance;

        float32 rotGroupCompareSigma;
        float32 refGroupCompareSigma;
        float32 latGroupCompareSigma;
        float32 dihGroupCompareSigma;
        float32 angelGroupGroupCompareSigma;

        float32 minFeatureLineLength;

        float32 symGroupCombineAngleTolerance;

        bool useICPWhileDetectingBaseElements;
        bool reuseSegmentation;
        bool searchLattice;
        bool runPreliminaryPlausibilityCheck;
        bool clusterFoundGroupsBeforeExtractGeometry;
        bool performTopologyCheckOnFoundGroup;

        card32 minNumRotations;

        Vector4f gidSelectedColor;
        Vector4f gidSymmetricColor;
        Vector4f gidDefaultColor;
        Vector4f gidNonrelevantColor;

        //int32 embeddingDimensions;
        float32 embeddingCutoffEigenvalue;

        bool doVeryConservativeRegionGrowing;
    };


    /**
    * Attachment in the scene describing the classification
    **/
    class PROJECTSXWU_API SymmGroupClass : public AttachedData
    {
        DEFINE_CLASS(SymmGroupClass);
    public:
        SymmGroupClass();
        virtual ~SymmGroupClass();

        static std::string getDefaultName() { return "SymmGroupClass"; }

        std::string loggedOutput;
        Statistics* stats;

        std::map<std::string, std::vector<int> > shapeMap;

        std::map<unsigned, std::vector<Matrix4f> > symmetryT;
        std::map<unsigned, std::vector<unsigned> > symmetryId;

        std::vector<sym::SymmetryGroupAttachment*> m_symGroup;
        IMPLEMENT_OBJECT_LIST_METHODS_STL(symGroup, sym::SymmetryGroupAttachment);

        std::map<unsigned, std::vector<unsigned> > regions;
        std::map<unsigned, std::vector<unsigned> > regionGroupsId;

        virtual void write(OutputObjectStream *s) const;
        virtual void read(InputObjectStream *s);
    };

    /**
    * Detector class, implement here main functions.
    **/
    class PROJECTSXWU_API PCISymmGroup : public PCInteractionTool
    {
        DEFINE_CLASS( PCISymmGroup )
    private:
        unsigned	m_ScreenWidth, m_ScreenHeight;
        SymmGroupSettings* m_Settings;
        SceneEditorWidget* sceneEditorW;

        void connectToSceneImpl(Scene *scene,OnSceneChangeCallback *onChange, QWidget *sceneEditorWidget, QWidget* toolWidget);

        UnstructuredInCorePointCloud* getGraphNode(SGObjectNode* node);
        UnstructuredInCorePointCloud* extractSegment(UnstructuredInCorePointCloud* cloud, int cid, bool localcid = true);

        void rebuildMaterials();

        std::vector<SGObjectNode*> segmentObject(UnstructuredInCoreTriangleMesh* mesh, InCorePCTopologyGraph* top, int& cid);

        //std::vector<sym::SymmetryGroupAttachment*> combineFoundGroups(UnstructuredInCorePointCloud* shape, std::vector<sym::SymmetryGroupAttachment*> groups);
        std::vector<sym::SymmetryGroupAttachment*> findOtherGroups(FeatureSet* fs, DVectorUObjectList* clusters, ProgressWindow* progress);
        std::vector<sym::SymmetryGroupAttachment*> findLatticeGroups(FeatureSet* _fs, DVectorUObjectList* _clusters, ProgressWindow* progress);

        //std::vector<UnstructuredInCorePointCloud*> extractLatticeGroupElements(std::vector<sym::SymmetryGroupAttachment*>& groups, UnstructuredInCorePointCloud* shape, InCorePCTopologyGraph* tpg, FeatureSet* fs, DVectorUObjectList* clusters, std::vector<char>& assigned);
        std::vector<UnstructuredInCorePointCloud*> extractOtherGroupElements(std::vector<sym::SymmetryGroupAttachment*>& groups, UnstructuredInCorePointCloud* shape, InCorePCTopologyGraph* tpg, FeatureSet* fs, DVectorUObjectList* clusters, std::vector<char>& assigned);

        pair<UnstructuredInCorePointCloud*, SymmGroupClass*> findPreliminarySymmetryGroups(const std::string& shapeName, UnstructuredInCoreTriangleMesh* selcloud, FeatureSet* fs, DVectorUObjectList* clusters);
        pair<UnstructuredInCorePointCloud*, std::vector<UnstructuredInCorePointCloud*>> extractFinalSymmetricGroups(UnstructuredInCorePointCloud* shape, FeatureSet* _fs, DVectorUObjectList* _clusters, SymmGroupClass* ccsg, InCorePCTopologyGraph* tpg);
        pair<UnstructuredInCorePointCloud*, std::vector<UnstructuredInCorePointCloud*>> detectAndExtractFinalSymmetries(const std::string& shapeName, UnstructuredInCoreTriangleMesh* shape, FeatureSet* fs, DVectorUObjectList* cluster);

        Vector4f symColorDetected;
        Vector4f symColorVirtual;
        Vector4f symRotColorAxis;
        float32  symRotAxisThickness;

        Vector3f m_2DPattern_U;
        Vector3f m_2DPattern_V;
        Vector2i m_2DPattern_Size;

        bool doRenderSelected;

        // selected render symmetries
        std::map<card32, float32> selectedSymDistanceMap;
        float32 selectedSymDistanceThreshold;
        bool doScreenshotWhileSelecting;

        typedef enum _State
        {
            IDLE,
            SELECT_GROUPS,
            SELECT_GENERATOR
        }State;

        State m_State;

        struct GroupSelection
        {
            std::vector<card32> markedGIDs;
            UnstructuredInCorePointCloud* nonsymmetric;
            std::vector<UnstructuredInCorePointCloud*> symmetric;
            GroupSelection() : nonsymmetric(nullptr) {}
        };

        // manual symmetry selection
        std::vector<card32> m_selectedGIDs;
        std::vector<GroupSelection> m_markedSymmetricGIDs;
        std::vector<card32> m_selectSymmetryRemainingGIDs;
        std::map<card32, char> m_selectSymmetryAssignedGID;
        UnstructuredInCorePointCloud* m_selectSymmetryPC;
        card32 lastSelectedGID;
        std::string selectSymmetryGroupName;
        std::string selectSymmetryPartsName;
        QMutex selectGeneratorMutex;
        QWaitCondition selectGeneratorCondition;
        bool extractSubgroupsWhileSelecting;
        bool showFoundGroups;
        bool recordSubgroupsWhileSelecting;

        struct GroupEdge
        {
            card32 A;
            card32 B;
            float32 weight;
            enum
            {
                PROXIMITY   = (1 << 0),
                SUBSET   = (1 << 1),
                OVERLAP = (1 << 2),
                SYMREL = (1 << 3),

                // region related properties
                OFFGRAPH = (1 << 4)
            };
            card32 type;
        };
        SGListNode* showGroupGraph;
        std::vector<GroupEdge> groupEdgeMap;
        struct SymmetricElement
        {
            bool nonsymmetric;
            sym::SymmetryGroup* group;
            std::string path;
            card32 index;
            UnstructuredInCorePointCloud* pc;
            Vector3f pcaValues;
            Vector3f pcaVectors[3];
            std::vector<card32> pts;
        };
        struct RegionElement
        {
            std::string path;
            card32 index;
            UnstructuredInCorePointCloud* pc;
            std::vector<card32> groups;

            //! compute minimal distance between both elements
            static float minimalDistance(const RegionElement& elem1, const RegionElement& elem2);
        };

        typedef std::vector<card32> PointIdList;
        // marks a collection of points of a shape which do belong to the same disjoint region
        // such a region must not be symmetric. it is build by a set intersection of all symmetric groups over the same region
        struct DisjointRegion
        {
            PointIdList points; // indices of points 
            std::vector<sym::SymmetryGroup*> groups; // groups the points belong to
        };


        // debug things
        int32 showCluster;
        bool drawFeatureLines;
        float32 featureLineSize;
        bool showSymmetryGraph;
        bool loadMatchForEmbedding;
        bool regionBasedCorrespondences; // is set to activate region based correspondencies, as to siggraph14 submission, previously only node based corr

        float32 embDistanceResultSigma;
        card64 embCurrentlySelectedNode;
        std::vector<DVectorF> tmp_embeddedVectors;
        std::vector<float> tmp_embeddedValues;
        card32 tmp_embeddedVectorsDim;
        card32 tmp_embeddedVectorsNum;
        //bool useSpectralEmbedding;
        bool showBackwardMatching;
        DMatrixF tmp_correpondenceMatrix;
        card32 tmp_currentlySelectedMatchingIdx;

        struct ShapeData
        {
            Vector3f pcaValues;
            card32 numpoints;
            UnstructuredInCorePointCloud* shape;
        };
        ShapeData parseSymmetryDataFile(const std::string& path);
        std::vector<GroupEdge> parseGraphFromFile(const std::string& path, const std::string& file = "data.edg");
        std::vector<SymmetricElement> parseGroupFromFile(const std::string& path, bool loadData = true);
        std::vector<RegionElement> parseRegionsFromFile(const std::string& path, bool loadData = true);
        std::vector<std::vector<card32>> computeRegionCorrespondence(const std::vector<RegionElement>& shapeRegions, const std::vector<SymmetricElement>& shapeB, const ShapeData& shapeBdata, const DMatrixF& correspondence, std::vector<float>& scores);
        void checkAndSetTrianglePointAssignmentCache();

        float computeGroup2GroupScore(
            const pair<SymmetricElement, SymmetricElement>& pair1, const pair<card32,card32>& idxA,
            const pair<SymmetricElement, SymmetricElement>& pair2, const pair<card32,card32>& idxB,
            const DMatrixF& vertexScore);

    protected:
        PCISymmGroup();
        ~PCISymmGroup();
        void execute(void);

        void attachSettingsToScene();

        void loadSelectedClass();
        void computeGraphs();

        //! Mark each point of the selected shape with a regionid identifying a disjoint region created from intersection of all symmetry groups
        void constructDisjointRegions();

        void detectFeatureLines();
        void clusterFeatureLines();
        FeatureSet* detectFeatureLines(UnstructuredInCoreTriangleMesh*);
        DVectorUObjectList* clusterFeatureLines(FeatureSet* fs);

        void extractFinalSymmetricGroups();
        void findPreliminarySymmetryGroups();
        //void findLatticeGroups();
        //void generate2DPattern();

        void debugRenderFeatureLine();
        void debugRenderCluster();

        void showStatisticsAndLogs();

        bool pickPoint(int32 x, int32 y, PointCloud *pc, mpcard &index);
        // ----------- Interaction Tool Interface ---------------------------------
        void keyDown(GeneralKey key);
        void keyUp(GeneralKey key) {}
        void mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
        void mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState) {}
        void mouseMoved(int32 x, int32 y);
        void areaResize(card32 width, card32 height){m_ScreenWidth = width; m_ScreenHeight = height;}
        void glDrawToolImplementation(GLContext *glContext);
        bool drawGLSceneRendering();
    }; 

};
#endif
