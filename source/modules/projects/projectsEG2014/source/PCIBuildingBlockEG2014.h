#ifndef PCIBuildingBlockEG2014_H
#define PCIBuildingBlockEG2014_H

#include "projectsEG2014.h"
#include "PCInteractionTool.h"
#include "UnstructuredInCoreTriangleMesh.h"
#include "UnstructuredInCorePointCloud.h"
#include "MajorKDTree.h"
#include "FastSphereQuerry.h"
#include "HierarchicalKNNIterator.h"
#include "TriangleMeshHashGrid.h"
#include "AttachmentBuildingBlockUpward.h"


namespace X4
{

    class PROJECTSEG2014_API PCIBuildingBlockEG2014: public PCInteractionTool{
        X4_CLASS(PCIBuildingBlockEG2014)

    private:
		// data
		UnstructuredInCorePointCloud * pcFullres; // full resolution point cloud. 
		UnstructuredInCorePointCloud * pcFeature; // feature point cloud
		UnstructuredInCorePointCloud * pcSegmentation;

		//folder name & file name
		string name_dirRoot; // name of root folder
		string name_dirPCD; // directory of pcd output
		string name_pcFullres; // name of full resolution pc
		string name_pcFeature; // name of features pc
		string name_pcSegmentation; // name for segmentation pc

		// scene graph nodes
		SGListNode *nodeRoot;
		SGObjectNode *sourceNO;

		//rendering
		std::vector<Vector3f> colorList;

		////-------------------------------------------------------------------------------------------------
		//// attachment
		AttachmentBuildingBlockUpward* att; // only store irregular data

		// feature
		card32 numFeature;

		// segment
		card32 numSegmentation;

		// clique
		card32 numCliqe;
		std::vector<DVectorF> clique;
		DVectorF idxTemplate2Feature;

		// building block
		card32 numBB;
		DVectorF idxBB2Clique;
		DMatrixF CooC_M_dist;

		// supervision
		card32 numTemplate;
		std::vector<DVectorF> userTemplateFeature;
		std::vector<DVectorF> idxKey2Feature; // for each PM, stores the index from all parts to pcFeature
		std::vector<Vector3f> pm_bbox_lowerCorner;
		std::vector<Vector3f> pm_bbox_upperCorner;
		std::vector<Vector3f> pm_bboxCenter;
		std::vector<DVectorF> pm_coordFrame;

		//-----------------------------------------------------------
		// functions
		//-----------------------------------------------------------
		// helper
		void recScene();
		void updateAttachment();
		void loadAttachment();
		
		//// rendering
		void generateColor(card32 num);
		void renderBB();
		void renderClique();


    public:

        PCIBuildingBlockEG2014();
        ~PCIBuildingBlockEG2014();

		// interaction tool interface
		virtual void keyDown(GeneralKey key);
		virtual void keyUp(GeneralKey key) {}
		virtual void mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
		virtual void mouseMoved(int32 x, int32 y);
		virtual void mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
		virtual void mouseWheelRotated(int32 rotatedDelta, Modifiers modifiersState);
		virtual void areaResize(card32 width, card32 height) {}
		virtual void glDrawTool(GLContext *glContext) {}
    };

}

#endif

/*
A demo tool for different features
*/