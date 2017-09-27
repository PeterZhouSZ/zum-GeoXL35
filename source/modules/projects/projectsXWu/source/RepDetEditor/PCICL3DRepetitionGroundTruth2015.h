#ifndef PCICL3DRepetitionGroundTruth2015_H
#define PCICL3DRepetitionGroundTruth2015_H

#include "CommonHdrXWu.h"
#include "PCInteractionTool.h"
#include "Box.h"
//#include "CLSettingsGroundTruth2015.h"
//
//#include "CLResultsBoxGroundTruth2015.h"
//#include "CLPRBoxGroundTruth2015.h"

namespace X4
{
	class PROJECTSXWU_API PCICL3DRepetitionGroundTruth2015 :public PCInteractionTool{
		X4_CLASS(PCICL3DRepetitionGroundTruth2015)
	private:

		// name
		std::string pcName;
		std::string dumpName;
	
		// parameters
		float32 m_unit_absDiagonal;
		float32 m_unit_relRadiusFullres;
		float32 m_unit_relRadiusFeature;
		int32 m_preprocess_NormalPCACamera_numNN;
		int32 m_preprocess_NormalPCA_numNN;
		Vector3i m_preprocess_split_v;
		int32 m_sw_numBasis;
		float32 m_sw_sigma;
		int32 m_localsnapping_numCandidate;

		//float32 m_unit_relSizeVoxel;
		//float32 m_unit_relRadiusValidVoxel;
		//int32 m_preprocess_minNumPointsValidVoxel;
		//

		// UI
		int start_x;
		int start_y;
		card32 ST_brushsize; /// size of the brush in pixels
		card32 ST_tolerance; /// tolerance value for color attrib selection
		card32 ST_mode; /// selection mode which is currently activated. 0=Cloud, 1=Point, 2=CircleBrush, 3=RectangleArea, 4=QuadBrush, 5=CircleArea
		bool ST_ctrlPressed; /// true while ctrl key is pressed. Used for deselection mode.
		bool ST_SurfaceMode; /// true if surface mode is activated. See corresponding X4 help file for details of surface mode.
		Vector3f ST_SelColor; /// color for selection by color attrib
		card32 w, h; /// temp var: width and height of window
		float32 WidthToHeight; /// temp var: factor of width to height
		bool leftMBdown; /// temp var: true while left mouse button pressed
		bool rightMBdown; /// temp var: true while right mouse button pressed
		int32 leftMBdownX, leftMBdownY; /// temp var for mouse x and y when mouse button pressed
		int32 leftMBmoveX, leftMBmoveY; /// temp var for mouse x and y when mouse moved
		int32 rightMBdownX, rightMBdownY; /// temp var for mouse x and y when mouse button pressed
		int32 rightMBmoveX, rightMBmoveY; /// temp var for mouse x and y when mouse moved

		// UI mode
		bool flag_createtemplate;
		bool flag_editbox;
		bool flag_manualdetect;
		bool flag_manualcreateclass;
		bool flag_manualcreateobject;

		// data
		UnstructuredInCorePointCloud * pcFullres;

		UnstructuredInCorePointCloud * pcFeature;
		AttachedIndexedOctree * knnOctreeFeature;
		FastSphereQuerry * spherequeryFeature;
		HierarchicalKNNIterator * knnItFeature;

		UnstructuredInCorePointCloud * pcTemplate;

		Box * boxCurrent;
		Box * boxTemplate;
		std::vector<std::vector<Box*>> boxRecord;
		int32 i_classCurrent;
		int32 i_boxCurrent;

		Vector3f translateEdit;
		Vector3f rotateEdit;

		std::vector<Vector3f> colorList;

	public: 
		PCICL3DRepetitionGroundTruth2015();
		~PCICL3DRepetitionGroundTruth2015();

		// preprocess IGN
		void PreprocessIGN();
		void removeoutlier(); // remove outliers 
		void splitdata(); // split big data into smaller scenes
		void points2x4obj(); // save ascii to x4obj

		// preprocess General
		void PreprocessGeneral();
		void flipyz();
		void computeDiagonal();

		// Create GT
		void initialdatastructure();
		void saveGT();
		void loadGT();

		// Create GT (Semi Auto)
		void createtempalte();
		void manualdetect();

		// Create GT (Manual)
		void manualcreatenewclass();
		void manualcreatenewobject();

		// edit
		void translateX(int32 step);
		void translateY(int32 step);
		void translateZ(int32 step);
		void rotateZ(int32 step);

		void semanticlabelassign(int32 l);
		void renderrigid();
		void rendersemantic();

		// misc
		void computeboxTemplate();
		void computeboxCurrent();
		bool checkpointinbox(Vector3f pos2center, Matrix3f world2loc, Vector3f boundary);
		void localsnapping(Vector3f posRef, int32 numCandidate);
		void initializeParameters();

		// UI
		void setPcName(std::string name);
		std::string getPcName() const { return pcName; }
		void setDumpName(std::string name);
		std::string getDumpName() const { return dumpName; }
		void setRelRadiusFullres(float32 RelRadiusFullres);
		float32 getRelRadiusFullres(){ return m_unit_relRadiusFullres; }

		// render
		void drawCube(const Vector3f &pbegin, const Vector3f &pend, const Vector3f &color, const float &linewidth);
		void drawQuad(const Vector3f &p1, const Vector3f &p2, const Vector3f &p3, const Vector3f &p4, const Vector3f &color, const float &linewidth);
		void drawCubeTR(const Vector3f &pbegin, const Vector3f &pend, const Vector3f &color, const float &linewidth, Vector3f T, Matrix3f R);
		void drawBox(Box * box, Vector3f color, float32 lindwidth);
		void generateColor(std::vector<Vector3f> & colorList, card32 num);

		// customized key
		void keyUpCleanboxCurrent();
		void keyUpSaveboxCurrent();
		void keyUpDeleteCurrent();
		
		//UI inherited rendering functions
		virtual void keyDown(GeneralKey key);
		virtual void keyUp(GeneralKey key);
		virtual void mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
		virtual void mouseMoved(int32 x, int32 y);
		virtual void mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState);
		virtual void mouseWheelRotated(int32 rotatedDelta, Modifiers modifiersState);
		virtual void areaResize(card32 width, card32 height);
		virtual void glDrawTool(GLContext *glContext);
		void drawRectangle(float32 x1, float32 y1, float32 x2, float32 y2);
		void drawFilledRectangle(float32 x1, float32 y1, float32 x2, float32 y2);
		void drawCircle(float32 x1, float32 y1, float32 r);
		void drawFilledCircle(float32 x1, float32 y1, float32 r);
		void renderSelectedObject();		
	};
}

#endif