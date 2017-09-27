#ifndef PCICLPointCloudProcessing_H
#define PCICLPointCloudProcessing_H

#include "CommonHdrXWu.h"
#include "PCInteractionTool.h"
#include "RegularVolume.h"


namespace X4
{
    class PROJECTSXWU_API PCICLPointCloudProcessing : public PCInteractionTool{
		X4_CLASS(PCICLPointCloudProcessing)

	private:
		//UnstructuredInCorePointCloud * pc;
		//UnstructuredInCorePointCloud * pcsampled;
		//UnstructuredInCorePointCloud * feat;

		struct Surfel
		{
			Vector3f pos;
			Vector3f normal;
			float weight;
			float dynamicWeight;
		};


	public:

		PCICLPointCloudProcessing();
		~PCICLPointCloudProcessing();
		//void setup(UnstructuredInCorePointCloud * pc_input); // set up the process for a point set 
		//UnstructuredInCorePointCloud* getPC();
		//UnstructuredInCorePointCloud* getPCSampled();
		//UnstructuredInCorePointCloud* getFeature();

		void computeSmoothNormal(PointCloud* pc, float32 m_CV_relBaseSigmaVertex);
		void computeSmoothNormal(PointCloud* pc, float32 m_CV_relBaseSigmaVertex, float32 m_AbsDiagonal);
		void computeSmoothNormalParallel(PointCloud* pc, float32 m_CV_relBaseSigmaVertex, float32 m_AbsDiagonal);
		void computeCurvatureRobust(PointCloud *pc, float32 baseSigma, BOOL usePCANormal, float32 maxCurvature);
		void computeCurvatureRobust(PointCloud *pc, float32 baseSigma, BOOL usePCANormal, float32 maxCurvature, float32 m_AbsDiagonal);
		void computeCurvatureRobustParallel(PointCloud *pc, float32 baseSigma, float32 maxCurvature, float32 m_AbsDiagonal);
		void computeCuratureKNN(PointCloud *pc, float32 maxCurvature, card32 numNN, float32 baseSigma, float32 diagonal);

		//void computeTF4Feature(UnstructuredInCorePointCloud* feature, float32 baseSigma, BOOL usePCANormal);
		//void computeStatisticsCurvature(UnstructuredInCorePointCloud* feature, float32 baseSigma, card32 numOrientationBins);
		//std::vector<std::vector<Vector3f>> computeFRFCurvature(UnstructuredInCorePointCloud* feature, card32 numOrientationBins);
		//std::vector<std::vector<Vector3f>> computeFRFUniformCurvature(UnstructuredInCorePointCloud* feature, card32 numOrientationBins);
		//std::vector<DVectorF> computeFeaturefromFRFCurvature(UnstructuredInCorePointCloud* feature, std::vector<std::vector<Vector3f>> frf);
		//
		//// compute normal
		void computeNormalPCAUnify(PointCloud * pc, float32 radiusResampler, card32 numNN, card32 numNNsampled);
		void computeNormalPCACamera(PointCloud * pc, card32 numNN);
		void computeNormalPCA(PointCloud * pc, card32 numNN);

		// compute slippage
		void computeSlippageTranslation(PointCloud * pc, float32 m_Slippage_relBaseSigma, float32 m_AbsDiagonal);

		// remove outline
		void removeOutlier_dist2KNN(PointCloud * pc, card32 k, float32 sigma_outlier);

		// tools
		void computeUnify(PointSet * ps, string name_attribute, card32 seed, card32 numNN);

		// local frame
		std::vector<std::vector<Vector3f>> computeRFUniformCurvature(PointCloud * pcFeature, card32 numOrientationBins);
		std::vector<DVectorF> computeFeaturefromRF(PointCloud* pcFeatureSingle, PointCloud* pcFeatureMulti, std::vector<std::vector<Vector3f>> rf);
		void computeFeaturefromRFAAT(PointCloud* pcFeatureSingle, PointCloud* pcFeatureMulti, std::vector<std::vector<Vector3f>> rf);

		// sampling from mesh
		void sampleMeshRandomly(UnstructuredInCoreTriangleMesh *sourceMesh, UnstructuredInCorePointCloud * pc, card64 expNumPoints, bool createTangentDiscs,
			float32 tangentDiscOversampling, bool surfaceNormalSmoothing, bool trackTriangleID);

		// clip mesh by RegularVolume
		void LinefromVoxel(Vector3f pbegin, Vector3f pend, int32 idx, Line3f &l);
		void facefromVoxel(Vector3f pbegin, Vector3f pend, std::vector<Plane3f> &p);
		void edgesfromTriangle(std::vector<Vector3f> p, std::vector<Line3f> & edges);
		void clipMeshbyRegularVolume(UnstructuredInCoreTriangleMesh * mesh, RegularVolume * volume, UnstructuredInCoreTriangleMesh * meshClipped, float32 clip_min_vertexdist_pos, float32 clip_min_vertexdist_neg, vector<vector<int>> & idx_voxel2triangleClipped);

		// icp
		bool computeICP_R2(PointSet * ps_source, PointSet * ps_target, const Vector3f &pos_ref_source, Vector3f &pos_ref_target, HierarchicalKNNIterator * m_hIt_source, HierarchicalKNNIterator * m_hIt_target, float32 m_MedPointDistance, int32 m_MaxNumIterations, float32 m_OutlierDistance, float32 m_ConvergeDistance, Vector3f &v_axis, Matrix3f &rotation);
		bool computeICP_R3(PointSet * ps_source, PointSet * ps_target, const Vector3f &pos_ref_source, Vector3f &pos_ref_target, HierarchicalKNNIterator * m_hIt_source, HierarchicalKNNIterator * m_hIt_target, float32 m_MedPointDistance, int32 m_MaxNumIterations, float32 m_OutlierDistance, float32 m_ConvergeDistance, Matrix3f &rotation);

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