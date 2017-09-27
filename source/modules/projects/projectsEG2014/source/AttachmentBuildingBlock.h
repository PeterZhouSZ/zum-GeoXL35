#ifndef AttachmentBuildingBlock_H
#define  AttachmentBuildingBlock_H
//---------------------------------------------------------------------------
#include "projectsEG2014.h"
//---------------------------------------------------------------------------
#include "Persistent.h"
#include "AttachedData.h"
#include "DynamicLinearAlgebra.h"
#include "SparseLinearAlgebra.inline.h"

//---------------------------------------------------------------------------
namespace X4{

	class PROJECTSEG2014_API AttachmentBuildingBlock: public AttachedData {
		X4_CLASS(AttachmentBuildingBlock)

	public:

		// data
		int32 numSamples;
		int32 numFeaturesKey;
		int32 numFeaturesDense;

		std::vector<DVectorF> featKey; // key features
		std::vector<DVectorF> featDense; // dense features
		card32 numClusters; // dictionary size
		DVectorF idxKey2Cluster; // cluster index for each key feature
		DVectorF idxClusterCenter2Key; // center index for each cluster
		DVectorF clusterSize; // size of cluster
		std::vector<DVectorF> clique;
		DMatrixF CooC_M_dist;
		int32 dimEmbed; 
		DVectorF CooC_cen_indices;
		bool flag_spec_success;
		card32 numClustersOptimized;
		std::vector<DVectorF> cliqueOptimized;
		std::vector<Vector6f> boxes;

		//folder name & file name
		string name_folderRoot; // name of root folder
		string name_pcInput; // name of input pc
		string name_pcSampled; // name of input pc
		string name_pcNonSlippage; // name of non-slippary pc
		string name_pcFeature; // name of feature pc
		string name_dirPCD; // directory of pcd output

		// paramters
		// sampling
		float32 m_SP_sigma;
		float32 m_SP_sampleSpacing;

		// slippage detection
		float32 m_SD_RelativeSlippageRadius;
		float32 m_SD_SlippageThreshold;

		// feature detection
		float32 m_FD_Spacing;
		float32 m_FD_Threshold;
		int32 m_FD_maxNumFeature;

		// features descriptor
		int32 m_FDS_dimSHOT; // SHOT feature dimension
		float32 m_FDS_Radius;

		// initial dictionary
		card32 m_DICT_minClusterSize; // minimum cluster size
		card32 m_DICT_maxNumCluster; // max number of clusters

		// sliding window detection
		float32 m_SW_ExpRatio;
		float32 m_SW_Threshold;
		float32 m_SW_Spacing;

		// spectral clustering
		card32 maxnumBB;
		/*
		*/

		AttachmentBuildingBlock(void);
		~AttachmentBuildingBlock(void);

		virtual void read(InputObjectStream *s);
		virtual void write(OutputObjectStream *s) const;
	};


}


#endif