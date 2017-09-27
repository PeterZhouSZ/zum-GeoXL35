//---------------------------------------------------------------------------
#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "projectsEG2014.h"
#include "AttachmentBuildingBlock.h"
#include "CopyObjectProperties.h"
#include "Object.h"
#include "DynamicLinearAlgebra.h"
#include "RegularVolume.h"
//---------------------------------------------------------------------------
using namespace X4;
//---------------------------------------------------------------------------

IMPLEMENT_X4_CLASS( AttachmentBuildingBlock , 0 )
{
	BEGIN_CLASS_INIT( AttachmentBuildingBlock );
	
	ADD_INT32_PROP(numSamples, 0);
	ADD_INT32_PROP(numFeaturesKey, 0);
	ADD_INT32_PROP(numFeaturesDense, 0);

	ADD_CARD32_PROP(numClusters, 0);
	ADD_CARD32_PROP(dimEmbed, 0);
	ADD_BOOLEAN_PROP(flag_spec_success, 0);
	
	ADD_STRING_PROP(name_folderRoot, 0);
	ADD_STRING_PROP(name_pcInput, 0);
	ADD_STRING_PROP(name_pcSampled, 0);
	ADD_STRING_PROP(name_pcNonSlippage, 0);
	ADD_STRING_PROP(name_pcFeature, 0);
	ADD_STRING_PROP(name_dirPCD, 0);

	// sampling
	ADD_FLOAT32_PROP(m_SP_sigma, 0);
	ADD_FLOAT32_PROP(m_SP_sampleSpacing, 0);

	// slippage detection
	ADD_FLOAT32_PROP(m_SD_RelativeSlippageRadius, 0);
	ADD_FLOAT32_PROP(m_SD_SlippageThreshold, 0);

	// feature detection
	ADD_FLOAT32_PROP(m_FD_Spacing, 0);
	ADD_FLOAT32_PROP(m_FD_Threshold, 0);
	ADD_INT32_PROP(m_FD_maxNumFeature, 0);

	// features descriptor
	ADD_INT32_PROP(m_FDS_dimSHOT, 0);
	ADD_FLOAT32_PROP(m_FDS_Radius, 0);

	// dictionary
	ADD_CARD32_PROP(m_DICT_minClusterSize, 0);
	ADD_CARD32_PROP(m_DICT_maxNumCluster, 0);

	// sliding window detection
	ADD_FLOAT32_PROP(m_SW_ExpRatio, 0);
	ADD_FLOAT32_PROP(m_SW_Threshold, 0);
	ADD_FLOAT32_PROP(m_SW_Spacing, 0);

	// spectral clustering
	ADD_CARD32_PROP(maxnumBB, 0);
	/*
	*/
	// segmentation
	ADD_CARD32_PROP(numClustersOptimized, 0);

}

AttachmentBuildingBlock::AttachmentBuildingBlock(void){

}

AttachmentBuildingBlock::~AttachmentBuildingBlock(void){

}

void AttachmentBuildingBlock::read( InputObjectStream *s ){
	AttachedData::read(s);

	featKey.resize(numFeaturesKey);
	for (mpcard i = 0; i < numFeaturesKey; i++){
		featKey[i].read(s);
	}

	featDense.resize(numFeaturesDense);
	for (mpcard i = 0; i < numFeaturesDense; i++){
		featDense[i].read(s);
	}

	idxKey2Cluster.read(s);

	idxClusterCenter2Key.read(s);

	clusterSize.read(s);

	CooC_M_dist.read(s);

	clique.resize(numClusters);
	for (mpcard i = 0; i < numClusters; i++){
		clique[i].read(s);
	}

	CooC_cen_indices.read(s);

	cliqueOptimized.resize(numClustersOptimized);
	for (mpcard i = 0; i < numClustersOptimized; i++){
		cliqueOptimized[i].read(s);
	}

	boxes.resize(numClustersOptimized);
	for (mpcard i = 0; i < numClustersOptimized; i++){
		boxes[i].read(s);
	}

}

void AttachmentBuildingBlock::write( OutputObjectStream *s ) const{
	AttachedData::write(s);

	for (mpcard i = 0; i < featKey.size(); i++){
		featKey[i].write(s);
	}

	for (mpcard i = 0; i < featDense.size(); i++){
		featDense[i].write(s);
	}

	idxKey2Cluster.write(s);

	idxClusterCenter2Key.write(s);

	clusterSize.write(s);

	CooC_M_dist.write(s);

	for (mpcard i = 0; i < clique.size(); i++){
		clique[i].write(s);
	}

	CooC_cen_indices.write(s);

	for (mpcard i = 0; i < numClustersOptimized; i++){
		cliqueOptimized[i].write(s);
	}

	for (mpcard i = 0; i < numClustersOptimized; i++){
		boxes[i].write(s);
	}
}