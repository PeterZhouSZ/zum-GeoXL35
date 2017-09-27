//---------------------------------------------------------------------------
#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "projectsEG2014.h"
#include "AttachmentBuildingBlockUpward.h"
#include "CopyObjectProperties.h"
#include "Object.h"
#include "DynamicLinearAlgebra.h"
#include "RegularVolume.h"
//---------------------------------------------------------------------------
using namespace X4;
//---------------------------------------------------------------------------

IMPLEMENT_X4_CLASS(AttachmentBuildingBlockUpward, 0)
{
	BEGIN_CLASS_INIT(AttachmentBuildingBlockUpward);
	ADD_CARD32_PROP(numFeature, 0);
	ADD_CARD32_PROP(numSegmentation, 0);
	ADD_CARD32_PROP(numCliqe, 0);
	ADD_CARD32_PROP(numBB, 0);
	ADD_CARD32_PROP(numTemplate, 0);
}

AttachmentBuildingBlockUpward::AttachmentBuildingBlockUpward(void){

}

AttachmentBuildingBlockUpward::~AttachmentBuildingBlockUpward(void){

}

void AttachmentBuildingBlockUpward::read(InputObjectStream *s){
	AttachedData::read(s);

	// clique
	clique.resize(numCliqe);
	for (mpcard i = 0; i < numCliqe; i++)
	{
		clique[i].read(s);
	}
	idxTemplate2Feature.read(s);

	// buiding block
	idxBB2Clique.read(s);
	CooC_M_dist.read(s);

	// supervision
	userTemplateFeature.resize(numTemplate);
	for (mpcard i = 0; i < numTemplate; i++)
	{
		userTemplateFeature[i].read(s);
	}
	idxKey2Feature.resize(numTemplate);
	for (mpcard i = 0; i < numTemplate; i++)
	{
		idxKey2Feature[i].read(s);
	}
	pm_bbox_lowerCorner.resize(numTemplate);
	for (mpcard i = 0; i < numTemplate; i++)
	{
		pm_bbox_lowerCorner[i].read(s);
	}
	pm_bbox_upperCorner.resize(numTemplate);
	for (mpcard i = 0; i < numTemplate; i++)
	{
		pm_bbox_upperCorner[i].read(s);
	}
	pm_bboxCenter.resize(numTemplate);
	for (mpcard i = 0; i < numTemplate; i++)
	{
		pm_bboxCenter[i].read(s);
	}
	pm_coordFrame.resize(numTemplate);
	for (mpcard i = 0; i < numTemplate; i++)
	{
		pm_coordFrame[i].read(s);
	}
}

void AttachmentBuildingBlockUpward::write(OutputObjectStream *s) const{
	AttachedData::write(s);

	// clique
	for (mpcard i = 0; i < clique.size(); i++){
		clique[i].write(s);
	}
	idxTemplate2Feature.write(s);

	// buiding block
	idxBB2Clique.write(s);
	CooC_M_dist.write(s);

	// supervision
	for (mpcard i = 0; i < userTemplateFeature.size(); i++){
		userTemplateFeature[i].write(s);
	}
	for (mpcard i = 0; i < idxKey2Feature.size(); i++){
		idxKey2Feature[i].write(s);
	}
	for (mpcard i = 0; i < pm_bbox_lowerCorner.size(); i++){
		pm_bbox_lowerCorner[i].write(s);
	}
	for (mpcard i = 0; i < pm_bbox_upperCorner.size(); i++){
		pm_bbox_upperCorner[i].write(s);
	}
	for (mpcard i = 0; i < pm_bboxCenter.size(); i++){
		pm_bboxCenter[i].write(s);
	}
	for (mpcard i = 0; i < pm_coordFrame.size(); i++){
		pm_coordFrame[i].write(s);
	}
}