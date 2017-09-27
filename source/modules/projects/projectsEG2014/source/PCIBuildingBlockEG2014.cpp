// Some info:
// useful channels: position, locX, locY, locZ
// idxBB2Clique(building block patterns, index to)->clique(feature cliques, index to)->pcFeature(features, sampled from)pcFullres
// numBB: number of building block patterns
// idxTemplate2Feature: index to feature, for template in each building block pattern

//---------------------------------------------------------------------------
#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "PCIBuildingBlockEG2014.h"
#include <iosfwd>
#include "PointCloudImporterReaderSMFOBJ.h"
#include "PointCloudImporterAdder.h"
#include "SceneEditorWidget.h"

//---------------------------------------------------------------------------
using namespace X4;
//---------------------------------------------------------------------------

IMPLEMENT_X4_CLASS( PCIBuildingBlockEG2014 ,0){
	BEGIN_CLASS_INIT( PCIBuildingBlockEG2014 );
	ADD_NOARGS_METHOD(renderBB);
	ADD_NOARGS_METHOD(renderClique);
}


PCIBuildingBlockEG2014::PCIBuildingBlockEG2014(void){
	pcFullres = new UnstructuredInCorePointCloud;
	pcFeature = new UnstructuredInCorePointCloud;
	pcSegmentation = new UnstructuredInCorePointCloud;

	name_dirRoot = "root";
	name_pcFullres = "pc1";
	name_pcFeature = "pcFeature";
	name_pcSegmentation = "pcSegmentation";

	colorList.resize(0);

	//-------------------------------------------------------------------------------------------------
	// attachment
	att = new AttachmentBuildingBlockUpward;

	// feature
	numFeature = 0;

	// clique
	numCliqe = 0;
	clique.resize(0);
	idxTemplate2Feature.setDim(0);

	// buiding block
	numBB = 0;
	idxBB2Clique.setDim(0);
	CooC_M_dist.setDimension(0, 0);

	// supervision
	numTemplate = 0;
	userTemplateFeature.resize(0);
	idxKey2Feature.resize(0);
	pm_bbox_lowerCorner.resize(0);
	pm_bbox_upperCorner.resize(0);
	pm_bboxCenter.resize(0);
	pm_coordFrame.resize(0);
	//-------------------------------------------------------------------------------------------------
}

PCIBuildingBlockEG2014::~PCIBuildingBlockEG2014(void){

}

void PCIBuildingBlockEG2014::keyDown(GeneralKey key) {
}

void PCIBuildingBlockEG2014::mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState){
	mouseDownPassToCam(x, y, buttonsState, modifiersState);
}

void PCIBuildingBlockEG2014::mouseMoved(int32 x, int32 y){
	mouseMovedPassToCam(x, y);
}

void PCIBuildingBlockEG2014::mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState){
	mouseUpPassToCam(x, y, buttonsState, modifiersState);
}

void PCIBuildingBlockEG2014::mouseWheelRotated(int32 rotatedDelta, Modifiers modifiersState){
	mouseWheelRotatedPassToCam(rotatedDelta, modifiersState);
}

// helper
void PCIBuildingBlockEG2014::recScene(){
	nodeRoot = dynamic_cast<SGListNode*>(getSceneGraphNode(getScene(), name_dirRoot));
	sourceNO = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), name_dirRoot + "/" + name_pcFullres));
	if (sourceNO) {
		sourceNO->setVisible(false);
		pcFullres = dynamic_cast<UnstructuredInCorePointCloud *>(sourceNO->getSceneObject());
	}

	sourceNO = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), name_dirRoot + "/" + name_pcFeature));
	if (sourceNO) {
		sourceNO->setVisible(true);
		pcFeature = dynamic_cast<UnstructuredInCorePointCloud*>(sourceNO->getSceneObject());
	}

	sourceNO = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), name_dirRoot + "/" + name_pcSegmentation));
	if (sourceNO) {
		sourceNO->setVisible(false);
		pcSegmentation = dynamic_cast<UnstructuredInCorePointCloud*>(sourceNO->getSceneObject());
	}

	loadAttachment();
}

void PCIBuildingBlockEG2014::updateAttachment(){
	att = new AttachmentBuildingBlockUpward;
	att->setup("buildingblocks", AttachedData::ADF_PERSISTENT);
	// feature
	att->numFeature = numFeature;

	// segment
	att->numSegmentation = numSegmentation;

	// clique
	att->numCliqe = numCliqe;
	att->clique.resize(numCliqe);
	for (mpcard i = 0; i < clique.size(); i++){
		att->clique[i] = clique[i];
	}
	att->idxTemplate2Feature = idxTemplate2Feature;

	// building block
	att->numBB = numBB;
	att->idxBB2Clique = idxBB2Clique;
	att->CooC_M_dist = CooC_M_dist;

	// supervision
	att->numTemplate = numTemplate;
	att->userTemplateFeature.resize(numTemplate);
	for (mpcard i = 0; i < numTemplate; i++){
		att->userTemplateFeature[i] = userTemplateFeature[i];
	}
	att->idxKey2Feature.resize(numTemplate);
	for (mpcard i = 0; i < numTemplate; i++){
		att->idxKey2Feature[i] = idxKey2Feature[i];
	}

	att->pm_bbox_lowerCorner.resize(numTemplate);
	for (mpcard i = 0; i < numTemplate; i++)
	{
		att->pm_bbox_lowerCorner[i] = pm_bbox_lowerCorner[i];
	}
	att->pm_bbox_upperCorner.resize(numTemplate);
	for (mpcard i = 0; i < numTemplate; i++)
	{
		att->pm_bbox_upperCorner[i] = pm_bbox_upperCorner[i];
	}
	att->pm_bboxCenter.resize(numTemplate);
	for (mpcard i = 0; i < numTemplate; i++)
	{
		att->pm_bboxCenter[i] = pm_bboxCenter[i];
	}
	att->pm_coordFrame.resize(numTemplate);
	for (mpcard i = 0; i < numTemplate; i++)
	{
		att->pm_coordFrame[i] = pm_coordFrame[i];
	}
}

void PCIBuildingBlockEG2014::loadAttachment(){
	att = dynamic_cast<AttachmentBuildingBlockUpward *>(pcFullres->getAttachments()->getData("buildingblocks"));
	// feature
	numFeature = att->numFeature;

	// segment
	numSegmentation = att->numSegmentation;

	// clique
	numCliqe = att->numCliqe;
	clique.resize(numCliqe);
	for (mpcard i = 0; i < clique.size(); i++){
		clique[i] = att->clique[i];
	}
	idxTemplate2Feature = att->idxTemplate2Feature;

	// building block
	numBB = att->numBB;
	idxBB2Clique = att->idxBB2Clique;
	CooC_M_dist = att->CooC_M_dist;

	// supervision
	numTemplate = att->numTemplate;
	userTemplateFeature.resize(numTemplate);
	for (mpcard i = 0; i < numTemplate; i++){
		userTemplateFeature[i] = att->userTemplateFeature[i];
	}
	idxKey2Feature.resize(numTemplate);
	for (mpcard i = 0; i < numTemplate; i++){
		idxKey2Feature[i] = att->idxKey2Feature[i];
	}
	pm_bbox_lowerCorner.resize(numTemplate);
	for (mpcard i = 0; i < numTemplate; i++)
	{
		pm_bbox_lowerCorner[i] = att->pm_bbox_lowerCorner[i];
	}
	pm_bbox_upperCorner.resize(numTemplate);
	for (mpcard i = 0; i < numTemplate; i++)
	{
		pm_bbox_upperCorner[i] = att->pm_bbox_upperCorner[i];
	}
	pm_bboxCenter.resize(numTemplate);
	for (mpcard i = 0; i < numTemplate; i++)
	{
		pm_bboxCenter[i] = att->pm_bboxCenter[i];
	}
	pm_coordFrame.resize(numTemplate);
	for (mpcard i = 0; i < numTemplate; i++)
	{
		pm_coordFrame[i] = att->pm_coordFrame[i];
	}
}

// rendering
void PCIBuildingBlockEG2014::generateColor(card32 num){
	colorList.resize(std::max<card32>(num, 10));		
	colorList[0] = makeVector3f(102.0f/255.0f, 153.0f/255.0f, 255.0f/255.0f);
	colorList[1] = makeVector3f(255.0f/255.0f, 127.0f/255.0f, 102.0f/255.0f);
	colorList[2] = makeVector3f(102.0f/255.0f, 255.0f/255.0f, 127.0f/255.0f);
	colorList[3] = makeVector3f(102.0f/255.0f, 230.0f/255.0f, 255.0f/255.0f);
	colorList[4] = makeVector3f(255.0f/255.0f, 204.0f/255.0f, 102.0f/255.0f);
	colorList[5] = makeVector3f(230.0f/255.0f, 255.0f/255.0f, 102.0f/255.0f);
	colorList[6] = makeVector3f(102.0f/255.0f, 255.0f/255.0f, 204.0f/255.0f);
	colorList[7] = makeVector3f(255.0f/255.0f, 102.0f/255.0f, 153.0f/255.0f);
	colorList[8] = makeVector3f(204.0f/255.0f, 102.0f/255.0f, 255.0f/255.0f);
	colorList[9] = makeVector3f(153.0f/255.0f, 255.0f/255.0f, 102.0f/255.0f);
	if (num > 10){
		for (mpcard i = 10; i < num; i++){
			colorList[i] = makeVector3f(rnd01(), rnd01(), rnd01());
		}
	}
}

void PCIBuildingBlockEG2014::renderBB(){
	recScene();
	generateColor(numBB);
	AAT pcFeatureposAAT = pcFeature->getAAT("position");
	AAT pcFeaturelocXAAT = pcFeature->getAAT("locX");
	AAT pcFeaturelocYAAT = pcFeature->getAAT("locY");
	AAT pcFeaturelocZAAT = pcFeature->getAAT("locZ");

	debugRenderer->beginRenderJob_OneFrame("", 0, true);
	for (card32 i = 0; i < numBB; i++){
		for (card32 j = 0; j < clique[idxBB2Clique[i]].size(); j++){
			Vector3f pos = pcFeature->getPointSet()->get3f(clique[idxBB2Clique[i]][j], pcFeatureposAAT);
			Vector3f locX = pcFeature->getPointSet()->get3f(clique[idxBB2Clique[i]][j], pcFeaturelocXAAT);
			Vector3f locY = pcFeature->getPointSet()->get3f(clique[idxBB2Clique[i]][j], pcFeaturelocYAAT);
			Vector3f locZ = pcFeature->getPointSet()->get3f(clique[idxBB2Clique[i]][j], pcFeaturelocZAAT);
			if (clique[idxBB2Clique[i]][j] == idxTemplate2Feature[idxBB2Clique[i]]){
				debugRenderer->addFastSphere(pos, 0.03f, colorList[i], true);
			}
			else{
				debugRenderer->addFastSphere(pos, 0.015f, colorList[i], true);
			}
			debugRenderer->addLine(pos, pos + locX / 15, makeVector3f(1, 1, 1), makeVector3f(1, 0, 0), 1);
			debugRenderer->addLine(pos, pos + locY / 15, makeVector3f(1, 1, 1), makeVector3f(0, 1, 0), 1);
			debugRenderer->addLine(pos, pos + locZ / 15, makeVector3f(1, 1, 1), makeVector3f(0, 0, 1), 1);
		}
	}
	debugRenderer->endRenderJob();
}

void PCIBuildingBlockEG2014::renderClique(){
	recScene();
	generateColor((card32)clique.size());
	AAT pcFeatureposAAT = pcFeature->getAAT("position");
	AAT pcFeaturelocXAAT = pcFeature->getAAT("locX");
	AAT pcFeaturelocYAAT = pcFeature->getAAT("locY");
	AAT pcFeaturelocZAAT = pcFeature->getAAT("locZ");
	for (mpcard i = 0; i < clique.size(); i++){
		debugRenderer->beginRenderJob_OneFrame("", i, true);
		for (card32 j = 0; j < clique[i].getDim(); j++){
			debugRenderer->addFastSphere(pcFeature->getPointSet()->get3f(clique[i][j], pcFeatureposAAT), 0.02f, makeVector3f(127.0f / 255.0f, 127.0f / 255.0f, 255.0f / 255.0f), false);
			Vector3f pos_render = pcFeature->getPointSet()->get3f(clique[i][j], pcFeatureposAAT);
			Vector3f locX_render = pcFeature->getPointSet()->get3f(clique[i][j], pcFeaturelocXAAT);
			Vector3f locY_render = pcFeature->getPointSet()->get3f(clique[i][j], pcFeaturelocYAAT);
			Vector3f locZ_render = pcFeature->getPointSet()->get3f(clique[i][j], pcFeaturelocZAAT);
			debugRenderer->addLine(pos_render, pos_render + locX_render / 20, makeVector3f(1.0f, 1.0f, 1.0f), makeVector3f(1.0f, 0.0f, 0.0f), 2);
			debugRenderer->addLine(pos_render, pos_render + locY_render / 20, makeVector3f(1.0f, 1.0f, 1.0f), makeVector3f(0.0f, 1.0f, 0.0f), 2);
			debugRenderer->addLine(pos_render, pos_render + locZ_render / 20, makeVector3f(1.0f, 1.0f, 1.0f), makeVector3f(0.0f, 0.0f, 1.0f), 2);
		}
		debugRenderer->endRenderJob();
	}
}