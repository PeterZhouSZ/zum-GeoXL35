//---------------------------------------------------------------------------
#include"StdAfx.h"
//---------------------------------------------------------------------------
#include "PCICL3DRepetitionGroundTruth2015EditorWidget.h"
#include "PCICL3DRepetitionGroundTruth2015.h"
#include "BasicStringUtils.h"
//---------------------------------------------------------------------------
using namespace X4;
//---------------------------------------------------------------------------

IMPLEMENT_CLASS_EDITOR(PCICL3DRepetitionGroundTruth2015)

PCICL3DRepetitionGroundTruth2015EditorWidget::PCICL3DRepetitionGroundTruth2015EditorWidget(QWidget * parent, Qt::WindowFlags f)
:QWidget(parent, f)
{
	//call ui.setupUi(this) to set up the layout from QtDesigner
	ui.setupUi(this);
	std::vector<std::string> semanticlabel;
	semanticlabel.push_back("window");
	semanticlabel.push_back("car");
	semanticlabel.push_back("balcony");
	semanticlabel.push_back("deco");
	for (mpcard i = 0; i < semanticlabel.size(); i++)
	{
		ui.cbSemanticLabel->addItem(QString::fromLatin1(semanticlabel[i].data(), (int)semanticlabel[i].size()));
	}
}

PCICL3DRepetitionGroundTruth2015EditorWidget::~PCICL3DRepetitionGroundTruth2015EditorWidget(void)
{
}

void PCICL3DRepetitionGroundTruth2015EditorWidget::setup(PCICL3DRepetitionGroundTruth2015 * obj)
{
	m_Object = obj;
}

void PCICL3DRepetitionGroundTruth2015EditorWidget::updateEditor()
{
	ui.lePcName->setText(m_Object->getPcName().c_str());
	ui.leDumpName->setText(m_Object->getDumpName().c_str());
	std::ostringstream sstream;
	sstream << m_Object->getRelRadiusFullres();
	std::string varAsString = sstream.str();
	ui.leRelRadiusFullres->setText(varAsString.c_str());
}

void PCICL3DRepetitionGroundTruth2015EditorWidget::on_lePcName_textEdited(const QString & text){
	m_Object->setPcName(qString2STLString(text));
}

void PCICL3DRepetitionGroundTruth2015EditorWidget::on_leDumpName_textEdited(const QString & text){
	m_Object->setDumpName(qString2STLString(text));
}

void PCICL3DRepetitionGroundTruth2015EditorWidget::on_leRelRadiusFullres_textEdited(const QString & text){
	m_Object->setRelRadiusFullres(std::stod(qString2STLString(text).c_str()));
}

// preprocess IGN
void PCICL3DRepetitionGroundTruth2015EditorWidget::on_pbPreprocessIGN_clicked(){
	m_Object->PreprocessIGN();
}

void PCICL3DRepetitionGroundTruth2015EditorWidget::on_pbRemoveOutlier_clicked(){
	m_Object->removeoutlier();
}

void PCICL3DRepetitionGroundTruth2015EditorWidget::on_pbSplitData_clicked(){
	m_Object->splitdata();
}

void PCICL3DRepetitionGroundTruth2015EditorWidget::on_pbPoints2x4obj_clicked(){
	m_Object->points2x4obj();
}

// preprocess General
void PCICL3DRepetitionGroundTruth2015EditorWidget::on_pbPreprocessGeneral_clicked(){
	m_Object->PreprocessGeneral();
}

void PCICL3DRepetitionGroundTruth2015EditorWidget::on_pbFlipYZ_clicked(){
	m_Object->flipyz();
}

void PCICL3DRepetitionGroundTruth2015EditorWidget::on_pbComputeDiagonal_clicked(){
	m_Object->computeDiagonal();
}

// Create GT
void PCICL3DRepetitionGroundTruth2015EditorWidget::on_pbInitialDatastructure_clicked(){
	m_Object->initialdatastructure();
}

void PCICL3DRepetitionGroundTruth2015EditorWidget::on_pbSaveGT_clicked(){
	m_Object->saveGT();
}

void PCICL3DRepetitionGroundTruth2015EditorWidget::on_pbLoadGT_clicked(){
	m_Object->loadGT();
}

// Create GT (Semi Auto)
void PCICL3DRepetitionGroundTruth2015EditorWidget::on_pbCreateTempalte_clicked(){

	// reset
	ui.hsTranslateX->setValue(50);
	ui.hsTranslateY->setValue(50);
	ui.hsTranslateZ->setValue(50);
	ui.hsRotateZ->setValue(50);

	m_Object->createtempalte();
}

void PCICL3DRepetitionGroundTruth2015EditorWidget::on_pbManualDetect_clicked(){

	// reset
	ui.hsTranslateX->setValue(50);
	ui.hsTranslateY->setValue(50);
	ui.hsTranslateZ->setValue(50);
	ui.hsRotateZ->setValue(50);

	m_Object->manualdetect();
}

// Create GT (Manual)
void PCICL3DRepetitionGroundTruth2015EditorWidget::on_pbManualCreateNewClass_clicked(){
	// reset
	ui.hsTranslateX->setValue(50);
	ui.hsTranslateY->setValue(50);
	ui.hsTranslateZ->setValue(50);
	ui.hsRotateZ->setValue(50);

	m_Object->manualcreatenewclass();
}

void PCICL3DRepetitionGroundTruth2015EditorWidget::on_pbManualCreateNewObject_clicked(){
	// reset
	ui.hsTranslateX->setValue(50);
	ui.hsTranslateY->setValue(50);
	ui.hsTranslateZ->setValue(50);
	ui.hsRotateZ->setValue(50);

	m_Object->manualcreatenewobject();
}

// edit
void PCICL3DRepetitionGroundTruth2015EditorWidget::on_hsTranslateX_valueChanged(){
	m_Object->translateX(ui.hsTranslateX->value() - 50);
}

void PCICL3DRepetitionGroundTruth2015EditorWidget::on_hsTranslateY_valueChanged(){
	m_Object->translateY(ui.hsTranslateY->value() - 50);
}

void PCICL3DRepetitionGroundTruth2015EditorWidget::on_hsTranslateZ_valueChanged(){
	m_Object->translateZ(ui.hsTranslateZ->value() - 50);
}

void PCICL3DRepetitionGroundTruth2015EditorWidget::on_hsRotateZ_valueChanged(){
	m_Object->rotateZ(ui.hsRotateZ->value() - 50);
}

void PCICL3DRepetitionGroundTruth2015EditorWidget::on_pbReset_clicked(){
	ui.hsTranslateX->setValue(50);
	ui.hsTranslateY->setValue(50);
	ui.hsTranslateZ->setValue(50);
	ui.hsRotateZ->setValue(50);
	m_Object->translateX(ui.hsTranslateX->value() - 50);
}

void PCICL3DRepetitionGroundTruth2015EditorWidget::on_cbSemanticLabel_currentIndexChanged(){
	debugOutput << "Current semantic label: " << ui.cbSemanticLabel->currentIndex() << "\n";
}

void PCICL3DRepetitionGroundTruth2015EditorWidget::on_pbSemanticLabelAssign_clicked(){
	m_Object->semanticlabelassign(ui.cbSemanticLabel->currentIndex());
}

void PCICL3DRepetitionGroundTruth2015EditorWidget::on_pbRenderRigid_clicked(){
	m_Object->renderrigid();
}


void PCICL3DRepetitionGroundTruth2015EditorWidget::on_pbRenderSemantic_clicked(){
	m_Object->rendersemantic();
}


#include "PCICL3DRepetitionGroundTruth2015EditorWidget_moc.h"

