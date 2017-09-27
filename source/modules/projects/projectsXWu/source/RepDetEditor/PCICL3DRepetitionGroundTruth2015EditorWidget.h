#ifndef PCICL3DRepetitionGroundTruth2015EditorWidget_H
#define PCICL3DRepetitionGroundTruth2015EditorWidget_H

#include "CommonHdrXWu.h"
#include "PCICL3DRepetitionGroundTruth2015EditorForm_ui.h"
#include "ClassEditor.h"
#include "CLLabelsGroundTruth2015.h"
#include "SGListNode.h"
#include "CLResultsBoxGroundTruth2015.h"
#include "CLPRBoxGroundTruth2015.h"


namespace X4
{

	class PCICL3DRepetitionGroundTruth2015;

	class PROJECTSXWU_API PCICL3DRepetitionGroundTruth2015EditorWidget : public QWidget
	{
		Q_OBJECT

	private:

		PCICL3DRepetitionGroundTruth2015	*	m_Object;
		Ui_PCICL3DRepetitionGroundTruth2015EditorForm ui;
		
	public:
		PCICL3DRepetitionGroundTruth2015EditorWidget(QWidget * parent = NULL, Qt::WindowFlags f = 0);
		~PCICL3DRepetitionGroundTruth2015EditorWidget(void);

		void setup(PCICL3DRepetitionGroundTruth2015 * obj);

		/// update editor content with object data
		void updateEditor();

		public slots:
		void on_lePcName_textEdited(const QString & text);
		void on_leDumpName_textEdited(const QString & text);
		void on_leRelRadiusFullres_textEdited(const QString & text);

		// preprocess IGN
		void on_pbPreprocessIGN_clicked();
		void on_pbRemoveOutlier_clicked();
		void on_pbSplitData_clicked();
		void on_pbPoints2x4obj_clicked();

		// preprocess General
		void on_pbPreprocessGeneral_clicked();
		void on_pbFlipYZ_clicked();
		void on_pbComputeDiagonal_clicked();

		// Create GT
		void on_pbInitialDatastructure_clicked();
		void on_pbSaveGT_clicked();
		void on_pbLoadGT_clicked();

		// Create GT (Semi Auto)
		void on_pbCreateTempalte_clicked();
		void on_pbManualDetect_clicked();

		// Create GT (Manual)
		void on_pbManualCreateNewClass_clicked();
		void on_pbManualCreateNewObject_clicked();

		// edit
		void on_hsTranslateX_valueChanged();
		void on_hsTranslateY_valueChanged();
		void on_hsTranslateZ_valueChanged();
		void on_hsRotateZ_valueChanged();
		void on_pbReset_clicked();

		void on_cbSemanticLabel_currentIndexChanged();
		void on_pbSemanticLabelAssign_clicked();
		void on_pbRenderRigid_clicked();
		void on_pbRenderSemantic_clicked();

	signals:
		void sigObjectChanged();
	};

	MAKE_CLASS_EDITOR(PROJECTSXWU_API, PCICL3DRepetitionGroundTruth2015)

}



#endif