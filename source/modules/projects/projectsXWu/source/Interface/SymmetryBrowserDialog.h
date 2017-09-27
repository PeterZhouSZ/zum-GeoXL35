//---------------------------------------------------------------------------
#ifndef SymmetryBrowserDialogH
#define SymmetryBrowserDialogH
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "Symmetry/SymmContainer.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
#include "SymmetryBrowserDialog_ui.h"
#include <QDialog>
//---------------------------------------------------------------------------

namespace NAMESPACE_VERSION
{

class PROJECTSXWU_API SymmetryBrowserDialog : public QDialog {
	Q_OBJECT

private:
	Ui_SymmetryBrowserDialog ui;

public:
	SymmetryBrowserDialog();
    void BuildItemList(const std::vector<SymmContainer::Ptr>& symmvec);
	std::vector<bool> GetShowList(void);

public slots:
    void acceptedSlot();

private:
    void updateShow(void);

private:
    typedef void(*StringFunc)(
        const Ui_SymmetryBrowserDialog& ui,
        const std::string& desc,
        QListWidgetItem*& listItem);
    std::map<std::string, StringFunc> stringToFuncMap;

private:
    std::vector<SymmContainer::Ptr> symmvec_;
};

}

#endif
