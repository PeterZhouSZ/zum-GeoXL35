//---------------------------------------------------------------------------
#ifndef Handle3DListWidgetH
#define Handle3DListWidgetH
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
#include "Interface/Handle3DList.h"
//---------------------------------------------------------------------------
#include "Handle3DListWidgetForm_ui.h"
#include <QDialog>
//---------------------------------------------------------------------------

class PROJECTSXWU_API Handle3DListWidget : public QDialog
{
	Q_OBJECT
public:
    typedef boost::shared_ptr< Handle3DListWidget > Ptr;
    typedef boost::shared_ptr< const Handle3DListWidget > ConstPtr;

public:
    explicit Handle3DListWidget(Handle3DList::Ptr handle3DList);
    void clear(void);
    void BuildItemList(void);

private:
	Ui_Handle3DListWidgetForm ui;

public slots:
    void itemClickedSlot(QListWidgetItem* item);

private:
    Handle3DList::Ptr handle3DList_;
};

#endif
