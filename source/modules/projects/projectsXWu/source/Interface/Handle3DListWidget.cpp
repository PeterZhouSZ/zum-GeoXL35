#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Handle3DListWidget.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
#include <QListWidgetItem>
//---------------------------------------------------------------------------

Handle3DListWidget::Handle3DListWidget(Handle3DList::Ptr handle3DList) : handle3DList_(handle3DList)
{
    ui.setupUi(this);

    connect(ui.listWidgetHandle3DGizmo, SIGNAL(itemClicked(QListWidgetItem*)),
        this, SLOT(itemClickedSlot(QListWidgetItem*)));
}

void Handle3DListWidget::clear(void)
{
    ui.listWidgetHandle3DGizmo->clear();
}

void Handle3DListWidget::itemClickedSlot(QListWidgetItem* item)
{
    (Qt::Checked == item->checkState()) ? item->setCheckState(Qt::Unchecked) : item->setCheckState(Qt::Checked);
    int row = ui.listWidgetHandle3DGizmo->row(item);
    handle3DList_->handles_[row].invertActive();
    //handle3DList_->glDraw();
}

void Handle3DListWidget::BuildItemList(void)
{
    ui.listWidgetHandle3DGizmo->clear();
    const std::deque<Handle3DGizmo>& handles = handle3DList_->handles_;
    for (size_t ii = 0; ii < handles.size(); ++ii) {
        std::ostringstream ss;
        ss << ii << ": " << handles[ii].size() << " points";
        QListWidgetItem* listItem = new QListWidgetItem(ss.str().c_str());
        if (handles[ii].isActive()) listItem->setCheckState(Qt::Checked);
        else listItem->setCheckState(Qt::Unchecked);
        ui.listWidgetHandle3DGizmo->addItem(listItem);
    }
}

#include "Handle3DListWidget_moc.h"
