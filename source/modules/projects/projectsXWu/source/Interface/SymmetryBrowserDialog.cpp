#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "SymmetryBrowserDialog.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
#include <QListWidgetItem>
//---------------------------------------------------------------------------

namespace NAMESPACE_VERSION
{

void AddReflectionListItem(const Ui_SymmetryBrowserDialog& ui,
    const std::string& desc,
    QListWidgetItem*& listItem)
{
    listItem = new QListWidgetItem(
        desc.c_str(), ui.listWidgetReflection);
    ui.listWidgetReflection->addItem(listItem);
}

void AddRotationListItem(const Ui_SymmetryBrowserDialog& ui,
    const std::string& desc,
    QListWidgetItem*& listItem)
{
    listItem = new QListWidgetItem(
        desc.c_str(), ui.listWidgetRotation);
    ui.listWidgetRotation->addItem(listItem);
}

void AddTranslationListItem(const Ui_SymmetryBrowserDialog& ui,
    const std::string& desc,
    QListWidgetItem*& listItem)
{
    listItem = new QListWidgetItem(
        desc.c_str(), ui.listWidgetTranslation);
    ui.listWidgetTranslation->addItem(listItem);
}

void AddCoPlaneListItem(const Ui_SymmetryBrowserDialog& ui,
                        const std::string& desc,
                        QListWidgetItem*& listItem)
{
    listItem = new QListWidgetItem(
        desc.c_str(), ui.listWidgetCoPlanarity);
    ui.listWidgetCoPlanarity->addItem(listItem);
}

SymmetryBrowserDialog::SymmetryBrowserDialog()
{
    ui.setupUi(this);

    stringToFuncMap["CoPlanarity"] = &AddCoPlaneListItem;
    stringToFuncMap["Reflection"] = &AddReflectionListItem;
    stringToFuncMap["Rotation"] = &AddRotationListItem;
    stringToFuncMap["Translation"] = &AddTranslationListItem;

    connect(this, SIGNAL(accepted()), this, SLOT(acceptedSlot()));
}

void SymmetryBrowserDialog::
BuildItemList(const std::vector<SymmContainer::Ptr>& symmvec)
{
    ui.listWidgetCoPlanarity->clear();
    ui.listWidgetReflection->clear();
    ui.listWidgetRotation->clear();
    ui.listWidgetTranslation->clear();
    symmvec_ = symmvec;

    for (size_t ii = 0; ii < symmvec.size(); ++ii) {
        std::string name = symmvec[ii]->GetName();
        std::string desc = symmvec[ii]->GetDescription();
        QListWidgetItem* listItem  = nullptr;
        stringToFuncMap[name](ui, desc, listItem);
        if (symmvec_[ii]->show_) listItem->setCheckState(Qt::Checked);
        else listItem->setCheckState(Qt::Unchecked);
        QVariant qv;
        qv.setValue(ii);
        listItem->setData(Qt::UserRole, qv);
    }
}

void SymmetryBrowserDialog::acceptedSlot()
{
    updateShow();
}

void SymmetryBrowserDialog::updateShow(void)
{
    for(int ii = 0; ii < ui.listWidgetCoPlanarity->count(); ii++) {
        QListWidgetItem* item = ui.listWidgetCoPlanarity->item(ii);
        QVariant qvPtr = item->data(Qt::UserRole);
        size_t idx = qvPtr.value<size_t>();
        if (Qt::Checked == item->checkState())
            symmvec_[idx]->show_ = true;
        else symmvec_[idx]->show_ = false;
    }
    for(int ii = 0; ii < ui.listWidgetReflection->count(); ii++) {
        QListWidgetItem* item = ui.listWidgetReflection->item(ii);
        QVariant qvPtr = item->data(Qt::UserRole);
        size_t idx = qvPtr.value<size_t>();
        if (Qt::Checked == item->checkState())
            symmvec_[idx]->show_ = true;
        else symmvec_[idx]->show_ = false;
    }
    for(int ii = 0; ii < ui.listWidgetRotation->count(); ii++) {
        QListWidgetItem* item = ui.listWidgetRotation->item(ii);
        QVariant qvPtr = item->data(Qt::UserRole);
        size_t idx = qvPtr.value<size_t>();
        if (Qt::Checked == item->checkState())
            symmvec_[idx]->show_ = true;
        else symmvec_[idx]->show_ = false;
    }
    for(int ii = 0; ii < ui.listWidgetTranslation->count(); ii++) {
        QListWidgetItem* item = ui.listWidgetTranslation->item(ii);
        QVariant qvPtr = item->data(Qt::UserRole);
        size_t idx = qvPtr.value<size_t>();
        if (Qt::Checked == item->checkState())
            symmvec_[idx]->show_ = true;
        else symmvec_[idx]->show_ = false;
    }
}

std::vector<bool> SymmetryBrowserDialog::GetShowList(void)
{
    std::vector<bool> showlist;
    for(int ii = 0; ii < symmvec_.size(); ++ii) {
        showlist.push_back(symmvec_[ii]->show_);
    }
    return showlist;
}

}

#include "SymmetryBrowserDialog_moc.h"
