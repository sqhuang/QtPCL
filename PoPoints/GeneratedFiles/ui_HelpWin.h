/********************************************************************************
** Form generated from reading UI file 'HelpWin.ui'
**
** Created by: Qt User Interface Compiler version 5.11.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_HELPWIN_H
#define UI_HELPWIN_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>

QT_BEGIN_NAMESPACE

class Ui_HelpWin
{
public:

    void setupUi(QDialog *HelpWin)
    {
        if (HelpWin->objectName().isEmpty())
            HelpWin->setObjectName(QStringLiteral("HelpWin"));
        HelpWin->resize(400, 300);

        retranslateUi(HelpWin);

        QMetaObject::connectSlotsByName(HelpWin);
    } // setupUi

    void retranslateUi(QDialog *HelpWin)
    {
        HelpWin->setWindowTitle(QApplication::translate("HelpWin", "HelpWin", nullptr));
    } // retranslateUi

};

namespace Ui {
    class HelpWin: public Ui_HelpWin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_HELPWIN_H
