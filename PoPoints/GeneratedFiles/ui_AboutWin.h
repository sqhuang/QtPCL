/********************************************************************************
** Form generated from reading UI file 'AboutWin.ui'
**
** Created by: Qt User Interface Compiler version 5.11.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ABOUTWIN_H
#define UI_ABOUTWIN_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_AboutWin
{
public:
    QLabel *label;
    QLabel *label_2;

    void setupUi(QWidget *AboutWin)
    {
        if (AboutWin->objectName().isEmpty())
            AboutWin->setObjectName(QStringLiteral("AboutWin"));
        AboutWin->resize(400, 300);
        label = new QLabel(AboutWin);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(60, 40, 291, 121));
        label->setPixmap(QPixmap(QString::fromUtf8(":/PoPoints/Resources/eglass-glass.png")));
        label_2 = new QLabel(AboutWin);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(130, 190, 135, 30));

        retranslateUi(AboutWin);

        QMetaObject::connectSlotsByName(AboutWin);
    } // setupUi

    void retranslateUi(QWidget *AboutWin)
    {
        AboutWin->setWindowTitle(QApplication::translate("AboutWin", "AboutWin", nullptr));
        label->setText(QString());
        label_2->setText(QApplication::translate("AboutWin", "Link Here", nullptr));
    } // retranslateUi

};

namespace Ui {
    class AboutWin: public Ui_AboutWin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ABOUTWIN_H
