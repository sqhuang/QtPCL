#pragma once

#include <QDialog>
#include "ui_HelpWin.h"

class HelpWin : public QDialog
{
	Q_OBJECT

public:
	HelpWin(QWidget *parent = Q_NULLPTR);
	~HelpWin();

private:
	Ui::HelpWin ui;
};
