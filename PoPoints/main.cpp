#include "PoPoints.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	PoPoints w;
	w.show();
	return a.exec();
}
