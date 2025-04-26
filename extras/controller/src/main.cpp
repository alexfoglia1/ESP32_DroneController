#include <qserialport.h>
#include <qapplication.h>
#include <iostream>
#include <qfont.h>

#include "ControllerWindow.h"


int main(int argc, char** argv)
{
	QApplication* ctrl;
	ctrl = new QApplication(argc, argv);

	QFont font("Verdana", 8);
	qApp->setFont(font);

	ControllerWindow* ctrlWindow = new ControllerWindow();
	ctrlWindow->setVisible(true);

	return ctrl->exec();
}
