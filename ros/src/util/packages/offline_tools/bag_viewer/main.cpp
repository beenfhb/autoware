/*
 * main.cpp
 *
 *  Created on: Dec 12, 2018
 *      Author: sujiwo
 */


#include <QApplication>

#include "RandomAccessBag.h"
#include "BagViewer.h"


int main (int argc, char *argv[])
{
	QApplication mainApp(argc, argv);
	BagViewer bv;

	bv.setBagFile(argv[1]);
	bv.show();

	return mainApp.exec();
}
