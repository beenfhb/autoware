#include "mainwindow.h"
#include "tunerBody.h"
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "tlr_tuner");

  QApplication a(argc, argv);
  MainWindow w;
  TunerBody tuner;

  w.show();
  tuner.launch();

  return a.exec();
}
