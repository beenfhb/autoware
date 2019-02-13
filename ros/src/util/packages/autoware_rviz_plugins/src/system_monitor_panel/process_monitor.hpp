#ifndef PROCESS_MONITOR_HPP
#define PROCESS_MONITOR_HPP


#ifndef Q_MOC_RUN
#include <QWidget>
#include <QLabel>
#include <QString>
#include <QVBoxLayout>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#endif
namespace autoware_rviz_plugins{
class ProcessMonitor:public QWidget{
    Q_OBJECT
  public:
    ProcessMonitor();
    void update();

  private:
    QLabel* label_;
};
}
#endif
