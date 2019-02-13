#ifndef MEMORY_MONITOR_HPP
#define MEMORY_MONITOR_HPP


#ifndef Q_MOC_RUN
#include <QWidget>
#include <QLabel>
#include <QString>
#include <iostream>
#include <sstream>
#include <fstream>
#include <QProgressBar>
#include <QGridLayout>
#include <QHBoxLayout>

#endif
namespace autoware_rviz_plugins{
class MemoryMonitor:public QWidget{
    Q_OBJECT
  public:
    MemoryMonitor(QString);
    void setValue(double total_memory_Gb, double available_memory_Gb);
    void updateValue();

  private:
    QLabel* name_;
    QLabel* percentage_label_;
    QProgressBar* progress_bar_;
    double percentage_;
    int red_thresh_;
};
}
#endif
