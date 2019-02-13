#ifndef CPU_MONITOR_HPP
#define CPU_MONITOR_HPP


#ifndef Q_MOC_RUN
#include <QWidget>
#include <QLabel>
#include <QString>
#include <QProgressBar>
#include <QVBoxLayout>
#endif
namespace autoware_rviz_plugins{
class CpuMonitor:public QWidget{
    Q_OBJECT
  public:
    CpuMonitor( int id);
    void setValue(double percentage);
    void updateValue();

  private:
    QLabel* name_;
    QLabel* percentage_label_;
    QProgressBar* progress_bar_;
    double percentage_;
    int red_thresh_;
    int cpu_id_;
    unsigned int previous_work_jiffies_;
    unsigned int previous_total_jiffies_;

};
}
#endif
