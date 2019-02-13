#ifndef SYSTEM_MONITOR_PANEL_HPP
#define SYSTEM_MONITOR_PANEL_HPP

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <rviz/panel.h>
#include <QTimer>
#include <QProgressBar>
#include <QString>
#include <unistd.h>
#include "cpu_monitor.hpp"
#include "memory_monitor.hpp"
#include "process_monitor.hpp"

#endif

// class QLineEdit;

namespace autoware_rviz_plugins
{

class SystemMonitorPanel: public rviz::Panel
{
Q_OBJECT
public:
  SystemMonitorPanel( QWidget* parent = 0 );

protected Q_SLOTS:
  void update();

protected:
  QTimer* timer_;
  std::vector<CpuMonitor*> cpu_monitors_;
  MemoryMonitor* memory_monitor_;
  ProcessMonitor* process_monitor_;
  //QProgressBar* cpu_usage_bar;
  int n_cpu_;
};
}

#endif
