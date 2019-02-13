#include <stdio.h>
#include <QHBoxLayout>

#include "system_monitor_panel.hpp"

namespace autoware_rviz_plugins
{
SystemMonitorPanel::SystemMonitorPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  n_cpu_ = sysconf(_SC_NPROCESSORS_ONLN);
  for (int i = 0; i < n_cpu_; i++){
    CpuMonitor* cm = new CpuMonitor( i );
    cpu_monitors_.push_back(cm);
  }

  memory_monitor_ = new MemoryMonitor(QString( "Memory" ));
  process_monitor_ = new ProcessMonitor();

  QHBoxLayout* horizontal_layout = new QHBoxLayout;
  QVBoxLayout* vertical_layout = new QVBoxLayout;

  horizontal_layout->setSpacing(0);
  horizontal_layout->setMargin(0);

  vertical_layout->addWidget(process_monitor_);
  vertical_layout->addWidget(memory_monitor_);

  for(auto cpu : cpu_monitors_){
    horizontal_layout->addWidget(cpu);
  }
  horizontal_layout->addLayout(vertical_layout);
  setLayout (horizontal_layout);

  timer_ = new QTimer(this);
  timer_->setInterval(1000);
  timer_->setSingleShot(false);
  connect ( timer_, SIGNAL(timeout()), this, SLOT(update()));
  timer_->start();
}

void SystemMonitorPanel::update(){
  for(auto cpu: cpu_monitors_){
    cpu->updateValue();
  }
  memory_monitor_->updateValue();
  process_monitor_->update();
}

} // end namespace rviz_plugin_tutorials

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( autoware_rviz_plugins::SystemMonitorPanel, rviz::Panel )
// END_TUTORIAL
