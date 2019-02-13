#include "memory_monitor.hpp"
#include <sstream>

namespace autoware_rviz_plugins{

MemoryMonitor::MemoryMonitor(QString name){
  progress_bar_ = new QProgressBar;
  percentage_label_ = new QLabel;
  percentage_label_->setAlignment(Qt::AlignCenter);
  name_ = new QLabel(name);

  red_thresh_ = 70;
  QGridLayout* grid_layout = new QGridLayout;

  grid_layout->addWidget(percentage_label_, 0, 0, 1, 2);
  grid_layout->addWidget(name_, 1, 0);
  grid_layout->addWidget(progress_bar_, 1, 1);

  setLayout(grid_layout);

}
void MemoryMonitor::updateValue(){
  std::ifstream proc_stat("/proc/meminfo");
  std::string line;

  //skip first line
  std::getline(proc_stat,line);
  std::string skip;
  double total_memory_kB, available_memory_kB;
  std::stringstream ss(line);
  ss >> skip >> total_memory_kB >> skip;

  std::getline(proc_stat,line);
  std::getline(proc_stat,line);
  ss.clear();
  ss.str(line);
  ss >> skip >> available_memory_kB >> skip;

  setValue((double)(total_memory_kB)/1000000,(double)(available_memory_kB)/1000000);
}

void MemoryMonitor::setValue(double total_memory_Gb, double available_memory_Gb){
  double percentage = (total_memory_Gb - available_memory_Gb )/total_memory_Gb * 100;

  QPalette palette = progress_bar_->palette();
  if(percentage > red_thresh_){
    palette.setColor(QPalette::Highlight, Qt::red);
  }else{
    palette.setColor(QPalette::Highlight, Qt::blue);
  }
  progress_bar_->setPalette(palette);
  progress_bar_->setValue(percentage);

  std::stringstream ss;
  ss.precision(1);
  ss << std::fixed << total_memory_Gb - available_memory_Gb << "GB/" << total_memory_Gb << "GB(" << percentage << "\%)";
  percentage_label_->setText(QString(ss.str().c_str()));

}
}
