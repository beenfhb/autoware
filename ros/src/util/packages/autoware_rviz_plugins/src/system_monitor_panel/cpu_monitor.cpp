#include <sstream>
#include <fstream>
#include "cpu_monitor.hpp"

namespace autoware_rviz_plugins{

CpuMonitor::CpuMonitor( int id)
{
  QVBoxLayout* vertical_layout = new QVBoxLayout;
  progress_bar_ = new QProgressBar;
  // progress_bar_->setFixedHeight(100);

  percentage_label_ = new QLabel;
  percentage_label_->setFixedWidth(30);
  percentage_label_->setMargin(0);
  cpu_id_ = id;
  name_ = new QLabel( QString( std::string("CPU" + std::to_string(cpu_id_)).c_str()) );
  red_thresh_ = 70;

  progress_bar_->setOrientation(Qt::Vertical);
  vertical_layout->addWidget(percentage_label_);
  vertical_layout->addWidget(progress_bar_);
  vertical_layout->addWidget(name_);
  vertical_layout->setAlignment(progress_bar_, Qt::AlignCenter);
  vertical_layout->setAlignment(percentage_label_, Qt::AlignCenter);
  vertical_layout->setSpacing(0);
  vertical_layout->setMargin(3);

  setLayout(vertical_layout);
  previous_work_jiffies_ = 0;
  previous_total_jiffies_ = 0;

}

void CpuMonitor::updateValue(){

  std::ifstream proc_stat("/proc/stat");
  std::string line;

  //skip lines
  std::getline(proc_stat,line);
  for(int i = 0; i < cpu_id_; i++){
    std::getline(proc_stat,line);
  }

  //get cpu usage info
  std::getline(proc_stat,line);

  //split into words
  std::stringstream iss(line);
  std::string cpu_name;
  unsigned int user, nice, system_tick, idle, iowait, irq, softirq, steal, guest, guest_nice;
  iss >> cpu_name >> user >> nice >> system_tick >> idle >> iowait >> irq >> softirq >> steal >> guest >> guest_nice;

  // calculate cpu_usage from jiffies
  unsigned int total_jiffies = user + nice + system_tick + idle + iowait + irq + softirq + steal + guest + guest_nice;
  unsigned int work_jiffies =  user + nice + system_tick;
  double cpu_usage = (double)( work_jiffies - previous_work_jiffies_) /( total_jiffies - previous_total_jiffies_ );
  setValue( cpu_usage * 100 );

  // std::cout << core << ":" << cpu_usage << std::endl;
  previous_work_jiffies_ = work_jiffies;
  previous_total_jiffies_ = total_jiffies;
}

void CpuMonitor::setValue(double percentage){
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
  ss << std::fixed << percentage << "%";

  percentage_label_->setText(QString(ss.str().c_str()));

}
}
