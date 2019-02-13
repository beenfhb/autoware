#include "process_monitor.hpp"

namespace autoware_rviz_plugins{
ProcessMonitor::ProcessMonitor(){
  label_ = new QLabel;
  // label_->setWordWrap(true);
  label_->setFixedWidth(200);
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(label_);
  setLayout(layout);

}

void ProcessMonitor::update(){
  // std::string command
  system(" ps au --sort -%cpu |  awk '{print $11 \" (\" $3 \"%)\"}' > /tmp/top_result.txt");
  // system("awk '{print $9 " " $12}' /tmp/top_result.txt  > /tmp/awk_result.txt" );
  std::ifstream top_result("/tmp/top_result.txt");
  std::string line;

  while(std::getline(top_result, line)){
    if(line.find("\%CPU")!= std::string::npos){
      break;
    }
  }
  int top_n = 5;
  std::stringstream ss;

  for (int i = 0; i < top_n; i++){
    std::getline(top_result, line);
    ss << line;
    if(i <top_n -1) ss << std::endl;
  }
  label_->setText(QString(ss.str().c_str()));
}

}
