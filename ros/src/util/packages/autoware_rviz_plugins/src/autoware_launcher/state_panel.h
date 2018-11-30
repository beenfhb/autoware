#ifndef AUTOWARE_LAUNCHER_RVIZ_CONTROLLER_HPP
#define AUTOWARE_LAUNCHER_RVIZ_CONTROLLER_HPP

#include <rviz/panel.h>

namespace autoware_rviz_plugins {

class RvizController : public rviz::Panel
{
  Q_OBJECT

  public:

    RvizController(QWidget* parent = 0);

  private:

    void processMessage(const std_msgs::String::ConstPtr &msg);
};

}

#endif
