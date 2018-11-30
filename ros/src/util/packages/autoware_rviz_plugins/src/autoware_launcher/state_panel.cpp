#include "state_panel.h"

namespace autoware_rviz_plugins {

RvizController(QWidget* parent = 0) : rviz::Panel(parent)
{
  
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::RvizController, rviz::Panel)