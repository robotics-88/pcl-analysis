/* 
© 2023 Robotics 88
Author: Gus Meyer <gus@robotics88.com> 
*/

#include "pcl_analysis/pcl_analysis.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_analysis");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle node;
  PCLAnalysis pcl_analysis(node);

  ros::spin();

  return 0;
}