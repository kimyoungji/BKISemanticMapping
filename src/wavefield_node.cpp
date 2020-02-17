#include <ros/ros.h>
#include "TMapping.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "wavefield_node");
  ros::NodeHandle nodeHandle("~");
  TMapping mapping(nodeHandle);

  // Spin
  ros::AsyncSpinner spinner(0);  // Use n threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
