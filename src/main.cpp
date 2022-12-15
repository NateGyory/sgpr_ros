#include <string>

#include <ros/console.h>
#include <ros/param.h>
#include <ros/ros.h>

#include <Pipeline.h>

void NodeSpin() {
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "sgpr_ros_node");
  ros::NodeHandle n;

  Pipeline pl;

  int dataset;
  ros::param::get("dataset", dataset);

  switch (dataset) {
  case 0:
  case 1:
    pl.PostProcess(dataset);
    break;
  case 2:
    pl.RealTime();
    break;
  default:
    ROS_DEBUG("NOT SUPPORTED");
  }

  ROS_DEBUG("Done Processing");

  return 1;
}
