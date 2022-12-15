#include <iostream>
#include <memory>
#include <string>

#include "ros/param.h"
#include "ros/ros.h"

#include <Pipeline.h>
#include <DataLoaders/RScanDataLoader.h>
#include <DataLoaders/Matterport3D.h>
#include <DataLoaders/SemanticKitti.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "sgpr_ros_node");
  ros::NodeHandle n;

  // TODO group these in a utils free function code block
  std::string dataset_dir, dataset_to_use, dataset_file_path;
  ros::param::get("dataset_dir", dataset_dir);
  ros::param::get("dataset_to_use", dataset_to_use);
  dataset_file_path = dataset_dir + "/" + dataset_to_use;

  spDataLoader dl;

  // TODO group these in a utils free function called GetDataLoader
  if (dataset_to_use == "3RScan") {
    dl = std::make_shared<RScanDataLoader>();
  } else if (dataset_to_use == "Matterport3D") {
    dl = std::make_shared<Matterport3D>();
  } else if (dataset_to_use == "SemanticKitti") {
    dl = std::make_shared<SemanticKitti>();
  }

  // TODO: lets think about what datastructure we want to contain the cloud map
  dl->GetSemanticCloudMap();

  /* ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  } */

  return 1;
}
