#include <ros/ros.h>
#include "../sptam/mapNode.hpp"

int main(int argc, char **argv){

  ros::init(argc,argv,"Map Node");
  ros::NodeHandle nh1;
  ros::NodeHandle nhp1("~");

  ROS_INFO("Map node running...");
  MapNode myMapNode(nh1, nhp1);

  ros::spin();
  
  return 0;
}


