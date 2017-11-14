#include <ros/ros.h>
#include "../sptam/tracker.hpp"

int main(int argc, char **argv){

  ros::init(argc,argv,"TrackerNode");
  ros::NodeHandle nh("");
  ros::NodeHandle nhp("~"); 

  ROS_INFO("Tracker node running...");
  Tracker myTracker(nh, nhp);

  ros::spin();
  return 0;
}
