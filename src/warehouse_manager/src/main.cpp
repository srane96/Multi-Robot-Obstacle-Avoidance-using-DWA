#include "ros/ros.h"
#include <iostream>
#include <warehouse_manager/environment_master.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_datatypes.h"



int main(int argv, char **argc) {
  ros::init(argv, argc, "warehouse_master");
  ros::NodeHandle nh_env;
  
  EnvironmentMaster env(nh_env);

  env.init();
 
  ros::spin();
    
  return 0;
}

