#ifndef ENVIRONMENT_MASTER_INCLUDED
#define ENVIRONMENT_MASTER_INCLUDED

#include "ros/ros.h"
#include "warehouse_manager/Robot_Task_Complete.h"
#include "warehouse_manager/Robot_Task_Request.h"
#include "warehouse_manager/Robot_Gen_Report.h"
#include "warehouse_manager/TaskInfo.h"
#include "warehouse_manager/RobotInfo.h"
#include <std_msgs/Int32.h>
#include <iostream>
#include <stdio.h>
#include <tuple>
#include <vector>
#include<queue>

class EnvironmentMaster {
private:
  std::map<int, std::vector<std::tuple<int, int>>> robot_tasks_;
  std::map<int, std::vector<float>> robot_time_;
  std::map<int, std::vector<float>> robot_distance_;
  ros::NodeHandle n_;

public:
  ros::ServiceServer report_task_completion;
  ros::ServiceServer request_available_task;
  ros::ServiceServer request_all_task_complete;
  ros::Subscriber collision_sub;
  
  double begin_;
  double end_;
  int robot_number_;
  int robot_count_;
  int num_collisions_;
  EnvironmentMaster();
  
  EnvironmentMaster(ros::NodeHandle nh) { this->n_ = nh; }

  void init();

  bool task_complete(warehouse_manager::Robot_Task_Complete::Request &req,
                     warehouse_manager::Robot_Task_Complete::Response &res);

  bool req_task(warehouse_manager::Robot_Task_Request::Request &req,
                     warehouse_manager::Robot_Task_Request::Response &res);

  bool all_task_complete(warehouse_manager::Robot_Gen_Report::Request &req,
                     warehouse_manager::Robot_Gen_Report::Response &res);

  void collisionCallback(const std_msgs::Int32::ConstPtr& msg);

  void add_to_report(int robot_number);

  ~EnvironmentMaster();
};

#endif
