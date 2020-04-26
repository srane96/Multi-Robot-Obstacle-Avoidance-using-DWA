#include <fstream>
#include <warehouse_manager/environment_master.h>
#include "nav_msgs/OccupancyGrid.h"


EnvironmentMaster::EnvironmentMaster() {
  std::cout << "Main constructor" << std::endl;
}

EnvironmentMaster::~EnvironmentMaster() {
  std::cout << "Terminate" << std::endl;
}

void EnvironmentMaster::init() {
  //Code block for reading robot task assignmnet from a text file
  robot_number_ = 0;
  robot_count_ = 0;
  std::ifstream f("/home/siddhesh/warehouse_sim/DWA-for-Decentralized-Multi-Agent-Warehouse-Simulation/src/warehouse_manager/task100robot10.txt");
  if(f.is_open()){
    while(f){
      std::string line;
      std::getline(f, line);
      std::istringstream iss(line);
      if (line != ""){
        int robot = 0;
        int robot_x = 0;
        int robot_y = 0;
        for(int i = 0; i < 3; i++){
          std::string temp;
          iss >> temp;
          if(i == 0) robot = std::stoi(temp);
          else if (i == 1) robot_x = std::stoi(temp);
          else if (i == 2) robot_y = std::stoi(temp);
        }
        std::tuple<int, int> temp_t(robot_x, robot_y);
        if(robot_tasks_.find(robot) == robot_tasks_.end()){
          std::vector<std::tuple<int, int>> v;
          v.push_back(temp_t);
          robot_tasks_.insert({robot, v});
          robot_number_ ++;
          robot_count_ ++;
        }
        else{
          robot_tasks_[robot].push_back(temp_t);
        }
      }
    }
  }
  
  // Init the publisher and server
  report_task_completion = this->n_.advertiseService(
      "report_task_complete", &EnvironmentMaster::task_complete, this);
  
  request_available_task = this->n_.advertiseService(
      "request_available_task", &EnvironmentMaster::req_task, this);

  request_all_task_complete = this->n_.advertiseService("gen_final_report", &EnvironmentMaster::all_task_complete, this);

  collision_sub = this->n_.subscribe("/collisions", 10, &EnvironmentMaster::collisionCallback, this);
  begin_ = ros::Time::now().toSec();
}

void EnvironmentMaster::collisionCallback(const std_msgs::Int32::ConstPtr& msg)
{
  num_collisions_ = msg->data;
}

//We are using this currently
bool EnvironmentMaster::req_task(warehouse_manager::Robot_Task_Request::Request &req,
                     warehouse_manager::Robot_Task_Request::Response &res){
                       ROS_INFO_STREAM("Goal Request arrived" << req.name);
                       int robot_number = std::stoi(req.name);
                       std::tuple<int, int> temp_t;
                       if(robot_tasks_[robot_number].size() > 0){
                         temp_t = robot_tasks_[robot_number][robot_tasks_[robot_number].size() - 1];
                         res.x = std::get<0>(temp_t);
                         res.y = std::get<1>(temp_t);
                         res.task_available = true;
                         robot_tasks_[robot_number].erase(robot_tasks_[robot_number].end());
                         return true;
                       }
                       else{
                         //Robot has finished all the tasks assigned to it.
                         //Add it to the report
                         res.x = -1;
                         res.y = -1;
                         res.task_available = false;
                         add_to_report(robot_number);
                         robot_count_ --;
                         if(robot_count_ == 0) add_to_report(-1); 
                         return true;
                       }
                     }

//Service to register goal completion message, not used now, have to be modified
bool EnvironmentMaster::task_complete(
    warehouse_manager::Robot_Task_Complete::Request &req,
    warehouse_manager::Robot_Task_Complete::Response &res) {
  int robot_number = std::stoi(req.robot_name);
  float time_taken  = req.time_taken;
  float shortest_distance = req.shortest_distance;
  if(robot_time_.find(robot_number) == robot_time_.end())
  {
    std::vector<float> temp;
    temp.push_back(time_taken);
    robot_time_.insert({robot_number,temp});
  }
  else
  {
    robot_time_[robot_number].push_back(time_taken);
  }

  if(robot_distance_.find(robot_number) == robot_distance_.end())
  {
    std::vector<float> temp_d;
    temp_d.push_back(shortest_distance);
    robot_distance_.insert({robot_number, temp_d});
  }
  else
  {
    robot_distance_[robot_number].push_back(shortest_distance);
  }
  
  return true;
}

bool EnvironmentMaster::all_task_complete(warehouse_manager::Robot_Gen_Report::Request &req,
                     warehouse_manager::Robot_Gen_Report::Response &res)
{
 if(req.generate_report){
 std::ofstream outfile;
 outfile.open("/home/siddhesh/warehouse_sim/DWA-for-Decentralized-Multi-Agent-Warehouse-Simulation/src/warehouse_manager/final_report.txt", std::ios_base::app);
 outfile << "\n" << "\n";
 outfile << "Time and distance: " << "\n";
 float total_time;
 float total_distance;
 float one_robot_time;
 float one_robot_distance;
 int temp_num = robot_number_;
 while(temp_num  >= 0)
 {
   for(int i = 0; i < robot_time_[temp_num].size(); i++)
   {
     one_robot_time += robot_time_[temp_num][i];
     one_robot_distance += robot_distance_[temp_num][i]; 
   }
   outfile << "Robot: " << temp_num << " Time: " << one_robot_time << " Distance: " << one_robot_distance << "\n";
   if(robot_time_[temp_num].size() > 0){
      total_time += (one_robot_time/robot_time_[temp_num].size());
      total_distance += (one_robot_distance/robot_time_[temp_num].size());  
    }  
   temp_num--;
 }
 outfile << "\n" << "\n";
 double diff_now = ros::Time::now().toSec() - begin_;
 outfile << "Total(1/Tasks)- "  << " Time: " << total_time << " Distance: " << total_distance << "\n";
 outfile << "Total Time(For all tasks)- " << diff_now << "\n"; 
 //Print the number of collisions
 //boost::shared_ptr<std_msgs::Int32 const> shared_count;
 //std_msgs::Int32 col_count;
 //shared_count = ros::topic::waitForMessage<std_msgs::Int32>("/collisions",this->n_);
 //if(shared_count != NULL)
 //{
 //  col_count = *shared_count;
 //}
 outfile << "\n";
 outfile << "Total number of collisions: " << num_collisions_ << "\n";
 outfile.close();
 }
}

void EnvironmentMaster::add_to_report(int robot_number)
{
  std::ofstream outfile;
  outfile.open("/home/siddhesh/warehouse_sim/DWA-for-Decentralized-Multi-Agent-Warehouse-Simulation/src/warehouse_manager/task100robot10_report.txt", std::ios_base::app);
  if(robot_number >= 0 )
  {
    outfile << "\n" << "\n";
    for(int i = 0; i < robot_time_[robot_number].size(); i++)
    {
      outfile << "Task: " << i << " Robot: " << robot_number << " Time: " << robot_time_[robot_number][i] << " Distance: " << robot_distance_[robot_number][i] << "\n"; 
    }
  }
  else
  {
    // Warehouse time
    end_ = ros::Time::now().toSec();
    double diff = end_ - begin_; 
    //Adding total time and total distance
    outfile << "\n" << "\n";
    outfile << "Time and distance: " << "\n";
    float total_time;
    float total_distance;
    float one_robot_time;
    float one_robot_distance;
    int temp_num = robot_number_;
    while(temp_num  >= 0)
    {
      for(int i = 0; i < robot_time_[temp_num].size(); i++)
      {
        one_robot_time += robot_time_[temp_num][i];
        one_robot_distance += robot_distance_[temp_num][i]; 
      }
      outfile << "Robot: " << temp_num << " Time: " << one_robot_time << " Distance: " << one_robot_distance << "\n";
      if(robot_time_[temp_num].size() > 0){
        total_time += (one_robot_time/robot_time_[temp_num].size());
        total_distance += (one_robot_distance/robot_time_[temp_num].size());  
      }
      temp_num  --;
    }
    outfile << "\n" << "\n";
    outfile << "Total(1/Tasks)- "  << " Time: " << total_time << " Distance: " << total_distance << "\n";
    outfile << "Total Time(For all tasks)- " << diff << "\n";
    //Print the number of collisions
    //boost::shared_ptr<std_msgs::Int32 const> shared_count;
    //std_msgs::Int32 col_count;
    //shared_count = ros::topic::waitForMessage<std_msgs::Int32>("/collisions",this->n_);
    //if(shared_count != NULL)
    //{
    //  col_count = *shared_count;
    //}
    outfile << "\n";
    outfile << "Total number of collisions: " << num_collisions_ << "\n";
  }
  outfile.close();
}
