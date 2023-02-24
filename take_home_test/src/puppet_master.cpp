/**
 * @file puppet_master.cpp
 * @author Karthick C K
 * @brief 
 * @version 0.1
 * @date 2023-02-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <take_home_test/GoToGoalAction.h>
#include <vector>
#include <tuple>

int main (int argc, char **argv)
{
  // Creating and registering a node
  ros::init(argc, argv, "puppet_master");

  // create the action client
  actionlib::SimpleActionClient<take_home_test::GoToGoalAction> ac("GoToGoal", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();
  
  // saving a vector of goals to be sent 
  std::tuple<double,double,double> tuple_pose;
  std::vector<std::tuple<double,double,double>> vec_goals;
  vec_goals.emplace_back(std::make_tuple<double,double,double>(0,5,0));
  vec_goals.emplace_back(std::make_tuple<double,double,double>(4,2,1.57));
  vec_goals.emplace_back(std::make_tuple<double,double,double>(9,1,0));
  vec_goals.emplace_back(std::make_tuple<double,double,double>(2,8,0));
  vec_goals.emplace_back(std::make_tuple<double,double,double>(8,8,0));

  bool finish_status = true;
  for (uint i = 0; i < vec_goals.size();i++){
    ROS_INFO("Sending goal(%f,%f)",std::get<0>(vec_goals[i]),std::get<1>(vec_goals[i]));
    // send a goal to the action
    take_home_test::GoToGoalGoal goal;
    goal.x = std::get<0>(vec_goals[i]);
    goal.y = std::get<1>(vec_goals[i]);
    goal.theta = std::get<2>(vec_goals[i]);
    ac.sendGoal(goal);

    //wait for the action to return
    // ac.cancelGoal();
    bool finished_before_timeout = ac.waitForResult();
    actionlib::SimpleClientGoalState state = ac.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Goal reached"); 
    }
    else if (state == actionlib::SimpleClientGoalState::PREEMPTED){
      ROS_ERROR("Goal preempted"); return 1;
    }
    else {
      ROS_ERROR("%s",ac.getResult()->error_message.c_str()); finish_status = false; return 1;
    }
  }
  if (finish_status) ROS_INFO("Success: all destinations complete!");

  //exit
  return 0;
}
