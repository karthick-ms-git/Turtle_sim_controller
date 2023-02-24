/**
 * @file take_home_test.cpp
 * @author Karthick C K
 * @brief This file implements an action server for moving a turtle sim robot to a given pose
 * @version 0.1
 * @date 2023-02-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <take_home_test/GoToGoalAction.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <math.h>

class GoToGoalAction
{
private:

	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<take_home_test::GoToGoalAction> as_; 
	std::string action_name_;
	take_home_test::GoToGoalFeedback feedback_;
	take_home_test::GoToGoalResult result_;
	ros::Subscriber pose_sub_;
	ros::Publisher vel_pub_;
	double curr_x_;
	double curr_y_;
	double curr_theta_;
	double dist_tol_;
	double ang_tol_;
	bool init_ = false;

public:

	GoToGoalAction(std::string name) :
	as_(nh_, name, boost::bind(&GoToGoalAction::executeCB, this, _1), false),
	action_name_(name)
	{
		as_.start();
		pose_sub_ = nh_.subscribe("turtle1/pose", 1000, &GoToGoalAction::poseCB,this);
		vel_pub_ = nh_.advertise<geometry_msgs :: Twist>("/turtle1/cmd_vel", 1000);
		dist_tol_ = 0.1;
		ang_tol_ = 0.005;
		init_ = false;
	}

	~GoToGoalAction(void)
	{
	}
	/**
	 * @brief calculates the euclidean distance of given points
	 * 
	 * @param x x co-ordinate of 1st point
	 * @param y y co-ordinate of 1st point
	 * @param x2 x co-ordinate of 2nd point
	 * @param y2 y co-ordinate of 2nd point
	 * @return double 
	 */
	double distance(double x, double y, double x2, double y2)
	{
		return sqrt(std::pow(x - x2, 2) + pow(y - y2, 2));
	}

	/**
	 * @brief callback for the subscriber of type pose
	 * 
	 * @param pose_msg 
	 */
	void poseCB(const turtlesim::Pose::ConstPtr& pose_msg)
	{	
		init_ = true;
		curr_x_ = pose_msg->x;
		curr_y_ = pose_msg->y;
		curr_theta_ = pose_msg->theta;
	}

	/**
	 * @brief callback for the action
	 * 
	 * @param goal 
	 */
	void executeCB(const take_home_test::GoToGoalGoalConstPtr &goal)
	{	
		int duration = 0;
		while (!init_ && duration<5){
			ros::Rate loop_rate(0.5);
			loop_rate.sleep();
			duration++;
		}
		if (init_){			

			// To check the goal is within range
			if (goal->x > 9 || goal->y > 9) {
				result_.goal_status = false;
				result_.error_message = "Too far from the goal range";
				// set the action state to fail
				as_.setAborted(result_);
				ROS_INFO("Too far from the goal range");
			}

			if (distance(curr_x_, curr_y_, goal->x, goal->y) < dist_tol_) goto label1;
			if (!SteerStart(goal)) return;
			feedback_.completion_rate = 30;as_.publishFeedback(feedback_);
			ROS_INFO("Steered to target Succeeded");

			if(!Throttle(goal)) return;
			feedback_.completion_rate = 75;as_.publishFeedback(feedback_);
			ROS_INFO("Position of target Succeeded");

			label1:
			if(!SteerFinal(goal)) return;
			feedback_.completion_rate = 100;as_.publishFeedback(feedback_);
			ROS_INFO("angle of target Succeeded");

			result_.goal_status = true;
			// set the action state to succeeded
			as_.setSucceeded(result_);
			ROS_INFO("Goal succeeded");
		}
		else {
			result_.goal_status = false;
			result_.error_message = "Did not receive pose message";
			// set the action state to fail
			as_.setAborted(result_);
			ROS_INFO("Did not receive pose message");
		}
	}

	/**
	 * @brief steering to the direction towards target
	 * 
	 * @param goal 
	 * @return true if everything is successful
	 * @return false otherwise
	 */
	bool SteerStart(const take_home_test::GoToGoalGoalConstPtr &goal)
	{
		// helper variables
		ros::Rate loop_rate(10);
		bool success = true;

		double goal_x = goal->x;
		double goal_y = goal->y;
		double goal_theta = goal->theta;

		// init_ executing steering init_
		while (std::abs(std::atan2(goal_y - curr_y_, goal_x - curr_x_) - curr_theta_) > ang_tol_)
		{
			if (as_.isPreemptRequested() || !ros::ok())
			{
				ROS_INFO("%s: Preempted", action_name_.c_str());
				// set the action state to preempted
				as_.setPreempted();
				ROS_INFO("Goal Preempted");
				return false;
			}
 			double Kp = 4;
			double angular_speed = Kp * (std::atan2(goal_y - curr_y_, goal_x - curr_x_) - curr_theta_);
			geometry_msgs :: Twist vel; 
			vel.angular.z = angular_speed;
			vel_pub_.publish(vel);
			loop_rate.sleep();
			ros :: spinOnce();
		}
		return true;
	}

	/**
	 * @brief throttling towards target
	 * 
	 * @param goal 
	 * @return true if everything is successful
	 * @return false otherwise
	 */
	bool Throttle(const take_home_test::GoToGoalGoalConstPtr &goal)
	{	
		// helper variables
		ros::Rate loop_rate(10);
		bool success = true;

		double goal_x = goal->x;
		double goal_y = goal->y;
		double goal_theta = goal->theta;
		
		// init_ executing throttle
		while (distance(curr_x_, curr_y_, goal_x, goal_y) > dist_tol_)
		{
			if (as_.isPreemptRequested() || !ros::ok())
			{
				ROS_INFO("%s: Preempted", action_name_.c_str());
				// set the action state to preempted
				as_.setPreempted();
				ROS_INFO("Goal Preempted");
				return false;
			}
			float kp = 0.25;
			double speed = kp * distance(curr_x_, curr_y_, goal_x, goal_y);
			geometry_msgs :: Twist vel;
			vel.linear.x = speed;
			vel_pub_.publish(vel);
			loop_rate.sleep();
			ros :: spinOnce();
		}
		return true;
	}

	/**
	 * @brief steering to the direction of target
	 * 
	 * @param goal 
	 * @return true if everything is successful
	 * @return false otherwise
	 */
	bool SteerFinal(const take_home_test::GoToGoalGoalConstPtr &goal)
	{	
		// helper variables
		ros::Rate loop_rate(10);
		bool success = true;

		double goal_x = goal->x;
		double goal_y = goal->y;
		double goal_theta = goal->theta;
		
		// init_ executing steering final
		while (std::abs(goal_theta - curr_theta_) > ang_tol_)
		{
			if (as_.isPreemptRequested() || !ros::ok())
			{
				ROS_INFO("%s: Preempted", action_name_.c_str());
				// set the action state to preempted
				as_.setPreempted();
				ROS_INFO("Goal Preempted");
				return false;
			}
			double Kp = 3;
			double angular_speed = Kp * (goal_theta - curr_theta_);
			geometry_msgs :: Twist vel; 
			vel.angular.z = angular_speed;
			vel_pub_.publish(vel);
			loop_rate.sleep();
			ros :: spinOnce();
		}
		return true;
	}

};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "take_home_test");

	GoToGoalAction GoToGoal("GoToGoal");
	ros::spin();

	return 0;
}
