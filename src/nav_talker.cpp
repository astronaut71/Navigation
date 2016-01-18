#include <ros/ros.h>
//path thread stuff
#include <boost/thread.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
//move_base actionlib stuff
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#define PRINT 1
#define POSE_SPACING 0.8

ros::Publisher path_pub; nav_msgs::Path leader_path;

int path_builder(void)
//builds a Path of goal poses
{
  leader_path.header.frame_id = "map";
  tf::TransformListener lr; tf::StampedTransform transform;

  geometry_msgs::PoseStamped pose; pose.header.frame_id = "map";
  pose.pose.position.z = 0; double dx,dy; int w;

  ros::Rate rate(10);
  while(ros::ok())
  {
	try
	{
		lr.lookupTransform("map", "laser_target", ros::Time(0), transform);
	}
	catch (tf::TransformException ex) {}
	pose.pose.position.x = transform.getOrigin().x(); pose.pose.position.y = transform.getOrigin().y();

	w = leader_path.poses.size() -1;
	if(w<1) dx=dy = POSE_SPACING;
	else
	{
		dx = pose.pose.position.x - leader_path.poses[w].pose.position.x;
		dy = pose.pose.position.y - leader_path.poses[w].pose.position.y;
	}
	if(sqrt(dx*dx+dy*dy) > POSE_SPACING)
	{
		tf::quaternionTFToMsg(transform.getRotation(), pose.pose.orientation);
		leader_path.poses.push_back(pose); path_pub.publish(leader_path);
		if(PRINT) ROS_INFO("Recieved goal: %f,%f", pose.pose.position.x,pose.pose.position.y);
	}
	rate.sleep();
  }
  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nav_stack_talker");
  ros::NodeHandle nh;

  path_pub = nh.advertise<nav_msgs::Path>("leader_path", 1);
  boost::thread path(boost::bind(path_builder));

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
  while(ros::ok() && !ac.waitForServer(ros::Duration(5.0))) ROS_INFO("Waiting for actionlib server...");
  move_base_msgs::MoveBaseGoal goal; size_t path_r=1;
  
  while(ros::ok())
  //feed goals to move_base. stop some points behind leader to maintain a reasonable dist
  {
	if(path_r < leader_path.poses.size()-2)
	{
		goal.target_pose = leader_path.poses[path_r];
		goal.target_pose.header.stamp = ros::Time::now();

		ac.sendGoal(goal); ac.waitForResult();
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			path_r++;
			if(PRINT) ROS_INFO("Goal successfully reached! Moving to next goal...");
		}
	}
  }

  path.join();
  return 0;
}
