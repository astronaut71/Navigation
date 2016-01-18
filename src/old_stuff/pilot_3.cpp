#include <ros/ros.h>
#include <boost/thread.hpp>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// vel limits for Stage
#define LINEAR_SPEED_CAP 0.3 // (m/s)
#define ANGULAR_SPEED_CAP 0.5 // (rad/s)

#define DIST_HOLD 1.2 // safety dist (m): include additional ~60 cm to front of unit from rear axle
#define PROXIMITY_THRESHOLD 0.75 //acceptable closeness to move to next point (m)
#define MIN_SPACING 0.20 //minimum distance between consecutive steps (m)

bool safe = 1;
int path_read_step = 0;
nav_msgs::Path leader_path;
ros::Publisher vel_pub, lr_odom_pub;

double hypotenuse(double& one, double& two)
{
  return sqrt(one*one + two*two);
}

int path_handler(void)
{
  tf::TransformListener lr;
  double ldr_h, dx,dy; int seq = 1;

  geometry_msgs::PoseStamped pose; tf::StampedTransform transform;
  leader_path.header.frame_id = pose.header.frame_id = "odom";

  pose.pose.position.x = pose.pose.position.y = pose.pose.position.z = 0;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
  leader_path.poses.push_back(pose);

  ros::Rate rate(5);
  while(ros::ok())
  {
	try
	{
		lr.lookupTransform("odom", "target", ros::Time(0), transform);
	}
	catch(tf::TransformException ex) {}

	pose.pose.position.x = transform.getOrigin().x();
	pose.pose.position.y = transform.getOrigin().y();
	//ldr_h = tf::getYaw(transform.getRotation()); pose.pose.orientation = tf::createQuaternionMsgFromYaw(ldr_h);

	dx = pose.pose.position.x - leader_path.poses[seq-1].pose.position.x;
	dy = pose.pose.position.y - leader_path.poses[seq-1].pose.position.y;
	if(hypotenuse(dx,dy) > MIN_SPACING)
	{
		pose.header.stamp = transform.stamp_;
		pose.header.seq = seq++;
		leader_path.poses.push_back(pose);
	}

	lr_odom_pub.publish(leader_path);
	rate.sleep();
  }
  return 0;
}

int waypoint_handler(void)
{
  tf::TransformBroadcaster waypoint_caster;
  geometry_msgs::TransformStamped point; point.transform.translation.z = 0;
  point.header.frame_id = "odom"; point.child_frame_id = "waypoint";
  point.transform.rotation = tf::createQuaternionMsgFromYaw(0);

  ros::Rate rate(10); while(!leader_path.poses.size()) rate.sleep();
  while(ros::ok())
  {
	point.transform.translation.x = leader_path.poses[path_read_step].pose.position.x;
	point.transform.translation.y = leader_path.poses[path_read_step].pose.position.y;
	//point.transform.rotation = leader_path.poses[path_read_step].pose.orientation;

	point.header.stamp = ros::Time::now();
	waypoint_caster.sendTransform(point);
	rate.sleep();
  }
  return 0;
}

void cap_speeds(double& dist, double& ang, geometry_msgs::Twist& msg)
{
  if(dist>=LINEAR_SPEED_CAP) msg.linear.x = LINEAR_SPEED_CAP;
  else msg.linear.x = dist;

  if(ang >= ANGULAR_SPEED_CAP) msg.angular.z = ANGULAR_SPEED_CAP;
  else if(ang <= -ANGULAR_SPEED_CAP) msg.angular.z = -ANGULAR_SPEED_CAP;
  else msg.angular.z = ang;
}

int velocity_handler(void)
{
  geometry_msgs::Twist vels;
  tf::TransformListener lr; tf::StampedTransform transform;
  double lin_d, ang, dx,dy;

  ros::Rate rate(5);
  while(ros::ok())
  {
	try
	{
		lr.lookupTransform("base_link", "waypoint", ros::Time(0), transform);
	}
	catch(tf::TransformException ex) {}

	dx = transform.getOrigin().x();	dy = transform.getOrigin().y();
	lin_d = hypotenuse(dx,dy); ang = atan2(dy,dx);

	if(!safe || path_read_step>=leader_path.poses.size()) vels.linear.x = vels.angular.z = 0;
	else
	{
		cap_speeds(lin_d, ang, vels);
		if(lin_d<PROXIMITY_THRESHOLD) path_read_step++;
	}

	vel_pub.publish(vels);
	rate.sleep();
  }
  return 0;
}

int safety(void)
{
  double dx,dy;
  tf::TransformListener lr; tf::StampedTransform transform;

  ros::Rate rate(5);
  while(ros::ok())
  {
	try
	{
		lr.lookupTransform("base_link", "target", ros::Time(0), transform);
	}
	catch(tf::TransformException ex) {}

	dx = transform.getOrigin().x();	dy = transform.getOrigin().y();
	if(hypotenuse(dx,dy)<DIST_HOLD || (ros::Time::now()-transform.stamp_).toSec()>0.5) safe = 0;
	else safe = 1;

	rate.sleep();
  }
  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wheelchair_controller");
  ros::NodeHandle nh;
  boost::thread_group threads;

  vel_pub = nh.advertise<geometry_msgs::Twist>("drive_rates", 1);
  lr_odom_pub = nh.advertise<nav_msgs::Path>("leader_path", 1);

  threads.create_thread(boost::bind(path_handler));
  threads.create_thread(boost::bind(waypoint_handler));
  threads.create_thread(boost::bind(velocity_handler));

  bool mode; nh.param("ftl_control/sim",mode, false);
  if(!mode) threads.create_thread(boost::bind(safety));
  else ROS_INFO("No safety thread");
  ros::spin();

  threads.join_all();
  return 0;
}
