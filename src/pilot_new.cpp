#include <ros/ros.h>
#include <boost/thread.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// vel limits for Stage
#define LINEAR_SPEED_CAP 0.6 // (m/s)
#define ANGULAR_SPEED_CAP 0.8 // (rad/s)

#define FULL_SPEED_THRESH 20 // turn degree threshold above which
							// speed throttling will apply

#define SAFETY_DISTANCE 1.6 //safety distance from wc to target (m)
							// + 0.6m from rear axle to front of wc

#define PROXIMITY_THRESHOLD 0.5 //acceptable closeness to move to next point (m)
#define MIN_SPACING 0.25 //minimum distance between consecutive steps (m)

#define hypotenuse(dx,dy) sqrt(dx*dx + dy*dy)

bool kinect_warn = 0,laser_warn = 0;
nav_msgs::Path leader_path; int path_write_step = -1;
ros::Publisher vel_pub, lr_odom_pub;

double gap_x=0,gap_y=0;
double ang_in;

double fix_ang(double in)
{
  ang_in = in;
  while(ang_in>M_PI) ang_in-= 2*M_PI;
  while(ang_in<-M_PI) ang_in+=2*M_PI;
  return ang_in;
}

void path_handler(const geometry_msgs::PoseStamped& msg)
{
  if(path_write_step>-1)
  {
	gap_x = msg.pose.position.x - leader_path.poses[path_write_step].pose.position.x;
	gap_y = msg.pose.position.y - leader_path.poses[path_write_step].pose.position.y;
  }

  if(hypotenuse(gap_x,gap_y) > MIN_SPACING || path_write_step==-1)
  {
	leader_path.poses.push_back(msg); path_write_step++;
	lr_odom_pub.publish(leader_path);
  }
}

int controller(void)
{
  geometry_msgs::Twist vels;
  tf::TransformListener lr; tf::StampedTransform transform;
  double lin_d, dyaw, wc_x,wc_y, dx,dy, rt_dx,rt_dy, heading, dx_,dy_,gain_next;
  const double speed_thresh = FULL_SPEED_THRESH * (M_PI/180);
  int path_read_step = 0;
  ros::Rate rate(10);

  while(path_write_step <= 0) rate.sleep();
  while(ros::ok())
  {
	try
	{
		lr.lookupTransform("map", "base_link", ros::Time(0), transform);
	}
	catch(tf::TransformException ex) {}
	wc_x = transform.getOrigin().x(); wc_y = transform.getOrigin().y();

	rt_dx = leader_path.poses[path_write_step].pose.position.x - wc_x;
	rt_dy = leader_path.poses[path_write_step].pose.position.y - wc_y;

	if((kinect_warn||laser_warn) || hypotenuse(rt_dx,rt_dy)<SAFETY_DISTANCE || path_read_step>=path_write_step-1)
	{
		vels.linear.x = vels.angular.z = 0;
	}
	else
	{
		dx = leader_path.poses[path_read_step].pose.position.x - wc_x;
		dy = leader_path.poses[path_read_step].pose.position.y - wc_y;
		lin_d = hypotenuse(dx,dy); heading = tf::getYaw(transform.getRotation());

		//set a speed proportional to the change in heading between next two points
		dx_ = leader_path.poses[path_read_step+1].pose.position.x - leader_path.poses[path_read_step].pose.position.x;
		dy_ = leader_path.poses[path_read_step+1].pose.position.y - leader_path.poses[path_read_step].pose.position.y;
		gain_next = cos(fix_ang(atan2(dy_,dx_)-heading)); if(gain_next<=speed_thresh) gain_next = 1;
		vels.linear.x = LINEAR_SPEED_CAP*gain_next;

		dyaw = fix_ang(atan2(dy,dx)-heading);
		vels.angular.z = dyaw*(lin_d/vels.linear.x);
		if(vels.angular.z>ANGULAR_SPEED_CAP) vels.angular.z = ANGULAR_SPEED_CAP;
		else if(vels.angular.z<-ANGULAR_SPEED_CAP) vels.angular.z = -ANGULAR_SPEED_CAP;

		if(lin_d<PROXIMITY_THRESHOLD) path_read_step++;
	}
	vel_pub.publish(vels); //rate.sleep();
  }
  return 0;
}

void kinect_safety_cb(const std_msgs::Bool::ConstPtr& b)
{
  kinect_warn = b->data;
}

void laser_safety_cb(const std_msgs::Bool::ConstPtr& b)
{
  laser_warn = b->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wheelchair_controller");
  ros::NodeHandle nh;

  leader_path.header.frame_id = "odom";

  ros::Subscriber sub = nh.subscribe("leader_pose", 1, path_handler);
  ros::Subscriber sub1 = nh.subscribe("kinect_obs", 1, kinect_safety_cb);
  ros::Subscriber sub2 = nh.subscribe("laser_obs", 1, laser_safety_cb);

  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  lr_odom_pub = nh.advertise<nav_msgs::Path>("leader_path", 1);

  boost::thread wc_driver(boost::bind(controller));
  ros::spin();

  wc_driver.join();
  return 0;
}
