#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <boost/thread.hpp>
#include <nav_msgs/Odometry.h>

#define HZ 10
#define USE_VERSION 2
#define LINEAR_SPEED_CAP 0.3

/* v1: the wheelchair 'lags' behind the target by so many waypoints
eg. at linear speed of 0.3 m/s, starting 3m behind gives 10s delay
waypoints = desired controller spin rate * time needed to cover initial lag (x Hz, 10 s) */
#define DELAY 5
#define LAG (HZ*DELAY)

/* v2: wheelchair just keeps a certain distance between itself and the target
doesn't strictly adhere to its path */
#define DIST_HOLD 1.2 //metres (include ~60 cm to front of unit from rear axle)

ros::Publisher pub, target_odom_pub;
geometry_msgs::Twist vels;

//v1: each waypoint has a cartesian position ([x y]) + a heading
double parr[LAG][2], harr[LAG];
//v1: keeps track of array position, and number of waypoints cleared
int arr_pos=0, passed=0;

//v2: doesn't take into account orientation of target, only displacement
double linear_d;

//general variables
double fpos_x,fpos_y,fol_h, tpos_x,tpos_y,tar_h, dx,dy;
double last_target_time;

double normalize(double in)
//keeps a value between -pi/pi
{
  if(in >= CV_PI) return in - 2*CV_PI;
  else if(in <= -CV_PI) return in + 2*CV_PI;
  else return in;
}

void controller_v1(void)
/* manages arrays of waypoint data
computes and publishes drive rates */
{
  parr[arr_pos][0] = tpos_x;
  parr[arr_pos][1] = tpos_y;
  harr[arr_pos] = tar_h;
  arr_pos++; if(arr_pos == LAG) arr_pos=0;

  if(passed < LAG)
  {
	vels.linear.x = 0.3;
	vels.angular.z = 0;
  }
  else
  {
	vels.linear.x = sqrt(pow(parr[arr_pos][0]-fpos_x,2) + pow(parr[arr_pos][1]-fpos_y,2)) * HZ;
	vels.angular.z = normalize(harr[arr_pos] - fol_h) * HZ;
  }

  passed++;
}

double cap_vel(double in)
{
  if(in>=LINEAR_SPEED_CAP) return LINEAR_SPEED_CAP;
  else return in;
}

void controller_v2(void)
{
	linear_d = sqrt(dx*dx+dy*dy);

	if(linear_d <= DIST_HOLD) vels.linear.x = 0;
	else vels.linear.x = cap_vel(linear_d - DIST_HOLD);

	vels.angular.z = atan2(dy,dx);
}

void leader_on_follower(void)
//computes latest target waypoint for controller
{
  ros::Rate rate(5);
  tf::TransformListener lr;

  while(ros::ok())
  {
	tf::StampedTransform transform;
	try
	{
		lr.lookupTransform("base_link", "target", ros::Time(0), transform);
	}
	catch(tf::TransformException ex) {}
	dx = transform.getOrigin().x();
	dy = transform.getOrigin().y();
	//tar_h = fol_h + tf::getYaw(transform.getRotation());

	last_target_time = transform.stamp_.toSec();
	rate.sleep();
  }
}

void leader_on_odom(void)
//computes latest target waypoint for controller
{
  ros::Rate rate(5);
  tf::TransformListener lr;

  nav_msgs::Odometry targ_odom; targ_odom.pose.pose.position.z = 0;
  targ_odom.header.frame_id = "odom"; targ_odom.child_frame_id = "target_real";

  while(ros::ok())
  {
	tf::StampedTransform transform;
	try
	{
		lr.lookupTransform("odom", "target", ros::Time(0), transform);
	}
	catch(tf::TransformException ex) {}
	targ_odom.pose.pose.position.x = tpos_x = transform.getOrigin().x();
	targ_odom.pose.pose.position.y = tpos_y = transform.getOrigin().y();
	tar_h = tf::getYaw(transform.getRotation()); targ_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(tar_h);

	last_target_time = transform.stamp_.toSec(); targ_odom.header.stamp = transform.stamp_;
	target_odom_pub.publish(targ_odom); rate.sleep();
  }
}

void follower_pose(const nav_msgs::Odometry::ConstPtr& fol)
//sets follower's latest pose for waypoint_gen
{
  fpos_x = fol->pose.pose.position.x;
  fpos_y = fol->pose.pose.position.y;
  fol_h = tf::getYaw(fol->pose.pose.orientation);
}

void spinner(void)
//times the controller thread
{
  ros::Rate rate(HZ);

  ROS_INFO("Press any key on 'Waypoints' window to start velocity controller");
  int wait = cvWaitKey(-1); ROS_INFO("Starting controller v%d", USE_VERSION);

  while(ros::ok())
  {
	if(ros::Time::now().toSec() - last_target_time > 0.5) vels.linear.x = vels.angular.z = 0;
	else
	{
		switch(USE_VERSION)
		{
			case 1:
			{
				controller_v1();
				break;
			}
			case 2:
			{
				controller_v2();
				break;
			}
			default: break;
		}
	}
	pub.publish(vels);
	rate.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wheelchair_controller"); 
  ros::NodeHandle nh;

  cvNamedWindow("Waypoints");
  cvStartWindowThread();

  if(USE_VERSION==1) ros::Subscriber sub = nh.subscribe("odom", 1, follower_pose);
  pub = nh.advertise<geometry_msgs::Twist>("drive_rates", 1);
  target_odom_pub = nh.advertise<nav_msgs::Odometry>("target_rl", 1);

  boost::thread controlThread(boost::bind(spinner));
  boost::thread odom_leader(boost::bind(leader_on_odom));
  boost::thread fol_leader(boost::bind(leader_on_follower));
  ros::spin();

  cvDestroyWindow("Waypoints");
  controlThread.join(); odom_leader.join(); fol_leader.join();
  return 0;
}
