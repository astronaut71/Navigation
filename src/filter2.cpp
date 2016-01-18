#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PoseStamped.h>

#define KALMAN_HZ 5

using namespace cv;

void setup_kalman(KalmanFilter& kalman)
{
  float a[4][4], dt = 1/KALMAN_HZ;
  for(int i=0; i<2; i++)
  {
	a[i][i] = a[i+2][i+2] = 1;
	a[i][i+2] = dt;
  }

  kalman.transitionMatrix = Mat(4, 4, CV_32FC1, a);
  setIdentity(kalman.measurementMatrix, Scalar(1));
  setIdentity(kalman.processNoiseCov, Scalar(0.5));
  setIdentity(kalman.measurementNoiseCov, Scalar(0.1));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_filter");
  ros::NodeHandle nh;

  tf::TransformListener lr;
  tf::StampedTransform laser_trans, kinect_trans;

  geometry_msgs::PoseStamped target_pose;
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("leader_pose", 1);
  target_pose.pose.position.z = 0; target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
  target_pose.header.frame_id = "map";

  Mat measurement = Mat(2, 1, CV_32FC1);
  KalmanFilter kalman = KalmanFilter(4,2,0);
  setup_kalman(kalman);

  ros::Rate kalman_rate(KALMAN_HZ);
  while(ros::ok())
  {
	try
	{
		lr.lookupTransform("map", "laser_target", ros::Time(0), laser_trans);
	}
	catch(tf::TransformException ex) {}
	/*try
	{
		lr.lookupTransform("odom", "kinect_target", ros::Time(0), kinect_trans);
	}
	catch(tf::TransformException ex) {}

	measurement.at<float>(0,0) = (laser_trans.getOrigin().x() + kinect_trans.getOrigin().x())/2;
	measurement.at<float>(1,0) = (laser_trans.getOrigin().y() + kinect_trans.getOrigin().y())/2;*/
	measurement.at<float>(0,0) = laser_trans.getOrigin().x();
	measurement.at<float>(1,0) = laser_trans.getOrigin().y();

	Mat predict = kalman.predict();	kalman.correct(measurement);

	target_pose.pose.position.x = kalman.statePost.at<float>(0,0);
	target_pose.pose.position.y = kalman.statePost.at<float>(1,0);
	target_pose.header.stamp = laser_trans.stamp_;

	pub.publish(target_pose);
	kalman_rate.sleep();
  }

  return 0;
}
