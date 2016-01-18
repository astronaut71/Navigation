#include <ros/ros.h>
#include <opencv/cv.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// !! Note: this node only handles x,y and yaw

//acceptable differences in measurement matrix to prediction matrix (m,m,rad)
#define X_TOL 0.5
#define Y_TOL 0.5
#define YAW_TOL 0.8

//filter spin rate
#define KALMAN_HZ 5

int i;

void setup_kalman(CvKalman*& kalman)
{
  float H[3][6] = {0}, F[6][6] = {0}, dt = 1.0/KALMAN_HZ;

  for(i=0; i<3; i++)
  {
	H[i][i] = 1;

	F[i][i] = F[i+3][i+3] = 1;
	F[i][i+3] = dt;
  }
  memcpy(kalman->measurement_matrix->data.fl,H, sizeof(H));
  memcpy(kalman->transition_matrix->data.fl,F, sizeof(F));

  cvSetIdentity(kalman->process_noise_cov, cvRealScalar(0.5)); 
  cvSetIdentity(kalman->measurement_noise_cov, cvRealScalar(1.0));
}

bool check_mats(CvMat*& one, CvMat*& two, double threshes[])
{
  for(i=0; i<3; i++)
  {
	if(fabs(cvmGet(one,i,0)-cvmGet(two,i,0)) > threshes[i]) return 0;
  }
  return 1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_filter");
  ros::NodeHandle nh;

  tf::TransformListener lr;
  tf::StampedTransform drec_trans, ar_pose_trans;
  ros::Time time; ros::Duration time_since_latest;

  tf::TransformBroadcaster bcaster;
  geometry_msgs::TransformStamped target_pose;
  target_pose.transform.translation.z = 0;
  target_pose.header.frame_id = "odom"; target_pose.child_frame_id = "target";

  CvMat* meas = cvCreateMat(3,1, CV_32FC1);
  double thresholds[3] = {X_TOL,Y_TOL,YAW_TOL};
  CvKalman* kalman = cvCreateKalman(6,3, 0);
  setup_kalman(kalman);

  ros::Rate rate(KALMAN_HZ);
  while(ros::ok())
  {
	try
	{
		lr.lookupTransform("odom", "ar_pose_target", ros::Time(0), ar_pose_trans);
	}
	catch(tf::TransformException ex) {}
	try
	{
		lr.lookupTransform("odom", "depth_target", ros::Time(0), drec_trans);
	}
	catch(tf::TransformException ex) {}

	if(ar_pose_trans.stamp_ >= drec_trans.stamp_)
	{
		cvmSet(meas, 0,0, ar_pose_trans.getOrigin().x()); cvmSet(meas, 1,0, ar_pose_trans.getOrigin().y());
		cvmSet(meas, 2,0, tf::getYaw(ar_pose_trans.getRotation())); time = ar_pose_trans.stamp_;
	}
	else
	{
		cvmSet(meas, 0,0, drec_trans.getOrigin().x()); cvmSet(meas, 1,0, drec_trans.getOrigin().y());
		cvmSet(meas, 2,0, tf::getYaw(drec_trans.getRotation())); time = drec_trans.stamp_;
	}

	time_since_latest = ros::Time::now() - time;
	cvKalmanPredict(kalman, 0);
	if(/*check_mats(meas, kalman->state_pre, thresholds) &&*/ time_since_latest.toSec() < 0.5)
	{
		cvKalmanCorrect(kalman, meas);
		target_pose.transform.translation.x = cvmGet(kalman->state_post,0,0);
		target_pose.transform.translation.y = cvmGet(kalman->state_post,1,0);
		target_pose.transform.rotation = tf::createQuaternionMsgFromYaw(cvmGet(kalman->state_post,2,0));
	}
	else
	{
		target_pose.transform.translation.x = cvmGet(kalman->state_pre,0,0);
		target_pose.transform.translation.y = cvmGet(kalman->state_pre,1,0);
		target_pose.transform.rotation = tf::createQuaternionMsgFromYaw(cvmGet(kalman->state_pre,2,0));
	}

	target_pose.header.stamp = time;
	bcaster.sendTransform(target_pose);
	rate.sleep();
  }

  cvReleaseKalman(&kalman); cvReleaseMat(&meas);
  return 0;
}
