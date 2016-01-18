#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <std_msgs/Bool.h>

//the ar_pose can be used here because I want to use it to start the process through a single launch file.
//however, I do want the depth sensor to do the bulk of the work

namespace enc = sensor_msgs::image_encodings;

struct blob_stuff
{
  float depth;
  int x,y;
};

//normal stuff
double y_rad; float px_total; int mask_px_count; 
cv::Moments blob_moments; blob_stuff blob_details;
const double half_view_angle_rads = 57/2 *(M_PI/180); //maximum horizontal view angle in radians /2
cv::Mat blob; const int ff_flags = cv::FLOODFILL_MASK_ONLY | 255<<8;
cv::Mat di_crop; const cv::Rect diROI(0, 0, 640, 360);

bool click_recieved = 0, first_blob_recorded = 0;
geometry_msgs::TransformStamped pose_trans;

//obstruction check stuff
int obs_px_count, rec_area;
cv::Mat last_unobs_di; cv::Rect mask_rec;
std::vector<std::vector<cv::Point> > v;
ros::Publisher pub; std_msgs::Bool pub_bool;

void mouse_cb(int event,int x,int y,int flags,void* param)
//mouse click handler (for touch screen if it's operational)
{
  if(event == CV_EVENT_LBUTTONUP)
  {
	blob_details.x = x; blob_details.y = y;
	click_recieved = 1; pub_bool.data = 0;
	ROS_INFO("Leader selected");
  }
}

void send_tf()
{
  static tf::TransformBroadcaster br;
  pose_trans.header.stamp = ros::Time::now();
  br.sendTransform(pose_trans);
}

bool obstacle_check(cv::Mat& source, cv::Mat& mask, cv::Mat& last_unobs, double& last_x)
{
  obs_px_count = 0;

  cv::findContours(mask, v, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
  mask_rec = cv::boundingRect(cv::Mat(v[0]));
  rec_area = (mask_rec.width-1) * (mask_rec.height-1);

  px_total = 0.0; mask_px_count = 0;

  for(int i=mask_rec.y; i<mask_rec.y+mask_rec.height-1; i++)
  {
	float* Ci = source.ptr<float>(i);
	float* Pi = last_unobs.ptr<float>(i);

	for(int j=mask_rec.x; j<mask_rec.x+mask_rec.width-1; j++)
	{
		if(Pi[j]-Ci[j]>0.75 && fabs(Pi[j]-last_x)<0.3) obs_px_count++;
		if(mask.at<bool>(j+1,i+1) && (Ci[j]>0.5 && Ci[j]<5.0))
		{
			px_total += Ci[j];
			mask_px_count++;
		}
	}
  }

  blob_details.depth = px_total/mask_px_count;
  if((double)obs_px_count/rec_area > 0.05) return 1;
  else return 0;
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
	cv_ptr = cv_bridge::toCvCopy(msg, enc::TYPE_32FC1);
  }
  catch(cv_bridge::Exception& e)
  {
	ROS_ERROR("Error obtaining depth image. %s", e.what());
	return;
  }
  di_crop = cv_ptr->image(diROI);

  if(click_recieved)
  {
	if(first_blob_recorded)
	{
		pub_bool.data = obstacle_check(di_crop,blob, last_unobs_di, pose_trans.transform.translation.x);
		pub.publish(pub_bool);
	}

	if(!pub_bool.data)
	{
		blob = cv::Mat::zeros(di_crop.rows+2,di_crop.cols+2, CV_8UC1); first_blob_recorded = 1;
		cv::floodFill(di_crop,blob, cv::Point(blob_details.x,blob_details.y), 255, NULL, 0.05,0.05, ff_flags); cv::imshow("show2", blob);

		blob_moments = cv::moments(blob, true);
		if(blob_moments.m00 > 0)
		{
			blob_details.x = blob_moments.m10/blob_moments.m00 -1;
			blob_details.y = blob_moments.m01/blob_moments.m00 -1;
		}

		pose_trans.transform.translation.x = blob_details.depth;
		y_rad = (320-(double)blob_details.x)/320 * half_view_angle_rads;
		pose_trans.transform.translation.y = pose_trans.transform.translation.x*tan(y_rad);
//ROS_INFO("Centroid pose %f, %f", pose_trans.transform.translation.x,pose_trans.transform.translation.y);
		last_unobs_di = di_crop.clone(); send_tf();
	}
  }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
	cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
	ROS_ERROR("Error obtaining RGB image. %s", e.what());
	return;
  }

  if(click_recieved && !pub_bool.data) cv::circle(cv_ptr->image, cv::Point(blob_details.x,blob_details.y), 4, CV_RGB(0,255,0), 2);
  else if(click_recieved && pub_bool.data) cv::circle(cv_ptr->image, cv::Point(blob_details.x,blob_details.y), 4, CV_RGB(255,0,0), 2);
  cv::imshow("show", cv_ptr->image);
}

void setup(void)
{
  pose_trans.header.frame_id = "camera"; pose_trans.child_frame_id = "kinect_target";
  pose_trans.transform.translation.z = 0;
  pose_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

  pub_bool.data = 0;
  blob_details.x = 320; blob_details.y = 240;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_img_listener");
  ros::NodeHandle nh;

  setup();

  cv::namedWindow("show"); cv::namedWindow("show2"); 
  cv::startWindowThread(); cv::setMouseCallback("show", mouse_cb, NULL);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber d_sub = it.subscribe("camera/depth/image_rect", 1, depthCallback);
  image_transport::Subscriber i_sub = it.subscribe("camera/rgb/image_raw", 1, imageCallback);
  pub = nh.advertise<std_msgs::Bool>("kinect_obs", 1);

  ros::Rate spin_rate(10);
  while(ros::ok())
  {
	ros::spinOnce();
	spin_rate.sleep();
  }
  cv::destroyWindow("show"); cv::destroyWindow("show2");
  return 0;
}
