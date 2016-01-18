#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <std_msgs/Int32MultiArray.h>

ros::Publisher pub;
sensor_msgs::CvBridge bridge;
std_msgs::Int32MultiArray corners_out;
IplImage* edges = cvCreateImage(cvSize(640,480), 8, 3);

const int max = 4;

int block_size = 10, i, corner_count = max; double quality_level = 0.25, min_distance = 10;
CvPoint2D32f corners[max]={0};

void init(void)
{
  corners_out.data.resize(8);
}

void send_array(void)
{
  /* 4 x co-ords followed by 4 y co-ords
  tl 0,4 tr 1,5 bl 2,6 br 3,7 */
  /*copy(id_corners,id_corners+8,corners_out.data.begin());	
  pub.publish(corners_out);

  cvCircle(edges, cvPoint(id_corners[0],id_corners[4]), 3, CV_RGB(255,0,0), CV_FILLED);
  cvCircle(edges, cvPoint(id_corners[1],id_corners[5]), 3, CV_RGB(0,255,0), CV_FILLED);
  cvCircle(edges, cvPoint(id_corners[2],id_corners[6]), 3, CV_RGB(0,0,255), CV_FILLED);
  cvCircle(edges, cvPoint(id_corners[3],id_corners[7]), 3, CV_RGB(255,255,255), CV_FILLED);*/
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cvZero(edges);
  IplImage *img_base = bridge.imgMsgToCv(msg, "mono8"),
  *eigI = cvCreateImage(cvSize(640,480), 32, 1),
  *tempI = cvCreateImage(cvSize(640,480), 32, 1);

  cvGoodFeaturesToTrack(img_base, eigI,tempI, corners,&corner_count, quality_level,min_distance);

  for(i=0; i<max; i++)
  {
	cvCircle(img_base, cvPoint((int)corners[i].x,(int)corners[i].y), 4,cvScalar(255),1);
  }

  //send_array();
  cvShowImage("Lines",img_base);
  cvReleaseImage(&eigI); cvReleaseImage(&tempI);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  init();
  cvNamedWindow("Lines");
  cvStartWindowThread();
  image_transport::ImageTransport it(nh); 
  image_transport::Subscriber sub = it.subscribe("segmented", 1, imageCallback);  
  pub = nh.advertise<std_msgs::Int32MultiArray>("corners", 1);
  ros::spin();
  cvDestroyWindow("Lines");
}
