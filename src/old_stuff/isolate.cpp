#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <std_msgs/Int32.h>

sensor_msgs::CvBridge bridge;
image_transport::Publisher pub;
std_msgs::Int32 exp_out; ros::Publisher pub1;

int h_bins = 30, s_bins = 30, hist_size[] = { h_bins,s_bins }, max_idx; 
float h_ranges[] = { 0,255 }, s_ranges[] = { 0,255 },
*ranges[] = { h_ranges,s_ranges }, *ranges2[] = { h_ranges }, last = 12; 
CvHistogram *hist = cvCreateHist(2, hist_size, CV_HIST_ARRAY, ranges, 1),
*roi_hist = cvCreateHist(1, &h_bins, CV_HIST_ARRAY, ranges2, 1);

CvPoint pt1;
bool hist_made=0, currently_drawing=0, make_hist=0;
IplImage* hist_mask = cvCreateImage(cvSize(1280,960), 8, 1);

int get_ideal_exposure(IplImage* v, IplImage* con)
{
  CvRect bound = cvBoundingRect(con, 0);
  cvFloodFill(con, cvPoint(bound.x+bound.width/2,bound.y+bound.height/2), cvScalar(255));

  cvCalcHist(&v, roi_hist, 0, con);
  cvGetMinMaxHistValue(roi_hist, NULL,NULL, NULL,&max_idx);

  last = (last + max_idx)/2;

  return last*10 + 1;
}

void mouseHandler(int event, int x, int y, int flags, void *param)
{
  switch(event)
  {
	case CV_EVENT_LBUTTONDOWN:
	{
		if(!hist_made)
		{
			currently_drawing = 1;
			pt1 = cvPoint(x,y);
		}
		break;
	}
	case CV_EVENT_LBUTTONUP:
	{
		currently_drawing = 0;
		cvRectangle(hist_mask, pt1,cvPoint(x,y), cvScalar(255),CV_FILLED);
		make_hist=1;
		break;
	}
	case CV_EVENT_RBUTTONDOWN:
	{
		if(hist_made)
		{
			cvClearHist(hist);
			ROS_INFO("Histogram cleared");
			hist_made=0;
		}
		break;
	}
	default: break;
  }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  IplImage* img_base = bridge.imgMsgToCv(msg, "bgr8");
  int w = img_base->width, h = img_base->height;

  IplImage *imgHSV = cvCreateImage(cvSize(w,h), 8, 3),
  *hue = cvCreateImage(cvSize(w,h), 8, 1),  
  *sat = cvCreateImage(cvSize(w,h), 8, 1),
  *val = cvCreateImage(cvSize(w,h), 8, 1),
  *planes[] = { hue,sat };

  //extract hue/sat maps from imgHSV, press 'h' to store a histogram, 'c' to empty it
  cvCvtColor(img_base, imgHSV, CV_BGR2HSV);  
  cvSplit(imgHSV, hue,sat,val,NULL); cvReleaseImage(&imgHSV);

  if(make_hist)
  {
	cvCalcHist(planes, hist, 0, hist_mask);
	hist_made=1; make_hist=0; cvZero(hist_mask);
	ROS_INFO("Histogram recorded");
  }

  //now that the histogram is filled, find backprojection of histogram on hue
  IplImage *backproject = cvCreateImage(cvSize(w,h), 8,1),
  *cont = cvCreateImage(cvSize(w,h), 8,1);

  if(hist_made)
  {
 	cvCalcBackProject(planes, backproject, hist);

	CvSeq *contour = NULL, *biggest_contour = NULL;
	CvMemStorage* storage = cvCreateMemStorage (0); 
	cvFindContours( backproject,storage, &contour,sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );

	double max_h_area = 0.0;
	while( contour != NULL )
	{
		double contour_area = fabs(cvContourArea( contour));	
		if (contour_area > max_h_area)
		{
			max_h_area = contour_area;
			biggest_contour = contour;
		}
		contour = contour->h_next;
	}
	cvZero(cont); cvDrawContours(cont, biggest_contour, cvScalar(255),cvScalar(255), -1,2);
	cvReleaseMemStorage(&storage);

	pub.publish(bridge.cvToImgMsg(cont, "mono8"));
	exp_out.data = get_ideal_exposure(val, cont); pub1.publish(exp_out);
  }
  else cvShowImage("Raw",img_base);
  cvReleaseImage(&backproject); cvReleaseImage(&cont);
  cvReleaseImage(&hue); cvReleaseImage(&sat); cvReleaseImage(&val);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener"); 
  ros::NodeHandle nh; 
  cvNamedWindow("Raw");
  cvStartWindowThread();
  cvSetMouseCallback( "Raw", mouseHandler, NULL );
  image_transport::ImageTransport it(nh); 
  image_transport::Subscriber sub = it.subscribe("usb_camera/image_raw", 1, imageCallback);  
  pub = it.advertise("segmented", 1);
  pub1 = nh.advertise<std_msgs::Int32>("usb_camera/exp_roi", 1);
  ros::spin();
  cvDestroyWindow("Raw");
}
