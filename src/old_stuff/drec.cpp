#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <std_msgs/Int32MultiArray.h>

//image params
#define WIDTH 640
#define HEIGHT 480

//contour params
#define PROX_THRESH 50 //max range (px) around centrepoint from ar_pose to be considered
#define PROX_PERF 3 //max range (px) around centrepoint to be considered perfect match
sensor_msgs::CvBridge bridge;
ros::Publisher pub; std_msgs::Int32MultiArray corners_out;

//contour ranking
double score, score_temp;
int datum_x,datum_y, datum_w,datum_h;
CvRect rec, r_hold;

//contour finding
IplImage *gray = cvCreateImage(cvSize(WIDTH,HEIGHT), 8, 1), *pyrms = cvCreateImage(cvSize(WIDTH,HEIGHT), 8, 3),
*conts = cvCreateImage(cvSize(WIDTH,HEIGHT), 8, 1), *cont_img = cvCreateImage(cvSize(WIDTH,HEIGHT), 8, 1);
CvSeq *contour, *best_contour, *approx; CvMemStorage* storage = cvCreateMemStorage (0);

//good features to track
const int max = 4; int corner_count; double quality_level = 0.2, min_distance = 10;
CvPoint2D32f corners[max]; bool a_fnd,b_fnd,c_fnd,d_fnd;
IplImage *eigI = cvCreateImage(cvSize(WIDTH,HEIGHT), 32, 1), *tempI = cvCreateImage(cvSize(WIDTH,HEIGHT), 32, 1);

//others
int i, hy1,hy2, ddist;
double last_ar_time;

void node_init(void)
{
  corners_out.data.resize(8);
}

void sort_corners(void)
{
  a_fnd=b_fnd=c_fnd=d_fnd=0;
  for(i=0; i<max; i++)
  {
	if((int)corners[i].y<datum_y)
	{
		if((int)corners[i].x<datum_x)
		{
			corners_out.data[0] = (int)corners[i].x; corners_out.data[4] = (int)corners[i].y;
			a_fnd = 1;
		}
		else
		{
			corners_out.data[1] = (int)corners[i].x; corners_out.data[5] = (int)corners[i].y;
			b_fnd = 1;
		}
	}
	else
	{
		if((int)corners[i].x<datum_x)
		{
			corners_out.data[2] = (int)corners[i].x; corners_out.data[6] = (int)corners[i].y;
			c_fnd = 1;
		}
		else
		{
			corners_out.data[3] = (int)corners[i].x; corners_out.data[7] = (int)corners[i].y;
			d_fnd = 1;
		}
	}
  }
  //in order: tl,tr,bl,br. x's from 0-3, y's from 4-7
  if(a_fnd&&b_fnd&&c_fnd&&d_fnd) pub.publish(corners_out);
}

void update_datum_internal(CvRect& r)
{
  datum_x = r.x + r.width/2; datum_y = r.y + r.height/2;
  datum_w = r.width; datum_h = r.height;
}

int hypotenuse_to_datum(CvRect& r)
{
  hy1 = (r.x+r.width/2)-datum_x; hy2 = (r.y+r.height/2)-datum_y;
  return sqrt(hy1*hy1 + hy2*hy2);
}

double per_match(double one, double two)
{
  if(one>two) return two/one;
  else return one/two;
}

double calc_cont_score(CvRect& rec_in)
{
  ddist = hypotenuse_to_datum(rec_in);
  if(ddist>PROX_THRESH) return 0.0; else if(ddist<PROX_PERF) return 1.0;
  else return (per_match(rec_in.height,datum_h)+per_match(rec_in.width,datum_w))/2;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  IplImage *img_base = bridge.imgMsgToCv(msg, "bgr8");
  cvPyrMeanShiftFiltering(img_base,pyrms, 2,40, 3); cvCvtColor(pyrms, gray, CV_BGR2GRAY);
  cvAdaptiveThreshold(gray,conts, 255, CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY_INV, 101, 15);
  cvMorphologyEx(conts,conts, 0,0, CV_MOP_CLOSE,1); cvShowImage("View",conts);
 
  cvClearMemStorage(storage); contour = best_contour = approx = NULL; score = 0.75;
  cvFindContours(conts,storage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  while(contour != NULL)
  {
	rec = cvBoundingRect(contour, 0);
	score_temp = calc_cont_score(rec);
	if(score_temp>score)
	{
		score = score_temp;
		r_hold = rec;
		best_contour = contour;
	}
	if(score==1.0) break; else contour = contour->h_next;
  }

  if(best_contour != NULL)
  {
	if(ros::Time::now().toSec() - last_ar_time > 0.25) update_datum_internal(r_hold);

	approx = cvApproxPoly(best_contour, sizeof(CvContour), storage, CV_POLY_APPROX_DP, 5, 0);
	cvZero(cont_img); cvDrawContours(cont_img, approx, cvScalar(255),cvScalar(255),-1, -1);

	corner_count=max; memset(corners,0,sizeof(corners)); min_distance = (double)r_hold.width/3;
	cvGoodFeaturesToTrack(cont_img, eigI,tempI, corners,&corner_count, quality_level,min_distance); 
	sort_corners();

	for(i=0; i<max; i++) cvCircle(img_base, cvPoint((int)corners[i].x,(int)corners[i].y), 4,CV_RGB(0,255,0),2);
  }

  cvShowImage("Raw",img_base); //cvShowImage("View2",cont_img);
}

void ar_centreCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
{
  // co-ords of centre (x,y), width, height
  datum_x = array->data[0]; datum_y = array->data[1];
  datum_w = array->data[2]; datum_h = array->data[3];
  last_ar_time = ros::Time::now().toSec();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener"); 
  ros::NodeHandle nh;
  node_init();

  cvNamedWindow("Raw"); cvNamedWindow("View"); cvNamedWindow("View2");
  cvStartWindowThread();

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
  ros::Subscriber sub2 = nh.subscribe("ar_marker_image_pos", 1, ar_centreCallback); 
  pub = nh.advertise<std_msgs::Int32MultiArray>("corners", 1);

  ros::spin();
  cvDestroyWindow("Raw"); //cvDestroyWindow("View"); cvDestroyWindow("View2");
  cvReleaseImage(&pyrms); cvReleaseImage(&gray); cvReleaseImage(&conts); cvReleaseImage(&cont_img);
  cvReleaseImage(&eigI); cvReleaseImage(&tempI); cvReleaseMemStorage(&storage);
  return 0;
}
