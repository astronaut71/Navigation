#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <std_msgs/Int32MultiArray.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>

#define PX_DIST_THRESH 50

int i,j, trackbar1_val,trackbar2_val;
CvKalman* kalman = cvCreateKalman(16,8, 0);
bool kalman_active=0, fault, good_data_set;
double dx_sq,dy_sq;

//IplImage *base = cvCreateImage(cvSize(1280,960), 8,3);
CvMat *last_meas = cvCreateMat( 8, 1, CV_32FC1 ), *last_post = cvCreateMat( 8, 1, CV_32FC1 );

ros::Publisher pub;
std_msgs::Int32MultiArray filter_out;

void main_init(void)
{
  filter_out.data.resize(8);
}

void send_array(void)
{
  copy(kalman->state_post->data.fl, kalman->state_post->data.fl+8, filter_out.data.begin());
  pub.publish(filter_out);  
}

void kalman_init(CvArr* z_k)
{
  //intialize state matrix, and measurement(H)/transition(F) matrices
  float H[8][16] = {0}, F[16][16] = {0}, dt = 1;

  for(i=0; i<8; i++)
  {
	cvmSet(kalman->state_post, i,0, cvmGet((CvMat*)z_k, i,0));
	cvmSet(kalman->state_post, i+8,0, 0);	

	H[i][i] = 1;

	F[i][i] = 1; F[i+8][i+8] = 1;
	F[i][i+8] = dt;
  }
  memcpy( kalman->measurement_matrix->data.fl,H, sizeof(H));
  memcpy( kalman->transition_matrix->data.fl,F, sizeof(F));

  //set other kalman parameters
  cvSetIdentity( kalman->process_noise_cov, cvRealScalar(trackbar1_val) ); 
  cvSetIdentity( kalman->measurement_noise_cov, cvRealScalar(255) );
}

bool mat_compare(CvArr* newer, CvArr* older)
{
  fault=0;
  for(i=0; i<4; i++)
  {
	dx_sq = pow(cvmGet((CvMat*)newer,i,0)-cvmGet((CvMat*)older,i,0), 2),
	dy_sq = pow(cvmGet((CvMat*)newer,i+4,0)-cvmGet((CvMat*)older,i+4,0), 2);	
  	if(sqrt(dx_sq+dy_sq) > PX_DIST_THRESH)
  	{
		fault = 1;
		break;	
 	}
  }

  return !fault;
}

bool kalman_correct(CvArr* z_k)
{
  good_data_set=1;
  cvKalmanPredict( kalman, 0 );

/*ROS_INFO("%f %f %f %f %f %f %f %f", cvmGet(kalman->state_post,8,0),cvmGet(kalman->state_post,9,0),cvmGet(kalman->state_post,10,0),
cvmGet(kalman->state_post,11,0),cvmGet(kalman->state_post,12,0),cvmGet(kalman->state_post,13,0),
cvmGet(kalman->state_post,14,0),cvmGet(kalman->state_post,15,0));*/

  if(mat_compare(z_k, kalman->state_pre)) cvKalmanCorrect( kalman, (CvMat*)z_k );
  else if(mat_compare(z_k, last_meas)) kalman_init(z_k);
  else 
  {
	//cvZero(base);
	ROS_ERROR("Unstable");
	good_data_set=0;
  }
  return good_data_set;
}

void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
{
  CvMat* arr_base = cvCreateMat( 8, 1, CV_32FC1 );
  for(i=0; i<8; i++) cvmSet(arr_base, i,0, array->data[i]);

  if(cvWaitKey(50) != -1)
  {
	kalman_init(arr_base);
 	kalman_active = 1;

	//last_meas = cvCloneMat(arr_base);
	//cvZero(base);
	ROS_INFO("Kalman reset");
  }

  if(kalman_active) 
  {
	last_post = cvCloneMat(kalman->state_post);	
	if(kalman_correct(arr_base)) send_array();
  }

  /*IplImage* kalman_rect = cvCreateImage(cvSize(1280,960), 8,3);
  for(i=0; i<4; i++)
  {
	//lines from point 0 to 1 and 2, and from 3 to 1 and 2 form rectangle
	int j=0, k=1;
	if(i>1) j=3; if(i==1 || i==3) k=2;

	//resultant rect (green)
	cvLine(kalman_rect, cvPoint((int)cvmGet(kalman->state_post, j,0),(int)cvmGet(kalman->state_post, j+4,0)),
	cvPoint((int)cvmGet(kalman->state_post, k,0),(int)cvmGet(kalman->state_post, k+4,0)), CV_RGB(0,255,0),1);

	//meas (red), kalman (green) lines
	cvLine(base, cvPoint((int)cvmGet(last_meas,i,0),(int)cvmGet(last_meas,i+4,0)),
	cvPoint((int)cvmGet(arr_base,i,0),(int)cvmGet(arr_base,i+4,0)), CV_RGB(255,0,0),1);
	cvLine(base, cvPoint((int)cvmGet(last_post,i,0),(int)cvmGet(last_post,i+4,0)),
	cvPoint((int)cvmGet(kalman->state_post,i,0),(int)cvmGet(kalman->state_post,i+4,0)), CV_RGB(0,255,0),2);
  }

  cvAdd(base,kalman_rect,kalman_rect); cvShowImage("Filtering", kalman_rect); cvReleaseImage(&kalman_rect);*/
  last_meas = cvCloneMat(arr_base); cvReleaseMat(&arr_base);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "array_listener"); 
  ros::NodeHandle nh;   image_transport::ImageTransport it(nh);
  main_init();
  cvNamedWindow("Filtering");
  cvCreateTrackbar( "Proc cov", "Filtering", &trackbar1_val, 50, NULL );
  //cvCreateTrackbar( "Meas cov", "Filtering", &trackbar2_val, 255, NULL );
  cvStartWindowThread();
  ros::Subscriber sub = nh.subscribe("corners", 1, arrayCallback);
  pub = nh.advertise<std_msgs::Int32MultiArray>("filter_out", 1);
  ros::spin();
  cvDestroyWindow("Filtering");
}
