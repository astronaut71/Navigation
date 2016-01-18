#include <ros/ros.h>
#include <opencv/cv.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//dimensions in m 
#define OBJECT_WIDTH 0.168
#define OBJECT_HEIGHT 0.168

int i; double rotation[3];
CvMat *object_points = cvCreateMat( 4, 3, CV_32FC1 ), *image_points = cvCreateMat( 4, 2, CV_32FC1 ),
*intrinsic_mat = cvCreateMat( 3, 3, CV_32FC1 ), *distort_coeffs = cvCreateMat( 1, 5, CV_32FC1 ),
*rotation_vec = cvCreateMat( 3, 1, CV_32FC1 ), *rotation_mat = cvCreateMat( 3, 3, CV_32FC1 ),
*translation_vec = cvCreateMat( 3, 1, CV_32FC1 );

geometry_msgs::TransformStamped pose_trans;

void send_tf(CvMat*& trans, double rot[])
{
  static tf::TransformBroadcaster br;

  pose_trans.transform.translation.x = cvmGet(trans, 2,0);
  pose_trans.transform.translation.y = -cvmGet(trans, 0,0);
  pose_trans.transform.translation.z = -cvmGet(trans, 1,0);
  pose_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(rot[0],rot[1],rot[2]);

  pose_trans.header.stamp = ros::Time::now();
  br.sendTransform(pose_trans);
}

void setup(void)
{
  float half_w = OBJECT_WIDTH/2, half_h = OBJECT_HEIGHT/2,
  obj_pts[4][3] = {-half_w, half_h, 0 , half_w, half_h, 0 , -half_w, -half_h, 0 , half_w, -half_h, 0},
  intrinsics[3][3] = {597.409771, 0, 315.293396, 0, 549.814336, 236.472741, 0, 0, 1},
  distort[1][5] = {0}; //{0.0329, -0.064376, 0.006359, 0.005131, 0}; 	

  memcpy(object_points->data.fl, obj_pts, sizeof(obj_pts));
  memcpy(intrinsic_mat->data.fl, intrinsics, sizeof(intrinsics));
  memcpy(distort_coeffs->data.fl, distort, sizeof(distort));

  pose_trans.header.frame_id = "camera";
  pose_trans.child_frame_id = "drec_target";
}

void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
{
  for(i=0; i<4; i++)
  {
	cvmSet(image_points, i,0, array->data[i]);
	cvmSet(image_points, i,1, array->data[i+4]);
  }
  cvZero(rotation_vec); cvZero(translation_vec); cvZero(rotation_mat);
  cvFindExtrinsicCameraParams2(object_points,image_points, intrinsic_mat,distort_coeffs, rotation_vec,translation_vec);
  cvRodrigues2(rotation_vec, rotation_mat, NULL);

  // [ roll pitch yaw ]
  rotation[0] = atan2(cvmGet(rotation_mat,1,0), cvmGet(rotation_mat,0,0));
  rotation[1] = M_PI-atan2(cvmGet(rotation_mat,2,1), cvmGet(rotation_mat,2,2));
  rotation[2] = atan2(cvmGet(rotation_mat,2,0), sqrt(pow(cvmGet(rotation_mat,2,1),2)+pow(cvmGet(rotation_mat,2,2),2)));

  send_tf(translation_vec, rotation);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "array_listener");
  ros::NodeHandle nh;
  setup();
  ros::Subscriber sub = nh.subscribe("corners", 1, arrayCallback);
  ros::spin();

  cvReleaseMat(&object_points); cvReleaseMat(&intrinsic_mat); cvReleaseMat(&distort_coeffs);
  cvReleaseMat(&image_points); cvReleaseMat(&translation_vec); cvReleaseMat(&rotation_mat); cvReleaseMat(&rotation_vec);
  return 0;
}
