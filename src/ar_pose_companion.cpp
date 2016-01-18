#include <ros/ros.h>
#include <ar_pose/ARMarker.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

geometry_msgs::TransformStamped target_pose;
double r,p,y;
btQuaternion quat;

void setup(void)
{
  target_pose.header.frame_id = "camera";
  target_pose.child_frame_id = "ar_pose_target";
}

void pose_cb(const ar_pose::ARMarker::ConstPtr& msg)
{
  target_pose.transform.translation.x = msg->pose.pose.position.z;
  target_pose.transform.translation.y = -msg->pose.pose.position.x;
  target_pose.transform.translation.z = -msg->pose.pose.position.y;

  tf::quaternionMsgToTF(msg->pose.pose.orientation, quat); btMatrix3x3(quat).getRPY(r,p,y);
  target_pose.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(y,M_PI-r,-p);

  target_pose.header.stamp = ros::Time::now();
  static tf::TransformBroadcaster bcaster;
  bcaster.sendTransform(target_pose);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ar_tfer");
  ros::NodeHandle nh;
  setup();

  ros::Subscriber sub = nh.subscribe("ar_pose_marker", 1, pose_cb);
  ros::spin();
  return 0;
}
