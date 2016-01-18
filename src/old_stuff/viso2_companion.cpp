#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

btQuaternion quat; double r,p,y;
geometry_msgs::TransformStamped wc_pose;
nav_msgs::Odometry odom_out;
ros::Publisher pub;

void setup(void)
{
  wc_pose.header.frame_id = odom_out.header.frame_id = "odom";
  wc_pose.child_frame_id = odom_out.child_frame_id = "base_link";
  wc_pose.transform.translation.z = odom_out.pose.pose.position.z = 0;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  wc_pose.transform.translation.x = odom_out.pose.pose.position.x = msg->pose.position.z; //-msg->pose.position.y; if camera pointed to ground
  wc_pose.transform.translation.y = odom_out.pose.pose.position.y = -msg->pose.position.x;
  tf::quaternionMsgToTF(msg->pose.orientation, quat); btMatrix3x3(quat).getRPY(r,p,y);
  wc_pose.transform.rotation = odom_out.pose.pose.orientation = tf::createQuaternionMsgFromYaw(-p);

  wc_pose.header.stamp = odom_out.header.stamp = msg->header.stamp;
  static tf::TransformBroadcaster bcaster;
  bcaster.sendTransform(wc_pose); pub.publish(odom_out);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "viso2_tfer");
  ros::NodeHandle nh;

  setup();
  pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
  ros::Subscriber sub = nh.subscribe("viso/pose", 1, pose_cb);

  ros::spin();
  return 0;
}
