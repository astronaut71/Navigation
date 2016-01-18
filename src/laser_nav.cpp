#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Bool.h>

//in m wrt laser
#define START_X	1.5
#define START_Y 0.3

#define within_p(a,b) (fabs(a-b) < 0.1)
#define hypotenuse(dx,dy) sqrt(dx*dx + dy*dy)
#define fix_ang(i) \
	if(i>M_PI) i-=2*M_PI; \
	else if(i<-M_PI) i+=2*M_PI;

double left_ang,right_ang, temp_x,temp_y, lx,ly,rx,ry, diffx,diffy;
double smallest, dist_to_datum;
int count, chain[50][2], point_counter, i, closest; bool chain_started;
geometry_msgs::TransformStamped target_real;

//obstruction stuff
bool initial_datum_set = 0, target_chain_obs = 0, target_chain_part_obs = 0, leader_vanished = 0;
int last_datum_l,last_datum_r, ol_l,ol_r, closest_obs, second_closest;
ros::Publisher pub; std_msgs::Bool pub_bool;
double close_chains[20][3]; int close_chains_count, close_chain_points[20][2];
double last_datum_ln, obs_x,obs_y, second_smallest;

//avoidance stuff
	//gap from laser
double left_bound_x,left_bound_y, right_bound_x,right_bound_y;
geometry_msgs::TransformStamped target_point;

//----------------------------------------------------------------
	//things in these dotted lines are for vis purposes only
ros::Publisher scan_pub, scan_pub2;
sensor_msgs::LaserScan scanout, leaderless;
//----------------------------------------------------------------

void scan_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  //extract 'chains' of scan points
  count = chain_started = 0; memset(chain, 0, sizeof chain);
  for(i=0; i<scan->ranges.size(); i++)
  {
	if(within_p(scan->ranges[i+1],scan->ranges[i]) && !chain_started && scan->ranges[i]!=0)
	{
		point_counter = chain_started = 1;
		chain[count][0] = i;
	}
	else if(chain_started && !within_p(scan->ranges[i+1],scan->ranges[i]))
	{
		chain_started = 0;
		if(point_counter >= 15)
		{
			chain[count][1] = i;
			count++;
		}
	}
	else if(chain_started && within_p(scan->ranges[i+1],scan->ranges[i])) point_counter++;
  }

  //find chain most likely to match target chain from previous scan, get x/y co-ordinates
  closest=second_closest = count; close_chains_count = 0;
  if(!pub_bool.data && initial_datum_set) smallest = 1.0; else smallest = 2.0;
  second_smallest = 4.0;
  for(i=0; i<count; i++)
  {
	left_ang = (chain[i][1] - 0.5*scan->ranges.size()) * scan->angle_increment;
	right_ang = (chain[i][0] - 0.5*scan->ranges.size()) * scan->angle_increment;
	lx = scan->ranges[chain[i][1]]*cos(left_ang); rx = scan->ranges[chain[i][0]]*cos(right_ang);
	ly = scan->ranges[chain[i][1]]*sin(left_ang); ry = scan->ranges[chain[i][0]]*sin(right_ang);
	temp_x = (lx+rx)/2; temp_y = (ly+ry)/2;

	diffx = target_real.transform.translation.x-temp_x; diffy = target_real.transform.translation.y-temp_y;
	dist_to_datum = hypotenuse(diffx,diffy);

	if(dist_to_datum<4.0)
	{
		close_chains[close_chains_count][0] = temp_x; close_chains[close_chains_count][1] = temp_y;
		close_chains[close_chains_count][2] = atan2(rx-lx,ly-ry); fix_ang(close_chains[close_chains_count][2]);
		close_chain_points[close_chains_count][0] = chain[i][0]; close_chain_points[close_chains_count][1] = chain[i][1];

		if(dist_to_datum<smallest)
		{
			smallest = dist_to_datum; closest = close_chains_count;
		}
		else if(dist_to_datum<second_smallest)
		{
			second_smallest = dist_to_datum; second_closest = close_chains_count;
		}
		close_chains_count++;
	}
  }

  if(!initial_datum_set)
  {
	last_datum_l = close_chain_points[closest][1]; last_datum_r = close_chain_points[closest][0];
	initial_datum_set = 1;
  }

  //check if leader has suddenly vanished due to obstruction or spiderman
  if(closest>close_chains_count && !pub_bool.data)
  {
	if(second_closest<=close_chains_count)
	{
		closest_obs = second_closest;
		target_chain_obs = 1;
	}
	else leader_vanished = 1;
  }
  else leader_vanished = 0;

  //if previously obstructed, track obstructing chain
  if(pub_bool.data && closest<=close_chains_count)
  {
	smallest = 1.0;
	for(i=0; i<close_chains_count; i++)
	{
		diffx = close_chains[i][0]-obs_x; diffy = close_chains[i][1]-obs_y;
		dist_to_datum = hypotenuse(diffx,diffy);

		if(dist_to_datum<smallest)
		{
			smallest = dist_to_datum; closest_obs = i;
		}
	}
	if(closest==closest_obs) target_chain_obs = 1;
	else target_chain_obs = 0;
  }

  //check for partial obstruction
  ol_l=ol_r = 0; last_datum_ln = (double)(last_datum_l-last_datum_r);
  if(!target_chain_obs && closest<close_chains_count-1)
  {
	for(i=close_chain_points[closest+1][0]; i<last_datum_l; i++)
	{
		if(scan->ranges[i]>0.5 && target_real.transform.translation.x-scan->ranges[i] > 0.3) ol_l++;
	}
  }
  if(!target_chain_obs && closest>0 && close_chains_count>closest)
  {
	for(i=close_chain_points[closest-1][1]; i>last_datum_r; i--)
	{
		if(scan->ranges[i]>0.5 && target_real.transform.translation.x-scan->ranges[i] > 0.3) ol_r++;
	}
  }

  target_chain_part_obs = 1;
  if(ol_l/last_datum_ln > 0.05) closest_obs = closest+1;
  else if(ol_r/last_datum_ln > 0.05) closest_obs = closest-1;
  else target_chain_part_obs = 0;

  //publish tf if unobstructed
  if(!target_chain_obs && !target_chain_part_obs && !leader_vanished)
  {
	last_datum_l = close_chain_points[closest][1]; last_datum_r = close_chain_points[closest][0];

	target_real.header.stamp = ros::Time::now();
	target_real.transform.translation.x = close_chains[closest][0];
	target_real.transform.translation.y = close_chains[closest][1];
	target_real.transform.rotation = tf::createQuaternionMsgFromYaw(close_chains[closest][2]);

	static tf::TransformBroadcaster broadcaster;
	broadcaster.sendTransform(target_real);
	pub_bool.data = 0;

	//gap point placement
	//need to find points which have potential to be collision
	//left_bound_x = ; left_bound_y = ;
	//right_bound_x = ; right_bound_y = ;
	//if //points are close, then it's a door. need to handle differently

//-------------------------------------------------------------------
  scanout.header = leaderless.header = target_real.header;
  scanout.time_increment = leaderless.time_increment = scan->time_increment;
  scanout.angle_min = leaderless.angle_min = scan->angle_min;
  scanout.angle_max = leaderless.angle_max = scan->angle_max;
  scanout.angle_increment = leaderless.angle_increment = scan->angle_increment;
  scanout.time_increment = leaderless.time_increment = scan->time_increment;
  scanout.scan_time = leaderless.scan_time = scan->scan_time;
  scanout.range_min = leaderless.range_min = scan->range_min;
  scanout.range_max = leaderless.range_max = scan->range_max;

  for(i = 0; i < scan->ranges.size(); i++)
  {
	scanout.ranges[i] = 0;
	leaderless.ranges[i] = scan->ranges[i];
  }
  for(i=last_datum_r; i < last_datum_l; i++)
  {
	scanout.ranges[i] = scan->ranges[i];
	leaderless.ranges[i] = 0;
  }
  scan_pub.publish(scanout);
//  leaderless.header.stamp = ros::Time::now(); leaderless.header.frame_id = "laser";
 scan_pub2.publish(leaderless);
//----------------------------------------------------------------
  }
  else
  {
	if(!leader_vanished)
	{
		obs_x = close_chains[closest_obs][0]; obs_y = close_chains[closest_obs][1];
	}
	pub_bool.data = 1;
  }
  pub.publish(pub_bool);
}

void setup(void)
{
  target_real.header.frame_id = "laser"; target_real.child_frame_id = "laser_target";
  target_real.transform.translation.x = START_X; target_real.transform.translation.y = START_Y;
  target_real.transform.translation.z = 0; target_real.transform.rotation = tf::createQuaternionMsgFromYaw(0);

  target_point.header.frame_id = "laser"; target_point.child_frame_id = "laser_target";
  target_point.transform.translation.z = 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lazorzzz"); 
  ros::NodeHandle nh;

  setup();

  ros::Subscriber sub = nh.subscribe("scan_throttle", 1, scan_cb);
  pub = nh.advertise<std_msgs::Bool>("laser_obs", 1);

//----------------------------------------------------------------
  scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan2", 1);
  scan_pub2 = nh.advertise<sensor_msgs::LaserScan>("scan3", 1);
  scanout.ranges.resize(800); leaderless.ranges.resize(800);
//----------------------------------------------------------------

  ros::Rate spin_rate(10);
  //ros::spin(); 
  while(ros::ok())
  {
	ros::spinOnce();
	//spin_rate.sleep();
  }
  return 0;
}
