#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf/transform_datatypes.h"
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/LaserScan.h"

bool keepMoving1=true; // Indicates whether the robot should continue moving
bool keepMoving2=true; // Indicates whether the robot should continue moving
bool keepMoving3=true; // Indicates whether the robot should continue moving
float ob_dir1=0.0,ob_dir2=0.0,ob_dir3=0.0;
float ob_len1=0.0,ob_len2=0.0,ob_len3=0.0;
const static double MIN_SCAN_ANGLE = -30.0/180*M_PI;
const static double MAX_SCAN_ANGLE = +30.0/180*M_PI;
const static float MIN_DIST_FROM_OBSTACLE = 0.8;
const static float TURTLE_BOT_R=0.4;
int time_num1=0,time_num2=0;
/*void scanCallback1(const sensor_msgs::LaserScan::ConstPtr& scan)
{

  bool isObstacleInFront_L = false;//障碍标志
  bool isObstacleInFront_R = false;//障碍标志
  bool isObstacleInFront_M = false;//障碍标志
    // Find the closest range between the defined minimum and maximum angles
    int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int middleIndex=(minIndex+maxIndex)/2;
    bool start=false;
    int obsIndexL=0,obsIndexR=0;
    float obld=0,obrd=0,obmd=0;
    int obsIndex_num=0;
    //扫描障碍范围
    for (int currIndex = minIndex + 1; currIndex <= (maxIndex-1); currIndex++)
      {
	if (scan->ranges[currIndex-1]>MIN_DIST_FROM_OBSTACLE&&scan->ranges[currIndex]<MIN_DIST_FROM_OBSTACLE)
	  {
	    start=true;
	    obsIndexL=currIndex;
	  }
	if(start)
	  {
	    if(scan->ranges[currIndex]<MIN_DIST_FROM_OBSTACLE)
	      {
		obsIndex_num++;
	      }
	    if (scan->ranges[currIndex-1]<MIN_DIST_FROM_OBSTACLE&&scan->ranges[currIndex]>MIN_DIST_FROM_OBSTACLE)
	      {
		obsIndexR=currIndex;
		break;
	      }
	  }
      }
    if(obsIndex_num>5)//障碍存在
      {
	if(obsIndexL>middleIndex)
	  {
	    obld=tan((obsIndexL-middleIndex)* scan->angle_increment)*MIN_DIST_FROM_OBSTACLE;
	    if(obld<TURTLE_BOT_R)
	      {
		isObstacleInFront_R=true;
		ob_len1=TURTLE_BOT_R+obld;
	      }
	  }
	else if(obsIndexR<middleIndex)
	  {
	    obrd=tan((middleIndex-obsIndexR)* scan->angle_increment)*MIN_DIST_FROM_OBSTACLE;
	    if(obrd<TURTLE_BOT_R)
	      {
		isObstacleInFront_L=true;
		ob_len1=TURTLE_BOT_R+obrd;
	      }
	  }
	else
	  {
	    	isObstacleInFront_M=true;
		obmd=(middleIndex-obsIndexR)<(obsIndexL-middleIndex)?(middleIndex-obsIndexR):(obsIndexL-middleIndex);
		obmd=tan(obmd*scan->angle_increment)*MIN_DIST_FROM_OBSTACLE;
		ob_len1=TURTLE_BOT_R+obmd;
	  }
      }
    if(isObstacleInFront_L)
      {
	ob_dir1=-M_PI/2.0;
      }
    else if(isObstacleInFront_R)
      {
	ob_dir1=M_PI/2.0;
      }
    else if(isObstacleInFront_M)
      {
	if(obmd==(middleIndex-obsIndexR))
	  {
	    ob_dir1=-M_PI/2.0;
	  }
	else
	  {
	    ob_dir1=M_PI/2.0;
	  }
      }

    if (isObstacleInFront_L||isObstacleInFront_R||isObstacleInFront_M)
      {
	// ROS_INFO("robot1 finds Obstacle!");
	keepMoving1 = false;
      }
    else
      {
	keepMoving1 = true;
      }
}

void scanCallback2(const sensor_msgs::LaserScan::ConstPtr& scan)
{

  bool isObstacleInFront_L = false;//障碍标志
  bool isObstacleInFront_R = false;//障碍标志
  bool isObstacleInFront_M = false;//障碍标志
    // Find the closest range between the defined minimum and maximum angles
    int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int middleIndex=(minIndex+maxIndex)/2;
    bool start=false;
    int obsIndexL=0,obsIndexR=0;
    float obld=0,obrd=0,obmd=0;
    int obsIndex_num=0;
    //扫描障碍范围
    for (int currIndex = minIndex + 1; currIndex <= (maxIndex-1); currIndex++)
      {

	if(scan->ranges[currIndex]<MIN_DIST_FROM_OBSTACLE)
	  {
	    start=true;
	    obsIndex_num++;
	  }
	if(start)
	  {
	    if (scan->ranges[currIndex-1]>MIN_DIST_FROM_OBSTACLE&&scan->ranges[currIndex]<MIN_DIST_FROM_OBSTACLE)
	      {
		obsIndexL=currIndex;
	      }
	    if (scan->ranges[currIndex-1]<MIN_DIST_FROM_OBSTACLE&&scan->ranges[currIndex]>MIN_DIST_FROM_OBSTACLE)
	      {
		obsIndexR=currIndex;
		break;
	      }
	  }

      }
    if(obsIndexL==0&&start)
      {
	obsIndexL=minIndex+1;
      }
    if(obsIndexR==0&&start)
      {
	obsIndexR=maxIndex-1;
      }
    if(start&&obsIndex_num>5)//障碍存在
      {
	if(obsIndexL>middleIndex)
	  {
	    obld=tan((obsIndexL-middleIndex)* scan->angle_increment)*MIN_DIST_FROM_OBSTACLE;
	    if(obld<TURTLE_BOT_R)
	      {
		isObstacleInFront_R=true;
		ob_len2=(obsIndexL-middleIndex)* scan->angle_increment;
	      }
	  }
	else if(obsIndexR<middleIndex)
	  {
	    obrd=tan((middleIndex-obsIndexR)* scan->angle_increment)*MIN_DIST_FROM_OBSTACLE;
	    if(obrd<TURTLE_BOT_R)
	      {
		isObstacleInFront_L=true;
		ob_len2=(middleIndex-obsIndexR)* scan->angle_increment;
	      }
	  }
	else
	  {
	    	isObstacleInFront_M=true;
		obmd=(middleIndex-obsIndexL)<(obsIndexR-middleIndex)?(middleIndex-obsIndexL):(obsIndexR-middleIndex);
		obmd=tan(obmd*scan->angle_increment)*MIN_DIST_FROM_OBSTACLE;
		ob_len2=M_PI/2.0;
	  }
      }
    if(isObstacleInFront_L)
      {
	ob_dir2=-M_PI/2.0;
      }
    else if(isObstacleInFront_R)
      {
	ob_dir2=M_PI/2.0;
      }
    else if(isObstacleInFront_M)
      {
	if(obmd==(middleIndex-obsIndexR))
	  {
	    ob_dir2=-M_PI/2.0;
	  }
	else
	  {
	    ob_dir2=M_PI/2.0;
	  }
      }

    if (isObstacleInFront_L||isObstacleInFront_R||isObstacleInFront_M)
      {
	ROS_INFO("robot2 Obstacle:L:%f A:%f !",ob_len2,ob_dir2);
	// ROS_INFO("robot1 finds Obstacle!");
	keepMoving2 = false;
      }
    else
      {
	keepMoving2 = true;
      }
}

void scanCallback3(const sensor_msgs::LaserScan::ConstPtr& scan)
{

  bool isObstacleInFront_L = false;//障碍标志
  bool isObstacleInFront_R = false;//障碍标志
  bool isObstacleInFront_M = false;//障碍标志
    // Find the closest range between the defined minimum and maximum angles
    int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int middleIndex=(minIndex+maxIndex)/2;
    bool start=false;
    int obsIndexL=0,obsIndexR=0;
    float obld=0,obrd=0,obmd=0;
    int obsIndex_num=0;
    //扫描障碍范围
    for (int currIndex = minIndex + 1; currIndex <= (maxIndex-1); currIndex++)
      {

	if(scan->ranges[currIndex]<MIN_DIST_FROM_OBSTACLE)
	  {
	    start=true;
	    obsIndex_num++;
	  }
	if(start)
	  {
	    if (scan->ranges[currIndex-1]>MIN_DIST_FROM_OBSTACLE&&scan->ranges[currIndex]<MIN_DIST_FROM_OBSTACLE)
	      {
	
		obsIndexL=currIndex;
	      }
 
	    if (scan->ranges[currIndex-1]<MIN_DIST_FROM_OBSTACLE&&scan->ranges[currIndex]>MIN_DIST_FROM_OBSTACLE)
	      {
		obsIndexR=currIndex;
		break;
	      }
	  }
	 
      }
    if(obsIndexL==0&&start)
      {
	obsIndexL=minIndex+1;
      }
    if(obsIndexR==0&&start)
      {
	obsIndexR=maxIndex-1;
      }
    if(obsIndex_num>5)//障碍存在
      {
	if(obsIndexL>middleIndex)
	  {
	    obld=tan((obsIndexL-middleIndex)* scan->angle_increment)*MIN_DIST_FROM_OBSTACLE;
	    if(obld<TURTLE_BOT_R)
	      {
		isObstacleInFront_R=true;
		ob_len3=(obsIndexL-middleIndex)* scan->angle_increment;
	      }
	  }
	else if(obsIndexR<middleIndex)
	  {
	    obrd=tan((middleIndex-obsIndexR)* scan->angle_increment)*MIN_DIST_FROM_OBSTACLE;
	    if(obrd<TURTLE_BOT_R)
	      {
		isObstacleInFront_L=true;
		ob_len3=(middleIndex-obsIndexR)* scan->angle_increment;
	      }
	  }
	else
	  {
	    	isObstacleInFront_M=true;
		obmd=(middleIndex-obsIndexR)<(obsIndexL-middleIndex)?(middleIndex-obsIndexR):(obsIndexL-middleIndex);
		obmd=tan(obmd*scan->angle_increment)*MIN_DIST_FROM_OBSTACLE;
		ob_len3=M_PI/2.0;
	  }
      }
    if(isObstacleInFront_L)
      {
	ob_dir3=-M_PI/2.0;
      }
    else if(isObstacleInFront_R)
      {
	ob_dir3=M_PI/2.0;
      }
    else if(isObstacleInFront_M)
      {

	if(obmd==(middleIndex-obsIndexR))
	  {
	    ob_dir3=-M_PI/2.0;
	  }
	else
	  {
	    ob_dir3=M_PI/2.0;
	  }
      }

    if (isObstacleInFront_L||isObstacleInFront_R||isObstacleInFront_M)
      {
	ROS_INFO("robot3 Obstacle:L:%f A:%f !",ob_len3,ob_dir3);
	// ROS_INFO("robot1 finds Obstacle!");
	keepMoving3 = false;
      }
    else
      {
	keepMoving3 = true;
      }
}
*/
void scanCallback2(const sensor_msgs::LaserScan::ConstPtr& scan)
{

  bool isObstacleInFront = false;//障碍标志
    // Find the closest range between the defined minimum and maximum angles
    int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int middleIndex=(minIndex+maxIndex)/2;
    for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
        if (scan->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE) {
        	isObstacleInFront = true;
		if(isObstacleInFront)
		  {
		    if((currIndex-middleIndex)>=0)
		      {
			ob_dir2=-1.0;
		      }
		    else
		      {
			ob_dir2=1.0;
		      }
		  }
            break;
        }
    }
    if (isObstacleInFront) {
      // ROS_INFO("robot2 finds Obstacle!");
      keepMoving2 = false;
      
    }
    else
      {
	keepMoving2 = true;
      }
}
void scanCallback3(const sensor_msgs::LaserScan::ConstPtr& scan)
{

  bool isObstacleInFront = false;//障碍标志
    // Find the closest range between the defined minimum and maximum angles
    int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int middleIndex=(minIndex+maxIndex)/2;
    for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
        if (scan->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE) {
        	isObstacleInFront = true;
		if(isObstacleInFront)
		  {
		    if((currIndex-middleIndex)>=0)
		      {
			ob_dir3=-1.0;
		      }
		    else
		      {
			ob_dir3=1.0;
		      }
		  }
            break;
        }
    }
    if (isObstacleInFront) {
      // ROS_INFO("robot3 finds Obstacle!");
      keepMoving3 = false;
    }
    else
      {
	keepMoving3 = true;
      }
      }


int main(int argc, char** argv){
  ros::init(argc, argv, "myturtlebot_location");

  ros::NodeHandle node;

  ros::Publisher turtlebot_vel2 =
    node.advertise<geometry_msgs::Twist>("/robot2/mobile_base/commands/velocity", 10);
  ros::Publisher turtlebot_vel3 =
    node.advertise<geometry_msgs::Twist>("/robot3/mobile_base/commands/velocity", 10);
  ros::Subscriber laserSub2= node.subscribe("robot2/scan", 1, scanCallback2);
  ros::Subscriber laserSub3= node.subscribe("robot3/scan", 1, scanCallback3);
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);

  geometry_msgs::TransformStamped transformStamped2;
  geometry_msgs::TransformStamped transformStamped3;
  geometry_msgs::TransformStamped transformStamped4;
  geometry_msgs::TransformStamped transformStamped5;
  transformStamped2.transform.translation.x=1.0;
  transformStamped2.transform.translation.y=1.0;
  transformStamped3.transform.translation.x=1.0;
  transformStamped3.transform.translation.y=1.0;
  geometry_msgs::Twist vel_msg2;
  geometry_msgs::Twist vel_msg3;
  bool avoid_succeed2=true;
  bool avoid_succeed3=true;
  //float obstacle_Agoal2=0,obstacle_Agoal3=0,obstacle_Lgoal2=0,obstacle_Lgoal3=0;
  //int avoid_num2=0,avoid_num3=0;
  while (node.ok())
    {
  
      if(keepMoving2&&avoid_succeed2)
	{
     
	//获取虚拟位置1的坐标
	  try{
	  transformStamped2 = tfBuffer.lookupTransform("robot2/base_link", "carrot1",
						       ros::Time(0));
	  }
	  catch (tf2::TransformException &ex) {
	  ROS_WARN("%s",ex.what());
	  ros::Duration(1.0).sleep();
	  continue;
	  }
	//前往虚拟位置1
	  vel_msg2.angular.z = 2.0 * atan2(transformStamped2.transform.translation.y,
					 transformStamped2.transform.translation.x);//调向虚拟位置方向
  
	  if( sqrt(pow(transformStamped2.transform.translation.x, 2) +
		   pow(transformStamped2.transform.translation.y, 2))>0.1)//前往目标位置的开始条件
	    {
	      if(transformStamped2.transform.translation.x>0)
		vel_msg2.linear.x = 1* sqrt(pow(transformStamped2.transform.translation.x, 2) +
					    pow(transformStamped2.transform.translation.y, 2));
	      else
		vel_msg2.linear.x = -1 * sqrt(pow(transformStamped2.transform.translation.x, 2) +
					      pow(transformStamped2.transform.translation.y, 2));
	      
	    }
	  else//前往目标的结束条件，调整最后姿态的开始条件
	    {
	      vel_msg2.linear.x=0;
	      try{
		transformStamped4 = tfBuffer.lookupTransform("robot1/base_link", "robot2/base_link",
							   ros::Time(0));
	      }
	      catch (tf2::TransformException &ex) {
		ROS_WARN("%s",ex.what());
		ros::Duration(1.0).sleep();
		continue;
	      }
	      tf::Quaternion quat2;
	      tf::quaternionMsgToTF(transformStamped4.transform.rotation, quat2);
	      // the tf::Quaternion has a method to acess roll pitch and yaw
	      double roll2, pitch2, yaw2;
	      tf::Matrix3x3(quat2).getRPY(roll2, pitch2, yaw2);
	      // the found angles are written in a geometry_msgs::Vector3
	      vel_msg2.angular.z=-1.0*yaw2;
	    }
	}
    if(keepMoving3&&avoid_succeed3)
      {
	//获取虚拟位置2的坐标
		try{
	  transformStamped3 = tfBuffer.lookupTransform("robot3/base_link", "carrot2",
						       ros::Time(0));
	}
	catch (tf2::TransformException &ex) {
	  ROS_WARN("%s",ex.what());
	  ros::Duration(1.0).sleep();
	  continue;
	}
	//前往虚拟位置2
	vel_msg3.angular.z = 2.0 * atan2(transformStamped3.transform.translation.y,
					 transformStamped3.transform.translation.x);//调向虚拟位置方向
  
	if( sqrt(pow(transformStamped3.transform.translation.x, 2) +
		 pow(transformStamped3.transform.translation.y, 2))>0.1)//前往目标位置的开始条件
	  {
	    if( transformStamped3.transform.translation.x>0)
	      vel_msg3.linear.x = 1 * sqrt(pow(transformStamped3.transform.translation.x, 2) +
					   pow(transformStamped3.transform.translation.y, 2));
	    else
	      vel_msg3.linear.x = -1 * sqrt(pow(transformStamped3.transform.translation.x, 2) +
					    pow(transformStamped3.transform.translation.y, 2));

	  }
	else//前往目标的结束条件，调整最后姿态的开始条件
	  {
	    vel_msg3.linear.x=0;
	    try{
	      transformStamped5 = tfBuffer.lookupTransform("robot1/base_link", "robot3/base_link",
							   ros::Time(0));
	    }
	    catch (tf2::TransformException &ex) {
	      ROS_WARN("%s",ex.what());
	      ros::Duration(1.0).sleep();
	      continue;
	    }
	    tf::Quaternion quat3;
	    tf::quaternionMsgToTF(transformStamped5.transform.rotation, quat3);
	    // the tf::Quaternion has a method to acess roll pitch and yaw
	    double roll3, pitch3, yaw3;
	    tf::Matrix3x3(quat3).getRPY(roll3, pitch3, yaw3);
	    // the found angles are written in a geometry_msgs::Vector3
	    vel_msg3.angular.z=-1.0*yaw3;
	  
	  }
	  }
    if((!keepMoving2))
      {

	//avoid_succeed2=false;
	vel_msg2.linear.x=0;
	vel_msg2.angular.z=ob_dir2;
      }
    if((!keepMoving3))
      {

	//avoid_succeed2=false;
	vel_msg3.linear.x=0;
	vel_msg3.angular.z=ob_dir3;
      }

    /* if((!keepMoving2)||(!avoid_succeed2))
      {
	avoid_succeed2=false;
	if(avoid_num2>100)//遇见避障死循环，反方向寻找活路
	  {
	    ob_dir2=ob_dir2*(-1.0);
	  }
	  if(!avoid_succeed2)//进行
	    {
	      try
		{
		  transformStamped2 = tfBuffer.lookupTransform("map","robot2/base_link",
							       ros::Time(0));
		}
	      catch (tf2::TransformException &ex) 
		{
		  ROS_WARN("%s",ex.what());
		  ros::Duration(1.0).sleep();
		  continue;
		}
	      tf::Quaternion quat2;
	      tf::quaternionMsgToTF(transformStamped2.transform.rotation, quat2);
	      // the tf::Quaternion has a method to acess roll pitch and yaw
	      double roll2, pitch2, yaw2;
	      tf::Matrix3x3(quat2).getRPY(roll2, pitch2, yaw2);
	      if(obstacle_Agoal2==0)
		{
		  obstacle_Agoal2=yaw2+ob_dir2;
		  vel_msg2.linear.x=0;
		}
	      vel_msg2.angular.z=(obstacle_Agoal2-yaw2);
	      if(abs(vel_msg2.angular.z)<0.1)
		{
		  if(!keepMoving2)
		    {
		      avoid_num2++;//用于恢复行为
		      obstacle_Lgoal2=0;
		      obstacle_Agoal2=0;
		      
		    }
		  else
		    {
		      if(obstacle_Lgoal2==0)
			{

			  obstacle_Lgoal2=sqrt(pow(transformStamped2.transform.translation.x, 2) +
					       pow(transformStamped2.transform.translation.y, 2))+ob_len2;
			}
		      
		      vel_msg2.linear.x=(obstacle_Lgoal2-sqrt(pow(transformStamped2.transform.translation.x, 2) +
							      pow(transformStamped2.transform.translation.y, 2)));
		      if(abs(vel_msg2.linear.x)<0.1)
			{
			  avoid_succeed2=true;
			  obstacle_Lgoal2=0;
			  obstacle_Agoal2=0;
			  avoid_num2=0;
			}
		    }
		}
	    
	    }

	  ROS_INFO("robot2 finds Obstacle!v:%f,w:%f",vel_msg2.linear.x,vel_msg2.angular.z);
	  }
	if((!keepMoving3)||(!avoid_succeed3))
	  {
	    avoid_succeed3=false;
	    if(avoid_num3>100)//遇见避障死循环，反方向寻找活路
	      {
		ob_dir3=ob_dir3*(-1.0);
	      }
	      if(!avoid_succeed3)//进行
		{
		  try
		    {
		      transformStamped3 = tfBuffer.lookupTransform("map","robot3/base_link",
								   ros::Time(0));
		    }
		  catch (tf2::TransformException &ex) 
		    {
		      ROS_WARN("%s",ex.what());
		      ros::Duration(1.0).sleep();
		      continue;
		    }
		  tf::Quaternion quat3;
		  tf::quaternionMsgToTF(transformStamped3.transform.rotation, quat3);
		  // the tf::Quaternion has a method to acess roll pitch and yaw
		  double roll3, pitch3, yaw3;
		  tf::Matrix3x3(quat3).getRPY(roll3, pitch3, yaw3);
		  if(obstacle_Agoal3==0)
		    {
		      obstacle_Agoal3=yaw3+ob_dir3;
		      vel_msg3.linear.x=0;
		    }
		  vel_msg3.angular.z=(obstacle_Agoal3-yaw3);
		  if(abs(vel_msg3.angular.z)<0.1)
		    {
		      if(!keepMoving3)
			{
			  avoid_num3++;//用于恢复行为
			  obstacle_Lgoal3=0;
			  obstacle_Agoal3=0;
		      
			}
		      else
			{
			  if(obstacle_Lgoal3==0)
			    {

			      obstacle_Lgoal3=sqrt(pow(transformStamped3.transform.translation.x, 2) +
						   pow(transformStamped3.transform.translation.y, 2))+ob_len3;
			    }
		      
			  vel_msg3.linear.x=(obstacle_Lgoal3-sqrt(pow(transformStamped3.transform.translation.x, 2) +
								  pow(transformStamped3.transform.translation.y, 2)));
			  if(abs(vel_msg3.linear.x)<0.1)
			    {
			      avoid_succeed3=true;
			      obstacle_Lgoal3=0;
			      obstacle_Agoal3=0;
			      avoid_num3=0;
			    }
			}
		    }
	    
		}

	      ROS_INFO("robot3 finds Obstacle!v:%f,w:%f",vel_msg3.linear.x,vel_msg3.angular.z);
	      }*/

	    vel_msg2.angular.z=vel_msg2.angular.z>1.0?1.0:vel_msg2.angular.z;
	    vel_msg2.angular.z=vel_msg2.angular.z<(-1.0)?(-1.0):vel_msg2.angular.z;
	    vel_msg2.linear.x=vel_msg2.linear.x>0.2?0.2:vel_msg2.linear.x;
	    vel_msg2.linear.x=vel_msg2.linear.x<(-0.2)?(-0.2):vel_msg2.linear.x;
	    vel_msg3.angular.z=vel_msg3.angular.z>1.0?1.0:vel_msg3.angular.z;
	    vel_msg3.angular.z=vel_msg3.angular.z<(-1.0)?(-1.0):vel_msg3.angular.z;
	    vel_msg3.linear.x=vel_msg3.linear.x>0.2?0.2:vel_msg3.linear.x;
	    vel_msg3.linear.x=vel_msg3.linear.x<(-0.2)?(-0.2):vel_msg3.linear.x;
	    turtlebot_vel2.publish(vel_msg2);
	    turtlebot_vel3.publish(vel_msg3);
	    ros::spinOnce();
	    rate.sleep();
	  }
	return 0;
      }
