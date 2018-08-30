#ifndef _NEWTURTLEBOT_LOCATION_H_
#define _NEWTURTLEBOT_LOCATION_H_
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf/transform_datatypes.h"
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "stdexcept"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/LaserScan.h"
class Scan_obstacle{

 private:
  ros::NodeHandle node;
  ros::Subscriber laserSub;
  std::string robot_name;
  static constexpr double MIN_SCAN_ANGLE = -30.0/180*M_PI;
  static constexpr double MAX_SCAN_ANGLE = +30.0/180*M_PI;
  static constexpr double MIN_DIST_FROM_OBSTACLE = 0.8;
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr&);
 public:
  bool keepMoving=true;
  double ob_dir=0.0;
 Scan_obstacle(std::string r_name="robot1"):robot_name(r_name)
    {
     laserSub= node.subscribe(robot_name+"/scan", 1,&Scan_obstacle::scanCallback,this);//订阅robotx/scan话题
    }
};
class Formation_control{
 public:
 Formation_control(std::string r_name="robot2"):robot_name(r_name),tfListener(tfBuffer)
  {
    transformStamped.transform.translation.x=100.0;
    turtlebot_vel =
      node.advertise<geometry_msgs::Twist>(robot_name+"/mobile_base/commands/velocity", 10);
  }
  bool receive_tf(const std::string&,const std::string&,geometry_msgs::TransformStamped&);//坐标转换  
  void moving_control(bool,double);
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::Twist vel_msg;
  bool succeedA_avoid=true;
  bool succeedL_avoid=true;
  bool succeed_avoid=true;
 private:
  std::string robot_name;
  ros::Publisher turtlebot_vel;
  ros::NodeHandle node;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  double avoiding_theta=M_PI/2;
  double avoiding_position=0.5;
  double global_dir=1.0;
  double obs_xx=0;
  double obs_aa=0;
  double get_yaw(geometry_msgs::TransformStamped &);
  bool gotoposition(void);//前往虚拟位置
  bool turnpose(void);//调整最后姿态
  void move_twist();
  bool avoiding_obstacle(double,bool);
};

#endif
