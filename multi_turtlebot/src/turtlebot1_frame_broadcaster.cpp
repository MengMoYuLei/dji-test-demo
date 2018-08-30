#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char **argv){
  ros::init(argc,argv,"turtlebot1_frame_broadcaster");
  ros::NodeHandle node;
  
  tf2_ros::TransformBroadcaster tfb2,tfb3;
  geometry_msgs::TransformStamped transformStamped2;
  geometry_msgs::TransformStamped transformStamped3;
  transformStamped2.header.frame_id="robot1/base_link";
  transformStamped2.child_frame_id="carrot1";
  transformStamped2.transform.translation.x=-2;
  transformStamped2.transform.translation.y=-2;
  transformStamped2.transform.translation.z=0.0;
  tf2::Quaternion q2;
  q2.setRPY(0,0,0);
  transformStamped2.transform.rotation.x=q2.x();
  transformStamped2.transform.rotation.y=q2.y();
  transformStamped2.transform.rotation.z=q2.z();
  transformStamped2.transform.rotation.w=q2.w();

  transformStamped3.header.frame_id="robot1/base_link";
  transformStamped3.child_frame_id="carrot2";
  transformStamped3.transform.translation.x=-2;
  transformStamped3.transform.translation.y=2;
  transformStamped3.transform.translation.z=0.0;
  tf2::Quaternion q3;
  q3.setRPY(0,0,0);
  transformStamped3.transform.rotation.x=q3.x();
  transformStamped3.transform.rotation.y=q3.y();
  transformStamped3.transform.rotation.z=q3.z();
  transformStamped3.transform.rotation.w=q3.w();

  ros::Rate rate(10.0);
  while(node.ok())
    {
       transformStamped2.header.stamp=ros::Time::now();
       transformStamped3.header.stamp=ros::Time::now();
      //transformStamped.transform.translation.x = 2.0*sin(ros::Time::now().toSec());
      //transformStamped.transform.translation.y = 2.0*cos(ros::Time::now().toSec());
      tfb2.sendTransform(transformStamped2);
      tfb3.sendTransform(transformStamped3);
      rate.sleep();
      printf("sending\n");
    }
}
