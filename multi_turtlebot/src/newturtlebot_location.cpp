#include"newturtlebot_location.h"

/***************************************扫描障碍********************************************/
void Scan_obstacle::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  bool isObstacleInFront_L=false;//障碍标志
  bool isObstacleInFront_R=false;//障碍标志

  int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);//激光扫描min，ceil向上取整
  int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);//激光扫描max，floor向下取整
  int midIndex=(minIndex+maxIndex)/2;//中心

  int obstaclebound_L=minIndex;//障碍左边界
  int obstaclebound_R=maxIndex;//障碍右边界
  //搜索中心以左障碍
  for (int currIndex = minIndex + 1; currIndex!=midIndex; ++currIndex)
    {
      if (scan->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE) 
	{
	  isObstacleInFront_L = true;
	  obstaclebound_L=currIndex;
	  break;
        }
    }
  //寻找中心以右障碍
  for (int currIndex = maxIndex-1; currIndex!=midIndex; --currIndex)
    {
      if (scan->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE) 
	{
	  isObstacleInFront_R = true;
	  obstaclebound_R=currIndex;
	  break;
        }
    }
  //存在障碍，进行避障方向选择
  if(isObstacleInFront_L&&isObstacleInFront_R)
    {
      ob_dir=1.0;
      keepMoving=false;
    }
  else if(isObstacleInFront_L)
    {
      ob_dir=-1.0;
      keepMoving=false;
    }
  else if(isObstacleInFront_R)
    {
      ob_dir=2.0;
      keepMoving=false;
    }
  else
    {
      ob_dir=0;
      keepMoving=true;
    }
  if(!keepMoving)
    {
      ROS_INFO("%s find obstacle!",robot_name.c_str());
    }
}


/**********************************************************获取坐标转换************************************/
bool Formation_control::receive_tf(const std::string& targetframe,const std::string& sourceframe,geometry_msgs::TransformStamped& gettransform)
{
  try{
    gettransform = tfBuffer.lookupTransform(targetframe,sourceframe,ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    return 0;
  }
  return 1;
}

/*******************************************************形成编队过程控制********************************/
bool Formation_control::gotoposition()
{
  if( receive_tf(robot_name+"/base_link",(robot_name=="robot2"?"carrot1":"carrot2"),transformStamped))
    {
      //调向虚拟位置方向
      vel_msg.angular.z = 2.0 * atan2(transformStamped.transform.translation.y,
				       transformStamped.transform.translation.x);
      //前往目标位置的开始条件
      if( sqrt(pow(transformStamped.transform.translation.x, 2) +
	       pow(transformStamped.transform.translation.y, 2))>0.1)
	{
	  //ROS_INFO("%s gotoposition",robot_name.c_str());
	  if(transformStamped.transform.translation.x>0)
	    vel_msg.linear.x = 1* sqrt(pow(transformStamped.transform.translation.x, 2) +
					pow(transformStamped.transform.translation.y, 2));
	  else
	    vel_msg.linear.x = -1 * sqrt(pow(transformStamped.transform.translation.x, 2) +
					  pow(transformStamped.transform.translation.y, 2));
	  return 0;  
	}
      else
	{
	  return 1;
	}
    }
  return 0;

}
/**********************************************************************形成编队最后姿态调整********************************/
bool Formation_control::turnpose()
{
  vel_msg.linear.x=0;
  //保持与领航者相同的姿态
  if( receive_tf("robot1/base_link",robot_name+"/base_link",transformStamped))
    {
      double yaw;
      yaw=get_yaw(transformStamped);
      vel_msg.angular.z=-1.0*yaw;
      global_dir=vel_msg.angular.z;
      if(abs(yaw)<0.1)
	{
	  // ROS_INFO("%s arrive at destination",robot_name.c_str());
	  return 1;
	}
      else
	{
	  //ROS_INFO("%s turnpose",robot_name.c_str());
	  return 0;
	}
    }
  return 0;
}

/**********************************************发布速度控制命令**********************************/
void Formation_control::move_twist()
{
  //速度限幅
  vel_msg.angular.z=vel_msg.angular.z>1.0?1.0:vel_msg.angular.z;
  vel_msg.angular.z=vel_msg.angular.z<(-1.0)?(-1.0):vel_msg.angular.z;
  vel_msg.linear.x=vel_msg.linear.x>0.2?0.2:vel_msg.linear.x;
  vel_msg.linear.x=vel_msg.linear.x<(-0.2)?(-0.2):vel_msg.linear.x;
  turtlebot_vel.publish(vel_msg);
}

/************************************获取航向角**************************************************/
double Formation_control::get_yaw(geometry_msgs::TransformStamped &transformStamped)
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(transformStamped.transform.rotation, quat);
  // the tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  //ROS_INFO("yaw:%f",yaw);
  return yaw;
}

/**********************************避障**********************************************************/
bool Formation_control::avoiding_obstacle(double turn_dir,bool keepmoving)
{
  if((!keepmoving)||(!succeed_avoid))//发现障碍，或者避障尚未成功
     {
       if(succeedA_avoid&&succeed_avoid&&succeedL_avoid)//发现障碍标志位重置
	 {
	   succeedA_avoid=false;//转向避障完成标志
	   succeedL_avoid=false;//直线行走避障完成标志
	   succeed_avoid=false;//成功避障标志

	   //设置初始避障转向与目的方向一致
	   if(receive_tf(robot_name+"/base_link",(robot_name=="robot2"?"carrot1":"carrot2"),transformStamped))
	     {

	       global_dir= atan2(transformStamped.transform.translation.y,
				 transformStamped.transform.translation.x);
	       if(global_dir<0)
		 {
		   global_dir=-0.5;
		 }
	       else
		 {
		   global_dir=0.5;

		 }
	     }
	 }
       //转向避障
       if((!succeed_avoid)&&(!succeedA_avoid))
	 {
	 
	
	   vel_msg.linear.x=0;
	   vel_msg.angular.z=global_dir;
	   obs_aa+=((vel_msg.angular.z>0)?vel_msg.angular.z:-vel_msg.angular.z)*0.1;
	   ROS_INFO("%s obs_aa:%f err %f",robot_name.c_str(),obs_aa,avoiding_theta-obs_aa);
	   if((avoiding_theta-obs_aa)<0.1&&(avoiding_theta-obs_aa)>-0.1)
	     {
	       //转向避障完成
	       vel_msg.angular.z=0;
	       succeedA_avoid=1;
	       obs_aa=0;
	     }
	   else
	     {
	       succeedA_avoid=0;
	     }
	  
	 }
       //直行走线避障
       if((!succeed_avoid)&&succeedA_avoid&&(!succeedL_avoid))
	 {
	   if(keepmoving)//前方障碍判断
	     {
	       vel_msg.linear.x=0.05;
	       obs_xx+= vel_msg.linear.x*0.1;
	       ROS_INFO("%s obs_xx:%f err %f",robot_name.c_str(),obs_xx,avoiding_position-obs_xx);
	       if((avoiding_position-obs_xx)<0.1&&(avoiding_position-obs_xx)>-0.1)//直线避障完成标志
		 {
		   succeedL_avoid=1;
		   obs_xx=0;
		 }
	       else
		 {
		   succeedL_avoid=0;
		 }
	     }
	   else//发现障碍重新开始转向避障
	     {
	       vel_msg.linear.x=0;
	       succeedA_avoid=0;
	       obs_xx=0;
	     }

	 }
       //扫描航线障碍，确认是否避障成功
       if(succeedL_avoid&&succeedA_avoid&&(!succeed_avoid))
	 {
	   vel_msg.linear.x=0;
	   vel_msg.angular.z=-global_dir;//转回原来方向
	   obs_aa+=((vel_msg.angular.z>0)?vel_msg.angular.z:-vel_msg.angular.z)*0.1;
	   ROS_INFO("%s obs_ss:%f err %f",robot_name.c_str(),obs_aa,avoiding_theta-obs_aa);
	   if((avoiding_theta-obs_aa)<0.1&&(avoiding_theta-obs_aa)>-0.1)
	     {
	       if(!keepmoving)//避障失败，重新转向避障
		 {
		   succeedA_avoid=0;
		   succeedL_avoid=0;
		   obs_aa=0;
		 }
	       else//避障成功
		 {
		   obs_aa=0;
		   succeed_avoid=1;
		   return 1;
		 }
	     }
	 }

     }
  else//清空转向和直线行走里程计数
     {
       obs_xx=0;
       obs_aa=0;
     }
   return 0;
}


void Formation_control::moving_control(bool keepmoving,double ob_dir)
{
  if(keepmoving&&succeed_avoid)//无障碍，进行正常编队控制
    {
      if(gotoposition())
	{
	  turnpose();
	}
    }
  if(avoiding_obstacle(ob_dir,keepmoving))
    {
      ROS_INFO("%s SUCCEED_avoidobstacle",robot_name.c_str());
    }
  move_twist();
}

int main(int argc,char **argv)
{
  ros::init(argc,argv,"newturtlebot_location");
  ros::NodeHandle node;

  Scan_obstacle robot2_obs("robot2");
  Scan_obstacle robot3_obs("robot3");

  Formation_control robot2_con("robot2");
  Formation_control robot3_con("robot3");

  //使用多线程订阅消息
  ros::AsyncSpinner spinner(2); // Use 2 threads
  spinner.start();
  ros::Rate rate(10.0);//循环速率10hz
  while(node.ok())
    {
      robot2_con.moving_control(robot2_obs.keepMoving,robot2_obs.ob_dir);
      robot3_con.moving_control(robot3_obs.keepMoving,robot3_obs.ob_dir);
      rate.sleep();
    }
  return 0;
}



