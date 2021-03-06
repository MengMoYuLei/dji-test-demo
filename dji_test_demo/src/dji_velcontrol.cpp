#include "dji_velcontrol.h"
#include "dji_sdk/dji_sdk.h"

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlVelPub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t current_gps_health = 0;
bool fly_suc=false;
sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Point current_local_pos;
geometry_msgs::Vector3Stamped current_vel;
Eigen::Vector4d cur_odom(0,0,0,1);
double x_odom=0.0,y_odom=0.0,z_odom=0.0;
cv::Mat slMat2cvMat(sl::Mat& input);
int main(int argc,char **argv)
{
	ros::init(argc,argv,"dji_velcontrol");
	ros::NodeHandle nh;
	
	
	/************************************dji_node*****************************************/
/*	
	// Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
    ros::Subscriber gpsHealth      = nh.subscribe("dji_sdk/gps_health", 10, &gps_health_callback);
    ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
    ros::Subscriber velocitySub=nh.subscribe("dji_sdk/velocity",10,&velocity_callback);
	
   // Publish the control signal
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
  
  // We could use dji_sdk/flight_control_setpoint_ENUvelocity_yawrate here
  ctrlVelPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10); 
       
    // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");
 
  ros::Duration(3).sleep();
  
   bool obtain_control_result = obtain_control();
  bool takeoff_result;
  
  if (!set_local_position()) // We need this for height
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    return 1;
  }

  if(is_M100())
  {
    ROS_INFO("M100 taking off!");
    takeoff_result = M100monitoredTakeoff();
  }
 
  if(takeoff_result)
  {
  	fly_suc=true;
    ROS_INFO("##### take off succeed ....");
  }
   */
   /*****************************************zed*****************************************/
   sl::Camera zed; // Create a ZED camera object
  
  // Set configuration parameters
    sl::InitParameters init_params;
    init_params.depth_mode = sl::DEPTH_MODE_PERFORMANCE; // Use PERFORMANCE depth mode
    init_params.coordinate_units = sl::UNIT_MILLIMETER; // Use millimeter units (for depth measurements)
  
    // Open the camera
    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::SUCCESS)
        exit(-1);

    // Set runtime parameters after opening the camera
    sl::RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = sl::SENSING_MODE_STANDARD; // Use STANDARD sensing mode
    
    
    // Prepare new image size to retrieve half-resolution images
    sl::Resolution image_size = zed.getResolution();
    int new_width = image_size.width/2;
    int new_height = image_size.height/2;

    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
    
    sl::Mat image_zed(new_width, new_height, sl::MAT_TYPE_8U_C4);
    cv::Mat image_ocv = slMat2cvMat(image_zed);
    sl::Mat depth_image_zed(new_width, new_height, sl::MAT_TYPE_8U_C4);
    cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed);
    sl::Mat point_cloud;
    // Loop until 'q' is pressed
    char key = ' ';
    int i=0;
    while(key!='q')
    {
       // A new image is available if grab() returns SUCCESS
       if (zed.grab(runtime_parameters) == sl::SUCCESS) 
       {
       	// Retrieve left image
            zed.retrieveImage(image_zed, sl::VIEW_LEFT,sl::MEM_CPU,new_width, new_height);
            // Retrieve depth map. Depth is aligned on the left image
            zed.retrieveMeasure(depth_image_zed, sl::MEASURE_DEPTH,sl::MEM_CPU,new_width, new_height);
            // Retrieve colored point cloud. Point cloud is aligned on the left image.
            zed.retrieveMeasure(point_cloud, sl::MEASURE_XYZRGBA,sl::MEM_CPU, new_width, new_height);
						
						  // Display image and depth using cv:Mat which share sl:Mat data
            cv::imshow("Image", image_ocv);
            cv::imshow("Depth", depth_image_ocv);
             // Handle key event
            key = cv::waitKey(10);
            
            // Get and print distance value in mm at the center of the image
            // We measure the distance camera - object using Euclidean distance
            int x = image_zed.getWidth() / 2;
            int y = image_zed.getHeight() / 2;
            sl::float4 point_cloud_value;
            point_cloud.getValue(x, y, &point_cloud_value);
						
						
            float distance = sqrt(point_cloud_value.x * point_cloud_value.x + point_cloud_value.y * point_cloud_value.y + point_cloud_value.z * point_cloud_value.z);
             i++;
           if(i>20)
       			{
       				ROS_INFO("x %f,y %f,z %f",cur_odom(0),cur_odom(1),cur_odom(2));
       				ROS_INFO("Distance to Camera at (%d, %d): %f mm", x, y, distance);
       				i=0;
       			}
         
       }
      
    	 //  ros::spinOnce();
  		 
    }
		zed.close();
    return 0;
}

/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat slMat2cvMat(sl::Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }
    

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}

//user programming

void fly_vel_control(double x_vel,double y_vel,double z_vel,double yaw_rate)
{

//limted 
  if(fly_suc)
  {
  x_vel=x_vel>3?3:x_vel;
  x_vel=x_vel<-3?-3:x_vel;
  
  y_vel=y_vel>3?3:y_vel;
  y_vel=y_vel<-3?-3:y_vel;
  
  z_vel=z_vel>3?3:z_vel;
  z_vel=z_vel<-3?-3:z_vel;
  
	sensor_msgs::Joy controlVelYaw;
  controlVelYaw.axes.push_back(x_vel);
  controlVelYaw.axes.push_back(y_vel);
  controlVelYaw.axes.push_back(z_vel);
  controlVelYaw.axes.push_back(yaw_rate);
  ctrlVelPub.publish(controlVelYaw);
  }
}

void fly_pos_control(double target_offset_x,double target_offset_y,double target_offset_z,double target_yaw)
{
	double xCmd,yCmd,zCmd,yawCmd; 
  xCmd = target_offset_x - current_local_pos.x;
  yCmd = target_offset_y - current_local_pos.y;
  zCmd = target_offset_z;
  //ROS_INFO("x %f,y %f,z %f",xCmd,yCmd,zCmd);
  yawCmd=target_yaw;
   if(fly_suc)
   {
  	    sensor_msgs::Joy controlPosYaw;
  			controlPosYaw.axes.push_back(xCmd);
  			controlPosYaw.axes.push_back(yCmd);
  			controlPosYaw.axes.push_back(zCmd);
  			controlPosYaw.axes.push_back(yawCmd);
  			ctrlPosYawPub.publish(controlPosYaw);
  }
 
      
}






/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates. Accurate when distances are small.
!*/
void
localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin)
{
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;

  deltaNed.y = deltaLat * deg2rad * C_EARTH;
  deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
  deltaNed.z = target.altitude - origin.altitude;
}

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
	//double alpha,beta,gamma;
  current_atti = msg->quaternion;
  double alpha=(double)((long)(toEulerAngle(current_atti).x*1000)/1000.0);
  double beta=(double)((long)(toEulerAngle(current_atti).y*1000)/1000.0);
  double gamma=(double)((long)(toEulerAngle(current_atti).z*1000)/1000.0);
  double d_x=(double)(((long)(current_vel.vector.x*10000)/10000.0)*0.01);
  double d_y=(double)(((long)(current_vel.vector.y*10000)/10000.0)*0.01);
  double d_z=(double)(((long)(current_vel.vector.z*10000)/10000.0)*0.01);
 //ROS_INFO("a %f,b %f,c %f",alpha,beta,gamma);
 
 
 //The odometer is not so troublesome,but we want to test the real-time operation of matrix operation.
 //Odometer model:x=x+vx y=y+vy z=z+vz yaw=yaw+w
  Eigen::Vector4d cur_ds(0,0,0,1);
  Eigen::Matrix<double, 4, 4> ZE_rx;
   							ZE_rx << 
  							1,     0,       0,       0,
  							0,cos(alpha),-sin(alpha),0,
  							0,sin(alpha),cos(alpha), 0,
  							0,     0,       0,       1;
  							
	Eigen::Matrix<double, 4, 4> ZE_ry;
								ZE_ry <<
								cos(beta),  0, sin(beta),0,
								  0,        1,    0,     0,
								-sin(beta), 0, cos(beta),0,
								   0,       0,    0,     1;
								   
	Eigen::Matrix<double, 4, 4> ZE_rz;
								ZE_rz <<
								cos(gamma), -sin(gamma),0,cur_odom(0),
								sin(gamma), cos(gamma), 0,cur_odom(1),
								  0,          0,        1,cur_odom(2),
								  0,          0,        0,		1;
								  
	
	
	Eigen::Matrix<double, 4, 4> ZE_f2w;
								ZE_f2w <<
								  1,          0,        0,   d_x,
								  0,          1,        0,   d_y,
								  0,          0,        1,   d_z,
								  0,          0,        0,		1;			  

  
  cur_odom=ZE_f2w*ZE_rz*ZE_ry*ZE_rx*cur_ds;
  
  
 
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  current_local_pos = msg->point;
  fly_pos_control(0,1,3,0);
  
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
 current_gps = *msg;

}

void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg) {
  current_gps_health = msg->data;
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	current_vel=*msg;
}

/*!
 * This function demos how to use M100 flight_status
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
      current_gps.altitude - home_altitude < 1.0)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }

  return true;
}

bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}




















