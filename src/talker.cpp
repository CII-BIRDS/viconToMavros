#include <ros/ros.h> 

#include <geometry_msgs/TransformStamped.h> 

#include <geometry_msgs/PoseStamped.h> 

#define _USE_MATH_DEFINES 

#include <cmath> 

 

struct Quaternion { 

 double w, x, y, z; 

}; 

 

struct EulerAngles { 

 double roll, pitch, yaw; 

}; 

 

EulerAngles ToEulerAngles(Quaternion q) { 

 EulerAngles angles; 

 

 // roll (x-axis rotation) 

 double sinr_cosp = 2 * (q.w * q.x + q.y * q.z); 

 double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y); 

 angles.roll = std::atan2(sinr_cosp, cosr_cosp); 

 

 // pitch (y-axis rotation) 

 double sinp = 2 * (q.w * q.y - q.z * q.x); 

 if (std::abs(sinp) >= 1) 

 angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range 

 else 

 angles.pitch = std::asin(sinp); 

 

 // yaw (z-axis rotation) 

 double siny_cosp = 2 * (q.w * q.z + q.x * q.y); 

 double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z); 

 angles.yaw = std::atan2(siny_cosp, cosy_cosp); 

 

 return angles; 

} 

 

geometry_msgs::TransformStamped current_state; 

void state_cb(const geometry_msgs::TransformStamped::ConstPtr& msg) 

{ 

 current_state = *msg; 

} 

 

 

int main(int argc, char **argv) 

{ 

 

 Quaternion quat; 

 EulerAngles eul; 

 ros::init(argc, argv, "viconToMavros"); 

 ros::NodeHandle nh; 

 


 ros::Subscriber state_sub = 
nh.subscribe<geometry_msgs::TransformStamped> 

 ("vicon/birds/birds", 10, state_cb); 

 ros::Publisher local_pos_pub = 
nh.advertise<geometry_msgs::PoseStamped> 

 ("mavros/vision_pose/pose", 10); 

 

 

 //the setpoint publishing rate MUST be faster than 2Hz 

 ros::Rate rate(50.0); 

 

 geometry_msgs::PoseStamped pose; 

 int count = 1; 

 

 // wait for FCU connection 

 while(ros::ok() ){ 

 

 //pose.header = current_state.header; 

 pose.header.stamp = ros::Time::now(); 

 pose.header.seq=count; 

 pose.header.frame_id = current_state.header.frame_id; 

 pose.pose.position.x = current_state.transform.translation.x; 

 pose.pose.position.y = current_state.transform.translation.y; 

 pose.pose.position.z = current_state.transform.translation.z; 

 

 pose.pose.orientation.x = current_state.transform.rotation.x; 

 pose.pose.orientation.y = current_state.transform.rotation.y; 

 pose.pose.orientation.z = current_state.transform.rotation.z; 

 pose.pose.orientation.w = current_state.transform.rotation.w; 

 if(count%50==0){ 

 quat.x = current_state.transform.rotation.x; 

 quat.y = current_state.transform.rotation.y; 

 quat.z = current_state.transform.rotation.z; 

 quat.w = current_state.transform.rotation.w; 

 eul = ToEulerAngles(quat); 

 std::cout<<"roll:"<<eul.roll*180/M_PI<<std::endl; 

 std::cout<<"pitch:"<<eul.pitch*180/M_PI<<std::endl; 

 std::cout<<"yaw:"<<eul.yaw*180/M_PI<<std::endl; 

 } 

 local_pos_pub.publish(pose); 

 count++; 

 ros::spinOnce(); 

 rate.sleep(); 

 

 } 

 

 

 

 

 return 0; 

 } 

 



// This code is from open-source community:
// https://discuss.px4.io/t/error-in-local-positioning-of-pixhawk4-using-vicon-data/16011/4
