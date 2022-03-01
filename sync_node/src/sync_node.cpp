/*
 * @Author: your name
 * @Date: 2022-01-20 09:58:11
 * @LastEditTime: 2022-03-01 13:58:20
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /src/time/src/timesync.cpp
 */
#include <message_filters/synchronizer.h>
//#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

#define PI (3.141592653)

string single_odometry_file_name;
string multi_odometry_file_name_1, multi_odometry_file_name_2;
string single_odometry_topic;
string multi_odometry_topic_1, multi_odometry_topic_2;

int msg_count = 0;
// vector<double> time_buffer;
vector<PoseStampedConstPtr> vision_buffer;
vector<nav_msgs::Odometry::ConstPtr> spin_buffer;

int single_msg_count = 0;
vector<nav_msgs::Odometry::ConstPtr> sigle_odometry_buffer;

// files option
ofstream vision_file, spin_file, single_odo_file;

ros::Subscriber sub_odometry;

void odometryCallBack(const nav_msgs::Odometry::ConstPtr &msg_in)
{
  std::cout << "----> receive a new message. " << std::endl;

  nav_msgs::Odometry::Ptr odometry_msg(new nav_msgs::Odometry(*msg_in));
  sigle_odometry_buffer.emplace_back(odometry_msg);
  
  ++ single_msg_count;

  std::cout << "----> have received " << single_msg_count << " messages." << std::endl;
}

void callback(const PoseStampedConstPtr& vision, const nav_msgs::Odometry::ConstPtr& spin_pose)
{
  // Solve all of perception here...
  std::cout << "----> receive a new message. " << std::endl;

  PoseStamped::Ptr vision_msg(new PoseStamped(*vision));
  nav_msgs::Odometry::Ptr spin_msg(new nav_msgs::Odometry(*spin_pose));

  vision_buffer.emplace_back(vision_msg);
  spin_buffer.emplace_back(spin_msg);
  
  ++ msg_count;

  std::cout << "----> have received " << msg_count << " messages." << std::endl;
}

void write_data_to_files_single()
{
  if(sigle_odometry_buffer.empty())
  {
    std::cout << "----> ERROR! sigle_odometry_buffer is empty." << std::endl;
    return;
  }

  std::cout << "----> odometry msg size is: " << sigle_odometry_buffer.size() << std::endl;

  single_odo_file.open("/home/pc/timesync/experiment_data/proposed/big/spin.txt", ios::app);


  double start_time = (sigle_odometry_buffer[0])->header.stamp.toSec();

  for(int i = 0; i < sigle_odometry_buffer.size(); ++i)
  {
    single_odo_file << (sigle_odometry_buffer[i])->header.stamp.toSec() - start_time << ","
                    << (sigle_odometry_buffer[i])->pose.pose.position.x << ","
                    << (sigle_odometry_buffer[i])->pose.pose.position.y << ","
                    << (sigle_odometry_buffer[i])->pose.pose.position.z << ","
                    << (sigle_odometry_buffer[i])->pose.pose.orientation.w << ","
                    << (sigle_odometry_buffer[i])->pose.pose.orientation.x << ","
                    << (sigle_odometry_buffer[i])->pose.pose.orientation.y << ","
                    << (sigle_odometry_buffer[i])->pose.pose.orientation.z << std::endl;        
  }

  single_odo_file.close();
}

void write_data_to_files()
{

  if(vision_buffer.empty() || spin_buffer.empty())
  {
    std::cout << "----> ERROR! vision_buffer or spin_buffer is empty." << std::endl;
    return;
  }

  std::cout << "----> vision msg size is: " << vision_buffer.size() << std::endl;
  std::cout << "----> spin msg size is: " << spin_buffer.size() << std::endl;
  
  // vision data
  vision_file.open("/home/pc/timesync/experiment_data/pre-calib/small/vision.txt", ios::app);
  spin_file.open("/home/pc/timesync/experiment_data/pre-calib/small/spin.txt", ios::app);

  Eigen::Vector3d eulerAngle;
  double start_time = (vision_buffer[0])->header.stamp.toSec();
  double start_pos_x, start_pos_y, start_pos_z, start_rot_x, start_rot_y, start_rot_z;
  start_pos_x = (vision_buffer[0])->pose.position.x;
  start_pos_y = (vision_buffer[0])->pose.position.y;
  start_pos_z = (vision_buffer[0])->pose.position.z;

  // Eigen::Quaterniond quaternion0((vision_buffer[0])->pose.orientation.w,
  //                                 (vision_buffer[0])->pose.orientation.x,
  //                                 (vision_buffer[0])->pose.orientation.y,
  //                                 (vision_buffer[0])->pose.orientation.z);
  // // 四元数转欧拉角(Z-Y-X，即RPY)
  // eulerAngle = quaternion0.matrix().eulerAngles(2, 1, 0);
  // start_rot_x = eulerAngle(2);
  // start_rot_y = eulerAngle(1);
  // start_rot_z = eulerAngle(0);

  for(int i = 0; i < vision_buffer.size(); ++i)
  {
    // Eigen::Quaterniond quaternion((vision_buffer[i])->pose.orientation.w,
    //                               (vision_buffer[i])->pose.orientation.x,
    //                               (vision_buffer[i])->pose.orientation.y,
    //                               (vision_buffer[i])->pose.orientation.z);
    // 四元数转欧拉角(Z-Y-X，即RPY)
    // eulerAngle = quaternion.matrix().eulerAngles(2, 1, 0);
    
    vision_file  << (vision_buffer[i])->header.stamp.toSec() - start_time << ","
                 << (vision_buffer[i])->pose.position.x - start_pos_x << ","
                 << (vision_buffer[i])->pose.position.y - start_pos_y << ","
                 << (vision_buffer[i])->pose.position.z - start_pos_z << ","
                 << (vision_buffer[i])->pose.orientation.w << ","
                 << (vision_buffer[i])->pose.orientation.x << ","
                 << (vision_buffer[i])->pose.orientation.y << ","
                 << (vision_buffer[i])->pose.orientation.z << std::endl;        
  }

  vision_file.close();

  start_time = (spin_buffer[0])->header.stamp.toSec();

  for(int i = 0; i < spin_buffer.size(); ++i)
  {
    // Eigen::Quaterniond quaternion((spin_buffer[i])->pose.pose.orientation.w,
    //                               (spin_buffer[i])->pose.pose.orientation.x,
    //                               (spin_buffer[i])->pose.pose.orientation.y,
    //                               (spin_buffer[i])->pose.pose.orientation.z);
    // 四元数转欧拉角(Z-Y-X，即RPY)
    // eulerAngle = quaternion.matrix().eulerAngles(2, 1, 0);
    
    spin_file    << (spin_buffer[i])->header.stamp.toSec() - start_time << ","
                 << (spin_buffer[i])->pose.pose.position.x << ","
                 << (spin_buffer[i])->pose.pose.position.y << ","
                 << (spin_buffer[i])->pose.pose.position.z << ","
                 << (spin_buffer[i])->pose.pose.orientation.w << ","
                 << (spin_buffer[i])->pose.pose.orientation.x << ","
                 << (spin_buffer[i])->pose.pose.orientation.y << ","
                 << (spin_buffer[i])->pose.pose.orientation.z << std::endl;        
  }

  spin_file.close();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sync_node");
  ros::NodeHandle nh;

  std::cout << "----> start sync node ." << std::endl;

  // update parameters
  // nh.param<string>("single_odometry_file_name", single_odometry_file_name);
  // nh.param<string>("multi_odometry_file_name_1", multi_odometry_file_name_1);
  // nh.param<string>("multi_odometry_file_name_2", multi_odometry_file_name_2);

  // nh.param<string>("single_odometry_topic", single_odometry_topic);
  // nh.param<string>("multi_odometry_topic_1", multi_odometry_topic_1);
  // nh.param<string>("multi_odometry_topic_2", multi_odometry_topic_2);

  sub_odometry = nh.subscribe("/lidar_Odometry", 1, odometryCallBack);

  // message_filters::Subscriber<PoseStamped> vision_sub(nh, "/mavros/vision_pose/pose", 1);
  // fastlio
  // message_filters::Subscriber<nav_msgs::Odometry> spin_sub(nh, "/lidar_Odometry", 1);
  // a-loam
  // message_filters::Subscriber<nav_msgs::Odometry> spin_sub(nh, "/aft_mapped_to_init", 1);
  // lio-sam
  // message_filters::Subscriber<nav_msgs::Odometry> spin_sub(nh, "/lio_sam/mapping/odometry", 1);

	// typedef sync_policies::ApproximateTime<PoseStamped, nav_msgs::Odometry> MySyncPolicy;
  // Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), vision_sub, spin_sub);
  // sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::Rate rate(200);
  bool status = ros::ok();

  while(status)
  {
    ros::spinOnce();

    status = ros::ok();
    rate.sleep();
  }
  
  // write data
  
  if(!vision_buffer.empty() && !spin_buffer.empty())
  {
    std::cout << "----> Have received multiple odometry messages." << std::endl;
    std::cout << "----> stop ros_node and start to write data to files, please wait ..." << std::endl;
    write_data_to_files();
    std::cout << "----> complete writing files." << std::endl;
  }

  if(!sigle_odometry_buffer.empty())
  {
    std::cout << "----> Have received only one odometry message." << std::endl;
    std::cout << "----> stop ros_node and start to write data to files, please wait ..." << std::endl;
    write_data_to_files_single();
    std::cout << "----> complete writing files." << std::endl;
  }

  return 0;
}
