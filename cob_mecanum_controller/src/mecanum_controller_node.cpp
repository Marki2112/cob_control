/*
 * Copyright 2020 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cob_mecanum_controller/mecanum_controller.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <exception>
class MecanumControllerNode
{
public:
  MecanumControllerNode() : nh_()
  {
    double lx, ly, r;
    bool all_parameters_set = true;
    ros::NodeHandle pnh("~");
    if (!pnh.getParam("lx", lx))
    {
      ROS_ERROR_STREAM("Parameter lx was not declared in the scope");
      all_parameters_set = false;
    }
    if (!pnh.getParam("ly", ly))
    {
      ROS_ERROR_STREAM("Parameter ly was not declared in the scope");
      all_parameters_set = false;
    }
    if (!pnh.getParam("r", r))
    {
      ROS_ERROR_STREAM("Parameter r was not declared in the scope");
      all_parameters_set = false;
    }

    if (!all_parameters_set)
    {
      throw std::runtime_error("At least one parameter is missing.");
    }

    static_frame_ = "base_footprint";
    pnh.getParam("static_frame", static_frame_);
    odom_frame_ = "odom_combined";
    pnh.getParam("odom_frame", odom_frame_);

    controller_ = std::make_shared<cob_mecanum_controller::MecanumController>(lx, ly, r);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10, false);
    joint_cmd_pub_ = nh_.advertise<sensor_msgs::JointState>("wheel_command", 10, false);

    twist_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &MecanumControllerNode::twistCallback, this);

    joint_state_sub_ =
        nh_.subscribe<sensor_msgs::JointState>("wheel_state", 1, &MecanumControllerNode::jointStateCallback, this);
  
  }

protected:
  ros::NodeHandle nh_;
  ros::Subscriber twist_sub_;
  ros::Subscriber joint_state_sub_;

  ros::Publisher odom_pub_;
  ros::Publisher joint_cmd_pub_;
  std::shared_ptr<cob_mecanum_controller::MecanumController> controller_;

  std::string static_frame_;
  std::string odom_frame_;

  // add new parameter for odometry
  ros::Time current_time = ros::Time::now();
  ros::Time last_time = ros::Time::now();
  
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
  
  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;

  // define broadcaster for tf between odom_combined and base_footprint
  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;

  void twistCallback(const geometry_msgs::Twist msg)
  {
    Eigen::Vector3d twist;
    twist << msg.linear.x, msg.linear.y, msg.angular.z;
    Eigen::Vector4d wheel_velocities = controller_->twistToWheel(twist);
    sensor_msgs::JointState joint_command;
    joint_command.velocity = std::vector<double>(
        wheel_velocities.data(), wheel_velocities.data() + wheel_velocities.rows() * wheel_velocities.cols());
    joint_cmd_pub_.publish(joint_command);
  }

  void jointStateCallback(const sensor_msgs::JointState msg)
  {
    Eigen::Vector4d wheel_velocities(msg.velocity.data());
    Eigen::Vector3d twist = controller_->wheelToTwist(wheel_velocities);

    // Calculation for the position 
    current_time = ros::Time::now();

    vx = twist.x();
    vy = twist.y();
    vth = twist.z();

    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;
    
    x += delta_x;
    y += delta_y;
    th += delta_th;

    // create a quaternion from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //publish transform from odom to base_footprint
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odom_frame_;
    odom_trans.child_frame_id = static_frame_;

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = static_frame_;

    // publish the new position
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.angular.z = vth;
    odom_pub_.publish(odom_msg);

    last_time = current_time;
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "mecanum_controller");
  MecanumControllerNode mcn;
  ros::spin();
}
