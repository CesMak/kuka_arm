#ifndef PLATE_CONTROL_NODE_H__
#define PLATE_CONTROL_NODE_H__

#include <ros/ros.h>
#include <ros/console.h>

// ROS messages:
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>  /* for matrix multiplication */
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <math.h>       /* sin, sqrt */

//#include <std_msgs/String.h>
//#include <std_msgs/Int32.h>

namespace kuka
{
class ControlNode
{
public:
  ControlNode(ros::NodeHandle& nh);

  void update();

  double update_rate_;
  double update_time_;

protected:
  // helper
  double convertDistanceToAngle(double dis);
  std::vector<double> convertVectorToAngle(std::vector<double> input_vec);

  void transformPoint(const tf::StampedTransform& transform,
                      const geometry_msgs::Point& point_in, geometry_msgs::Point& point_out) const;
  void transformVelocity(const tf::StampedTransform& transform,
                         const geometry_msgs::Point& vel_in, geometry_msgs::Point& vel_out) const;

//  std::vector<double> toEulerAngle(double x, double y, double z, double w);

//  std::vector<double> convertToAngleVel(std::vector<double> input_vec);
//  double linearVelToAngleVel(double linear_vel);

  // UI callbacks

  // ROS API callbacks
  void jointsCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg);
  void controlCommandCallback(const geometry_msgs::PointConstPtr& command_point_msg);
  void ballCallback(const nav_msgs::OdometryConstPtr& ball_state_msg);

  // class members
  tf::TransformListener tf_listener_;

  //Control2D control2D_;
  geometry_msgs::Point input_command_point_msg_;
  nav_msgs::Odometry  input_ball_state_msg_;

  // states:
  double alpha_;
  double dalpha_;
  double beta_;
  double dbeta_ ;
  double ball_x_ ;
  double dball_x_ ;
  double ball_y_;
  double dball_y_ ;

  // helper variables:
  //sensor_msgs::Imu previous_imu_msg_;
  //  sensor_msgs::ImageConstPtr current_image_;
  sensor_msgs::JointStateConstPtr previous_joint_state_msg_;

  // parameters from the parameter server
  std::vector<double> control_Matrix_R_;
  std::vector<double> control_Matrix_F_;

  Eigen::Matrix <double , 2 , 8 > eigen_control_Matrix_R_;
  Eigen::Matrix <double , 2 , 2 > eigen_control_Matrix_F_;
  Eigen::Matrix <double , 8, 1  > states_;
  Eigen::Matrix <double , 2, 1  > eigen_w_;
  Eigen::Matrix <double , 2, 1  > eigen_u_;


  std::string controller_type_;
  std::string motors_controller_type_;

   // subscriber
  //ros::Subscriber imu_sub_;
  ros::Subscriber joints_sub_;
  ros::Subscriber command_sub_;
  ros::Subscriber ball_state_sub_;

  //publisher:
  ros::Publisher joint_commands_1_pub_;
  ros::Publisher joint_commands_2_pub_;
  ros::Publisher joint_commands_3_pub_;
  ros::Publisher joint_commands_4_pub_;
  ros::Publisher joint_commands_5_pub_;
  ros::Publisher joint_commands_6_pub_;

  // action server
};

} // end namespace
#endif
