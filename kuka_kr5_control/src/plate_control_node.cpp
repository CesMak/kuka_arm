#include <kuka_kr5_control/plate_control_node.hpp>

namespace kuka
{
  //constructor is called only once:
  ControlNode::ControlNode(ros::NodeHandle& nh)
  {
    //get Params:
    //controller_type_ = nh.param("control_constants/controller_type", std::string("2D"));
    //motors_controller_type_ = nh.param("control_constants/motors_controller_type", std::string("VelocityJointInterface"));

    // get 2D control gains:
    //gains_2D_Kxz_=nh.param("control_constants/gains_2D_Kxz",(std::vector<double> ) {1.0,2.0,3.0,4.0});

    //Subscriber:
      joints_sub_ = nh.subscribe("/kuka/joint_states", 5, &ControlNode::jointsCallback, this);
      ball_state_sub_ = nh.subscribe("/gazebo/ball/odom", 5, &ControlNode::ballCallback, this);
      command_sub_ = nh.subscribe("/kuka/control_command", 5, &ControlNode::controlCommandCallback, this);

    //Publisher:
    // this can be effort, velocity or position controllers as defined in the controller_node.yaml (in config folder)
    joint_commands_1_pub_ = nh.advertise<std_msgs::Float64>("/kuka/link_1_controller/command", 5);
    joint_commands_2_pub_ = nh.advertise<std_msgs::Float64>("/kuka/link_2_controller/command", 5);
    joint_commands_3_pub_ = nh.advertise<std_msgs::Float64>("/kuka/link_3_controller/command", 5);
    joint_commands_4_pub_ = nh.advertise<std_msgs::Float64>("/kuka/link_4_controller/command", 5);
    joint_commands_5_pub_ = nh.advertise<std_msgs::Float64>("/kuka/link_5_controller/command", 5);
    joint_commands_6_pub_ = nh.advertise<std_msgs::Float64>("/kuka/link_6_controller/command", 5);

    //Publish rpy angles:
    // rpy_pub_ =  nh.advertise<geometry_msgs::PointStamped>("rpy_angles", 5);
    // calc_ball_odom_pub_ = nh.advertise<geometry_msgs::PointStamped>("ball_odom_", 5);

    //Publish desired torques:
   // desired_torques_pub_ = nh.advertise<geometry_msgs::Vector3>("desired_torques", 5);

    //action server:
  }

  // rostopic pub /kuka/control_command geometry_msg/Point "x: 1.0 y: 0.0   z: 0.0"
  void ControlNode::controlCommandCallback(const geometry_msgs::PointConstPtr& command_point_msg)
  {
    input_command_point_msg_ = *command_point_msg;
  }

  void ControlNode::ballCallback(const nav_msgs::OdometryConstPtr& ball_state_msg)
  {
    input_ball_state_msg_ = *ball_state_msg;
  }

  void ControlNode::jointsCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg)
  {
    previous_joint_state_msg_ = *joint_state_msg;

    std_msgs::Float64 pos_link1;
    std_msgs::Float64 pos_link2;
    std_msgs::Float64 pos_link3;
    std_msgs::Float64 pos_link4;
    std_msgs::Float64 pos_link5;
    std_msgs::Float64 pos_link6;

    pos_link1.data = 0.0;
    pos_link2.data = 0.0;
    pos_link3.data = 0.0;
    pos_link4.data = 0.0;
    pos_link5.data = 0.0;
    pos_link6.data = 0.0;

    if(input_command_point_msg_.x == 1.0 && input_command_point_msg_.y == 0.0 && input_command_point_msg_.z == 0.0 )
    {
      ROS_INFO_ONCE("driving to zero state");
      pos_link4.data = 3.14159;
      pos_link5.data = 1.57079632679;
    }
    else if(input_command_point_msg_.x == 2.0 && input_command_point_msg_.y == 0.0 && input_command_point_msg_.z == 0.0 )
    {
      ROS_INFO_ONCE("driving to freaky state");
      pos_link1.data = 1.5159;
      pos_link2.data = -1.74159;
      pos_link3.data = -0.74159;
      pos_link4.data = 3.14159;
      pos_link5.data = 1.57079632679;
      pos_link6.data = 1.57079632679;
    }
    else
    {
      pos_link1.data = 0.0;
      pos_link2.data = 0.0;
      pos_link3.data = 0.0;
      pos_link4.data = 0.0;
      pos_link5.data = 0.0;
      pos_link6.data = 0.0;
    }

    joint_commands_1_pub_.publish(pos_link1);
    joint_commands_2_pub_.publish(pos_link2);
    joint_commands_3_pub_.publish(pos_link3);
    joint_commands_4_pub_.publish(pos_link4);
    joint_commands_5_pub_.publish(pos_link5);
    joint_commands_6_pub_.publish(pos_link6);
  }

  //update (publish messages...) every 10ms. This is only updated every 100 ms! but why?!
  void ControlNode::update()
  {
    // inputs:
    // previous_joint_state_msg_; // angles (4,5)
    // ball position in plate_system:
    // rostopic echo /gazebo/ball/odom


    //calc2MotorCommands_withOdometry();

    // note that the wheel torque is limited to ca. 4.1Nm.
//    std_msgs::Float64 realT1;
//    std_msgs::Float64 realT2;
//    std_msgs::Float64 realT3;

//    realT1.data=realT_.at(0);
//    realT2.data=realT_.at(1);
//    realT3.data=realT_.at(2);

//    joint_commands_1_pub_.publish(realT1);
//    joint_commands_2_pub_.publish(realT2);
//    joint_commands_3_pub_.publish(realT3);
  }

}  // end namespace ballbot

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kuka_kr5_control_node");
  ROS_INFO("start kuka_kr5_control_node");

  ros::NodeHandle nh;

  // this wait is needed to ensure this ros node has gotten
  // simulation published /clock message, containing
  // simulation time.
  ros::Time last_ros_time_;
  bool wait = true;
  while (wait)
  {
    last_ros_time_ = ros::Time::now();
    if (last_ros_time_.toSec() > 0)
      wait = false;
  }

  kuka::ControlNode node(nh);
  ros::Rate loop_rate(nh.param("/kuka/control_constants/update_rate", 100.0));

  // TODO wait here to start publishing nodes information
  // until first non zero value from gazebo is received!
  while (ros::ok())
  {
    ros::spinOnce();
    node.update();
    loop_rate.sleep();
  }

  return 0;
}

