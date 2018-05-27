#include <kuka_kr5_control/plate_control_node.hpp>

namespace kuka
{
  //constructor is called only once:
  ControlNode::ControlNode(ros::NodeHandle& nh)
  {
    //get Params:
    //controller_type_ = nh.param("control_constants/controller_type", std::string("2D"));
    //motors_controller_type_ = nh.param("control_constants/motors_controller_type", std::string("VelocityJointInterface"));

    // control Matrix:
    control_Matrix_R_ = nh.param("control_constants/R_", (std::vector<double> )  {1.0, 2.0});
    control_Matrix_F_ = nh.param("control_constants/F_", (std::vector<double> )  {1.0, 2.0});

    int u = 0;
    for(int i=0; i<2; i++) {
      for( int j=0; j<8; j++){
        eigen_control_Matrix_R_(i , j ) = control_Matrix_R_[u];
        std::cout<<eigen_control_Matrix_R_<<std::endl;
        u++;
      }
    }

    u=0;
    for(int i=0; i<2; i++) {
      for( int j=0; j<2; j++){
        eigen_control_Matrix_F_(j , i ) = control_Matrix_F_[u];
        u++;
      }
    }

    std::cout<<eigen_control_Matrix_R_<<std::endl;
    std::cout<<eigen_control_Matrix_F_<<std::endl;

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

//  void BallTrackingNode::imageCb(const sensor_msgs::ImageConstPtr& image)
//  {
//    current_image_ = image;
//  }

  void ControlNode::jointsCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg)
  {
    previous_joint_state_msg_ = joint_state_msg;

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

  void ControlNode::update()
  {
    // Convert vel and position of ball in plate frame:
    std::string to_frame = std::string("ee_link");
    std::string from_frame = std::string("world");

    geometry_msgs::Point ball_pos_in_plate_frame;
    geometry_msgs::Point ball_vel_in_plate_frame;

      try
      {
        if (tf_listener_.canTransform(to_frame, from_frame, ros::Time()))
        {
          tf::StampedTransform transform;
          tf_listener_.lookupTransform(to_frame, from_frame, ros::Time(), transform);
          transformPoint(transform, input_ball_state_msg_.pose.pose.position, ball_pos_in_plate_frame);
          geometry_msgs::Point input_ball_vel;
          input_ball_vel.x =  input_ball_state_msg_.twist.twist.linear.x;
          input_ball_vel.y =  input_ball_state_msg_.twist.twist.linear.y;
          input_ball_vel.z =  input_ball_state_msg_.twist.twist.linear.z;

          transformVelocity(transform, input_ball_vel, ball_vel_in_plate_frame);
        }
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("[BallTrackingNode] transformBallState: %s", ex.what());
      }

    // how to optain control law? --> see scripts/control_plate.py
    // u = Fw - Rx
    //    F=
    //    [[0.         3.03124099]
    //     [3.03124099 0.        ]]
    //R:=
    //[[ 6.21845521e+01  7.55026005e-04 -0.00000000e+00 -0.00000000e+00
    //  -0.00000000e+00 -0.00000000e+00 -3.58060099e+00  4.59203044e+00]
    // [-0.00000000e+00 -0.00000000e+00  3.07779971e+00  3.95866474e-01
    //  -3.58060099e+00 -2.02082733e+00 -0.00000000e+00 -0.00000000e+00]]

    // Plate states x:
    if(previous_joint_state_msg_)
    {
      double alpha  = previous_joint_state_msg_->position[3]-3.1415898225691254;
      double dalpha = previous_joint_state_msg_->velocity[3];
      double beta   = previous_joint_state_msg_->position[4]- 1.5707947869759256;
      double dbeta  = previous_joint_state_msg_->velocity[4];
    }

    // Ball states:
    double ball_x  = ball_pos_in_plate_frame.x;
    double dball_x = ball_vel_in_plate_frame.x;
    double ball_y  = ball_pos_in_plate_frame.y;
    double dball_y = ball_vel_in_plate_frame.y;

    // w:
    double des_ball_pos_y = 0.1;
    double des_ball_pos_x = 0.2;
  }

  void ControlNode::transformPoint(const tf::StampedTransform& transform, const geometry_msgs::Point& point_in, geometry_msgs::Point& point_out) const
  {
    tf::Point p;
    tf::pointMsgToTF(point_in, p);
    p = transform * p;
    tf::pointTFToMsg(p, point_out);
  }

  void ControlNode::transformVelocity(const tf::StampedTransform& transform, const geometry_msgs::Point& vel_in, geometry_msgs::Point& vel_out) const
  {
    tf::Point p;
    tf::pointMsgToTF(vel_in, p);

    tf::StampedTransform transform_tmp = transform;
    transform_tmp.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    p = transform_tmp * p;
    tf::pointTFToMsg(p, vel_out);
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

