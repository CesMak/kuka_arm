#include <kuka_kr5_control/plate_control_node.hpp>

//using namespace Eigen;

namespace kuka
{
  //constructor is called only once:
  ControlNode::ControlNode(ros::NodeHandle& nh)
    : alpha_(0.0)
    , dalpha_(0.0)
    , beta_(0.0)
    , dbeta_(0.0)
    , ball_x_(0.0)
    , dball_x_(0.0)
    , ball_y_(0.0)
    , dball_y_(0.0)
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
        //std::cout<<eigen_control_Matrix_R_<<std::endl;
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

//    std::cout<<eigen_control_Matrix_R_<<std::endl;
//    std::cout<<eigen_control_Matrix_F_<<std::endl;

    //Subscriber:
    joints_sub_ = nh.subscribe("/kuka/joint_states", 5, &ControlNode::jointsCallback, this);
    ball_state_sub_ = nh.subscribe("/gazebo/ball/odom", 5, &ControlNode::ballCallback, this);

    command13_sub_ = nh.subscribe("/kuka/control13_command", 5, &ControlNode::controlCommandCallback_13, this);
    command46_sub_ = nh.subscribe("/kuka/control46_command", 5, &ControlNode::controlCommandCallback_46, this);

    //Publisher:
    // this can be effort, velocity or position controllers as defined in the controller_node.yaml (in config folder)
    // 1-4 are position controllers:
    joint_commands_1_pub_ = nh.advertise<std_msgs::Float64>("/kuka/link_1_controller/command", 5);
    joint_commands_2_pub_ = nh.advertise<std_msgs::Float64>("/kuka/link_2_controller/command", 5);
    joint_commands_3_pub_ = nh.advertise<std_msgs::Float64>("/kuka/link_3_controller/command", 5);
    joint_commands_4_pub_ = nh.advertise<std_msgs::Float64>("/kuka/link_4_controller/command", 5);

    // caution these are effort controllers!
    joint_commands_5_pub_ = nh.advertise<std_msgs::Float64>("/kuka/link_5_controller/command", 5);
    joint_commands_6_pub_ = nh.advertise<std_msgs::Float64>("/kuka/link_6_controller/command", 5);

    //Publish rpy angles:
    // rpy_pub_ =  nh.advertise<geometry_msgs::PointStamped>("rpy_angles", 5);
    // calc_ball_odom_pub_ = nh.advertise<geometry_msgs::PointStamped>("ball_odom_", 5);

    //Publish desired torques:
   // desired_torques_pub_ = nh.advertise<geometry_msgs::Vector3>("desired_torques", 5);

    //action server:
  }

  void ControlNode::set_all_Link_Positions(double pos1, double pos2, double pos3, double pos4, double pos5, double pos6)
  {
    set_Position_of_Joint(pos1, 1);
    set_Position_of_Joint(pos2, 2);
    set_Position_of_Joint(pos3, 3);
    set_Position_of_Joint(pos4, 4);
    set_Position_of_Joint(pos5, 5);
    set_Position_of_Joint(pos6, 6);
  }

  // of joints 1 - joint 6 (as link_nubers)
  void ControlNode::set_Position_of_Joint(double des_pos_angle_rad, double link_number)
  {
    ros::Publisher link_command;
    if(link_number==1)
        link_command=joint_commands_1_pub_;
    if(link_number==2)
        link_command=joint_commands_2_pub_;
    if(link_number==3)
        link_command=joint_commands_3_pub_;
    if(link_number==4)
        link_command=joint_commands_4_pub_;
    if(link_number==5)
        link_command=joint_commands_5_pub_;
    if(link_number==6)
        link_command=joint_commands_6_pub_;

    if( (link_number == 5) || (link_number == 4) )
    {
      // these joints are torque controlled!
      link_number = link_number -1; // for link number conversion!
      double default_torque = 5;
      control_Position_of_Link_with_Torque(des_pos_angle_rad, default_torque, link_number, link_command);
    }
    else
    {
      // these joints are position controlled:
      std_msgs::Float64 pos_link_data;
      pos_link_data.data = des_pos_angle_rad;
      link_command.publish(pos_link_data);
    }
  }

  // important only link 4, 5 are torque control links
  // only for these links this method was written:
  // also note that the turning direction of the of the des_torque so the sign
  // does not matter the code always turns in a way to minimize the error between the des_angle and the current angle
  // the amount of the des_torque determines somehow how fast the joint is turned.
  // cause only link_number 3 = link 4 and link_number 4 = link 5 are torque controlled
  void ControlNode::control_Position_of_Link_with_Torque(double des_pos_angle, double des_torque, int link_number, ros::Publisher link_pub)
  {
    if(previous_joint_state_msg_)
    {
      double curr_angle = previous_joint_state_msg_->position[link_number];

          if(link_number == 3 && des_pos_angle<curr_angle)
          {
            des_torque=des_torque*-1;
          }

          if(link_number == 4 && des_pos_angle<curr_angle)
          {
            des_torque=des_torque*-1;
          }

      double error = std::abs(curr_angle-des_pos_angle);
      std_msgs::Float64 pos_link;
      //std::cout<<"pre angle "<<previous_joint_state_msg_->position[link_number]<<" des angle: "<<des_pos_angle<<" error "<<error<<std::endl;

      if(error > 0.0174533) // accuracy of 1 degree
      {
        pos_link.data = des_torque;
      }
      else
      {
        pos_link.data = -des_torque;
      }
      link_pub.publish(pos_link);
    }

  }

  void ControlNode::controlCommandCallback_13(const geometry_msgs::PointConstPtr& command13_angle_msg)
  {
    input_command13_angle_msg_ = *command13_angle_msg;
  }

  void ControlNode::controlCommandCallback_46(const geometry_msgs::PointConstPtr& command46_angle_msg)
  {
    input_command46_angle_msg_ = *command46_angle_msg;
  }

  void ControlNode::ballCallback(const nav_msgs::OdometryConstPtr& ball_state_msg)
  {
    input_ball_state_msg_ = *ball_state_msg;
  }

  void ControlNode::jointsCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg)
  {
    previous_joint_state_msg_ = joint_state_msg;
  }

  void ControlNode::joints_manual_control()
  {
      set_all_Link_Positions(input_command13_angle_msg_.x, input_command13_angle_msg_.y, input_command13_angle_msg_.z,
                             input_command46_angle_msg_.x, input_command46_angle_msg_.y, input_command46_angle_msg_.z);
  }

  void ControlNode::update()
  {
    joints_manual_control();
    //control_Position_of_Link_with_Torque(0.5, 7, 4, joint_commands_5_pub_);
    //control_Position_of_Link_with_Torque(0.5, 7, 3, joint_commands_4_pub_);


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

    // Plate states x:
    if(previous_joint_state_msg_)
    {
      double alpha_  = previous_joint_state_msg_->position[3]-3.1415898225691254;
      double dalpha_ = previous_joint_state_msg_->velocity[3];
      double beta_   = previous_joint_state_msg_->position[4]- 1.5707947869759256;
      double dbeta_  = previous_joint_state_msg_->velocity[4];
    }

    // w
    double des_ball_pos_x = 0.1;
    double des_ball_pos_y = 0.2;

    eigen_w_(0,0) = des_ball_pos_x;
    eigen_w_(1,0) = des_ball_pos_y;

    // Ball states:
    double ball_x_  = ball_pos_in_plate_frame.x;
    double dball_x_ = ball_vel_in_plate_frame.x;
    double ball_y_  = ball_pos_in_plate_frame.y;
    double dball_y_ = ball_vel_in_plate_frame.y;

    states_(0,0) = alpha_;
    states_(1,0) = dalpha_;
    states_(2,0) = beta_;
    states_(3,0) = dbeta_;
    states_(4,0) = ball_x_;
    states_(5,0) = dball_x_;
    states_(6,0) = ball_y_;
    states_(7,0) = dball_y_;

    eigen_u_ = eigen_control_Matrix_F_*eigen_w_ -eigen_control_Matrix_R_*states_;
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

