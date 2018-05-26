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
    std::string to_frame = std::string("ee_link");
    std::string from_frame = std::string("world");
    std::cout<<"input"<<input_ball_state_msg_.pose.pose.position<<std::endl;
    geometry_msgs::Point ball_pos_in_plate_frame;

    double des_ball_pos_y = 0.1;
    double des_ball_pos_x = 0.2;
      try
      {
        if (tf_listener_.canTransform(to_frame, from_frame, ros::Time()))
        {
          tf::StampedTransform transform;
          tf_listener_.lookupTransform(to_frame, from_frame, ros::Time(), transform);
          transformPoint(transform, input_ball_state_msg_.pose.pose.position, ball_pos_in_plate_frame);
        }
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("[BallTrackingNode] transformBallState: %s", ex.what());
      }

    std::cout<<"result"<<ball_pos_in_plate_frame<<std::endl;

    /*
     * States x:
     alpha : Angle around link4
     dalpha: angular vel around link 4
     beta  : Angle around link5
     dbeta : angular vel around link5
     x     : ball x pos relative to plate
     dx    : ball vel x
     y     : ball y pos relative to plate
     dy    : ball vel y

     * Inputs u (torques of link4, link5):
     MR = M4
     MP = M5

     * non linear control model:
     [dalpha       x2
     ddalpha       f1(x, u)
     dbeta         x4
     ddbeta     =  f2(x, u)
     dx            x6
     ddx           f3(x, u)
     dy            x8
     ddy]          f4(x, u)

     * linear control model:
    dx = Ax+Bu
    y =  Cx = [x; y]
        0  1  0  0  0  0  0  0        0  0
        0  -b 0  0  0  0  -c 0        h  0
        0  0  0  1  0  0  0  0        0  0        0 0 0 0 1 0 0 0
   A =  0  0  0 -d -f  0  0  0    B = 0  i    C = 0 0 0 0 0 0 1 0
        0  0  0  0  0  0  1  0        0  0
        0  0  -a -e -g 0  0  0        0  j
        0  0  0  0  0  0  0  0        0  0
        -a -b 0  0  0  0  -g 1        k  0

    with constants:
    a = (g m*r^2)/( m r^2 + TB)
    b = (dR r) / (TP,xP)  <<TODO (I do not have a Rahmen!
    c = (g m)  / (TP,xP)
    h = (g m r)/ (TP,xP)
    d = (dP )  / (TP,yP)
    e = (dP r) / (TP,yP)
    f = (g m)  / (TP,yP)
    g = (g m r)/ (TP,yP)

    h = 1      / (TP,xP)
    i = 1      / (TP,yP)
    j = r      / (TP,yP)
    k = r      / (TP,xP)

    description of parameters:
    name            value    unit       description
    m               0.056    kg         ball mass
    g               9.81     kg/m^2     earth force
    r               0.032    m          ball radius
    dP              0        Nm s       damping Plate
    dR              0.1835   Nm s       damping Rahmen
    TB              0.000038 kg m^2     ball inertia (ixx=iyy=izz)
    TP,xP           0.0138   kg m^2     Plate inertia (ixx value in gazebo)
    TP,yP           0.0138   kg m^2     Plate inertia (iyy)

    Using these values and linearizing around x0 = [0...0], u0 = [0, 0]:

    */

    // ouptputs
    //joint_commands_1_pub_

    // control law:


  }

  void ControlNode::transformPoint(const tf::StampedTransform& transform, const geometry_msgs::Point& point_in, geometry_msgs::Point& point_out) const
  {
    tf::Point p;
    tf::pointMsgToTF(point_in, p);
    p = transform * p;
    tf::pointTFToMsg(p, point_out);
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

