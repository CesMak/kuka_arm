#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float64.h"
#include "ball_on_plate_gazebo/MoveArm.h"
#include "sensor_msgs/JointState.h"
#include "math.h"
#include "geometry_msgs/Point.h"

using namespace std_msgs;

//  global parameters
const int	N = 300;
double		dt;
double		current_joint_states[3];
bool 		is_trajectory_possible;

// global ros publisher objects
ros::Publisher joint_1_pub;
ros::Publisher joint_2_pub;
ros::Publisher joint_3_pub;
ros::Publisher end_effector_state_pub;

void calc_cartezian_trajectory(double CT[3][N] , Float64 desX , Float64 desY , Float64 desZ , double initial_pose_end_effector[3])
{
    double trajectory_length;
    double trajectory_dir_vec[3];
    double v_max;
    double duration = dt * N;
    double t=0; //incremental running time
    double step_size[N];
    int i=1;

    trajectory_length = sqrt(pow((desX.data - initial_pose_end_effector[0]),2) + pow((desY.data - initial_pose_end_effector[1]),2) + pow((desZ.data - initial_pose_end_effector[2]),2));

    //ROS_INFO("trajectory length: %f", trajectory_length);

    trajectory_dir_vec[0] = (desX.data - initial_pose_end_effector[0]) / trajectory_length;
    trajectory_dir_vec[1] = (desY.data - initial_pose_end_effector[1]) / trajectory_length;
    trajectory_dir_vec[2] = (desZ.data - initial_pose_end_effector[2]) / trajectory_length;

    //ROS_INFO("trajectory x direction: %f",trajectory_dir_vec[0]);
    //ROS_INFO("trajectory y direction: %f",trajectory_dir_vec[1]);
    //ROS_INFO("trajectory z direction: %f",trajectory_dir_vec[2]);

    v_max = (4 * trajectory_length)/(3 * duration);

    //ROS_INFO("max velocity: %f",v_max);

    //the trajectory divided to three different parts.
    //1st part is quarter of the duration time that have acceleration.
    //2nd part is two quarter of the duration time with steady velocity.
    //3rd part is another quarter of the duration time that have deceleration.
    step_size[0] = 2 * v_max / duration * pow(dt,2);
    t+=dt;
    while (t < duration / 4) //1st part
    {
      step_size[i] = (4 * v_max /duration) * t * dt + 2 * v_max /duration * pow(dt,2);
      i++;
      t+=dt;
    }
    while (t < 3 * duration / 4) //2nd part
    {
      step_size[i] = v_max * dt;
      i++;
      t+=dt;
    }
    while (t <= duration) //3rd part
    {
      step_size[i] = (4 * v_max - 4 / duration * v_max * t) * dt - 2 * v_max / duration * pow(dt,2);
      i++;
      t+=dt;
    }

    //calculating the cartezian trajectory
    for (i=0; i<3; i++)
    {
      CT[i][0] = step_size[0] * trajectory_dir_vec[i] + initial_pose_end_effector[i];
    }
    //ROS_INFO("cartezian step #1 : [%f , %f , %f]",CT[0][0],CT[1][0],CT[2][0]);
    ROS_INFO("");
    for (i=1; i<N ; i++)
    {
      for (int j=0; j<3 ; j++)
      {
        CT[j][i] = step_size[i] * trajectory_dir_vec[j] + CT[j][i-1];
      }
      //ROS_INFO("cartezian step #%d : [%f , %f , %f]",i+1,CT[0][i],CT[1][i],CT[2][i]);
    }
}

void calc_joint_trajectory(double CT[3][N] , double JT[3][N], double initial_pose_end_effector[3])
{
    double jacobian_inverse[3][3];
    double th1, th2, th3;
    double matrix_multiply;

    is_trajectory_possible = true;

    th1 = current_joint_states[0];
    th2 = current_joint_states[1];
    th3 = current_joint_states[2];

    //ROS_INFO("current joints state: [%f , %f , %f]",th1,th2,th3);

    //set the initial joint states into JT
    for (int i=0; i<3 ;i++)
    {
      JT[i][0] = current_joint_states[i];
    }

    for (int i=0; i<N; i++)
    {
      //calculating current inverse jacobian
      jacobian_inverse[0][0] = -(100000*sin(th1))/(48600*sin(th2 + th3) - 9000*cos(th2 + th3) + 1253*cos(th2) + 36500*sin(th2) + 6247.0);
      jacobian_inverse[0][1] = (100000*cos(th1))/(48600*sin(th2 + th3) - 9000*cos(th2 + th3) + 1253*cos(th2) + 36500*sin(th2) + 6247.0);
      jacobian_inverse[0][2] = 0.0;
      jacobian_inverse[1][0] = -(100000*(1357200*cos(th1) + 979235*cos(th1)*cos(th3) + 216331*cos(th1)*sin(th3) + 62470*cos(th1)*sin(th2)*sin(th3) + 5400*cos(th2)*sin(th1)*sin(th3) + 5400*cos(th3)*sin(th1)*sin(th2) + 1000*sin(th1)*sin(th2)*sin(th3) - 991765*cos(2*th2)*cos(th1)*cos(th3) - 148669*cos(2*th2)*cos(th1)*sin(th3) - 148669*sin(2*th2)*cos(th1)*cos(th3) + 991765*sin(2*th2)*cos(th1)*sin(th3) - 1267200*cos(2*th2)*cos(2*th3)*cos(th1) - 486000*cos(2*th2)*sin(2*th3)*cos(th1) - 486000*cos(2*th3)*sin(2*th2)*cos(th1) + 1267200*sin(2*th2)*sin(2*th3)*cos(th1) - 62470*cos(th1)*cos(th2)*cos(th3) + 337338*cos(th1)*cos(th2)*sin(th3) + 337338*cos(th1)*cos(th3)*sin(th2) - 1000*cos(th2)*cos(th3)*sin(th1)))/(45643842000*cos(th2 + 2*th3) - 35471014757*cos(th2 - th3) + 9123062955*sin(th2 - th3) + 19326801600*sin(th2 + 2*th3) + 36013140243*cos(th2 + th3) + 6669100045*sin(th2 + th3) - 49537800000*cos(th2) + 2702839514*cos(th3) + 1700571600*sin(th2) - 12234562090*sin(th3));
      jacobian_inverse[1][1] = (100000*(1000*cos(th1)*sin(th2)*sin(th3) - 979235*cos(th3)*sin(th1) - 216331*sin(th1)*sin(th3) - 1357200*sin(th1) - 337338*cos(th2)*sin(th1)*sin(th3) - 337338*cos(th3)*sin(th1)*sin(th2) - 62470*sin(th1)*sin(th2)*sin(th3) + 991765*cos(2*th2)*cos(th3)*sin(th1) + 148669*cos(2*th2)*sin(th1)*sin(th3) + 148669*sin(2*th2)*cos(th3)*sin(th1) - 991765*sin(2*th2)*sin(th1)*sin(th3) + 1267200*cos(2*th2)*cos(2*th3)*sin(th1) + 486000*cos(2*th2)*sin(2*th3)*sin(th1) + 486000*cos(2*th3)*sin(2*th2)*sin(th1) - 1267200*sin(2*th2)*sin(2*th3)*sin(th1) - 1000*cos(th1)*cos(th2)*cos(th3) + 5400*cos(th1)*cos(th2)*sin(th3) + 5400*cos(th1)*cos(th3)*sin(th2) + 62470*cos(th2)*cos(th3)*sin(th1)))/(45643842000*cos(th2 + 2*th3) - 35471014757*cos(th2 - th3) + 9123062955*sin(th2 - th3) + 19326801600*sin(th2 + 2*th3) + 36013140243*cos(th2 + th3) + 6669100045*sin(th2 + th3) - 49537800000*cos(th2) + 2702839514*cos(th3) + 1700571600*sin(th2) - 12234562090*sin(th3));
      jacobian_inverse[1][2] = -(100000*(27*cos(th2 + th3) + 5*sin(th2 + th3)))/(216331*cos(th3) - 979235*sin(th3));
      jacobian_inverse[2][0] = (500*(3776780009*cos(th1) + 15654982*cos(th1)*cos(th2) + 3525246000*cos(th1)*cos(th3) + 456031000*cos(th1)*sin(th2) + 250600*cos(th2)*sin(th1) + 778791600*cos(th1)*sin(th3) + 7300000*sin(th1)*sin(th2) - 1330679991*cos(2*th2)*cos(th1) + 91469000*sin(2*th2)*cos(th1) + 112446000*cos(th1)*sin(th2)*sin(th3) + 9720000*cos(th2)*sin(th1)*sin(th3) + 9720000*cos(th3)*sin(th1)*sin(th2) + 1800000*sin(th1)*sin(th2)*sin(th3) - 3570354000*cos(2*th2)*cos(th1)*cos(th3) - 535208400*cos(2*th2)*cos(th1)*sin(th3) - 535208400*sin(2*th2)*cos(th1)*cos(th3) + 3570354000*sin(2*th2)*cos(th1)*sin(th3) - 2280960000*cos(2*th2)*cos(2*th3)*cos(th1) - 874800000*cos(2*th2)*sin(2*th3)*cos(th1) - 874800000*cos(2*th3)*sin(2*th2)*cos(th1) + 2280960000*sin(2*th2)*sin(2*th3)*cos(th1) - 112446000*cos(th1)*cos(th2)*cos(th3) + 607208400*cos(th1)*cos(th2)*sin(th3) + 607208400*cos(th1)*cos(th3)*sin(th2) - 1800000*cos(th2)*cos(th3)*sin(th1)))/(9*(45643842000*cos(th2 + 2*th3) - 35471014757*cos(th2 - th3) + 9123062955*sin(th2 - th3) + 19326801600*sin(th2 + 2*th3) + 36013140243*cos(th2 + th3) + 6669100045*sin(th2 + th3) - 49537800000*cos(th2) + 2702839514*cos(th3) + 1700571600*sin(th2) - 12234562090*sin(th3)));
      jacobian_inverse[2][1] = -(500*(250600*cos(th1)*cos(th2) - 3776780009*sin(th1) + 7300000*cos(th1)*sin(th2) - 15654982*cos(th2)*sin(th1) - 3525246000*cos(th3)*sin(th1) - 456031000*sin(th1)*sin(th2) - 778791600*sin(th1)*sin(th3) + 1330679991*cos(2*th2)*sin(th1) - 91469000*sin(2*th2)*sin(th1) + 1800000*cos(th1)*sin(th2)*sin(th3) - 607208400*cos(th2)*sin(th1)*sin(th3) - 607208400*cos(th3)*sin(th1)*sin(th2) - 112446000*sin(th1)*sin(th2)*sin(th3) + 3570354000*cos(2*th2)*cos(th3)*sin(th1) + 535208400*cos(2*th2)*sin(th1)*sin(th3) + 535208400*sin(2*th2)*cos(th3)*sin(th1) - 3570354000*sin(2*th2)*sin(th1)*sin(th3) + 2280960000*cos(2*th2)*cos(2*th3)*sin(th1) + 874800000*cos(2*th2)*sin(2*th3)*sin(th1) + 874800000*cos(2*th3)*sin(2*th2)*sin(th1) - 2280960000*sin(2*th2)*sin(2*th3)*sin(th1) - 1800000*cos(th1)*cos(th2)*cos(th3) + 9720000*cos(th1)*cos(th2)*sin(th3) + 9720000*cos(th1)*cos(th3)*sin(th2) + 112446000*cos(th2)*cos(th3)*sin(th1)))/(9*(45643842000*cos(th2 + 2*th3) - 35471014757*cos(th2 - th3) + 9123062955*sin(th2 - th3) + 19326801600*sin(th2 + 2*th3) + 36013140243*cos(th2 + th3) + 6669100045*sin(th2 + th3) - 49537800000*cos(th2) + 2702839514*cos(th3) + 1700571600*sin(th2) - 12234562090*sin(th3)));
      jacobian_inverse[2][2] = (500*(48600*cos(th2 + th3) + 9000*sin(th2 + th3) + 36500*cos(th2) - 1253*sin(th2)))/(9*(216331*cos(th3) - 979235*sin(th3)));

      for (int j=0; j<3; j++)
      {
        matrix_multiply=0;
        if (i == 0)
        {
          for (int k=0; k<3; k++)
          {
          matrix_multiply += jacobian_inverse[j][k] * (CT[k][i] - initial_pose_end_effector[k]);
          }
        }
        else if (i > 0)
        {
          JT[j][i] = JT[j][i-1];

          for (int k=0; k<3; k++)
          {
          matrix_multiply += jacobian_inverse[j][k] * (CT[k][i] - CT[k][i-1]);
          }
        }
        JT[j][i] += matrix_multiply;
      }

      //updating th1,th2,th3 for the upcoming recalaulte of the inverse jacobian
      th1 = JT[0][i];
      th2 = JT[1][i];
      th3 = JT[2][i];

      if ((abs(th2) > 2)  ||  (abs(th3) > 2))
      {
        ROS_INFO("Sorry, the trajectory can't be execute!");
        is_trajectory_possible = false;
        break;
      }

      //ROS_INFO("joints step #%d : [%f , %f , %f]",i+1,th1,th2,th3);
    }
}

void calc_pose_end_effector_state(double cjs[3])
{
  double th1, th2, th3;

    th1 = current_joint_states[0];
    th2 = current_joint_states[1];
    th3 = current_joint_states[2];

    cjs[0] = (6247*cos(th1))/100000.0 + sin(th1)/1000.0 + (1253*cos(th1)*cos(th2))/100000.0 + (73*cos(th1)*sin(th2))/200.0 + (9*cos(th1)*sin(th2)*sin(th3))/100.0 - (9*cos(th1)*cos(th2)*cos(th3))/100.0 + (243*cos(th1)*cos(th2)*sin(th3))/500.0 + (243*cos(th1)*cos(th3)*sin(th2))/500.0;
    cjs[1] = (6247*sin(th1))/100000.0 - cos(th1)/1000.0 + (1253*cos(th2)*sin(th1))/100000.0 + (73*sin(th1)*sin(th2))/200.0 + (243*cos(th2)*sin(th1)*sin(th3))/500.0 + (243*cos(th3)*sin(th1)*sin(th2))/500.0 + (9*sin(th1)*sin(th2)*sin(th3))/100.0 - (9*cos(th2)*cos(th3)*sin(th1))/100.0;
    cjs[2] = (243*cos(th2 + th3))/500.0 + (9*sin(th2 + th3))/100.0 + (73*cos(th2))/200.0 - (1253*sin(th2))/100000.0 + 169.0/500.0;

}

void control_joint_trajectory(double JT[3][N])
{
  Float64 msg;
  double current_end_effector_pose[3];
  geometry_msgs::Point state_;
    for (int i=0; i<N; i++)
    {
      //publish command for joint_1
      msg.data = JT[0][i];
      joint_1_pub.publish(msg);

      //publish command for joint_2
      msg.data = JT[1][i];
      joint_2_pub.publish(msg);

      //publish command for joint_3
      msg.data = JT[2][i];
      joint_3_pub.publish(msg);

      //wait dt seconds
      ros::Duration(dt).sleep();

      //calc and publish end effector pose
      calc_pose_end_effector_state(current_end_effector_pose);
      state_.x = current_end_effector_pose[0];
      state_.y = current_end_effector_pose[1];
      state_.z = current_end_effector_pose[2];
      end_effector_state_pub.publish(state_);
    }
}

void init_end_effector_pose()
{
  Float64 reset;
  reset.data = 0.0;

  ROS_INFO("Initialazing end effector position");

  ros::Duration(2.0).sleep(); //waiting few seconds for the publishers set-up

  joint_1_pub.publish(reset);
  joint_2_pub.publish(reset);
  joint_3_pub.publish(reset);

  ros::Duration(3.0).sleep(); //wait for the movement to be complete


}

bool move_arm(manipulator_gazebo::MoveArm::Request  &req,
              manipulator_gazebo::MoveArm::Response &res)
{
  double cartezian_trajectory[3][N];
  double joint_trajectory[3][N];
  double initial_joint_states[3];
  double initial_pose_end_effector[3];


  Float64 desX		= req.desired_xyz[0];
  Float64 desY		= req.desired_xyz[1];
  Float64 desZ		= req.desired_xyz[2];

  dt					= req.duration.data/N;


  calc_pose_end_effector_state(initial_pose_end_effector);



  ROS_INFO("current cartezian joint states: %.4f  %.4f  %.4f",  	initial_pose_end_effector[0],
                                  initial_pose_end_effector[1],
                                  initial_pose_end_effector[2]);

  ROS_INFO("I got:");
  ROS_INFO("desired [ x y z ]          = [ %.4f %.4f %.4f ]",desX.data,desY.data,desZ.data);
  ROS_INFO("desired iteration time     = %f",dt);

  ROS_INFO("Calculating cartezian trajectory");
  calc_cartezian_trajectory( cartezian_trajectory , desX ,desY , desZ , initial_pose_end_effector);

  ROS_INFO("Calculating joint trajectory");
  calc_joint_trajectory(cartezian_trajectory , joint_trajectory , initial_pose_end_effector);

  if (is_trajectory_possible == true)
  {
  ROS_INFO("Sending joint trajectory and controlling");
    control_joint_trajectory(joint_trajectory);
    res.success.data = "True";
    return true;
  }

  else //if trajectory isn't possible
  {
  res.success.data = "False";
  return false;
  }
}

void jointCallback(const sensor_msgs::JointState &msg)
{
  current_joint_states[0] = msg.position[0];
  current_joint_states[1] = msg.position[1];
  current_joint_states[2] = msg.position[2];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_arm_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("move_arm", move_arm);
  ros::Subscriber joints_sub_ = n.subscribe("/manipulator/joint_states", 1000, jointCallback );

  joint_1_pub = n.advertise<std_msgs::Float64>("/manipulator/link_1_controller/command", 1000);
  joint_2_pub = n.advertise<std_msgs::Float64>("/manipulator/link_2_controller/command", 1000);
  joint_3_pub = n.advertise<std_msgs::Float64>("/manipulator/link_3_controller/command", 1000);
  end_effector_state_pub = n.advertise<geometry_msgs::Point>("/manipulator/end_effector_pose", 1000);

  init_end_effector_pose();

  ROS_INFO("Ready for commands:");

  ros::spin();

  return 0;
}
