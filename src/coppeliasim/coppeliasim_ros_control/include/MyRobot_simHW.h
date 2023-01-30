#pragma once
#include <ros/ros.h>

// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <hardware_interface/joint_state_interface.h>


#include <transmission_interface/transmission_parser.h>
#include <urdf/model.h>

namespace coppeliasim_ros_control
{
  /// \brief This is the hardware interface for MyRobot simulated in sim.
  class RobotSimHW : public hardware_interface::RobotHW
  {
  public:
    RobotSimHW();
    bool init(ros::NodeHandle* model_nh);
    bool read();
    bool write();

    // Get the URDF XML from the parameter server
    static std::string getURDF(ros::NodeHandle* nh, std::string param_name);

    // Get Transmissions from the URDF
    static bool parseTransmissionsFromURDF(const std::string& urdf_string,  
                    std::vector<transmission_interface::TransmissionInfo> &transmissions);

  protected:
    enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID};


    unsigned int n_dof_;
    hardware_interface::JointStateInterface    js_interface_;
    hardware_interface::EffortJointInterface   ej_interface_;
    hardware_interface::PositionJointInterface pj_interface_;
    hardware_interface::VelocityJointInterface vj_interface_;

    std::vector<int> sim_joints_;

    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;

    std::vector<double> joint_effort_command_;
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocity_command_;
    std::vector<double> last_joint_position_command_;

    std::vector<std::string> joint_names_;
    std::vector<ControlMethod> joint_control_methods_;
    std::vector<control_toolbox::Pid> pid_controllers_;
  };


} // namespace MR.
