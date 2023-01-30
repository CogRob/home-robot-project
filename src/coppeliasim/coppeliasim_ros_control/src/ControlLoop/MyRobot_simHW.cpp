#include "MyRobot_simHW.h"
#include "simLib.h"

#include <string>
#include <vector>
#include <iostream>


namespace coppeliasim_ros_control
{

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  RobotSimHW::RobotSimHW() : hardware_interface::RobotHW()
  {
    ROS_DEBUG_ONCE(" initialize MyRobot hardware_interface!");
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool RobotSimHW::init( ros::NodeHandle* nh)
  {
    ROS_DEBUG_ONCE("RobotSimHW::init");
    std::string robot_description = "robot_description"; // default
    std::vector<transmission_interface::TransmissionInfo> transmissions;
    std::string urdf_string = RobotSimHW::getURDF(nh, robot_description);

    if (!RobotSimHW::parseTransmissionsFromURDF(urdf_string, transmissions))
    {
      ROS_ERROR_NAMED("RobotSimHW", "Error parsing URDF in sim_ros_control plugin, plugin not active.\n");
      return false;
    }
    else{
      ROS_DEBUG("parseTransmissionsFromURDF finished successfully!");
    }

    // check all the loaded joint 
    ROS_DEBUG_STREAM("joint_number:" <<transmissions.size() <<", joint_names: " );
    for(unsigned int j=0; j < transmissions.size(); j++)
      ROS_DEBUG_STREAM("joint " <<j << ": "<<transmissions[j].joints_[0].name_ ); 

    // Resize vectors to our DOF
    n_dof_ = transmissions.size();
    joint_names_.resize(n_dof_);
    joint_control_methods_.resize(n_dof_);

    joint_position_.resize(n_dof_);
    joint_velocity_.resize(n_dof_);
    joint_effort_.resize(n_dof_);

    joint_effort_command_.resize(n_dof_);
    joint_position_command_.resize(n_dof_);
    joint_velocity_command_.resize(n_dof_);

    
    // Initialize values for each joint
    for(unsigned int j=0; j < n_dof_; j++)
    {
      // Check that this transmission has one joint
      if(transmissions[j].joints_.size() == 0)
      {
        ROS_WARN_STREAM_NAMED("RobotSimHW","Transmission " << transmissions[j].name_
          << " has no associated joints.");
        continue;
      }
      else if(transmissions[j].joints_.size() > 1)
      {
        ROS_WARN_STREAM_NAMED("RobotSimHW","Transmission " << transmissions[j].name_
          << " has more than one joint. Currently the default robot hardware simulation "
          << " interface only supports one.");
        continue;
      }

      
      std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;
      
      if (joint_interfaces.empty() &&
          !(transmissions[j].actuators_.empty()) &&
          !(transmissions[j].actuators_[0].hardware_interfaces_.empty()))
      {
        // TODO: Deprecate HW interface specification in actuators in ROS J
        joint_interfaces = transmissions[j].actuators_[0].hardware_interfaces_;
        ROS_WARN_STREAM_NAMED("RobotSimHW", "The <hardware_interface> element of tranmission " <<
          transmissions[j].name_ << " should be nested inside the <joint> element, not <actuator>. " <<
          "The transmission will be properly loaded, but please update " <<
          "your robot model to remain compatible with future versions of the plugin.");
      }
      
      if (joint_interfaces.empty())
      {
        ROS_WARN_STREAM_NAMED("RobotSimHW", "Joint " << transmissions[j].joints_[0].name_ <<
          " of transmission " << transmissions[j].name_ << " does not specify any hardware interface. " <<
          "Not adding it to the robot hardware simulation.");
        continue;
      }
      
      else if (joint_interfaces.size() > 1)
      {
        ROS_WARN_STREAM_NAMED("RobotSimHW", "Joint " << transmissions[j].joints_[0].name_ <<
          " of transmission " << transmissions[j].name_ << " specifies multiple hardware interfaces. " <<
          "Currently the default robot hardware simulation interface only supports one. Using the first entry!");
        //continue;
      }


      // Add data from transmission
      joint_names_[j] = transmissions[j].joints_[0].name_;
      joint_position_[j] = 0.0;
      joint_velocity_[j] = 0.0;
      joint_effort_[j]   = 0.0;  // N/m for continuous joints
      joint_effort_command_[j] = 0.0;
      joint_position_command_[j] = 0.0;
      joint_velocity_command_[j] = 0.0;

      const std::string& hardware_interface = joint_interfaces.front();

      // Debug
      ROS_DEBUG_STREAM_NAMED("RobotSimHW","Loading joint '" << joint_names_[j] << "' of type '" << hardware_interface << "'");

      // Create joint state interface for all joints
      js_interface_.registerHandle(hardware_interface::JointStateHandle(
          joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

      // Decide what kind of command interface this actuator/joint has
      hardware_interface::JointHandle joint_handle;
      if(hardware_interface == "hardware_interface/EffortJointInterface")
      {
        // Create effort joint interface
        joint_control_methods_[j] = EFFORT;
        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                       &joint_effort_command_[j] );
        ej_interface_.registerHandle(joint_handle);
      }

      else if(hardware_interface == "hardware_interface/PositionJointInterface")
      {
        // Create position joint interface
        joint_control_methods_[j] = POSITION;
        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                       &joint_position_command_[j]);

        float initpos;
        if (simGetJointPosition(simGetObjectHandle(joint_names_[j].c_str()), &initpos) != -1){
          ROS_DEBUG("init joint %s with value %f", joint_names_[j].c_str(), initpos);
          joint_handle.setCommand(initpos);
        }
          
        pj_interface_.registerHandle(joint_handle);
      }

      else if(hardware_interface == "hardware_interface/VelocityJointInterface")
      {
        // Create velocity joint interface
        joint_control_methods_[j] = VELOCITY;
        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                       &joint_velocity_command_[j]);
        vj_interface_.registerHandle(joint_handle);
      }

      else
      {
        ROS_FATAL_STREAM_NAMED("RobotSimHW","No matching hardware interface found for '"
          << hardware_interface );
        return false;
      }

      int simJointsHandle = -1; //joint ==> simJointsHandle
      simJointsHandle = simGetObjectHandle(joint_names_[j].c_str()); //joint ==> simJointsHandle
      ROS_DEBUG_STREAM("joint '"<< joint_names_[j]<< "' Handle number is: "<< simJointsHandle );   

      // case joint not found:
      if (simJointsHandle == -1)
        ROS_ERROR_STREAM("No handle available for '" << joint_names_[j] << "' in coppeliasim" );
      else
        sim_joints_.push_back(simJointsHandle);


      // set joint mode to toraue/force mode
      if ( simSetJointMode(sim_joints_[j], sim_jointmode_force , 0) == -1 ) // -1 mean unscuccesful operation
        ROS_DEBUG_STREAM("can not set joint mode to force/torque mode for joint '"<< joint_names_[j]  << "' " );
      

      // enable joint motor 
      if ( simSetObjectInt32Parameter(sim_joints_[j], sim_jointintparam_motor_enabled, 1) == -1  ) 
        ROS_DEBUG_STREAM("can not enable joint motor for joint '"<< joint_names_[j]  << "' " );

      // init position vs velocity vs effort mode
      switch (joint_control_methods_[j])
      {
        case POSITION:
        {
          ROS_DEBUG_STREAM("Enabling PID position control for joint '"<< joint_names_[j]  << "' " );
          
          // enable PID control loop for position control  
          if ( simSetObjectInt32Parameter(sim_joints_[j], sim_jointintparam_ctrl_enabled, 1) == -1  )
            ROS_DEBUG_STREAM("can not enable PID position contol loop for joint '"<< joint_names_[j]  << "' " );
      

          // set P, I, D values for the PID position controller  
          if ( simSetObjectFloatParameter(sim_joints_[j], sim_jointfloatparam_pid_p, 0.6) == -1 ||
               simSetObjectFloatParameter(sim_joints_[j], sim_jointfloatparam_pid_i, 0.05) == -1 ||
               simSetObjectFloatParameter(sim_joints_[j], sim_jointfloatparam_pid_d, 0.0) == -1  )
            ROS_DEBUG_STREAM("can not enable PID position contol loop for joint '"<< joint_names_[j]  << "' " );
        }
        break;


        case VELOCITY:
        {
          ROS_DEBUG_STREAM("enable velocity control mode for joint '"<< joint_names_[j]  << "' " );
          // disable pid position control loop
          if ( simSetObjectInt32Parameter(sim_joints_[j], sim_jointintparam_ctrl_enabled, 0) == -1  )
            ROS_DEBUG_STREAM("can not enable velocity mode for joint '"<< joint_names_[j]  << "' " );
        }
        break;


        case EFFORT:
        {
          ROS_DEBUG_STREAM("enable effort control mode for joint '"<< joint_names_[j]  << "' " );
          // disable pid position control loop
          if ( simSetObjectInt32Parameter(sim_joints_[j], sim_jointintparam_ctrl_enabled, 0) == -1  )
            ROS_DEBUG_STREAM("can not enable effor mode for joint '"<< joint_names_[j]  << "' " );

          // set velocity to target velocity to zero
          if ( simSetJointTargetVelocity(sim_joints_[j], 0.0) == -1 )
            ROS_DEBUG_STREAM("can not reset velocity for effor mode for joint '"<< joint_names_[j]  << "' " );

          // lock joint for 0 target vel 
          if ( simSetObjectInt32Parameter(sim_joints_[j], sim_jointintparam_velocity_lock , 1) == -1  )
            ROS_DEBUG_STREAM("can not enable joint_lock mode for effor mode for joint '"<< joint_names_[j]  << "' " );
            
          // set max applied joint force to zero
          if ( simSetJointForce(sim_joints_[j], 0.0) == -1 )
            ROS_DEBUG_STREAM("can not reset force/torque for effor mode for joint '"<< joint_names_[j]  << "' " );  
        }
        break;
      } // end of switch
    } //end of for_loop j till size of transmission

    // Register all available interfaces
    registerInterface(&js_interface_);
    registerInterface(&ej_interface_);
    registerInterface(&pj_interface_);
    registerInterface(&vj_interface_);

    return true;
  } // end of init


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool RobotSimHW::read()
  {
    ROS_DEBUG_ONCE("RobotSimHW::read");
    for(unsigned int j=0; j < n_dof_; j++)
    {
      float pos, vel, eff;
      if (simGetJointPosition(sim_joints_[j], &pos) == -1 ||
          simGetObjectFloatParameter(sim_joints_[j], sim_jointfloatparam_velocity, &vel) == -1 || // Velocity.
          simGetJointForce(sim_joints_[j], &eff) == -1)
        ROS_ERROR_STREAM("RobotSimHW not able to get state for '" << joint_names_[j] << "'." );
         
      joint_position_[j] = pos;
      joint_velocity_[j] = vel;
      joint_effort_[j] = eff;

    }

    return true;
  } //end of read

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool RobotSimHW::write()
  {
    ROS_DEBUG_ONCE("RobotSimHW::write");
    // std::string joint_data("");
    // for (std::vector<double>::const_iterator i = joint_position_command_.begin(); i != joint_position_command_.end(); ++i)
    //   joint_data = joint_data + std::to_string(*i) + ' ';
    // ROS_DEBUG("RobotSimHW::write joint %s", joint_data.c_str());

    for(unsigned int j=0; j < n_dof_; j++)
    {
      switch (joint_control_methods_[j])
      {
        case POSITION:
       {
          if (simSetJointTargetPosition(sim_joints_[j], joint_position_command_[j]) == -1)
            ROS_DEBUG_STREAM_ONCE("sim_ros_control not able to update position command for '" << joint_names_[j] << "' " );
        }
        break;


        case VELOCITY:
        {
          if (simSetJointTargetVelocity(sim_joints_[j], joint_velocity_command_[j]) == -1)
            ROS_DEBUG_STREAM("sim_ros_control not able to update velocity command for '" << joint_names_[j] << "' " );
        }
        break;


        case EFFORT:
        {
          ROS_DEBUG_STREAM_ONCE("EFFORT mode'" << joint_names_[j] << "' " );              
          ROS_DEBUG_STREAM("EFFORT mode joint:'" << joint_names_[j] << "', effort_value= " << joint_effort_command_[j]);              
          if (joint_effort_command_[j] > 0.0000)             
            if (simSetJointTargetVelocity(sim_joints_[j], 10000.0) == -1)
              ROS_DEBUG_STREAM_ONCE("sim_ros_control not able to set max_velcoity for '" << joint_names_[j] << "' " ); 
          else
            if (simSetJointTargetVelocity(sim_joints_[j], -10000.0) == -1)
              ROS_DEBUG_STREAM_ONCE("sim_ros_control not able to set max_velcoity for '" << joint_names_[j] << "' " );     

          if (simSetJointForce(sim_joints_[j], joint_effort_command_[j]) == -1)
            ROS_DEBUG_STREAM_ONCE("sim_ros_control not able to set force/torque for '" << joint_names_[j] <<"' ");
        }
        break;
      } //en of switch
    } //end of for_loop
    return true;
  } //end of write



  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Get the URDF XML from the parameter server
  std::string RobotSimHW::getURDF(ros::NodeHandle* nh, std::string param_name) 
  {
    std::string urdf_string, result;
    // search and wait for robot_description on param server
    while (urdf_string.empty())
    {
      if (nh->searchParam(param_name, urdf_string)) {
        if(!nh->getParam(urdf_string, result)) 
          ROS_FATAL("THIS SHOULD NEVER BE PRINTED ... ?! (%s)", urdf_string.c_str());
        else{
          ROS_DEBUG_ONCE("'%s' has been loaded", param_name.c_str());
        }
          
      }
      else
        ROS_DEBUG_ONCE("waiting '%s' to be loaded on the ROS parameter server", param_name.c_str());
    }
    return result;
  }


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Get Transmissions from the URDF
  bool RobotSimHW::parseTransmissionsFromURDF(const std::string& urdf_string, 
                        std::vector<transmission_interface::TransmissionInfo> &transmissions_)
  {
    transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);
    return true;
  }

  
} // namespace MR.
