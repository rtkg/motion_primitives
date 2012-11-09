/**
 * @file   srh_position_velocity_controller.cpp
 * @author Robert Krug
 * @date   Sun, Oct 21 2012
 *
 */


#include "tracking_controllers/srh_system_identification_controller.hpp"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"
#include <sstream>
#include <math.h>
#include "sr_utilities/sr_math_utils.hpp"
#include <std_msgs/Float64.h>

PLUGINLIB_DECLARE_CLASS(tracking_controllers, SrhSystemIdentificationController, controller::SrhSystemIdentificationController, pr2_controller_interface::Controller)

using namespace std;

namespace controller {


  SrhSystemIdentificationController::SrhSystemIdentificationController()
    : SrController(), max_velocity_(1.0), min_velocity_(-1.0),
      position_deadband(0.05), motor_min_force_threshold(0)
  {
  }

  SrhSystemIdentificationController::~SrhSystemIdentificationController()
  {
    sub_command_.shutdown();
  }

  bool SrhSystemIdentificationController::init(pr2_mechanism_model::RobotState *robot, const std::string &joint_name,
                                                     boost::shared_ptr<control_toolbox::Pid> pid_position,
                                                     boost::shared_ptr<control_toolbox::Pid> pid_velocity)
  {
    ROS_DEBUG(" --------- ");
    ROS_DEBUG_STREAM("Init: " << joint_name);

    assert(robot);
    robot_ = robot;
    last_time_ = robot->getTime();

    //joint 0s
    if( joint_name.substr(3,1).compare("0") == 0)
      {
	has_j2 = true;
	std::string j1 = joint_name.substr(0,3) + "1";
	std::string j2 = joint_name.substr(0,3) + "2";
	ROS_DEBUG_STREAM("Joint 0: " << j1 << " " << j2);

	joint_state_ = robot_->getJointState(j1);
	if (!joint_state_)
	  {
	    ROS_ERROR("SrhMixedPositionVelocityController could not find joint named \"%s\"\n",
		      joint_name.c_str());
	    return false;
	  }

	joint_state_2 = robot_->getJointState(j2);
	if (!joint_state_2)
	  {
	    ROS_ERROR("SrhMixedPositionVelocityController could not find joint named \"%s\"\n",
		      joint_name.c_str());
	    return false;
	  }
	if (!joint_state_2->calibrated_)
	  {
	    ROS_ERROR("Joint %s not calibrated for SrhSystemIdentificationController", j2.c_str());
	    return false;
	  }
      }
    else //"normal" joints
      {
	has_j2 = false;

	joint_state_ = robot_->getJointState(joint_name);
	if (!joint_state_)
	  {
	    ROS_ERROR("SrhMixedPositionVelocityController could not find joint named \"%s\"\n",
		      joint_name.c_str());
	    return false;
	  }
	if (!joint_state_->calibrated_)
	  {
	    ROS_ERROR("Joint %s not calibrated for SrhSystemIdentificationController", joint_name.c_str());
	    return false;
	  }
      }
    friction_compensator = boost::shared_ptr<sr_friction_compensation::SrFrictionCompensator>(new sr_friction_compensation::SrFrictionCompensator(joint_name));

    //get the min and max value for the current joint:
    get_min_max( robot_->model_->robot_model_, joint_name );

    pid_controller_position_ = pid_position;
    pid_controller_velocity_ = pid_velocity;

    //initialize the system identification
    system_identification_.init(node_,joint_state_);

    serve_set_gains_ = node_.advertiseService("set_gains", &SrhSystemIdentificationController::setGains, this);

    ROS_DEBUG_STREAM(" joint_state name: " << joint_state_->joint_->name);
    ROS_DEBUG_STREAM(" In Init: " << getJointName() << " This: " << this
                     << " joint_state: "<<joint_state_ );

#ifdef DEBUG_PUBLISHER
    if( std::string("FFJ3").compare(getJointName()) == 0)
      {
	ROS_INFO("Publishing debug infor for FFJ3 mixed position/velocity controller");
	std::stringstream ss2;
	ss2 << getJointName() << "debug_velocity";
	debug_pub = n_tilde_.advertise<std_msgs::Float64>(ss2.str(), 2);
      }
#endif

    after_init();
    return true;
  }

  bool SrhSystemIdentificationController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
  {
    assert(robot);
    node_ = n;

    std::string joint_name;
    if (!node_.getParam("joint", joint_name)) {
      ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }

    boost::shared_ptr<control_toolbox::Pid> pid_position = boost::shared_ptr<control_toolbox::Pid>( new control_toolbox::Pid() );;
    if (!pid_position->init(ros::NodeHandle(node_, "position_pid")))
      return false;

    boost::shared_ptr<control_toolbox::Pid> pid_velocity = boost::shared_ptr<control_toolbox::Pid>( new control_toolbox::Pid() );;
    if (!pid_velocity->init(ros::NodeHandle(node_, "velocity_pid")))
      return false;

    controller_state_publisher_.reset(
				      new realtime_tools::RealtimePublisher<sr_robot_msgs::JointControllerState>
				      (node_, "state", 1));

    return init(robot, joint_name, pid_position, pid_velocity);
  }


  void SrhSystemIdentificationController::starting()
  {
    if( has_j2 )
      command_ = joint_state_->position_ + joint_state_2->position_;
    else
      command_ = joint_state_->position_;

    pid_controller_position_->reset();
    pid_controller_velocity_->reset();
    read_parameters();
    ROS_WARN("Reseting PID");
    last_time_ = robot_->getTime();
  }

  bool SrhSystemIdentificationController::setGains(sr_robot_msgs::SetMixedPositionVelocityPidGains::Request &req,
                                                         sr_robot_msgs::SetMixedPositionVelocityPidGains::Response &resp)
  {
    ROS_INFO_STREAM("New parameters: " << "PID pos: [" <<req.position_p << ", " <<req.position_i << ", " <<req.position_d << ", " <<req.position_i_clamp << "] PID vel: ["<< req.velocity_p << ", " <<req.velocity_i << ", " <<req.velocity_d << ", " <<req.velocity_i_clamp << "], max force: " <<req.max_force << ", friction deadband: "<< req.friction_deadband << " pos deadband: "<<req.position_deadband << " min and max vel: ["<<req.min_velocity << ", " << req.max_velocity <<"]");

    pid_controller_position_->setGains(req.position_p,req.position_i,req.position_d,req.position_i_clamp,-req.position_i_clamp);

    pid_controller_velocity_->setGains(req.velocity_p,req.velocity_i,req.velocity_d,req.velocity_i_clamp,-req.velocity_i_clamp);
    max_force_demand = req.max_force;
    friction_deadband = req.friction_deadband;
    position_deadband = req.position_deadband;

    //setting the position controller parameters
    min_velocity_ = req.min_velocity;
    max_velocity_ = req.max_velocity;

    return true;
  }

  void SrhSystemIdentificationController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
  {
    pid_controller_position_->getGains(p,i,d,i_max,i_min);
  }
  void SrhSystemIdentificationController::getGains_velocity(double &p, double &i, double &d, double &i_max, double &i_min)
  {
    pid_controller_velocity_->getGains(p,i,d,i_max,i_min);
  }

  void SrhSystemIdentificationController::update()
  {
    if( !has_j2)
      {
	if (!joint_state_->calibrated_)
	  return;
      }

    assert(robot_ != NULL);
    ros::Time time = robot_->getTime();
    assert(joint_state_->joint_);
    dt_= time - last_time_;

    if (!initialized_)
      {
	initialized_ = true;
	if( has_j2 )
	  command_ = joint_state_->position_ + joint_state_2->position_;
	else
	  command_ = joint_state_->position_;
      }

    bool in_deadband=false;
    double commanded_velocity = 0.0;
    double error_velocity = 0.0;
    double commanded_effort = 0.0;
    double error_position = 0.0;

    if (system_identification_.isActive())
      {
	commanded_effort=system_identification_.getEffort();
        system_identification_.update();

        // if( has_j2 )
	//   command_ = joint_state_->position_ + joint_state_2->position_;
	// else
	//   command_ = joint_state_->position_;
      }
    else
      {
	//Clamps the command to the correct range.
	command_ = clamp_command( command_ );

	////////////
	// POSITION
	/////
	//Compute velocity demand from position error:

	if( has_j2 )
	  {
	    error_position = command_ - (joint_state_->position_ + joint_state_2->position_);
	    ROS_DEBUG_STREAM("j0: " << joint_state_->position_ + joint_state_2->position_);
	  }
	else
	  error_position = command_ - joint_state_->position_;

	//are we in the deadband?
	in_deadband = hysteresis_deadband.is_in_deadband(command_, error_position, position_deadband);

	if( in_deadband ) //consider the error as 0 if we're in the deadband
	  {
	    error_position = 0.0;
	    pid_controller_position_->reset();
	  }

	//compute the velocity demand using the position pid loop
	commanded_velocity = pid_controller_position_->updatePid(error_position, dt_);
	//saturate the velocity demand
	commanded_velocity = max( commanded_velocity, min_velocity_ );
	commanded_velocity = min( commanded_velocity, max_velocity_ );

	////////////
	// VELOCITY
	/////

	//velocity loop:
	if( !in_deadband ) //don't compute the error if we're in the deadband
	  {
	    //we're not in the deadband, compute the error
	    if( has_j2 )
	      error_velocity = commanded_velocity - (joint_state_->velocity_ + joint_state_2->velocity_);
	    else
	      error_velocity = commanded_velocity - joint_state_->velocity_;
	  }
	commanded_effort = pid_controller_velocity_->updatePid(error_velocity, dt_);

      }

    //Friction compensation, only if we're not in the deadband.
    int friction_offset = 0;
    if( !in_deadband )
      {
	if( has_j2 )
	  friction_offset = friction_compensator->friction_compensation( joint_state_->position_ + joint_state_2->position_ , joint_state_->velocity_ + joint_state_2->velocity_, int(commanded_effort), friction_deadband );
	else
	  friction_offset = friction_compensator->friction_compensation( joint_state_->position_ , joint_state_->velocity_, int(commanded_effort), friction_deadband );

	commanded_effort += friction_offset;
      }

    //clamp the result to max force
    commanded_effort = min( commanded_effort, max_force_demand );
    commanded_effort = max( commanded_effort, -max_force_demand );

    //if the demand is too small to be executed by the motor, then we ask for a force
    // of 0
    if( fabs(commanded_effort) <= motor_min_force_threshold )
      commanded_effort = 0.0;

    if( has_j2 )
      joint_state_2->commanded_effort_ = commanded_effort;
    else
      joint_state_->commanded_effort_ = commanded_effort;

    if(loop_count_ % 10 == 0)
      {
	if(controller_state_publisher_ && controller_state_publisher_->trylock())
	  {
	    controller_state_publisher_->msg_.header.stamp = time;
	    controller_state_publisher_->msg_.set_point = command_;
	    if( has_j2 )
	      {
		controller_state_publisher_->msg_.process_value = joint_state_->position_ + joint_state_2->position_;
		controller_state_publisher_->msg_.process_value_dot = joint_state_->velocity_ + joint_state_2->velocity_;
	      }
	    else
	      {
		controller_state_publisher_->msg_.process_value = joint_state_->position_;
		controller_state_publisher_->msg_.process_value_dot = joint_state_->velocity_;
	      }
	    controller_state_publisher_->msg_.commanded_velocity = commanded_velocity;

	    controller_state_publisher_->msg_.error = error_position;
	    controller_state_publisher_->msg_.time_step = dt_.toSec();

	    controller_state_publisher_->msg_.command = commanded_effort;
	    controller_state_publisher_->msg_.measured_effort = joint_state_->measured_effort_;

	    controller_state_publisher_->msg_.friction_compensation = friction_offset;

	    double dummy;
	    getGains(controller_state_publisher_->msg_.position_p,
		     controller_state_publisher_->msg_.position_i,
		     controller_state_publisher_->msg_.position_d,
		     controller_state_publisher_->msg_.position_i_clamp,
		     dummy);

	    getGains_velocity(controller_state_publisher_->msg_.velocity_p,
			      controller_state_publisher_->msg_.velocity_i,
			      controller_state_publisher_->msg_.velocity_d,
			      controller_state_publisher_->msg_.velocity_i_clamp,
			      dummy);

	    controller_state_publisher_->unlockAndPublish();
	  }
      }
    loop_count_++;

    last_time_ = time;
  }

  void SrhSystemIdentificationController::read_parameters()
  {
    node_.param<double>("position_pid/min_velocity", min_velocity_, -1.0);
    node_.param<double>("position_pid/max_velocity", max_velocity_, 1.0);
    node_.param<double>("position_pid/position_deadband", position_deadband, 0.015);

    node_.param<int>("velocity_pid/friction_deadband", friction_deadband, 5);
    node_.param<double>("velocity_pid/max_force", max_force_demand, 1023.0);
    node_.param<int>("motor_min_force_threshold", motor_min_force_threshold, 0);
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
