/**
 * @file   srh_position_velocity_controller.cpp
 * @author Robert Krug
 * @date   Sun, Oct 21 2012
 *
 */

#include "tracking_controllers/srh_inverse_dynamics_controller.hpp"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"
#include <sstream>
#include <math.h>
#include "sr_utilities/sr_math_utils.hpp"
#include <Eigen/Core>
//#include <std_msgs/Float64.h>
//#include <queue>

/// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(tracking_controllers, SrhInverseDynamicsController, controller::SrhInverseDynamicsController, pr2_controller_interface::Controller)

namespace controller {

  //----------------------------------------------------------------------------------
  SrhInverseDynamicsController::SrhInverseDynamicsController()
    : SrController(), command_pos_(0.0), command_vel_(0.0), command_acc_(0.0),
      deadband(0.05), motor_min_force_threshold(0), filtered_pos_(0.0), filtered_vel_(0.0)
  {
  }
  //----------------------------------------------------------------------------------
  SrhInverseDynamicsController::~SrhInverseDynamicsController()
  {
    sub_command_.shutdown();
  }
  //----------------------------------------------------------------------------------
  bool SrhInverseDynamicsController::init(pr2_mechanism_model::RobotState *robot, const std::string &joint_name,
					  boost::shared_ptr<control_toolbox::Pid> pid,
					  boost::shared_ptr<SSModel> model)
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
	    ROS_ERROR("SrhInverseDyanmicsController could not find joint named \"%s\"\n",
		      joint_name.c_str());
	    return false;
	  }

	joint_state_2 = robot_->getJointState(j2);
	if (!joint_state_2)
	  {
	    ROS_ERROR("SrhInverseDynamicsController could not find joint named \"%s\"\n",
		      joint_name.c_str());
	    return false;
	  }
	if (!joint_state_2->calibrated_)
	  {
	    ROS_ERROR("Joint %s not calibrated for SrhInverseDynamicsController", j2.c_str());
	    return false;
	  }
      }
    else //"normal" joints
      {
	has_j2 = false;

	joint_state_ = robot_->getJointState(joint_name);
	if (!joint_state_)
	  {
	    ROS_ERROR("SrhInverseDynamicsController could not find joint named \"%s\"\n",
		      joint_name.c_str());
	    return false;
	  }
	if (!joint_state_->calibrated_)
	  {
	    ROS_ERROR("Joint %s not calibrated for SrhInverseDynamicsController", joint_name.c_str());
	    return false;
	  }
      }
    friction_compensator = boost::shared_ptr<sr_friction_compensation::SrFrictionCompensator>(new sr_friction_compensation::SrFrictionCompensator(joint_name));

    //get the min and max value for the current joint:
    get_min_max( robot_->model_->robot_model_, joint_name );

    pid_=pid;
    model_=model;

    serve_set_gains_ = node_.advertiseService("set_gains", &SrhInverseDynamicsController::setGains, this);

    ROS_DEBUG_STREAM(" joint_state name: " << joint_state_->joint_->name);
    ROS_DEBUG_STREAM(" In Init: " << getJointName() << " This: " << this
                     << " joint_state: "<<joint_state_ );


    sub_command_ = node_.subscribe<sensor_msgs::JointState>("command", 1, &SrhInverseDynamicsController::commandCallback, this);
  

    return true;
  }
  //----------------------------------------------------------------------------------
  bool SrhInverseDynamicsController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
  {
    ROS_ASSERT(robot);
    node_ = n;

    std::string joint_name;
    if (!node_.getParam("joint", joint_name)) 
      {
	ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
	return false;
      }

    int pos_filter_window;
    if (!node_.getParam("pos_filter_window", pos_filter_window)) 
      {
	ROS_ERROR("No position filter window given (namespace: %s)", node_.getNamespace().c_str());
	return false;
      }

   int vel_filter_window;
    if (!node_.getParam("vel_filter_window", vel_filter_window)) 
      {
	ROS_ERROR("No velocity filter window given (namespace: %s)", node_.getNamespace().c_str());
	return false;
      }

    ROS_ASSERT(pos_filter_window >= 1);
    ROS_ASSERT(vel_filter_window >= 1);

    pos_filter_.resize(pos_filter_window);
    vel_filter_.resize(vel_filter_window);

    XmlRpc::XmlRpcValue model_param;
    if (!node_.getParam("model_param", model_param)) 
      {
	ROS_ERROR("No joint model parameters given (namespace: %s)", node_.getNamespace().c_str());
	return false;
      }

    ROS_ASSERT(model_param.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(model_param.size() == 9);
    for (int32_t i = 0; i < model_param.size(); ++i) 
	ROS_ASSERT(model_param[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    //initialize the state space model
    Eigen::MatrixXd A(2,2),B(2,1),C(1,2), D(1,1);
    Eigen::VectorXd g(6,1); //parameters for the Makkar friction model
    A.setZero(); B.setZero(); C.setZero(); D.setZero(); g.setZero();
   
    A(0,1)=1.0; A(1,0)=(double)model_param[0]/(double)model_param[2]*(-1.0); A(1,1)=(double)model_param[1]/(double)model_param[2]*(-1.0);
    B(1,0)=1.0/(double)model_param[2];
    C(0,0)=1.0;

    for (int i=0; i<5; i++)
      g(i)=(double)model_param[i+3];


    boost::shared_ptr<SSModel> ss_model= boost::shared_ptr<SSModel>(new SSModel());
    if (!ss_model->init(A,B,C,D,g))
      return false;

    boost::shared_ptr<control_toolbox::Pid> pid = boost::shared_ptr<control_toolbox::Pid>( new control_toolbox::Pid() );
    if (!pid->init(ros::NodeHandle(node_, "pid")))
      return false;


    controller_state_publisher_.reset( new realtime_tools::RealtimePublisher<motion_primitives_msgs::JointControllerState>(node_, "state", 1));

    return init(robot, joint_name, pid,ss_model);
  }

  //----------------------------------------------------------------------------------
  void SrhInverseDynamicsController::starting()
  {
    if( has_j2 )
	command_pos_ = joint_state_->position_ + joint_state_2->position_;
    else
	command_pos_ = joint_state_->position_;

    command_vel_=0.0;
    command_acc_=0.0;

    //initialize the filters for the measured position/velocity with the current values
    for (unsigned int i=0;i<pos_filter_.size();i++)
      pos_filter_[i]=command_pos_;

    for (unsigned int i=0;i<vel_filter_.size();i++)
      vel_filter_[i]=command_vel_;
  

    pid_->reset();
    read_parameters();
    ROS_WARN("Reseting PID");
    last_time_ = robot_->getTime();
  }
  //----------------------------------------------------------------------------------
  bool SrhInverseDynamicsController::setGains(sr_robot_msgs::SetPidGains::Request &req,sr_robot_msgs::SetPidGains::Response &resp)
  {
    ROS_INFO_STREAM("New parameters: " << "PID: [" <<req.p << ", " <<req.i << ", " <<req.d << ", " <<req.i_clamp << "] ");

    pid_->setGains(req.p,req.i,req.d,req.i_clamp,-req.i_clamp);
    max_force_demand = req.max_force;
    friction_deadband = req.friction_deadband;
    deadband = req.deadband;

    return true;
  }
  //----------------------------------------------------------------------------------
  void SrhInverseDynamicsController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
  {
    pid_->getGains(p,i,d,i_max,i_min);
  }
  //----------------------------------------------------------------------------------
  void SrhInverseDynamicsController::update()
  {
    if( !has_j2)
	if (!joint_state_->calibrated_)
	  return;
     
    assert(robot_ != NULL);
    ros::Time time = robot_->getTime();
    assert(joint_state_->joint_);
    dt_= time - last_time_;

    //Make sure the controller is initialized properly
    if (!initialized_)
      {
	initialized_ = true;
	if( has_j2 )
	    command_pos_ = joint_state_->position_ + joint_state_2->position_;
	else
	    command_pos_ = joint_state_->position_;

	command_vel_=0.0;
	command_acc_=0.0;

	for (unsigned int i=0;i<pos_filter_.size();i++)
	  pos_filter_[i]=command_pos_;

	for (unsigned int i=0;i<vel_filter_.size();i++)
	  vel_filter_[i]=command_vel_;

      }

    //get the filtered current position and velocity
    updateFilters();

    double error_position = 0.0;
    double error_velocity = 0.0;
    double friction_offset = 0.0;
    double commanded_effort = 0.0;
    double commanded_acceleration = 0.0;

    //Clamps the  commands to the correct range.
    command_pos_ = clamp_command( command_pos_ );

    //compute the errors
    error_position=command_pos_-filtered_pos_;
    error_velocity=command_vel_-filtered_vel_;

    bool in_deadband = hysteresis_deadband.is_in_deadband(command_pos_, error_position, deadband);

    if( in_deadband ) //consider the error as 0 if we're in the deadband
      {
	error_position = 0.0;
	pid_->reset();
      }

    //OUTER PID LOOP -> generates a reference acceleration
    commanded_acceleration = command_acc_+pid_->updatePid(error_position,error_velocity, dt_);

    //INNER INVERSE DYNAMCIS LOOP
    Eigen::VectorXd x(2,1),dx(2,1);
    x << filtered_pos_, filtered_vel_;
    dx << filtered_vel_, commanded_acceleration;

    model_->setState(x);
    model_->setStateDerivative(dx);
    model_->computeControl();
    commanded_effort=model_->getControl();

    //Friction compensation, only if we're not in the deadband.
    if( !in_deadband )
      friction_offset = friction_compensator->friction_compensation(filtered_pos_ , filtered_vel_, int(commanded_effort), friction_deadband );

   commanded_effort += friction_offset;

    //clamp the result to max force
    commanded_effort = std::min( commanded_effort, max_force_demand );
    commanded_effort = std::max( commanded_effort, -max_force_demand );

    //if the demand is too small to be executed by the motor, then we ask for a force
    // of 0
    if( fabs(commanded_effort) <= motor_min_force_threshold )
      commanded_effort = 0.0;

    //SET THE COMPUTED EFFORT
    joint_state_->commanded_effort_ = commanded_effort;

    if(loop_count_ % 10 == 0)
      {
	if(controller_state_publisher_ && controller_state_publisher_->trylock())
	  {
	    controller_state_publisher_->msg_.header.stamp = ros::Time::now();//time;
	    controller_state_publisher_->msg_.set_point = command_pos_;
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

	    controller_state_publisher_->msg_.process_value_filtered = filtered_pos_;
	    controller_state_publisher_->msg_.process_value_dot_filtered = filtered_vel_;

	    controller_state_publisher_->msg_.commanded_velocity = command_vel_;

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

	    controller_state_publisher_->unlockAndPublish();
	  }
      }
    loop_count_++;

    last_time_ = time;
  }
  //----------------------------------------------------------------------------------
  void SrhInverseDynamicsController::read_parameters()
  {
    node_.param<double>("pid/deadband", deadband, 0.015);
    node_.param<int>("pid/friction_deadband", friction_deadband, 5);
    node_.param<double>("pid/max_force", max_force_demand, 1023.0);
    node_.param<int>("motor_min_force_threshold", motor_min_force_threshold, 0);
  }
  //----------------------------------------------------------------------------------
  void SrhInverseDynamicsController::commandCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    lock_.lock();
    command_pos_=msg->position[0];
    command_vel_ = msg->velocity[0];
    command_acc_ = msg->effort[0]; //abuse the effort field for acceleration
    lock_.unlock();
  }
  //----------------------------------------------------------------------------------
  void SrhInverseDynamicsController::updateFilters()
  {
    pos_filter_.pop_front();
    vel_filter_.pop_front();

    if( has_j2 )
      {
	pos_filter_.push_back(joint_state_->position_ + joint_state_2->position_);
	vel_filter_.push_back(joint_state_->velocity_ + joint_state_2->velocity_);
      }
    else
      {
	pos_filter_.push_back(joint_state_->position_);
	vel_filter_.push_back(joint_state_->velocity_);
      }


 double filtered_pos=0.0;
 for (unsigned int i=0;i<pos_filter_.size();i++)
     filtered_pos+=pos_filter_[i];

  double filtered_vel=0.0;
    for (unsigned int i=0;i<vel_filter_.size();i++)
      filtered_vel+=vel_filter_[i];

    filtered_pos_=filtered_pos/pos_filter_.size();
    filtered_vel_=filtered_vel/vel_filter_.size();

  }
  // //----------------------------------------------------------------------------------
  // double SrhInverseDynamicsController::filterCurrPos()
  // {
  //   double filtered_pos_=0.0;
  //   for (unsigned int i=0;i<pos_filter_.size();i++)
  //     filtered_pos_+=pos_filter_[i];




  //   return filtered_pos_/pos_filter_.size();
  // }

  // //----------------------------------------------------------------------------------
  // double SrhInverseDynamicsController::filterCurrVel()
  // {
  //   double filtered_vel_=0.0;
  //   for (unsigned int i=0;i<vel_filter_.size();i++)
  //     filtered_vel_+=vel_filter_[i];

  //   return filtered_vel_/vel_filter_.size();
  // }
  //----------------------------------------------------------------------------------
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/


