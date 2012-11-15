/**
 * @file   srh_position_velocity_controller.cpp
 * @author Robert Krug
 * @date   Sun, Oct 21 2012
 *
 */

// #include "tracking_controllers/srh_position_velocity_controller.hpp"
// #include "angles/angles.h"
// #include "pluginlib/class_list_macros.h"
// #include <sstream>
// #include <math.h>
// #include "sr_utilities/sr_math_utils.hpp"
//#include <std_msgs/Float64.h>

#include "tracking_controllers/system_identification.hpp"
#include <iostream> 
#include <fstream> 

namespace controller {

  double PI=acos(-1.0);

  //----------------------------------------------------------------------------------
  SystemIdentification::SystemIdentification() :  joint_state_(NULL), active_(false), effort_(0.0), t_(0.0), t_init_(0.0), T_(0.0), K_(0.0), w_0_(0.0), type_(0), magnitude_(0.0), n_data_(0)
  { 
  } 
  //----------------------------------------------------------------------------------
  SystemIdentification::~SystemIdentification()
  {
    ol_cmd_srv_.shutdown();
    save_data_srv_.shutdown();
  }
  //----------------------------------------------------------------------------------
  double SystemIdentification::getEffort()
  {
    return effort_;
  }
  //----------------------------------------------------------------------------------
  bool  SystemIdentification::isActive()
  {
    return active_;
  }
  //----------------------------------------------------------------------------------
  void  SystemIdentification::init(ros::NodeHandle &node,pr2_mechanism_model::JointState *joint_state)
  {
    active_=false;
    node_=node;
    joint_state_=joint_state;

    ol_cmd_srv_=node_.advertiseService("open_loop_command",&SystemIdentification::olCommandCB,this);
    save_data_srv_=node_.advertiseService("save_ol_data",&SystemIdentification::saveDataCB,this);
  }
  //----------------------------------------------------------------------------------
  bool SystemIdentification::olCommandCB(motion_primitives_msgs::OpenLoopCommand::Request  &req, motion_primitives_msgs::OpenLoopCommand::Response &res)
  {
    ROS_ASSERT(req.duration > 0);
    ROS_ASSERT(req.w_0 >= 0);
    ROS_ASSERT(req.w_f >= 0);

    lock_.lock();
    active_=true;
    effort_=0.0;
    T_=req.duration;
    type_=req.type;
    magnitude_=req.magnitude;
    K_=std::exp(std::log(req.w_f/req.w_0)/T_);
    w_0_=req.w_0;

    t_=0.0;
    t_init_=0.0;
    n_data_=0;

    data_.resize(0,5);
    lock_.unlock();
   
    ROS_INFO("Processing open loop command ...");

    return true;
  }
  //----------------------------------------------------------------------------------
  bool SystemIdentification::saveDataCB(motion_primitives_msgs::SaveFile::Request  &req, motion_primitives_msgs::SaveFile::Response &res)
  {
    res.success=false;
    lock_.lock();
    
    if (active_)
       {
	lock_.unlock();
	ROS_WARN("Open loop command in process - cannot write to file.");
	return res.success;
      }

    if (data_.rows()==0)
      {
	lock_.unlock();
	ROS_WARN("No open loop data available - cannot write to file.");
	return res.success;
      }

    std::ofstream file(req.path.c_str(),std::ios::out | std::ios::trunc);
    if (!file.is_open())
      {
	lock_.unlock();
	ROS_WARN("Could not open file %s - cannot write fo file.",req.path.c_str());
	return res.success;
      }

    for (unsigned int i=0; i<data_.rows();i++)
      file  << data_.row(i)  << '\n';

    lock_.unlock();
    file.close();
    res.success=true;

    return res.success;
  }
  //----------------------------------------------------------------------------------
  void SystemIdentification::update()
  {
    if (n_data_==0)
      t_init_= ros::Time::now().toSec();
    else
      t_=ros::Time::now().toSec()-t_init_;
     
    if (t_ > T_)
      {
        active_=false;
        ROS_INFO("Open loop command finished.");
        return;
      }

    //store data
    data_.conservativeResize(n_data_+1,5);
    data_(n_data_,0)=t_; 
    data_(n_data_,1)=joint_state_->position_; 
    data_(n_data_,2)=joint_state_->velocity_;
    data_(n_data_,4)=effort_;

    //acceleration is obtained by numerically differentiating the velocities
    double dt=0.0;
    if (n_data_ > 0)
      {
	dt=t_-data_(n_data_-1,0);  //compute last time step
	if (dt < 1e-6)
	  data_(n_data_,3)=data_(n_data_-1,3); //keep the previous acceleration if time doesnt change between steps 
	else 
	  data_(n_data_,3)=(data_(n_data_,2)-data_(n_data_-1,2))/dt;
      }
    else
      data_(n_data_,3)=0.0;

    //update the effort
    switch (type_)
      {
      case STEP:
        effort_=magnitude_;
	break;

      case SINE:
	effort_=magnitude_*sin(t_*PI/T_);
	break;

     case SWEEP:
        effort_=magnitude_*sin(2*PI*w_0_*((std::pow(K_,t_)-1)/std::log(K_)));
	break;

      default:
	ROS_ERROR("Invalid open loop command type: %d. Cannot execute the command", type_);
	active_=false;
        return;
      }

    n_data_++;
  }
  //----------------------------------------------------------------------------------
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


