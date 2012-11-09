/**
 * @author Robert Krug
 * @date   Fri, Oct 9, 2012
 *
 */

#include "primitive_controllers/primitive_controllers.h"
#include <string.h>
#include <Eigen/Core>
// #include <sys/time.h>
// #include <time.h>

namespace PrimitiveControllers
{ 

  PrimitiveControllers::PrimitiveControllers() : nh_private_("~"), Td_(0.01),output_flag_(false)
  {
    active_joints_.clear();

    std::string searched_param;
    XmlRpc::XmlRpcValue dof_config;
    XmlRpc::XmlRpcValue dmp;

    if(nh_private_.searchParam("sample_time",searched_param))
      nh_private_.getParam(searched_param, Td_);

    if (nh_private_.searchParam("dof_config", searched_param))
      {
	nh_.getParam(searched_param,dof_config);
	ROS_ASSERT(dof_config.getType() == XmlRpc::XmlRpcValue::TypeArray); 
    
	for (int32_t i = 0; i < dof_config.size(); ++i) 
	  {    
	    ROS_ASSERT(dof_config[i]["joint"].getType() == XmlRpc::XmlRpcValue::TypeString);
	    ROS_ASSERT(dof_config[i]["state_topic"].getType() == XmlRpc::XmlRpcValue::TypeString);
	    ROS_ASSERT(dof_config[i]["command_topic"].getType() == XmlRpc::XmlRpcValue::TypeString);
            ROS_ASSERT(dof_config[i]["tracking_tolerance"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	    dmp=dof_config[i]["dmp"];
	    for (int32_t j = 0; j <dmp.size();j++) 
	      {
		ROS_ASSERT(dmp[j]["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
		ROS_ASSERT(dmp[j]["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
		ROS_ASSERT(dmp[j]["nD"].getType() == XmlRpc::XmlRpcValue::TypeInt);
		ROS_ASSERT(dmp[j]["nBF"].getType() == XmlRpc::XmlRpcValue::TypeInt);
		ROS_ASSERT(dmp[j]["ab"].getType() == XmlRpc::XmlRpcValue::TypeArray); 
		ROS_ASSERT(dmp[j]["kappa"].getType() == XmlRpc::XmlRpcValue::TypeDouble);     
		for (int32_t k = 0; k <dmp[j]["ab"].size();k++)
		  ROS_ASSERT(dmp[j]["ab"][k].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		ROS_ASSERT(dmp[j]["pBF"].getType() == XmlRpc::XmlRpcValue::TypeArray);
		for (int32_t k = 0; k <dmp[j]["pBF"].size();k++)
		  ROS_ASSERT(dmp[j]["pBF"][k].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		ROS_ASSERT(dmp[j]["w"].getType() == XmlRpc::XmlRpcValue::TypeArray);
		for (int32_t k = 0; k <dmp[j]["w"].size();k++)
		  ROS_ASSERT(dmp[j]["w"][k].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		ROS_ASSERT(dmp[j]["P"].getType() == XmlRpc::XmlRpcValue::TypeArray);
		for (int32_t k = 0; k <dmp[j]["P"].size();k++)
		  ROS_ASSERT(dmp[j]["P"][k].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		ROS_ASSERT(dmp[j]["q0_ref"].getType() == XmlRpc::XmlRpcValue::TypeArray);
		for (int32_t k = 0; k <dmp[j]["q0_ref"].size();k++)
		  ROS_ASSERT(dmp[j]["q0_ref"][k].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	      }

	    dofs_[(std::string)dof_config[i]["joint"]] = new DoF(dof_config[i]);
	  }
      }
    else
      {
	ROS_ERROR("The DoF configurations are not specified - cannot start the Primitive Movement Controllers");
	exit(0);
      }

    trigger_mvmt_srv_ = nh_.advertiseService("trigger_movement",&PrimitiveControllers::triggerMovement,this);
    stop_ctrls_srv_ = nh_.advertiseService("stop_controllers",&PrimitiveControllers::stopControllers,this);
  }
  //-----------------------------------------------------------------------------------------------
  PrimitiveControllers::~PrimitiveControllers()
  {
    for (std::map<std::string,DoF*>::iterator it=dofs_.begin() ; it != dofs_.end(); it++)
      delete (*it).second;
  }
  //-----------------------------------------------------------------------------------------------
  void PrimitiveControllers::spin()
  {
    // struct timeval start, end;
    // double c_time;
    // gettimeofday(&start,0);

    if (!active_joints_.empty())
      {
	//compute the control inputs for the next time step
	for (unsigned int i=0; i<active_joints_.size(); i++)
	  dofs_[active_joints_[i]]->update(cs_.getPhaseVariable());

        //iterate the phase variable
	cs_.update(Td_);

	//generate the output if the phase variable reaches 1 (t=Tau) -> controllers are still active afterwards
        if ((!output_flag_) & cs_.isMovementFinished())
	  {
	    output_flag_=true;
	    ROS_INFO("Movement finished.");

	    Eigen::Vector2d state;
            double goal;
	    for (unsigned int i=0; i<active_joints_.size(); i++)
	      {
		state=dofs_[active_joints_[i]]->getState();
		goal=dofs_[active_joints_[i]]->getGoal();

		ROS_INFO("%s-> q: %f dq: %f e_q: %f e_dq: %f",active_joints_[i].c_str(),state(0),state(1), goal-state(0),0.0-state(1));
	      }
	  }

	// gettimeofday(&end,0);
	// c_time = end.tv_sec - start.tv_sec + 0.000001 * (end.tv_usec - start.tv_usec);
	// std::cout<<"Computation time: "<<c_time<<" s"<<std::endl;
      }

    ros::spinOnce();
  }
  //-----------------------------------------------------------------------------------------------
  bool PrimitiveControllers::stopControllers(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
  {
    lock_.lock();
    active_joints_.clear();
    output_flag_=false;
    lock_.unlock();

    ROS_INFO("Primitive controllers stopped.");
    return true;
  }
  //-----------------------------------------------------------------------------------------------
  bool PrimitiveControllers::triggerMovement(motion_primitives_msgs::TriggerMovement::Request  &req, motion_primitives_msgs::TriggerMovement::Response &res)
  {
    res.success=false;
    lock_.lock();
 
    output_flag_=false;
    for (unsigned int i=0; i<req.movement.target_posture.name.size(); i++)
      if(dofs_.find(req.movement.target_posture.name[i]) == dofs_.end())
	{
	  ROS_ERROR("%s is not a valid joint! Cannot trigger movement",req.movement.target_posture.name[i].c_str());
	  lock_.unlock();
	  return res.success;
	}
   
    cs_.reset();
    cs_.setTau(req.movement.tau);

    active_joints_=req.movement.target_posture.name;
    for (unsigned int i=0; i<active_joints_.size(); i++)
      dofs_[active_joints_[i]]->reset(req.movement.id,req.movement.target_posture.position[i],req.movement.tau,Td_);

    lock_.unlock();

    res.success=true;
    return res.success;
  }
  //-----------------------------------------------------------------------------------------------
}//end namespace
