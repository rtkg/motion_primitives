/**
 * @author Robert Krug
 * @date   Fri, Oct 9, 2012
 *
 */

#include "coordinate_motions/coordinate_motions.h"
#include <stdio.h>
#include <Eigen/Core>
#include <math.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/find_iterator.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <motion_primitives_msgs/TriggerMovement.h>

namespace CoordinateMotions
{ 

  CoordinateMotions::CoordinateMotions() : nh_private_("~"), start_conf_(NULL), approach_conf_(NULL), end_conf_(NULL)
  {

  hand_joints_.resize(18);

  hand_joints_[0] = "THJ1";
  hand_joints_[1] = "THJ2";
  hand_joints_[2] = "THJ3";
  hand_joints_[3] = "THJ4";
  hand_joints_[4] = "THJ5";
  hand_joints_[5] = "FFJ0";
  hand_joints_[6] = "FFJ3";
  hand_joints_[7] = "FFJ4";
  hand_joints_[8] = "MFJ0";
  hand_joints_[9] = "MFJ3";
  hand_joints_[10] = "MFJ4";
  hand_joints_[11] = "RFJ0";
  hand_joints_[12] = "RFJ3";
  hand_joints_[13] = "RFJ4";
  hand_joints_[14] = "LFJ0";
  hand_joints_[15] = "LFJ3";
  hand_joints_[16] = "LFJ4";
  hand_joints_[17] = "LFJ5";

    std::string searched_param;
    if(!nh_private_.searchParam("data_dir", searched_param))
    {
      ROS_ERROR("No grasp data directory specified on the parameter server - cannot start the coordinate_motions node.");
      ROS_BREAK();
    }     
    nh_private_.param(searched_param, data_dir_, std::string());
    ROS_INFO("Grasp data directory set to: %s",data_dir_.c_str()); 

    traj_client_ = new  actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction >(TRAJ_ACTION_SERVER);

    //  wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0)))
      ROS_INFO("Waiting for the joint_trajectory_action server");

    joint_states_sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_states", 1, &CoordinateMotions::listenJointStatesCB, this);
    actionlib_feedback_sub_ = nh_.subscribe<control_msgs::FollowJointTrajectoryActionFeedback>(TRAJ_ACTION_SERVER+"/feedback", 1, &CoordinateMotions::listenActionlibFeedbackCB, this);
    set_conf_srv_ = nh_.advertiseService("set_configuration",&CoordinateMotions::setConfigurationCB,this);
    trigger_traj_srv_ = nh_.advertiseService("trigger_trajectory",&CoordinateMotions::triggerTrajectoryCB,this);
    grasp_motion_srv_ = nh_.advertiseService("grasp_motion",&CoordinateMotions::graspMotionCB,this);
    trigger_grasp_clt_ = nh_.serviceClient<motion_primitives_msgs::TriggerMovement>("trigger_grasp");
    trigger_grasp_clt_.waitForExistence();
  }
  //-----------------------------------------------------------------------------------------------
  CoordinateMotions::~CoordinateMotions()
  {
    delete traj_client_;
    delete start_conf_;
    delete approach_conf_;
    delete end_conf_;
  }
  //-----------------------------------------------------------------------------------------------
  void CoordinateMotions::spin()
  {

    ros::spinOnce();
  }
  //-----------------------------------------------------------------------------------------------
  bool CoordinateMotions::setConfigurationCB(coordinate_motions::SetConfiguration::Request  &req, coordinate_motions::SetConfiguration::Response &res)
  {
    res.success=false;
  
    unsigned int nJ=req.joint_names.size();
    if (nJ != req.positions.size())
      {
	ROS_ERROR("Number of joint names has to be equal to the number of given positions");
        return res.success;
      }

    if (req.positions.empty())
      {
	ROS_ERROR("Positions array is empty");
        return res.success;
      }
       
    Configuration* conf=NULL;
    lock_.lock();
    if (req.type==START)
      {
	delete start_conf_;
        start_conf_=new Configuration;
	conf=start_conf_;
      }
    else if (req.type==APPROACH)
      {
	delete approach_conf_;
	approach_conf_=new Configuration;
	conf=approach_conf_;
      }
    else if (req.type==END)
      {
	delete end_conf_;
	end_conf_=new Configuration;
	conf=end_conf_;
      }
    else 
      {
	ROS_ERROR("Invalid configuration type - cannot set configuration");
	lock_.unlock();
	return res.success;
      }

    for (unsigned int i=0; i<nJ; i++)
      {
	conf->joint_names_.push_back(req.joint_names[i]);
	conf->target_pos_.push_back(req.positions[i]);
      }
    conf->duration_=req.duration;
    // // conf->goal_tolerance_=req.goal_tolerance;
    // conf->goal_time_tolerance_=req.goal_time_tolerance;

    lock_.unlock();
  
    res.success=true;
    ROS_INFO("Configuration type %i set",req.type);
    return res.success;
  }
  //-----------------------------------------------------------------------------------------------
  bool CoordinateMotions::triggerTrajectoryCB(coordinate_motions::TriggerTrajectory::Request  &req, coordinate_motions::TriggerTrajectory::Response &res)
  {
    lock_.lock();
    res.success=triggerTrajectory(req.type);
    lock_.unlock();

    return res.success;
  }
  //-----------------------------------------------------------------------------------------------
  bool CoordinateMotions::triggerTrajectory(unsigned int type)
  {
    Configuration* conf=NULL;

    if (type==START)
      conf=start_conf_;
    else if (type==APPROACH)
      conf=approach_conf_;
    else if (type==END)
      conf=end_conf_;
    else 
      {
	ROS_ERROR("Invalid configuration type - cannot trigger the trajectory");
	return false;
      }

    if (conf==NULL)
      {
	ROS_ERROR("No trajectory configuration with given type - cannot trigger the trajectory");
	return false;
      }

    control_msgs::FollowJointTrajectoryGoal goal;

   minimumJerkTraj(goal.trajectory,conf,TRAJ_RES);
   //singlePointTraj(goal.trajectory,conf);
 
    goal.goal_time_tolerance=ros::Duration(GOAL_TIME_TOLERANCE);
    //JointTolerance[] goal_tolerance

    //    std::cout<<"time start: "<<ros::Time::now().toSec()<<std::endl;
    traj_client_->sendGoalAndWait(goal);
    //SimpleClientGoalState 	sendGoalAndWait (const Goal &goal, const ros::Duration &execute_timeout=ros::Duration(0, 0), const ros::Duration &preempt_timeout=ros::Duration(0, 0))
    //    std::cout<<"time end: "<<ros::Time::now().toSec()<<std::endl;


    ROS_INFO("Triggered trajectory of type %i.",type);
    return true;
  }
  //-----------------------------------------------------------------------------------------------
  void CoordinateMotions::minimumJerkTraj(trajectory_msgs::JointTrajectory & traj, const Configuration * const goal_conf,unsigned int resolution)
  {
    traj.header.stamp = ros::Time::now() - ros::Duration(0.00001);
    double duration=goal_conf->duration_;
    double dt=duration/resolution;
    double t=0;
   
    unsigned int nJ=goal_conf->joint_names_.size();

    //find the initial (current) positions
    Eigen::VectorXd xI(nJ);
    for (unsigned int i=0; i<nJ; i++)
      for(unsigned int j=0; j<joint_states_.name.size(); j++)
	if(std::strcmp(goal_conf->joint_names_[i].c_str(),joint_states_.name[j].c_str())==0)
          xI(i)=joint_states_.position[j];

    traj.points.resize(resolution);
    traj.joint_names.resize(nJ);
    for (unsigned int i=0; i<nJ; i++)
      {
	traj.joint_names[i]=goal_conf->joint_names_[i];
	for (unsigned int j=0; j<=resolution-1; j++)
	  {
	    t=t+dt;
            traj.points[j].positions.push_back(xI(i)+(goal_conf->target_pos_[i]-xI(i))*(10*std::pow(t/duration,3)-15*std::pow(t/duration,4)+6*std::pow(t/duration,5)));
            traj.points[j].velocities.push_back((goal_conf->target_pos_[i]-xI(i))*(30/duration*std::pow(t/duration,2)-60/duration*std::pow(t/duration,3)+30/duration*std::pow(t/duration,4)));
            traj.points[j].time_from_start = ros::Duration(t);
	  }
	t=0;
      }
  }
  //-----------------------------------------------------------------------------------------------
  void CoordinateMotions::singlePointTraj(trajectory_msgs::JointTrajectory & traj, const Configuration * const goal_conf)
  {
 traj.header.stamp = ros::Time::now() - ros::Duration(0.00001);

    unsigned int nJ=goal_conf->joint_names_.size();
    traj.points.resize(1);
    for (unsigned int i=0; i<nJ; i++)
      {
    	traj.joint_names.push_back(goal_conf->joint_names_[i]);
    	traj.points[0].positions.push_back(goal_conf->target_pos_[i]);
    	traj.points[0].velocities.push_back(0.0);
    	//std::cout<<goal.trajectory.joint_names[i]<<" "<<goal.trajectory.points[0].positions[i]<<" "<<goal.trajectory.points[0].velocities[i]<<std::endl;
    	//goal.goal_tolerance.push_back(conf->goal_tolerance_);
      }
    traj.points[0].time_from_start = ros::Duration(goal_conf->duration_);

}
  //-----------------------------------------------------------------------------------------------
  void CoordinateMotions::listenActionlibFeedbackCB(const control_msgs::FollowJointTrajectoryActionFeedback::ConstPtr& feedback)
    {
      std::cout<<"getting feedback"<<std::endl;
}
  //-----------------------------------------------------------------------------------------------
  void CoordinateMotions::listenJointStatesCB(const sensor_msgs::JointState::ConstPtr& joint_states)
{
  lock_.lock();
  joint_states_=(*joint_states);
  lock_.unlock();
}
  //-----------------------------------------------------------------------------------------------
  bool CoordinateMotions::graspMotionCB(coordinate_motions::GraspMotion::Request  &req, coordinate_motions::GraspMotion::Response &res)
  {
    //READ THE GRASP FROM FILE
    std::ifstream file;
    res.success=false;
    lock_.lock();

    if( approach_conf_==NULL)
      {
	ROS_ERROR("No approach configuration specified - cannot execute grasp motion");
	lock_.unlock();
	return res.success;
      }

    file.open((data_dir_+req.grasp_name+".txt").c_str());

    //can't find the file
    if( !file.is_open())
      {
	ROS_ERROR("Couldn't open the file %s",(data_dir_+req.grasp_name+".txt").c_str());
	lock_.unlock();
	return res.success;
      }
 
    unsigned int row_id=0;
    std::string line;
    std::vector<std::string> splitted_string;
    std::vector<std::vector<std::string> > parsed_file;
    while( !file.eof() )
      {
	getline(file, line);

	//remove leading and trailing whitespace
	line = boost::algorithm::trim_copy(line);

	//ignore empty line
	if( line.size() == 0 )
	  continue;


	boost::split(splitted_string, line, boost::is_any_of("\t #"));
	splitted_string.erase( std::remove_if(splitted_string.begin(), splitted_string.end(), boost::bind( &std::string::empty, _1 )), splitted_string.end()); 

	parsed_file.push_back(splitted_string);
	row_id++;
      }
    if( row_id > 2)
      {
	ROS_ERROR("Grasp in file %s contains more then two lines",(data_dir_+req.grasp_name+".txt").c_str());
	lock_.unlock();
	return res.success;
      }

    motion_primitives_msgs::TriggerMovement grasp_mvmt;
    grasp_mvmt.request.movement.id=req.type_id;
    grasp_mvmt.request.movement.tau=approach_conf_->duration_;


    for (unsigned int i=0; i<parsed_file[0].size(); i++)
      for (unsigned int j=0; j<hand_joints_.size(); j++)
	if (std::strcmp(parsed_file[0][i].c_str(),hand_joints_[j].c_str())==0)
	  {
	    grasp_mvmt.request.movement.target_posture.name.push_back(hand_joints_[j]);
	    grasp_mvmt.request.movement.target_posture.position.push_back(convertToDouble(parsed_file[1][i]));
	  }
    trigger_grasp_clt_.call(grasp_mvmt);

    lock_.unlock();
    res.success=true;
    return res.success;

  }
  //-----------------------------------------------------------------------------------------------
  // void CoordinateMotions::listenContactForceCB(const std_msgs::Float64::ConstPtr& ctct_force)
  // {
  //   lock_.lock();

  //   if(std::abs(ctct_force->data) > max_force_)
  //     {
  // 	stopControllers();
  // 	ROS_WARN("Maximal Contact Force exceeded - stopping primitive controllers.");
  //     } 

  //   lock_.unlock();
  // }
  // //-----------------------------------------------------------------------------------------------
  // bool CoordinateMotions::stopControllersCB(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
  // {
  //   lock_.lock();
  //   stopControllers();
  //   lock_.unlock();

  //   return true;
  // }
  // //-----------------------------------------------------------------------------------------------


}//end namespace
