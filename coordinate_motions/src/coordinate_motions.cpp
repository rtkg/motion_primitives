/**
 * @author Robert Krug
 * @date   Fri, Oct 9, 2012
 *
 */

#include "coordinate_motions/coordinate_motions.h"
// #include <string.h>
// #include <Eigen/Core>
// #include <stdlib.h>
// #include <sys/time.h>
// #include <time.h>




//#include <trajectory_msgs/JointTrajectory.h>


//   //! Sends the command to start a given trajectory
//   void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
//   {
//     //    std::cout<<"before started traj"<<std::endl;
//     // When to start the trajectory: 1s from now
//     goal.trajectory.header.stamp = ros::Time::now();//- ros::Duration(0.01);

//     traj_client_->sendGoal(goal);
//     // std::cout<<"started traj"<<std::endl;
//   }

//   //! Wait for currently running trajectory to finish
//   void waitTrajectory() {
//     while(!getState().isDone() && ros::ok()) { usleep(50000); }
//   }

//   //! Generates a simple trajectory to move the arm.
//   /*! Note that this trajectory contains three waypoints, joined together
//     as a single trajectory. Alternatively, each of these waypoints could
//     be in its own trajectory - a trajectory can have one or more waypoints
//     depending on the desired application.
//   */
//   control_msgs::FollowJointTrajectoryGoal arm_movement()
//   {
//     //our goal variable
//     control_msgs::FollowJointTrajectoryGoal goal;

//     // First, the joint names, which apply to all waypoints
//     goal.trajectory.joint_names.push_back("ShoulderJRotate");
//     goal.trajectory.joint_names.push_back("ShoulderJSwing");
//     goal.trajectory.joint_names.push_back("ElbowJSwing");
//     goal.trajectory.joint_names.push_back("ElbowJRotate");
//     goal.trajectory.joint_names.push_back("WRJ2");
//     goal.trajectory.joint_names.push_back("WRJ1");

//     // Set number of waypoints in this goal trajectory
//     goal.trajectory.points.resize(3);

//     // First trajectory point
//     // Positions
//     int ind = 0;
//     goal.trajectory.points[ind].positions.resize(6);
//     goal.trajectory.points[ind].positions[0] = 0.0;
//     goal.trajectory.points[ind].positions[1] = 0.0;
//     goal.trajectory.points[ind].positions[2] = 1.57;
//     goal.trajectory.points[ind].positions[3] = -0.78;
//     goal.trajectory.points[ind].positions[4] = 0.0;
//     goal.trajectory.points[ind].positions[5] = 0.0;

//     // Points also have velocities
//     goal.trajectory.points[ind].velocities.resize(6);
//     goal.trajectory.points[ind].velocities[0] = 0.0;
//     goal.trajectory.points[ind].velocities[1] = 0.0;
//     goal.trajectory.points[ind].velocities[2] = 0.0;
//     goal.trajectory.points[ind].velocities[3] = 0.0;
//     goal.trajectory.points[ind].velocities[4] = 0.0;
//     goal.trajectory.points[ind].velocities[5] = 0.0;

//     // To be reached 4.0 second after starting along the trajectory
//     goal.trajectory.points[ind].time_from_start = ros::Duration(10.0);

//     // 2nd trajectory point
//     ind += 1;
//     goal.trajectory.points[ind].positions.resize(6);
//     goal.trajectory.points[ind].positions[0] = 0.4;
//     goal.trajectory.points[ind].positions[1] = 0.78;
//     goal.trajectory.points[ind].positions[2] = 0.78;
//     goal.trajectory.points[ind].positions[3] = 0.78;
//     goal.trajectory.points[ind].positions[4] = 0.1;
//     goal.trajectory.points[ind].positions[5] = 0.1;

//     // Points also have velocities
//     goal.trajectory.points[ind].velocities.resize(6);
//     goal.trajectory.points[ind].velocities[0] = 0.3;
//     goal.trajectory.points[ind].velocities[1] = 0.0;
//     goal.trajectory.points[ind].velocities[2] = 0.0;
//     goal.trajectory.points[ind].velocities[3] = 0.0;
//     goal.trajectory.points[ind].velocities[4] = 0.0;
//     goal.trajectory.points[ind].velocities[5] = 0.0;

//     // To be reached 4.0 second after starting along the trajectory
//     goal.trajectory.points[ind].time_from_start = ros::Duration(16.0);
//     // 3rd trajectory point
//     ind += 1;
//     goal.trajectory.points[ind].positions.resize(6);
//     goal.trajectory.points[ind].positions[0] = 0.6;
//     goal.trajectory.points[ind].positions[1] = 0.48;
//     goal.trajectory.points[ind].positions[2] = 0.78;
//     goal.trajectory.points[ind].positions[3] = 0.78;
//     goal.trajectory.points[ind].positions[4] = 0.1;
//     goal.trajectory.points[ind].positions[5] = 0.1;

//     // Points also have velocities
//     goal.trajectory.points[ind].velocities.resize(6);
//     goal.trajectory.points[ind].velocities[0] = 0.0;
//     goal.trajectory.points[ind].velocities[1] = 0.0;
//     goal.trajectory.points[ind].velocities[2] = 0.0;
//     goal.trajectory.points[ind].velocities[3] = 0.0;
//     goal.trajectory.points[ind].velocities[4] = 0.0;
//     goal.trajectory.points[ind].velocities[5] = 0.0;

//     // To be reached 4.0 second after starting along the trajectory
//     goal.trajectory.points[ind].time_from_start = ros::Duration(20.0);
//     //    goal.trajectory.header.stamp=ros::Time::now();
//     return goal;
//   }

//   //! Returns the current state of the action
//   actionlib::SimpleClientGoalState getState()
//   {
//     return traj_client_->getState();
//   }

// };



namespace CoordinateMotions
{ 

  CoordinateMotions::CoordinateMotions() : start_conf_(NULL), approach_conf_(NULL), end_conf_(NULL)
  {

    traj_client_ = new  actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction >(TRAJ_ACTION_SERVER);

    //  wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0)))
      ROS_INFO("Waiting for the joint_trajectory_action server");


    // trigger_mvmt_srv_ = nh_.advertiseService("trigger_movement",&CoordinateMotions::triggerMovementCB,this);
    set_conf_srv_ = nh_.advertiseService("set_configuration",&CoordinateMotions::setConfigurationCB,this);
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
      conf=start_conf_;
    else if (req.type==APPROACH)
      conf=approach_conf_;
    else if (req.type==END)
      conf=end_conf_;
    else 
      {
	ROS_ERROR("Invalid configuration type - cannot set configuration");
	lock_.unlock();
	return res.success;
      }
    
    delete conf;
    conf=new Configuration;
    for (unsigned int i=0; i<nJ; i++)
      {
	conf->joint_names_.push_back(req.joint_names[i]);
	conf->target_pos_.push_back(req.positions[i]);
      }
    conf->duration_=req.duration;
    // conf->goal_tolerance_=req.goal_tolerance;
    conf->goal_time_tolerance_=req.goal_time_tolerance;

    lock_.unlock();

    res.success=true;
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

    unsigned int nJ=conf->joint_names_.size();

    //only send a single point
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.header.stamp = ros::Time::now();//- ros::Duration(0.01);
    goal.trajectory.points.resize(1);
    for (unsigned int i=0; i<nJ; i++)
      {
	goal.trajectory.joint_names.push_back(conf->joint_names_[i]);
	goal.trajectory.points[0].positions.push_back(conf->target_pos_[i]);
	goal.trajectory.points[0].velocities.push_back(0.0);
	//  goal.goal_tolerance.push_back(conf->goal_tolerance_);
      }
    goal.trajectory.points[0].time_from_start = ros::Duration(conf->duration_);
    goal.goal_time_tolerance=ros::Duration(conf->goal_time_tolerance_);
    //JointTolerance[] goal_tolerance
    //duration goal_time_tolerance

    traj_client_->sendGoal(goal);
    return true;
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
