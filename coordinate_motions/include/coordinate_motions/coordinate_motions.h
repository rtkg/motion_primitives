/**
 * @author Robert Krug
 * @date   Fri, Oct 9, 2012
 *
 */


#ifndef coordinate_motions_h___
#define coordinate_motions_h___

#include "ros/ros.h"
#include <boost/thread/mutex.hpp>
/* #include "primitive_controllers/dof.h" */
/* #include "primitive_controllers/canonical_system.h" */
/* #include <map> */
#include <vector>
 #include <string>  
#include <actionlib/client/simple_action_client.h>
/* #include <motion_primitives_msgs/TriggerMovement.h> */
/* #include <std_srvs/Empty.h> */
/* #include <std_msgs/Float64.h> */
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <coordinate_motions/SetConfiguration.h>
#include <coordinate_motions/TriggerTrajectory.h>

namespace CoordinateMotions
{

#define START    0
#define APPROACH 1
#define END      2

  std::string const TRAJ_ACTION_SERVER="/r_arm_controller/joint_trajectory_action";

struct Configuration
{
Configuration() : duration_(0.0),  goal_time_tolerance_(0.0)
  {
    joint_names_.clear();
    target_pos_.clear();
  }

  std::vector<std::string> joint_names_;
  std::vector<double> target_pos_;
  double duration_;
  //  double goal_tolerance_;
  double goal_time_tolerance_;
};

  /**
   *@brief ...
   */
  class CoordinateMotions
  {
  public:

    CoordinateMotions();
    ~CoordinateMotions();

    /**
     *@brief ...
     */
    void spin();

  private:

    ros::NodeHandle nh_;// nh_private_;
    boost::mutex lock_; 
    
   
    actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > * traj_client_;
    Configuration* start_conf_;
    Configuration* approach_conf_;
Configuration* end_conf_;
    ros::ServiceServer set_conf_srv_;
    /* std::map<std::string,DoF*> dofs_; */
    /* double Td_; */
    /* CanonicalSystem cs_; */
    /* std::vector<std::string> active_joints_; */
    /* bool output_flag_; */
    /* double max_force_; */
    /* ros::ServiceServer trigger_mvmt_srv_; */
    /* ros::ServiceServer stop_ctrls_srv_; */
    /* void stopControllers(); */
    /* ros::V_Subscriber ctct_force_subs_; */

    bool triggerTrajectory(unsigned int type);

    /* ///////////////// */
    /* //  CALLBACKS  // */
    /* ///////////////// */

    /* void listenContactForceCB(const std_msgs::Float64::ConstPtr& ctct_force); */
    /* bool triggerMovementCB(motion_primitives_msgs::TriggerMovement::Request  &req, motion_primitives_msgs::TriggerMovement::Response &res); */
    bool setConfigurationCB(coordinate_motions::SetConfiguration::Request  &req, coordinate_motions::SetConfiguration::Response &res);
 bool triggerTrajectoryCB(coordinate_motions::TriggerTrajectory::Request  &req, coordinate_motions::TriggerTrajectory::Response &res);
  };
}//end namespace
#endif
