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
#include <control_msgs/FollowJointTrajectoryActionFeedback.h>
#include <coordinate_motions/SetConfiguration.h>
#include <coordinate_motions/TriggerTrajectory.h>
#include <coordinate_motions/GraspMotion.h>
#include <sensor_msgs/JointState.h>

namespace CoordinateMotions
{

#define START    0
#define APPROACH 1
#define END      2

#define TRAJ_RES 100
#define GOAL_TIME_TOLERANCE 1.0

  std::string const TRAJ_ACTION_SERVER="/r_arm_controller/joint_trajectory_action";

  struct Configuration
  {
  Configuration() : duration_(0.0)
    {
      joint_names_.clear();
      target_pos_.clear();
    }

    double duration_;
    std::vector<std::string> joint_names_;
    std::vector<double> target_pos_;
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

    ros::NodeHandle nh_, nh_private_;
    boost::mutex lock_; 
    
   
    actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > * traj_client_;
    Configuration* start_conf_;
    Configuration* approach_conf_;
    Configuration* end_conf_;
    std::string data_dir_;
    std::vector<std::string> hand_joints_;
    ros::ServiceServer set_conf_srv_;
    ros::ServiceServer trigger_traj_srv_;
    ros::ServiceServer grasp_motion_srv_;
    ros::ServiceClient trigger_grasp_clt_;
    /* std::map<std::string,DoF*> dofs_; */
    /* double Td_; */
    /* CanonicalSystem cs_; */
    /* std::vector<std::string> active_joints_; */
    /* bool output_flag_; */
    /* double max_force_; */
    /* ros::ServiceServer trigger_mvmt_srv_; */
    /* ros::ServiceServer stop_ctrls_srv_; */
    /* void stopControllers(); */
    ros::Subscriber actionlib_feedback_sub_;
    ros::Subscriber joint_states_sub_;
    sensor_msgs::JointState joint_states_;
    void minimumJerkTraj(trajectory_msgs::JointTrajectory & traj, const Configuration * const goal_conf,unsigned int resolution);
    void singlePointTraj(trajectory_msgs::JointTrajectory & traj, const Configuration * const goal_conf);
    bool triggerTrajectory(unsigned int type);

 inline double convertToDouble(std::string const& s)
  {
    std::istringstream i(s);
    double x;
    if (!(i >> x))
      ROS_ERROR("Bad trajectory file: %s", s.c_str());
    return x;
  }

    /* ///////////////// */
    /* //  CALLBACKS  // */
    /* ///////////////// */

    void listenJointStatesCB(const sensor_msgs::JointState::ConstPtr& joint_states); 
    void listenActionlibFeedbackCB(const control_msgs::FollowJointTrajectoryActionFeedback::ConstPtr& feedback); 
    /* bool triggerMovementCB(motion_primitives_msgs::TriggerMovement::Request  &req, motion_primitives_msgs::TriggerMovement::Response &res); */
    bool setConfigurationCB(coordinate_motions::SetConfiguration::Request  &req, coordinate_motions::SetConfiguration::Response &res);
    bool triggerTrajectoryCB(coordinate_motions::TriggerTrajectory::Request  &req, coordinate_motions::TriggerTrajectory::Response &res);
    bool graspMotionCB(coordinate_motions::GraspMotion::Request  &req, coordinate_motions::GraspMotion::Response &res);
    //bool triggerGrasp(std::string const & serialized_model,geometry_msgs::Pose const & initial_pose);

  };
}//end namespace
#endif
