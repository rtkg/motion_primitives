/**
 * @file   srh_position_velocity_controller.cpp
 * @author Robert Krug
 * @date   Sun, Oct 21 2012
 *
 */


#ifndef _SYSTEM_IDENTIFICATION_HPP_
#define _SYSTEM_IDENTIFICATION_HPP_

#include <ros/node_handle.h>

// #include <pr2_controller_interface/controller.h>
// #include <control_toolbox/pid.h>
// #include <boost/scoped_ptr.hpp>
// #include <boost/thread/condition.hpp>
// #include <realtime_tools/realtime_publisher.h>
// #include <std_msgs/Float64.h>
// #include <pr2_controllers_msgs/JointControllerState.h>

// #include <utility>

// #include <sr_robot_msgs/SetPidGains.h>

// #include <sr_utilities/sr_deadband.hpp>
#include <motion_primitives_msgs/SaveFile.h>
#include <motion_primitives_msgs/OpenLoopCommand.h>
#include <pr2_controller_interface/controller.h>
#include <boost/thread/mutex.hpp>
#include <Eigen/Core>
#include <math.h>

namespace controller
{
#define STEP  1
#define SINE  2
#define SWEEP 3

  class SystemIdentification
  {
  public:

    SystemIdentification();
   ~SystemIdentification();

    bool isActive();
    void init(ros::NodeHandle &node,pr2_mechanism_model::JointState *joint_state);
    double getEffort();
    void update();
    
  private: 

    ros::NodeHandle node_;
    pr2_mechanism_model::JointState* joint_state_;
    bool active_;
    double effort_;
    double t_, t_init_,T_;
    double K_,w_0_;
    int type_;
    double magnitude_;
    unsigned int n_data_;

    ros::ServiceServer ol_cmd_srv_;
    ros::ServiceServer save_data_srv_;

    Eigen::Matrix<double, Eigen::Dynamic, 5> data_;

    boost::mutex lock_;

    /////////////////
    //  CALLBACKS  //
    /////////////////


bool olCommandCB(motion_primitives_msgs::OpenLoopCommand::Request  &req, motion_primitives_msgs::OpenLoopCommand::Response &res);
bool saveDataCB(motion_primitives_msgs::SaveFile::Request  &req, motion_primitives_msgs::SaveFile::Response &res);

  };
} // namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


#endif
