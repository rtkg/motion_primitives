/**
 * @file   srh_position_velocity_controller.cpp
 * @author Robert Krug
 * @date   Sun, Oct 21 2012
 *
 */

#ifndef SRH_INVERSE_DYNAMICS_CONTROLLER_H
#define SRH_INVERSE_DYNAMICS_CONTROLLER_H

#include <sr_mechanism_controllers/sr_controller.hpp>
#include <sr_robot_msgs/SetPidGains.h>
#include <motion_primitives_msgs/JointControllerState.h>
#include "sensor_msgs/JointState.h"
#include <boost/thread/mutex.hpp>
//#include <deque>
//#include "tracking_controllers/system_identification.hpp"
#include "tracking_controllers/ss_model.hpp"

namespace controller
{
  class SrhInverseDynamicsController : public SrController
  {
  public:

    SrhInverseDynamicsController();
    ~SrhInverseDynamicsController();

    bool init( pr2_mechanism_model::RobotState *robot, const std::string &joint_name,
               boost::shared_ptr<control_toolbox::Pid> pid,boost::shared_ptr<control_toolbox::Pid> pid_vel,
               boost::shared_ptr<SSModel> model);
    bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

    virtual void starting();

    /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
     */
    virtual void update();

    virtual void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
    bool setGains(sr_robot_msgs::SetPidGains::Request &req, sr_robot_msgs::SetPidGains::Response &resp);

  private:
    boost::shared_ptr<control_toolbox::Pid> pid_;
    boost::shared_ptr<control_toolbox::Pid> pid_vel_; //hack an additional pid just to get the integral for velocity
    //publish our joint controller state
    boost::shared_ptr<realtime_tools::RealtimePublisher<motion_primitives_msgs::JointControllerState> > controller_state_publisher_;

    //ros::Publisher filter_pub_;

    ros::Subscriber sub_command_;
    double command_pos_, command_vel_, command_acc_;
    ros::ServiceServer serve_set_gains_;

    ///the deadband on the position demand
    double deadband;

    ///We're using an hysteresis deadband.
    sr_deadband::HysteresisDeadband<double> hysteresis_deadband_;

    ///read all the controller settings from the parameter server
    void read_parameters();

    ///smallest demand we can send to the motors
    int motor_min_force_threshold;

    double filtered_pos_, filtered_vel_;
    void updateFilters();
    // double filterCurrPos();
    // double filterCurrVel();

    boost::mutex lock_;
    std::deque<double> pos_filter_, vel_filter_;

    boost::shared_ptr<SSModel> model_;

    /////////////
    //CALLBACKS//
    /////////////

  void commandCallback(const sensor_msgs::JointState::ConstPtr& msg);

  };
} // namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


#endif
