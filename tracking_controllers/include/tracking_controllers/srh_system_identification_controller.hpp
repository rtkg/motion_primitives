/**
 * @file   srh_system_identification_controller.cpp
 * @author Robert Krug
 * @date   Sun, Oct 21 2012
 *
 */

#ifndef SRH_SYSTEM_IDENTIFICATION_CONTROLLER_H
#define SRH_SYSTEM_IDENTIFICATION_CONTROLLER_H

#include <sr_mechanism_controllers/sr_controller.hpp>
#include <sr_robot_msgs/SetMixedPositionVelocityPidGains.h>
#include <sr_robot_msgs/JointControllerState.h>
#include "sensor_msgs/JointState.h"
#include <boost/thread/mutex.hpp>
#include <deque>
#include "tracking_controllers/system_identification.hpp"

namespace controller
{
  class SrhSystemIdentificationController : public SrController
  {
  public:

    SrhSystemIdentificationController();
    ~SrhSystemIdentificationController();

    bool init( pr2_mechanism_model::RobotState *robot, const std::string &joint_name,
               boost::shared_ptr<control_toolbox::Pid> pid_position,
               boost::shared_ptr<control_toolbox::Pid> pid_velocity);

    bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

    virtual void starting();

    /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
     */
    virtual void update();

    virtual void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
    virtual void getGains_velocity(double &p, double &i, double &d, double &i_max, double &i_min);
    bool setGains(sr_robot_msgs::SetMixedPositionVelocityPidGains::Request &req, sr_robot_msgs::SetMixedPositionVelocityPidGains::Response &resp);

  private:
    boost::shared_ptr<control_toolbox::Pid> pid_controller_position_;       /**< Internal PID controller for the position loop. */
    boost::shared_ptr<control_toolbox::Pid> pid_controller_velocity_;       /**< Internal PID controller for the velocity loop. */

    //publish our joint controller state
    boost::shared_ptr<realtime_tools::RealtimePublisher<sr_robot_msgs::JointControllerState> > controller_state_publisher_;

    /// The values for the velocity demand saturation
    double max_velocity_, min_velocity_;

    ros::Subscriber sub_command_;
    ros::ServiceServer serve_set_gains_;

    ///the deadband on the position demand
    double position_deadband;

    ///We're using an hysteresis deadband.
    sr_deadband::HysteresisDeadband<double> hysteresis_deadband;

    ///read all the controller settings from the parameter server
    void read_parameters();

    ///smallest demand we can send to the motors
    int motor_min_force_threshold;


    boost::mutex lock_;

    SystemIdentification system_identification_;

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
