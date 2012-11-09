/**
 * @file   srh_position_velocity_controller.cpp
 * @author Robert Krug
 * @date   Sun, Oct 21 2012
 *
 */


#ifndef _SS_MODEL_HPP_
#define _SS_MODEL_HPP_

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
// #include <motion_primitives_msgs/SaveFile.h>
// #include <motion_primitives_msgs/OpenLoopCommand.h>
// #include <pr2_controller_interface/controller.h>
// #include <boost/thread/mutex.hpp>
#include <Eigen/Core>

namespace controller
{
  class SSModel
  {
  public:

    SSModel();
   ~SSModel();

    bool init(const Eigen::MatrixXd & A,const Eigen::MatrixXd & B,const Eigen::MatrixXd & C, const Eigen::MatrixXd & D);    
    //void computeDiscreteSystem(double Td);
    void setState(const Eigen::MatrixXd & x);
    Eigen::MatrixXd getState();
    void setStateDerivative(const Eigen::MatrixXd & dx);
    Eigen::MatrixXd getStateDerivative();
    void computeControl();
    double getOrder();
    void update();
    double getOutput();
     double getControl();
     void setControl(double u);
    bool isInitialized();
    //    double getSampleTime();

  private: 

    Eigen::MatrixXd A_,B_,pinv_B_,C_,D_, x_, dx_, u_,y_;
    unsigned int order_;
    bool initialized_;
    // ros::ServiceServer ol_cmd_srv_;
    // ros::ServiceServer save_data_srv_;

    // Eigen::Matrix<double, Eigen::Dynamic, 5> data_;

    // boost::mutex lock_;

    /////////////////
    //  CALLBACKS  //
    /////////////////


// bool olCommandCB(motion_primitives_msgs::OpenLoopCommand::Request  &req, motion_primitives_msgs::OpenLoopCommand::Response &res);
// bool saveDataCB(motion_primitives_msgs::SaveFile::Request  &req, motion_primitives_msgs::SaveFile::Response &res);

  };
} // namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


#endif
