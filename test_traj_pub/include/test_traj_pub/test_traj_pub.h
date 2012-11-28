/**
 * @author Robert Krug
 * @date   Fri, Oct 9, 2012
 *
 */


#ifndef test_traj_pub_h___
#define test_traj_pub_h___

#include "ros/ros.h"
#include <boost/thread/mutex.hpp> 
#include <sensor_msgs/JointState.h>
/* #include "primitive_controllers/dof.h" */
/* #include "primitive_controllers/canonical_system.h" */
/* #include <map> */
/* #include <vector>  */
/* #include <string>  */
#include <test_traj_pub/PublishTrajectory.h> 
/* #include <std_srvs/Empty.h> */

#include <Eigen/Core>
#include <iostream>
#include <sstream>
#include <string>
#include <Eigen/Core>
#include <fstream>

namespace TestTrajPub
{

  /**
   *@brief ...
   */
  class TestTrajPub
  {
  public:

    TestTrajPub();
    ~TestTrajPub();

    // bool loadTraj(std::string & const path);



     void spin();

  private:

    ros::NodeHandle nh_, nh_private_;
    boost::mutex lock_;
    unsigned int ind_;
    bool active_;
    ros::Publisher pub_;
    ros::ServiceServer pub_traj_srv_;
    Eigen::MatrixX3d traj_;
    sensor_msgs::JointState msg_;
    //std::string pub_topic_;
    /* std::map<std::string,DoF*> dofs_; */
    /* double Td_; */
    /* CanonicalSystem cs_; */
    /* std::vector<std::string> active_joints_; */
    /* bool output_flag_; */

    /* ros::ServiceServer trigger_mvmt_srv_; */
    /* ros::ServiceServer stop_ctrls_srv_; */

  inline double convertToDouble(std::string const& s)
  {
    std::istringstream i(s);
    double x;
    if (!(i >> x))
      ROS_ERROR("Bad trajectory file: %s", s.c_str());
    return x;
  }
    /////////////////
    //  CALLBACKS  //
    /////////////////

    bool publishTrajectory(test_traj_pub::PublishTrajectory::Request  &req, test_traj_pub::PublishTrajectory::Response &res); 
    /* bool stopControllers(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res); */

  };
}//end namespace
#endif
