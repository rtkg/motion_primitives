/**
 * @author Robert Krug
 * @date   Fri, Oct 9, 2012
 *
 */


#ifndef dof_h___
#define dof_h___

#include "ros/ros.h"
#include <boost/thread/mutex.hpp>
#include "motion_primitives_msgs/JointControllerState.h"
#include <map>
#include <string>
#include <Eigen/Core>
#include <math.h>
#include "SQProblem.hpp"
#include <iostream>
#include <fstream>

namespace PrimitiveControllers
{
  // const double PI = std::acos(-1.0);

  /**
   *@brief ...
   */
  struct DMPParameters
  {
    DMPParameters();
    DMPParameters(XmlRpc::XmlRpcValue dmp_param);

    int id_;
    std::string name_;
    int nD_;
    int nBF_;
    double kappa_;
    Eigen::Vector2d ab_;
    Eigen::VectorXd pBF_;
    Eigen::MatrixXd w_;
    Eigen::Matrix2d P_;
    Eigen::VectorXd q0_ref_;

  };

  class DoF
  {
  public:

    DoF(XmlRpc::XmlRpcValue dof_config);
    ~DoF();

    void reset(int id,double goal, double Tau, double Td);
    void update(double s);
    Eigen::Vector2d getState();
    double getGoal();

  private:

    DoF(){};

    std::string joint_;
    std::string state_topic_;
    std::string command_topic_;

    boost::mutex lock_;
    std::map <int, DMPParameters*> param_map_;
    DMPParameters* param_;

    Eigen::Vector2d x_;
    Eigen::MatrixXd x_ref_;
    Eigen::MatrixXd ps_ref_;
    Eigen::VectorXd lmbd_;

    double Td_,track_tol_,goal_;
    Eigen::Matrix2d A_;
    Eigen::Vector2d B_;
    Eigen::Vector2d K_;
    Eigen::Matrix2d Ad_; 
 
    qpOASES::SQProblem qp_;

    ros::NodeHandle nh_;
    ros::Publisher  state_pub_;

#ifndef OPEN_LOOP
    ros::Subscriber state_sub_;
#endif

#ifdef DEBUG
    ros::Publisher ref_state_pub_;
#endif

    void updateParticularSolutions(double s);
    void updateLambda();
    double gaussBF(double s, double sigma, double c);

    /* std::ofstream file_x_; */
    /* std::ofstream file_x_ref_; */
    /* std::ofstream file_s_; */
    /* std::ofstream file_ps_ref_; */
    /* std::ofstream file_lmbd_; */

    /////////////////
    //  CALLBACKS  //
    /////////////////

#ifndef OPEN_LOOP
    void stateCallback(const motion_primitives_msgs::JointControllerState::ConstPtr& msg);
#endif
    
  };
}//end namespace
#endif
