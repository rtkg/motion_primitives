/**
 * @file   srh_position_velocity_controller.cpp
 * @author Robert Krug
 * @date   Sun, Oct 21 2012
 *
 */

// #include "tracking_controllers/srh_position_velocity_controller.hpp"
// #include "angles/angles.h"
// #include "pluginlib/class_list_macros.h"
// #include <sstream>
// #include <math.h>
// #include "sr_utilities/sr_math_utils.hpp"
//#include <std_msgs/Float64.h>

#include <unsupported/Eigen/MatrixFunctions>
#include "tracking_controllers/ss_model.hpp"
#include <Eigen/SVD>



// #include <math.h>
// #include <iostream> 
// #include <fstream> 

namespace controller {

  //----------------------------------------------------------------------------------
  SSModel::SSModel() : order_(0), initialized_(false)
  { 

  } 
  //----------------------------------------------------------------------------------
  SSModel::~SSModel()
  {
  }
  //----------------------------------------------------------------------------------
  bool SSModel::init(const Eigen::MatrixXd & A,const Eigen::MatrixXd & B,const Eigen::MatrixXd & C,const Eigen::MatrixXd & D)
  {
    order_=A.cols();
    ROS_ASSERT(A.cols()==A.rows()); //make sure A is square
    ROS_ASSERT(B.rows()==order_); ROS_ASSERT(B.cols()==1); //only support single input systems
    ROS_ASSERT(C.rows()==1); ROS_ASSERT(C.cols()==order_);
    ROS_ASSERT(D.rows()==1); ROS_ASSERT(D.cols()== 1);

    //Check stability
    double s_tol=-1e-8;
    if(A.eigenvalues().real().maxCoeff() > s_tol)
      {
	ROS_ERROR("Dynamic matrix A is not stable - cannot initialize the state space model!");
	return false; 
      }

    A_=A;
    B_=B;
    C_=C;
    D_=D;
    x_.resize(order_,1); x_.setZero();
    dx_.resize(order_,1); x_.setZero();
    u_.resize(1,1); u_.setZero();
    y_.resize(1,1); y_.setZero();


    //compute pseudo-inverse of the control vector via SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(B_.cast<float>(), Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::RowVectorXf S_inv(order_); S_inv.setZero();
    double i_tol=1e-6;
    for (int i=0; i < svd.nonzeroSingularValues(); i++)
      if ((svd.singularValues())(i) > i_tol)
  	S_inv(i)=1/(svd.singularValues())(i);

    pinv_B_= (svd.matrixV()*S_inv*svd.matrixU().transpose()).cast<double>();

    initialized_=true;

    return true;
  }    
  //----------------------------------------------------------------------------------
  double SSModel::getOutput()
  {
    return y_(0,0);
  }
  //----------------------------------------------------------------------------------
  bool SSModel::isInitialized()
  {
    return initialized_;
  }
  //----------------------------------------------------------------------------------
  double SSModel::getControl()
  {
    return u_(0,0);
  }
  //----------------------------------------------------------------------------------
  void SSModel::setControl(double u)
  {
    u_(0,0)=u;
  }
  //----------------------------------------------------------------------------------
  void SSModel::update()
  {
    dx_=A_*x_+B_*u_; 
    y_=C_*x_+D_*u_;
  }
  //----------------------------------------------------------------------------------
  void SSModel::setState(const Eigen::MatrixXd & x)
  {
    x_=x;
  }
  //----------------------------------------------------------------------------------
  Eigen::MatrixXd SSModel::getState()
  {
    return x_;
  }
  //----------------------------------------------------------------------------------
  void SSModel::setStateDerivative(const Eigen::MatrixXd & dx)
  {
    dx_=dx;
  }
  //----------------------------------------------------------------------------------
  Eigen::MatrixXd SSModel::getStateDerivative()
  {
    return dx_;
  }
  //----------------------------------------------------------------------------------
  // double SSModel::getSampleTime()
  // {
  //   return Td_;
  // }
  //----------------------------------------------------------------------------------
  double SSModel::getOrder()
  {
    return order_;
  }
  //----------------------------------------------------------------------------------
  void SSModel::computeControl()
  {
    ROS_ASSERT(initialized_);

    // std::cout<<std::endl<<"A "<<A_<<std::endl;
    // std::cout<<std::endl<<"B "<<B_<<std::endl;
    // std::cout<<std::endl<<"C "<<C_<<std::endl;
    // std::cout<<std::endl<<"D "<<D_<<std::endl;
    // std::cout<<std::endl<<"pinv_B "<<pinv_B_<<std::endl;
    // std::cout<<std::endl<<"x "<<x_<<std::endl;
    // std::cout<<std::endl<<"dx "<<dx_<<std::endl;

    u_=pinv_B_*(dx_ - A_*x_);
  }
  //----------------------------------------------------------------------------------
  // void SSModel::computeDiscreteSystem(double Td)
  // {
  //   ROS_ASSERT(initialized_);

  //   Td_=Td;
  //   Ad_=(A_*Td_).exp(); //compute discrete transition matrix via matrix exponential
  //   Bd_=A_.inverse()*(Ad_-Eigen::MatrixXd::Identity(order_,order_))*B_; //discrete control vector

  //   //compute pseudo-inverse of the control vector via SVD
  //   Eigen::JacobiSVD<Eigen::MatrixXf> svd(Bd_.cast<float>(), Eigen::ComputeFullU | Eigen::ComputeFullV);
  //   Eigen::RowVectorXf S_inv(order_); S_inv.setZero();
  //   double tol=1e-6;
  //   for (int i=0; i < svd.nonzeroSingularValues(); i++)
  //     if ((svd.singularValues())(i) > tol)
  // 	S_inv(i)=1/(svd.singularValues())(i);

  //   pinv_Bd_= (svd.matrixV()*S_inv*svd.matrixU().transpose()).cast<double>();
  // }
  // ------------------------------------------------------------------------------
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/


