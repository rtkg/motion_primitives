/**
 * @author Robert Krug
 * @date   Fri, Oct 9, 2012
 *
 */

#include <unsupported/Eigen/MatrixFunctions>
#include "primitive_controllers/dof.h"
#include <string.h>
#include <sstream>
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include <math.h>
namespace PrimitiveControllers
{
  //-------------------------------------------------------------------------------------------------
  DMPParameters::DMPParameters() : id_(0), name_("default"), nD_(0), nBF_(0), kappa_(0.0) {} 
  //-------------------------------------------------------------------------------------------------
  DMPParameters::DMPParameters(XmlRpc::XmlRpcValue dmp_param)
  {
    id_=(int)dmp_param["id"];
    name_=(std::string)dmp_param["name"];
    nD_=(int)dmp_param["nD"];
    nBF_=(int)dmp_param["nBF"];
    kappa_=(double)dmp_param["kappa"];

    ROS_ASSERT(dmp_param["ab"].size() == 2);
    ab_ << (double)dmp_param["ab"][0], (double)dmp_param["ab"][1];

    ROS_ASSERT(dmp_param["pBF"].size() == 2*nBF_);
    pBF_.resize(2*nBF_);
    for (int i=0; i<2*nBF_; i++)
      pBF_(i)=(double)dmp_param["pBF"][i];

    ROS_ASSERT(dmp_param["w"].size() == nBF_*nD_);
    w_.resize(nBF_,nD_);
    int k=0;
    for (int j=0; j<nD_; j++)
      for (int i=0; i<nBF_; i++)
	{
	  w_(i,j)=(double)dmp_param["w"][k];
	  k++;
	}

    ROS_ASSERT(dmp_param["P"].size() == 4);
    P_ << (double)dmp_param["P"][0], (double)dmp_param["P"][1],(double)dmp_param["P"][2],(double)dmp_param["P"][3];

    ROS_ASSERT(dmp_param["q0_ref"].size() == nD_);
    q0_ref_.resize(nD_);
    for (int i=0; i<nD_; i++)
      q0_ref_(i)=(double)dmp_param["q0_ref"][i];
  }
  //-------------------------------------------------------------------------------------------------
  DoF::DoF(XmlRpc::XmlRpcValue dof_config) :  param_(NULL), Td_(0.0), track_tol_(0.0),goal_(0.0)
  {
    joint_=(std::string)dof_config["joint"];
    state_topic_=(std::string)dof_config["state_topic"];
    command_topic_=(std::string)dof_config["command_topic"];
    track_tol_=(double)dof_config["tracking_tolerance"];

    x_.setZero();
    x_ref_.setZero();
    ps_ref_.setZero();
    lmbd_.setZero();
    A_.setZero(); A_(0,1)=1;
    B_.setZero(); B_(1)=1;
    K_.setZero();
    Ad_.setZero();

    for (int i=0; i<dof_config["dmp"].size(); i++)
      param_map_[(int)dof_config["dmp"][i]["id"]]=new DMPParameters(dof_config["dmp"][i]);

#ifdef MPV_CONTROL
    state_pub_ = nh_.advertise<std_msgs::Float64>(command_topic_,1);
#else
    state_pub_ = nh_.advertise<sensor_msgs::JointState>(command_topic_,1);  
#endif

#ifndef OPEN_LOOP
#ifdef MPV_CONTROL
    //std::cout<<"subscribing to shadow_msgs"<<std::endl;
    state_sub_= nh_.subscribe<sr_robot_msgs::JointControllerState>(state_topic_, 1, &DoF::stateCallback,this);
#else
    // std::cout<<"subscribing to motion_primitives_msgs"<<std::endl;
    state_sub_= nh_.subscribe<motion_primitives_msgs::JointControllerState>(state_topic_, 1, &DoF::stateCallback,this);
#endif
#endif

#ifdef DEBUG
    ref_state_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_+"/ref",1);
#endif
  }
  //-----------------------------------------------------------------------------------------------
  DoF::~DoF() 
  {
    for (std::map<int,DMPParameters*>::iterator it=param_map_.begin() ; it != param_map_.end(); it++)
      delete (*it).second;
  }
  //-----------------------------------------------------------------------------------------------
  void DoF::reset(int id,double goal, double Tau, double Td)
  {

    // file_x_.open(("/home/rkg/Desktop/DMP/matlab/mpc_dmp/x_.txt"), std::ios::out | std::ios::trunc );
    // file_x_ref_.open(("/home/rkg/Desktop/DMP/matlab/mpc_dmp/x_ref_.txt"), std::ios::out | std::ios::trunc );
    // file_s_.open(("/home/rkg/Desktop/DMP/matlab/mpc_dmp/s_.txt"), std::ios::out | std::ios::trunc );
    // file_ps_ref_.open(("/home/rkg/Desktop/DMP/matlab/mpc_dmp/ps_ref_.txt"), std::ios::out | std::ios::trunc );
    // file_lmbd_.open(("/home/rkg/Desktop/DMP/matlab/mpc_dmp/lmbd_.txt"), std::ios::out | std::ios::trunc );
    // file_x_.close();
    // file_x_ref_.close();
    // file_s_.close();
    // file_ps_ref_.close();
    // file_lmbd_.close();

    param_=param_map_.at(id);
   
    x_(0)=x_(0)+goal_-goal; //correct the change of variable for the current state

    goal_=goal;
    Td_=Td;

    x_ref_.resize(2,param_->nD_);  x_ref_.setZero();
    x_ref_.row(0)=param_->q0_ref_;
    ps_ref_.resize(2,param_->nD_);  ps_ref_.setZero(); 
    lmbd_.resize(param_->nD_);  lmbd_.setZero();

    A_(1,0)=param_->ab_(0)/(Tau*Tau);
    A_(1,1)=param_->ab_(1)/Tau;
    B_(1)=1/(Tau*Tau);
    Ad_=(A_*Td).exp();
    K_=A_.inverse()*(Ad_-Eigen::MatrixXd::Identity(2,2))*B_;

    //set up a new QP
    qp_=qpOASES::SQProblem(param_->nD_,1);
    qp_.setPrintLevel(qpOASES::PL_NONE);
    qp_.setEpsForRegularisation(1e-8);
  }
  //-----------------------------------------------------------------------------------------------
  void DoF::update(double s)
  {
    updateParticularSolutions(s);

    lock_.lock();

    updateLambda();

    Eigen::MatrixXd x_ref(Ad_*x_ref_+ps_ref_); //update the reference states
    Eigen::Vector2d x(Ad_*x_+ps_ref_*lmbd_); //update the next state
    double ddq=(x(1)-x_(1))/Td_; //compute current acceleration

    // file_x_.open("/home/rkg/Desktop/DMP/matlab/mpc_dmp/x_.txt", std::ios::out | std::ios::app );
    // file_x_ref_.open("/home/rkg/Desktop/DMP/matlab/mpc_dmp/x_ref_.txt", std::ios::out | std::ios::app );
    // file_s_.open(("/home/rkg/Desktop/DMP/matlab/mpc_dmp/s_.txt"), std::ios::out | std::ios::app );
    // file_ps_ref_.open(("/home/rkg/Desktop/DMP/matlab/mpc_dmp/ps_ref_.txt"), std::ios::out | std::ios::app );
    // file_lmbd_.open(("/home/rkg/Desktop/DMP/matlab/mpc_dmp/lmbd_.txt"), std::ios::out | std::ios::app );
    //  if (std::abs(x_(0)) > 1e-6)
    //    {
    //      file_x_<<ros::Time::now().toNSec()<<" ";
    //      file_x_ref_<<ros::Time::now().toNSec()<<" ";
    //      for(unsigned int i=0; i<x_.size();i++)
    // 	 file_x_<<x_(i)<<" ";

    //      file_x_<<ddq<<"\n";
    //      file_x_.close();
    
    //      for(unsigned int i=0; i<x_ref_.cols();i++)
    // 	 for(unsigned int j=0; j<x_ref_.rows();j++)
    // 	   file_x_ref_<<x_ref_(j,i)<<" ";

    //      file_x_ref_<<"\n";
    //      file_x_ref_.close();

    //      file_s_<<s<<"\n";
    //      file_s_.close();

    //      for(unsigned int i=0; i<ps_ref_.cols();i++)
    // 	 for(unsigned int j=0; j<ps_ref_.rows();j++)
    // 	   file_ps_ref_<<ps_ref_(j,i)<<" ";

    //      file_ps_ref_<<"\n";
    //      file_ps_ref_.close();

    //      for(unsigned int i=0; i<lmbd_.size();i++)
    // 	 file_lmbd_<<lmbd_(i)<<" "; 

    //      file_lmbd_<<"\n";
    //      file_lmbd_.close();
    //    }

    //PUBLISH (change q back)
#ifdef MPV_CONTROL
    std_msgs::Float64 msg;
    msg.data=x_(0)+goal_;
#else
    sensor_msgs::JointState msg;
    msg.header.stamp=ros::Time::now();
    msg.name.push_back(joint_);
    msg.position.push_back(x_(0)+goal_);
    msg.velocity.push_back(x_(1));
    msg.effort.push_back(ddq); //abuse the effort field i.o.t. publish the acceleration
#endif

    state_pub_.publish(msg);

    x_ref_=x_ref;
    x_=x;

#ifdef DEBUG
    sensor_msgs::JointState x_ref_msg;
    x_ref_msg.header.stamp=ros::Time::now();
    x_ref_msg.name.resize(param_->nD_);
    x_ref_msg.position.resize(param_->nD_);
    x_ref_msg.velocity.resize(param_->nD_);

    for (int i=0; i<param_->nD_;i++)
      {
	std::stringstream num;
	num<<i;
	x_ref_msg.name[i]=joint_+"_ref_"+num.str();
	x_ref_msg.position[i]=x_ref_(0,i)+goal_;
	x_ref_msg.velocity[i]=x_ref_(1,i);
      }

    ref_state_pub_.publish(x_ref_msg);
#endif

    lock_.unlock();
  }
  //-----------------------------------------------------------------------------------------------
  void DoF::updateLambda()
  {
    //Hessian
    Eigen::MatrixXd H(x_ref_.transpose()*param_->P_*x_ref_);
   
    //prioritized and weighted distances
    Eigen::RowVectorXd f(param_->nD_);
    for (int i=0; i<param_->nD_; i++)
      f(i)=param_->kappa_*(param_->P_*x_-param_->P_*x_ref_.col(i)).norm(); 

    //gradient vector
    f=f-x_.transpose()*param_->P_*x_ref_;
  
    //Constraints & Bounds
    Eigen::RowVectorXd A(param_->nD_); A.fill(1.0);
    double lbA[1]={1.0};
    double ubA[1]={1.0};
    Eigen::VectorXd lb(param_->nD_); lb.fill(0.0);
    Eigen::VectorXd ub(param_->nD_); ub.fill(1.0);

    //solve QP 
    int nWSR = 100;
    qpOASES::returnValue status;
    if (qp_.isInitialised())  
      status=qp_.hotstart(H.data(),f.data(),A.data(),lb.data(),ub.data(),lbA,ubA, nWSR,0); //called at the subsequent time steps
    else
      status=qp_.init(H.data(),f.data(),A.data(),lb.data(),ub.data(),lbA,ubA, nWSR,0); //called at the first time step

    
    if (status != qpOASES::SUCCESSFUL_RETURN)
      {
	std::string s;
	if (status==qpOASES::RET_INIT_FAILED_CHOLESKY )
	  s="RET_INIT_FAILED_CHOLESKY";
	else if (status==qpOASES::RET_INIT_FAILED)
	  s="RET_INIT_FAILED";
	else if (status==qpOASES::RET_HOTSTART_FAILED)
	  s="RET_HOTSTART_FAILED";
	else if (status==qpOASES::RET_HOTSTART_FAILED_TO_INIT) 
          s="RET_HOTSTART_FAILED_TO_INIT";
	else if (status==qpOASES::RET_HOTSTART_FAILED_AS_QP_NOT_INITIALISED)
	  s="RET_HOTSTART_FAILED_AS_QP_NOT_INITIALISED";
	else
	  {
	    std::stringstream out;
	    out<< status;
	    s=out.str();
	  }

	ROS_WARN("QP not successfuly solved! Exit status: %s",s.c_str());
      }
    else
      qp_.getPrimalSolution(lmbd_.data());

  }
  //-----------------------------------------------------------------------------------------------
  void DoF::updateParticularSolutions(double s)
  {
    Eigen::RowVectorXd psi(param_->nBF_);
    for (int i=0; i<param_->nBF_;i++)
      psi(i)=gaussBF(s, param_->pBF_(2*i),param_->pBF_(2*i+1));
	
    for (int i=0; i<param_->nD_;i++)
      ps_ref_.col(i)= K_*psi*param_->w_.col(i);
  }

  //-----------------------------------------------------------------------------------------------
  double DoF::gaussBF(double s, double sigma, double c)
  {
    return exp(-pow((s-c),2.0)/(2.0*pow(sigma,2.0)));
  }
  //-----------------------------------------------------------------------------------------------
  Eigen::Vector2d DoF::getState()
  {
    lock_.lock();
    Eigen::Vector2d state(x_);
    state(0)=x_(0)+goal_;
    lock_.unlock();

    return state;
  }
  //-------------------------------------------------------------------------------------------------
  double DoF::getGoal()
  {
    lock_.lock();
    double goal=goal_;
    lock_.unlock();

    return goal;
  }
  //-------------------------------------------------------------------------------------------------
  void DoF::stateCallback(const motion_primitives_msgs::JointControllerState::ConstPtr& msg)
  {

    lock_.lock();
    //set the current state to the measured state in case the tracking error is too large
    if (std::abs(x_(0) - msg->process_value_filtered+goal_) > track_tol_)
      {
	ROS_DEBUG("Tracking tolerance exceeded in joint %s.",joint_.c_str());
	x_(0)=msg->process_value_filtered-goal_;
	x_(1)=msg->process_value_dot_filtered;
      }

    lock_.unlock();
  }
 //-------------------------------------------------------------------------------------------------
  void DoF::stateCallback(const sr_robot_msgs::JointControllerState::ConstPtr& msg)
  {

    lock_.lock();
    //set the current state to the measured state in case the tracking error is too large
    if (std::abs(x_(0) - msg->process_value+goal_) > track_tol_)
      {
	ROS_DEBUG("Tracking tolerance exceeded in joint %s.",joint_.c_str());
	x_(0)=msg->process_value-goal_;
	x_(1)=msg->process_value_dot;
      }

    lock_.unlock();
  }
  //-------------------------------------------------------------------------------------------------
}//end namespace
