/**
 * @author Robert Krug
 * @date   Fri, Oct 9, 2012
 *
 */


#ifndef primitive_controllers_h___
#define primitive_controllers_h___

#include "ros/ros.h"
#include <boost/thread/mutex.hpp>
#include "primitive_controllers/dof.h"
#include "primitive_controllers/canonical_system.h"
#include <map>
#include <vector> 
#include <string> 
#include <motion_primitives_msgs/TriggerMovement.h>
#include <std_srvs/Empty.h>
//#include <std_msgs/Float64.h>
#include <kcl_msgs/KCL_ContactStateStamped.h>

namespace PrimitiveControllers
{

  /**
   *@brief ...
   */
  class PrimitiveControllers
  {
  public:

    PrimitiveControllers();
    ~PrimitiveControllers();

    /**
     *@brief ...
     */
    void spin();

  private:

    ros::NodeHandle nh_, nh_private_;
    boost::mutex lock_;
    std::map<std::string,DoF*> dofs_;
    double Td_;
    CanonicalSystem cs_;
    std::vector<std::string> active_joints_;
    bool output_flag_;
    double max_force_;
    ros::ServiceServer trigger_mvmt_srv_;
    ros::ServiceServer stop_ctrls_srv_;
    void stopControllers();
    ros::V_Subscriber ctct_force_subs_;
    /////////////////
    //  CALLBACKS  //
    /////////////////
    void listenContactForceCB(const kcl_msgs::KCL_ContactStateStamped::ConstPtr& ctct_force);
    bool triggerMovementCB(motion_primitives_msgs::TriggerMovement::Request  &req, motion_primitives_msgs::TriggerMovement::Response &res);
    bool stopControllersCB(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
  };
}//end namespace
#endif
