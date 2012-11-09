/**
 * @author Robert Krug
 * @date   Fri, Oct 9, 2012
 *
 */

#include <ros/ros.h>
#include "primitive_controllers/primitive_controllers.h"
#include <string.h>

  /////////////////////////////////
  //           MAIN              //
  /////////////////////////////////
//----------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_server");


  double sample_time=0.01;
  std::string searched_param;
  ros::NodeHandle nh_private_("~");

  if(nh_private_.searchParam("sample_time",searched_param))
    nh_private_.getParam(searched_param, sample_time);

  PrimitiveControllers::PrimitiveControllers pc;
  ROS_INFO("Primitive Movement Controllers ready");
  
  ros::Rate r(1/sample_time); 
  while(ros::ok())
    {
      pc.spin();
      r.sleep();
    }
  return 0;
}
//----------------------------------------------------------------------------------------
