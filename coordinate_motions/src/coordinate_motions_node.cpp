/**
 * @author Robert Krug
 * @date   Fri, Oct 9, 2012
 *
 */

#include <ros/ros.h>
#include "coordinate_motions/coordinate_motions.h"
// #include <arm_navigation_msgs/GetPlanningScene.h>
// #include <arm_navigation_msgs/MoveArmActionGoal.h>



/////////////////////////////////
//           MAIN              //
/////////////////////////////////
//----------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "coordinate_motions");
  ros::NodeHandle nh;
  //   ros::ServiceClient get_p_scene_clt;
  //   ros::Publisher action_goal_pub;
  //   action_goal_pub=nh.advertise<arm_navigation_msgs::MoveArmActionGoal>("/move_right_arm/goal",1,true);
  //   get_p_scene_clt = nh.serviceClient<arm_navigation_msgs::GetPlanningScene>("/environment_server/get_planning_scene");
  //   get_p_scene_clt.waitForExistence();

  //   arm_navigation_msgs::GetPlanningScene scene;
  //   get_p_scene_clt.call(scene);

  //   // for (int i=0; i<scene.response.planning_scene.robot_state.joint_state.name.size(); i++)
  //   //   std::cout<< scene.response.planning_scene.robot_state.joint_state.name[i]<<std::endl;

  // arm_navigation_msgs::MoveArmActionGoal action_goal;

  //  action_goal.goal_id.id="my_test";
  //  action_goal.goal_id.stamp=ros::Time::now();
  //  action_goal.header.stamp=ros::Time::now();
  //  action_goal.goal.planner_service_name="/ompl_planning/plan_kinematic_path";

  //  while (true)
  //  action_goal_pub.publish(action_goal);
  //  //action_goal.motion_plan_request.

  CoordinateMotions::CoordinateMotions cm;
  ROS_INFO("CoordinateMotions node ready");

  // ShadowTrajectory sj;
  // sj.startTrajectory(sj.arm_movement());
  // sj.waitTrajectory();

  // ros::Rate r(1/sample_time); 
  while(ros::ok())
      cm.spin();
 
  return 0;
}
//----------------------------------------------------------------------------------------
