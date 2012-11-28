/**
 * @author Robert Krug
 * @date   Fri, Oct 9, 2012
 *
 */

#include <ros/ros.h>
#include "test_traj_pub/test_traj_pub.h"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/find_iterator.hpp>


namespace TestTrajPub
{ 
  //----------------------------------------------------------------------------------------
  TestTrajPub::TestTrajPub() : nh_private_("~"), ind_(0), active_(false)
  {
    std::string topic;
  std::string searched_param;

  if(nh_private_.searchParam("topic",searched_param))
    nh_private_.param(searched_param,topic,std::string());

    traj_.resize(0,3);

    pub_ = nh_.advertise<sensor_msgs::JointState>(topic,1);
    pub_traj_srv_ = nh_.advertiseService("publish_trajectory",&TestTrajPub::publishTrajectory,this);
    // 
  }
  //----------------------------------------------------------------------------------------
  TestTrajPub::~TestTrajPub()
  {
    pub_.shutdown();
  }
  //----------------------------------------------------------------------------------------
  void TestTrajPub::spin()
  {
 
    if (active_)
      {
	msg_.position.clear();
	msg_.velocity.clear();
	msg_.effort.clear();
        msg_.header.stamp=ros::Time::now();

	msg_.position.push_back(traj_(ind_,0));
	msg_.velocity.push_back(traj_(ind_,1));
	msg_.effort.push_back(traj_(ind_,2));

	pub_.publish(msg_);

	ind_++;

        if (ind_ >= traj_.rows())
	  {
	    active_=false;
            ind_=0;
	    ROS_INFO("Finished publishing Trajectory.");
          }
      }

    ros::spinOnce();
  }
  //----------------------------------------------------------------------------------------
  bool TestTrajPub::publishTrajectory(test_traj_pub::PublishTrajectory::Request  &req, test_traj_pub::PublishTrajectory::Response &res)
  {
    std::ifstream joint_traj_file;
 
    lock_.lock();

    joint_traj_file.open(req.path.c_str());

    //can't find the file
    if( !joint_traj_file.is_open())
      {
	ROS_ERROR("Couldn't open the file %s", req.path.c_str());
	lock_.unlock();
	return false;
      }
 
    unsigned int row_id=0;
    std::string line;
    std::vector<std::string> splitted_string;
    while( !joint_traj_file.eof() )
      {
	getline(joint_traj_file, line);

	//remove leading and trailing whitespace
	line = boost::algorithm::trim_copy(line);

	//ignore empty line
	if( line.size() == 0 )
	  continue;

	boost::split(splitted_string, line, boost::is_any_of("\t "));
	splitted_string.erase( std::remove_if(splitted_string.begin(), splitted_string.end(), boost::bind( &std::string::empty, _1 )), splitted_string.end()); 

	traj_.conservativeResize(row_id+1,3);       
        for (unsigned int i=0; i<splitted_string.size(); i++)
          traj_(row_id,i)=convertToDouble(splitted_string[i]);

	row_id++;
      }

    //std::cout<<"Trajectory:"<<std::endl<<traj_<<std::endl;

    // convertToDouble(splitted_string[col_id]);

    //joint_traj_->resize(rows_,cols_);   


    active_=true;
    ind_=0;
    msg_.name.clear();
    msg_.name.push_back(req.name);
    lock_.unlock();
    return true;
  }
  //----------------------------------------------------------------------------------------
}//end namespace


//----------------------------------------------------------------------------------------
/////////////////////////////////
//           MAIN              //
/////////////////////////////////
//----------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_traj_pub");


  double sample_time=0.01;
  std::string searched_param;
  ros::NodeHandle nh_private_("~");

  if(nh_private_.searchParam("sample_time",searched_param))
    nh_private_.getParam(searched_param, sample_time);

  TestTrajPub::TestTrajPub test_traj_pub;
  
  ROS_INFO("Test Trajectory Publisher ready");
  
  ros::Rate r(1/sample_time); 
  while(ros::ok())
    {
      
      //std::cout<<sample_time<<std::endl;
      test_traj_pub.spin();
      r.sleep();
      // std::cout<<"before sleep"<<std::endl;
      // std::cout<<"expected: "<<r.expectedCycleTime().toSec()<<std::endl;

  

      //r.sleep();
      //std::cout<<"after sleep"<<std::endl;
    }


  return 0;
}
//----------------------------------------------------------------------------------------

