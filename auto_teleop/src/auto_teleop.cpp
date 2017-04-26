#include "ros/ros.h"
#include "std_msgs/String.h"


#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <limits>

#include <sstream>

#include <termios.h>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <auto_teleop/a_teleopConfig.h>

class auto_teleop_class
{
  public:

    geometry_msgs::PoseStamped targetPoint;
    geometry_msgs::PoseStamped droneposPoint;
    double tol_xy;  // toleranze in x and y in meter
    double tol_yaw;    // toleranz in grad

    auto_teleop_class()
    {
        sub_targetPoint = node.subscribe("/mavros/setpoint_position/local", 1, &auto_teleop_class::targetPointCallback, this);
        sub_droneposPoint =node.subscribe("/mavros/local_position/pose", 1, &auto_teleop_class::droneposPointCallback, this);
        pub_teleop = node.advertise<std_msgs::String>("/teleop/keyinput", 1000);


    }
    ~auto_teleop_class(){

    }

    void targetPointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
      targetPoint.pose=msg->pose;
      //ROS_INFO_STREAM("targetPointCallback");

    }

    void droneposPointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
      droneposPoint.pose=msg->pose;
      //ROS_INFO_STREAM("droneposPoint");
    }

    void configCallback(auto_teleop::a_teleopConfig &config, uint32_t level)
{
     ROS_INFO("Reconfigure");
     tol_xy=config.tol_xy;
     tol_yaw=config.tol_yaw;
    
} // end configCallback()




ros::Publisher pub_teleop;

  private:
    ros::NodeHandle node;
    ros::Subscriber sub_targetPoint;
    ros::Subscriber sub_droneposPoint;
    
};

double distance(double alpha, double beta) {
        double phi = abs(beta - alpha) % 360;       // This is either the distance or 360 - distance
        double distance = phi > 180 ? 360 - phi : phi;
        return distance;
    }


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "auto_teleop");

  auto_teleop_class *at = new auto_teleop_class;
  
  ros::Rate loop_rate(10);

  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  dynamic_reconfigure::Server<auto_teleop::a_teleopConfig> dr_srv;
  dynamic_reconfigure::Server<auto_teleop::a_teleopConfig>::CallbackType cb;
  cb = boost::bind(&auto_teleop_class::configCallback,at, _1, _2);
  dr_srv.setCallback(cb);


  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can
  // be run simultaneously while using different parameters.
  // Parameters defined in the .cfg file do not need to be initialized here
  // as the dynamic_reconfigure::Server does this for you.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("tol_xy", at->tol_xy, double(0.05));
  private_node_handle_.param("tol_yaw", at->tol_yaw, double(5));


  std_msgs::String msg;

  double errorx=std::numeric_limits<double>::max();
  double errory=std::numeric_limits<double>::max();
  double erroryaw=std::numeric_limits<double>::max();

  

  bool atposition=false;
  bool timestarted=false;
  ros::Time start_time;
  while (ros::ok())
  {

    //check if pose is close enogghu to target
    //calc erorrs (only x,y yaw are of interest)
    errorx=fabs(at->targetPoint.pose.position.x-at->droneposPoint.pose.position.x);
    errory=fabs(at->targetPoint.pose.position.y-at->droneposPoint.pose.position.y);
    erroryaw = distance(tf::getYaw(at->targetPoint.pose.orientation)*(180/M_PI),tf::getYaw(at->droneposPoint.pose.orientation)*(180/M_PI));
    
    //check if error is in margin
    if(errorx<at->tol_xy && errory< at->tol_xy && erroryaw < at->tol_yaw)
    {
      atposition=true;
    }
    else
    {
      atposition=false;
      timestarted=false;
    }

    if(atposition==true && timestarted==false)
    {
      start_time = ros::Time::now();  
      timestarted=true;
    }

    if(ros::Time::now() - start_time > ros::Duration(5.0) && atposition==true)
    {
      msg.data="n";
      ROS_INFO_STREAM("sending");
      at->pub_teleop.publish(msg);
      timestarted=false;
    }

    ros::spinOnce();
    //ROS_INFO_STREAM(" errorx: " << errorx);
    //ROS_INFO_STREAM("tol xy: " << at->tol_xy);
    
    loop_rate.sleep();
  }


  return 0;
}