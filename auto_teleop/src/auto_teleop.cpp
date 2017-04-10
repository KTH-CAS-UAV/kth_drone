#include "ros/ros.h"
#include "std_msgs/String.h"


#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <cmath>
#include <tf/transform_datatypes.h>

#include <sstream>

#include <termios.h>


class auto_teleop
{
  public:

    geometry_msgs::PoseStamped targetPoint;
    geometry_msgs::PoseStamped droneposPoint;

    auto_teleop()
    {
        sub_targetPoint = node.subscribe("/mavros/setpoint_position/local", 10, &auto_teleop::targetPointCallback, this);
        sub_droneposPoint =node.subscribe("/mavros/local_position/pose", 10, &auto_teleop::droneposPointCallback, this);
        pub_teleop = node.advertise<std_msgs::String>("/teleop/keyinput", 1000);


    }
    ~auto_teleop(){

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

  auto_teleop at;
  
  ros::Rate loop_rate(10);

  std_msgs::String msg;

  double errorx=1000;
  double errory=1000;
  double erroryaw=360;

  double tolxy=0.05;
  double tolyaw=5;

  bool atposition=false;
  bool timestarted=false;
  ros::Time start_time;
  while (ros::ok())
  {

    //check if pose is close enogghu to target
    //calc erorrs (only x,y yaw are of interest)
    errorx = pow(pow(at.targetPoint.pose.position.x,2)-pow(at.droneposPoint.pose.position.x,2),2);
    errory = pow(pow(at.targetPoint.pose.position.y,2)-pow(at.droneposPoint.pose.position.y,2),2);
    erroryaw = distance(tf::getYaw(at.targetPoint.pose.orientation)*(180/M_PI),tf::getYaw(at.droneposPoint.pose.orientation)*(180/M_PI));
    
    //check if error is in margin
    if(errorx<tolxy && errory< tolxy && erroryaw < tolyaw)
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
      at.pub_teleop.publish(msg);
      timestarted=false;
    }


    //ROS_INFO_STREAM(errorx);
    //ROS_INFO_STREAM(errory);
    //ROS_INFO_STREAM(erroryaw);
    //at.pub_teleop.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}