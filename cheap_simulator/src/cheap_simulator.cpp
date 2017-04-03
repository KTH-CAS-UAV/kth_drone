#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
//#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"


class cheap_simulator
{
  public:
    cheap_simulator()
    {
      //sub = node.subscribe("/mavros/local_position/pose", 10, &tf_pub::mavrosposeCallback, this);
      subrviz = node.subscribe("mavros/setpoint_position/local", 10, &cheap_simulator::rvizCallback, this);
      pubgazebo = node.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000, this);
    }

  private:
    void rvizCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){

      geometry_msgs::PoseStamped pose;

  

    geometry_msgs::Pose gazpose;
    gazpose.position.x = msg->pose.position.x;
    gazpose.position.y = msg->pose.position.y;
    gazpose.position.z = msg->pose.position.z;
    gazpose.orientation.x = msg->pose.orientation.x;
    gazpose.orientation.y = msg->pose.orientation.y;
    gazpose.orientation.z = msg->pose.orientation.z;
    gazpose.orientation.w =msg->pose.orientation.w;

    geometry_msgs::Twist gaztwist;
    gaztwist.linear.x = 0;
    gaztwist.linear.y = 0;
    gaztwist.linear.z = 0;
    gaztwist.angular.x = 0.0;
    gaztwist.angular.y = 0.0;
    gaztwist.angular.z = 0.0;

    
    gazebo_msgs::ModelState modelstate;
    modelstate.model_name = "iris_cam"; 
    modelstate.reference_frame = "world";
    modelstate.twist = gaztwist;
    modelstate.pose = gazpose;



    pubgazebo.publish(modelstate);

    
  }

  ros::NodeHandle node;
  ros::Subscriber subrviz;
  ros::Publisher pubgazebo;

};

int main(int argc, char **argv){
  ros::init(argc, argv, "cheap_simulator");
  cheap_simulator cs;  

  ros::spin();
  
  return 0;
};