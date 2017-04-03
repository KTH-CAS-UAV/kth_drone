#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include "gazebo_msgs/ModelStates.h"


class tf_pub
{
  public:
    tf_pub()
    {
      sub = node.subscribe("/mavros/local_position/pose", 10, &tf_pub::mavrosposeCallback, this);
      //subgazebo = node.subscribe("/gazebo/model_states", 10, &tf_pub::gazeboposeCallback, this);
      //pubgazebo = node.advertise<geometry_msgs::PoseStamped>("/vision_pose/pose", 1000, this);
    }

  private:
    void mavrosposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    
    tf::Transform transform;
    
    transform.setOrigin( tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) );
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    //q.setRPY(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "drone_base"));
  }

/*
  void gazeboposeCallback(const gazebo_msgs::ModelStates::ConstPtr& msg){
    
    tf::Transform transform;
    //ROS_INFO_STREAM(msg->pose[2].position.x)
    transform.setOrigin( tf::Vector3(msg->pose[2].position.x, msg->pose[2].position.y, msg->pose[2].position.z) );
    tf::Quaternion q(msg->pose[2].orientation.x, msg->pose[2].orientation.y, msg->pose[2].orientation.z, msg->pose[2].orientation.w);
    //q.setRPY(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "drone_base"));

    geometry_msgs::PoseStamped posemsg;
  
    posemsg.header.stamp = ros::Time::now();
    posemsg.header.frame_id = "/world";
    posemsg.pose.position.x = msg->pose[2].position.x;
    posemsg.pose.position.y = msg->pose[2].position.y;
    posemsg.pose.position.z = msg->pose[2].position.z;

    posemsg.pose.orientation.x = msg->pose[2].orientation.x;
    posemsg.pose.orientation.y = msg->pose[2].orientation.y;
    posemsg.pose.orientation.z = msg->pose[2].orientation.z;
    posemsg.pose.orientation.w = msg->pose[2].orientation.w;
  
    pubgazebo.publish(posemsg);
  }
*/

  tf::TransformBroadcaster br;
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Subscriber subgazebo;
  ros::Publisher pubgazebo;

};

int main(int argc, char **argv){
  ros::init(argc, argv, "drone_tf_pub");
  tf_pub tp;  

  ros::spin();
  
  return 0;
};