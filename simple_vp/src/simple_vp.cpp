#include <ros/ros.h>
#include <inttypes.h>

#include <nav_msgs/OccupancyGrid.h>
#include <ros/console.h>

//TF stuff
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <math.h>


//teleop
#include "std_msgs/String.h"
#include <string>

// mavros stuff
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>



using namespace std;

class simple_vp
{
  public:

    bool circleflag;   
    std::vector<tf::Vector3> vp_vec_reachable;
    std::vector<geometry_msgs::PoseStamped> vp_vec_all;
    mavros_msgs::State current_state;
    int view_point_number;

    double viewpoint_radius;
    double num_viepoints;

      //drone mavros com
    ros::Subscriber state_sub;
    ros::Publisher local_pos_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;


    simple_vp():circleflag(false),view_point_number(5)
    {

      // sub
      vp_pub = node.advertise<geometry_msgs::PoseArray>( "view_points", 0 );
      sub_map = node.subscribe("/projected_map", 10, &simple_vp::map2DCallback, this);

      // drone mavros stuff
     state_sub = node.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &simple_vp::state_cb, this);
    
    local_pos_pub = node.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    arming_client = node.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = node.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //teleop subscribe
    sub_teleop_vp_num = node.subscribe("/teleop/keyinput", 10, &simple_vp::keyinputCallback, this);


      vp_vec_all.clear();
      /*
      nav_msgs::OccupancyGrid
      
      sub_map = node.subscribe("/mavros/local_position/pose", 10, &tf_pub::mavrosposeCallback, this);
      subgazebo = node.subscribe("/gazebo/model_states", 10, &tf_pub::gazeboposeCallback, this);
      pubgazebo = node.advertise<geometry_msgs::PoseStamped>("/vision_pose/pose", 1000, this);
      */

    }

    void keyinputCallback(const std_msgs::String::ConstPtr& msg){
      ROS_INFO("kezboard input recived");
      ROS_INFO(msg->data.c_str());

      std::string strinput =msg->data.c_str();
      
      if(strinput=="n"){
        view_point_number=view_point_number+1;
          if(view_point_number>=vp_vec_all.size())
            view_point_number=0;

          ROS_INFO("Next View point");
      }
      if (strinput=="b"){
            view_point_number=view_point_number-1;
          if(view_point_number<0)
            view_point_number=vp_vec_all.size();

          ROS_INFO("previous View point");
        }
        if (strinput=="f"){
 
          ROS_INFO("Unknown command");
        }

    }

    
    void state_cb(const mavros_msgs::State::ConstPtr& msg){
      current_state = *msg;
    }

    void map2DCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg){
      
      //nav_msgs::OccupancyGrid  srcRos;
      

    }

    void circular_vp(std::string target, double radius, double num_vp){

      tf::StampedTransform transform;

      // get tf transform from world to target frame
      try{
        listener.lookupTransform("/world", "/target",  
                                 ros::Time(0), transform);

          tf::Vector3 tp(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z()); // target point 3D
          double angle = 2*M_PI/num_vp;

          // generatepoints vp around target given radius and num_vp
          // store points in verctor

          geometry_msgs::PoseStamped temp_vp;
          double temp_angle;
            for(int i=0;i<num_vp;i++){
              temp_angle=angle*i;
              temp_vp.pose.position.x=(tp.getX()+radius*cos(temp_angle));
              temp_vp.pose.position.y=(tp.getY()+radius*sin(temp_angle));
              temp_vp.pose.position.z=(tp.getZ());
              temp_vp.pose.orientation = tf::createQuaternionMsgFromYaw(temp_angle+M_PI);
              //temp_vp.pose.orientation.z=temp_angle;
              vp_vec_all.push_back(temp_vp);
            } 
          circleflag=true;
        }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.1).sleep();
        }      

    }

    void check_vp(std::vector<tf::Vector3> vp_vec_temp){

      //TODO::check if points are reachible or not using simple means (A* or similar)

    }

    void sort_vp(std::vector<tf::Vector3> vp_possiblevec){

      // TODO::sort vp in sensible order (shortest distance or so (best would be costs))
    }

    void publish_vp(std::vector<geometry_msgs::PoseStamped> vp_pubvec){

      int vec_length=vp_pubvec.size();
      geometry_msgs::PoseArray poseArray; 
      poseArray.header.stamp = ros::Time::now();
      poseArray.header.frame_id = "/world";
      for (int i = 0; i < vec_length; ++i)
     {      
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.pose.position.x = vp_pubvec[i].pose.position.x;
        temp_pose.pose.position.y = vp_pubvec[i].pose.position.y;
        temp_pose.pose.position.z = vp_pubvec[i].pose.position.z;

        temp_pose.pose.orientation.x = vp_pubvec[i].pose.orientation.x;
        temp_pose.pose.orientation.y = vp_pubvec[i].pose.orientation.y;
        temp_pose.pose.orientation.z = vp_pubvec[i].pose.orientation.z;
        temp_pose.pose.orientation.w = vp_pubvec[i].pose.orientation.w;;
        poseArray.poses.push_back(temp_pose.pose);
     }
     vp_pub.publish(poseArray);

    }

    void getcyrcleparam(){
      //check if param are ther else use defult of 0.75 and 10
      double d;
      if (node.getParam("/simple_vp/vp_radius", d))
      {
        ROS_INFO_THROTTLE(1 , "Got param: %f", d);
        viewpoint_radius= d;
      }
      else
      {
        ROS_WARN_THROTTLE(1, "Failed to get param '/simple_vp/vp_radius' setting to defult (0.55)");
        viewpoint_radius=0.55;
      }

      if (node.getParam("/simple_vp/num_vp", d))
      {
        ROS_INFO_THROTTLE(1, "Got param: %f", d);
        num_viepoints= d;
      }
      else
      {
        ROS_WARN_THROTTLE(1, "Failed to get param '/simple_vp/num_vp' setting to defult (10)");
        num_viepoints=10;
      }


    }

    double getzparam(){

      double d;
      double vp_z;
      if (node.getParam("/simple_vp/vp_z", d))
      {
        ROS_INFO_THROTTLE(1, "Got param: %f", d);
        vp_z= d;
      }
      else
      {
        ROS_WARN_THROTTLE(1, "Failed to get param '/simple_vp/vp_z' setting to defult (1.0)");
        vp_z=1.0;
      }

      return vp_z;
    }



  private:   
  tf::TransformBroadcaster br;
  tf::TransformListener listener;
  ros::NodeHandle node;
  ros::Subscriber sub_map;
  ros::Subscriber sub_teleop_vp_num;
  ros::Subscriber subgazebo;
  ros::Publisher pubgazebo;
  ros::Publisher vp_pub;  
};




int main(int argc, char **argv){
  ros::init(argc, argv, "simple_vp"); 

  simple_vp vp; 

  ros::Duration(5.0).sleep(); // sleep a little 
  ros::Rate rate(20.0);
  vp.getcyrcleparam();
  
  //basic view planner#
  /*
  1. generate possible view points around target in circle
  2. check if view points are reachible
  3. sort points by shortes distance to each other
  4. send list ready to execute

  */
  


  

  while(!vp.circleflag){

    // use parameter to set number of view points and radius
    vp.circular_vp("/target",(double)vp.viewpoint_radius,(double)vp.num_viepoints);
    //vp.circular_vp("/target",1.0,10.0);
    
    rate.sleep();
  }
 

  //conect drone
  ROS_INFO("Starting");
     // wait for FCU connection
    while(ros::ok() && vp.current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected");

    //TODO: read in current pose and add onlz Z!!!
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.5;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1.0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        vp.local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();     

    while(ros::ok() ){
        pose=vp.vp_vec_all[vp.view_point_number];
        // read z from param server
        pose.pose.position.z = vp.getzparam();

        ros::spinOnce();
        vp.publish_vp(vp.vp_vec_all);

        
        if( vp.current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(1.0))){
            if( vp.set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !vp.current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(1.0))){
                if( vp.arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        vp.local_pos_pub.publish(pose);
        
    }
  
  return 0;
};