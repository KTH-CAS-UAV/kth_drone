#include <ros/ros.h>

//MRPT

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/nav/planners/PlannerSimple2D.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt_bridge/map.h>

#include <mrpt/random.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/system/filesystem.h> // for fileExists()
#include <mrpt/system/string_utils.h> // for lowerCase()


#include <inttypes.h>

#include <nav_msgs/OccupancyGrid.h>
#include <ros/console.h>

//TF stuff
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>



#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
/*
#include <mrpt/version.h>
#if MRPT_VERSION>=0x130
#       include <mrpt/maps/COccupancyGridMap2D.h>
        using mrpt::maps::COccupancyGridMap2D;
#else
#       include <mrpt/slam/COccupancyGridMap2D.h>
        using mrpt::slam::COccupancyGridMap2D;
#endif
*/

#include "mrpt_bridge/map.h"
#include "mrpt_bridge/pose.h"
#include <nav_msgs/OccupancyGrid.h>
#include <ros/console.h>
#include <mrpt/random.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/version.h>
#include <ros/console.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
//using namespace mrpt::maps;
using namespace mrpt::nav;
using namespace mrpt::math;
using namespace mrpt::poses;


class simple_vp
{
  public:

    bool circleflag;
    std::vector<tf::Vector3> vp_vec_all;
    std::vector<tf::Vector3> vp_vec_reachable;


    simple_vp():circleflag(false)
    {

      // constructor
      vp_pub = node.advertise<visualization_msgs::Marker>( "view_points", 0 );
      sub_map = node.subscribe("/projected_map", 10, &simple_vp::map2DCallback, this);
      
      vp_vec_all.clear();
      /*
      nav_msgs::OccupancyGrid
      
      sub_map = node.subscribe("/mavros/local_position/pose", 10, &tf_pub::mavrosposeCallback, this);
      subgazebo = node.subscribe("/gazebo/model_states", 10, &tf_pub::gazeboposeCallback, this);
      pubgazebo = node.advertise<geometry_msgs::PoseStamped>("/vision_pose/pose", 1000, this);
      */

    }

    void map2DCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg){
      //bool mrpt_bridge::convert ( const nav_msgs::OccupancyGrid &   src, COccupancyGridMap2D &   des )
      /*COccupancyGridMap2D gridmap;
      bool consug = mrpt_bridge::convert (msg, gridmap);
      ROS_INFO_STREAM(consug);
      */

      nav_msgs::OccupancyGrid  srcRos;
      //COccupancyGridMap2D gridmap;
    
      //mrpt_bridge::convert(srcRos, desMrpt);
      

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

        tf::Vector3 temp_vp;
        double temp_angle;
          for(int i=0;i<num_vp;i++){
            temp_angle=angle*i;
            temp_vp.setX(tp.getX()+radius*cos(temp_angle));
            temp_vp.setY(tp.getY()+radius*sin(temp_angle));
            temp_vp.setZ(tp.getZ());
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

      //check if points are reachible or not using simple means (A* or similar)




    }

    void sort_vp(std::vector<tf::Vector3> vp_possiblevec){

      // sort vp in sensible order (shortest distance or so (best would be costs))
    }

    void publis_vp(std::vector<tf::Vector3> vp_pubvec){

      int vec_length=vp_pubvec.size();
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/world";
      marker.header.stamp = ros::Time();
     marker.ns = "view_point_sphere_list";
     marker.id = 0;
     marker.type = visualization_msgs::Marker::SPHERE_LIST;
     marker.action = visualization_msgs::Marker::ADD;
     marker.pose.orientation.x = 0.0;
     marker.pose.orientation.y = 0.0;
     marker.pose.orientation.z = 0.0;
     marker.pose.orientation.w = 1.0;
     //marker.pose.position.x = x_pos;
     marker.scale.x = 0.05;
     marker.scale.y = 0.05;
     marker.scale.z = 0.05;
     marker.color.r = 1.0;
     marker.color.g = 1.0;
     marker.color.b = 1.0;
     marker.color.a = 1.0;
     for (int i = 0; i < vec_length; ++i)
     {      
        geometry_msgs::Point p;
        p.x = vp_pubvec[i].getX();
        p.y = vp_pubvec[i].getY();
        p.z = vp_pubvec[i].getZ();
 
        marker.points.push_back(p);

     }

     vp_pub.publish(marker);

    }




  private:

    /*
    void mavrosposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    
    tf::Transform transform;
    
    transform.setOrigin( tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) );
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    //q.setRPY(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "drone_base"));
  }

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
  tf::TransformListener listener;
  ros::NodeHandle node;
  ros::Subscriber sub_map;
  ros::Subscriber subgazebo;
  ros::Publisher pubgazebo;
  ros::Publisher vp_pub;

};

int main(int argc, char **argv){
  ros::init(argc, argv, "simple_vp");
  simple_vp vp; 


  //COccupancyGridMap2D gridmap;

  //basic view planner#
  /*
  1. generate possible view points around target in circle
  2. check if view points are reachible
  3. sort points by shortes distance to each other
  4. send list ready to execute

  */
  ros::Rate rate(20.0);
  while(!vp.circleflag){
    vp.circular_vp("/target",0.75,10);
    rate.sleep();
    //ROS_INFO_STREAM("while 1");

  }

  
      
  

    // wait for FCU connection
    while(ros::ok() ){
        ros::spinOnce();
        vp.publis_vp(vp.vp_vec_all);
        //ROS_INFO_STREAM("while 2");
        
    }
  
  return 0;
};