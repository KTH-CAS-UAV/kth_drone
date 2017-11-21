// simple offboard control node for px4 with setpoints acceseble with action

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
//action
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <save_cloud/cloud_snapshootAction.h> 
#include <save_cloud/take_cloud_snapshootAction.h> 

#include <math.h>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <limits>

#include <sstream>

#include <termios.h>
//PCL
#include <sensor_msgs/PointCloud2.h>
//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

// PCL includes
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//write to file
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <string> 
#include <fstream>


using namespace std;


//snapshot class
class take_cloud_snapshoot
{
  protected:
    //node handler
    ros::NodeHandle nh_;
    ros::Subscriber sub_cloud;
    //config action server
    actionlib::SimpleActionClient<save_cloud::cloud_snapshootAction> ac_cloud_snap;

    actionlib::SimpleActionServer<save_cloud::take_cloud_snapshootAction> as_take_cloud_snap;
    save_cloud::take_cloud_snapshootFeedback feedback_take_cloud_snapshoot; // create messages that are used to published feedback
    save_cloud::take_cloud_snapshootResult result_take_cloud_snapshoot;    // create messages that are used to published result
    

  public:
    bool save_cloud;
    bool save_help_cloud;
    int snapshot_count;
    int snapshot_help_count;
    //string snapshoot_path;
    geometry_msgs::PoseStamped cloud_pose;

    take_cloud_snapshoot():ac_cloud_snap("cloud_snapshoot", true),as_take_cloud_snap(nh_, "take_cloud_snapshoot", boost::bind(&take_cloud_snapshoot::t_cloud_snapshoot, this, _1), false),snapshot_count(0),save_cloud(false),snapshot_help_count(0),save_help_cloud(false)
    {
      as_take_cloud_snap.start(); 
      ac_cloud_snap.waitForServer(); //wait for nbv server

      //sub_cloud = nh_.subscribe("/camera/depth/points", 1, &cloud_snapshoot::input_cloud_cb, this);

      sub_cloud = nh_.subscribe("/camera/depth_registered/points", 1, &take_cloud_snapshoot::input_cloud_cb, this);



    }

    ~take_cloud_snapshoot()
    {

    }
    //recive take snapshoot comand
    void t_cloud_snapshoot(const save_cloud::take_cloud_snapshootGoalConstPtr &goal)
  {
    int command=goal->command;
    cloud_pose=goal->cloud_pose;
    if(command==1){
      save_help_cloud=true;

    }
    if(command==2){
      save_cloud=true;
    }

    result_take_cloud_snapshoot.confirmation = 1;

    as_take_cloud_snap.setSucceeded(result_take_cloud_snapshoot);    
  }


   //action is action
  void send_snapshoot_path(string snapshoot_path)
  {
    // send the path to the cloud
    save_cloud::cloud_snapshootGoal goal;
    goal.cloud_path = snapshoot_path;
    goal.cloud_pose = cloud_pose;
    // Need boost::bind to pass in the 'this' pointer
    ac_cloud_snap.sendGoal(goal,
                boost::bind(&take_cloud_snapshoot::confirmation_snapshoot, this, _1, _2),
                actionlib::SimpleActionClient<save_cloud::cloud_snapshootAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<save_cloud::cloud_snapshootAction>::SimpleFeedbackCallback());
  }

  void confirmation_snapshoot(const actionlib::SimpleClientGoalState& state,
              const save_cloud::cloud_snapshootResultConstPtr& result)
  {
    //ROS_DEBUG_NAMED("game_master", "Finished in state [%s]", state.toString().c_str());
    //ai_move =result->best_move;
}

  //calback for the pointcloud
  void input_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    if(save_help_cloud)
    {
        //save the helper clouds
      stringstream ss_num_help;
      ss_num_help << snapshot_help_count;
      string snapshoot_path_help = ros::package::getPath("save_cloud") + "/clouds/cloud_help_" + ss_num_help.str() + ".pcd";   // Read the file
      
      ROS_INFO_STREAM("Received point help cloud with " << cloud_msg->height*cloud_msg->width << " points.");


      pcl::PCDWriter writer;
      pcl::PointCloud<pcl::PointXYZRGBNormal> pclCloud_help;
      fromROSMsg(*cloud_msg, pclCloud_help);
      pcl::io::savePCDFileASCII (snapshoot_path_help, pclCloud_help);

      std::ofstream outfile;
      string path_help = ros::package::getPath("save_cloud") + "/clouds/cloud_help" + ss_num_help.str() + ".txt";

      outfile.open(path_help.c_str(), std::ios_base::app);

                  outfile << cloud_pose.pose.position.x << "," << cloud_pose.pose.position.y << "," << cloud_pose.pose.position.z << ",";
                  outfile << cloud_pose.pose.orientation.x << "," << cloud_pose.pose.orientation.y << "," << cloud_pose.pose.orientation.z << "," << cloud_pose.pose.orientation.w ; 
                  outfile << "\n";               

      save_help_cloud=false;
      snapshot_help_count++;


    }


    if(!save_cloud)
      return;

    stringstream ss_num;
    ss_num << snapshot_count;
    string snapshoot_path = ros::package::getPath("save_cloud") + "/clouds/cloud_" + ss_num.str() + ".pcd";   // Read the file
    
    ROS_INFO_STREAM("Received point cloud with " << cloud_msg->height*cloud_msg->width << " points.");


    pcl::PCDWriter writer;
    pcl::PointCloud<pcl::PointXYZRGBNormal> pclCloud;
    fromROSMsg(*cloud_msg, pclCloud);
    pcl::io::savePCDFileASCII (snapshoot_path, pclCloud);

    std::ofstream outfile;
    string path = ros::package::getPath("save_cloud") + "/clouds/cloud_" + ss_num.str() + ".txt";

    outfile.open(path.c_str(), std::ios_base::app);

                outfile << cloud_pose.pose.position.x << "," << cloud_pose.pose.position.y << "," << cloud_pose.pose.position.z << ",";
                outfile << cloud_pose.pose.orientation.x << "," << cloud_pose.pose.orientation.y << "," << cloud_pose.pose.orientation.z << "," << cloud_pose.pose.orientation.w ; 
                outfile << "\n";               



    save_cloud=false;
    snapshot_count++;
    send_snapshoot_path(snapshoot_path);

  }

  
};




int main(int argc, char **argv)
{

  ros::init(argc, argv, "save_cloud");
  //node handler
  ros::NodeHandle nh;
  ROS_INFO("Start save_cloud");
  
  take_cloud_snapshoot cs;
  
  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  ros::spin();    
 
  return 0;

}
