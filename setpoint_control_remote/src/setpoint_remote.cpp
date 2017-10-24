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
#include <setpoint_control_remote/setpoint_control_commandsAction.h>
#include <setpoint_control_remote/cloud_snapshootAction.h> 

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

using namespace std;


class drone_setpoint
{
  protected:
    //node handler
    ros::NodeHandle nh_;
    ros::Subscriber sub_current_pose;
    //config action server
    actionlib::SimpleActionServer<setpoint_control_remote::setpoint_control_commandsAction> as_drone_setpoint; 
    std::string action_name_;
    setpoint_control_remote::setpoint_control_commandsFeedback feedback_drone_setpoint; 
    setpoint_control_remote::setpoint_control_commandsResult result_drone_setpoint;

  public:
    //vars
    geometry_msgs::PoseStamped target_pose;
    geometry_msgs::PoseStamped current_pose;
    int mode; //0=IDLE, 1 = moving to targed, 2= holding position
    int at_position_counter;
    int at_position_threshold;
    //position tolleranc
    double errorx;
    double errory;
    double errorz;
    double erroryaw;
    double tolx;
    double toly;
    double tolz;
    double tolyaw;
    //pointcloudsnapshoot

  
  //constructor
  drone_setpoint(std::string name) :
    as_drone_setpoint(nh_, name, boost::bind(&drone_setpoint::recived_command, this, _1), false),
    action_name_(name), mode(0),at_position_counter(0),at_position_threshold(50),tolx(0.1),toly(0.1),tolz(0.1),tolyaw(15.0)
  {

    //starting action server
    as_drone_setpoint.start();
    ROS_INFO("DS: drone setpoint server started");

    //subscribe to current pose
    sub_current_pose=nh_.subscribe("/mavros/local_position/pose", 1, &drone_setpoint::current_pointCallback, this);
        

    //init pose
    target_pose.pose.position.x = 0;
    target_pose.pose.position.y = 0;
    target_pose.pose.position.z = 0;
    target_pose.pose.orientation.x = 0;
    target_pose.pose.orientation.y = 0;
    target_pose.pose.orientation.z = 0;
    target_pose.pose.orientation.w = 1.0;

  }

  //deconstructor
  ~drone_setpoint(void)
  {

  }

  
  void current_pointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
      current_pose.pose=msg->pose;
      //check position
      //compare target with current pos
      //calculate the error
      
      errorx=fabs(target_pose.pose.position.x-current_pose.pose.position.x);
      errory=fabs(target_pose.pose.position.y-current_pose.pose.position.y);
      errorz=fabs(target_pose.pose.position.z-current_pose.pose.position.z); 
      erroryaw = distance(tf::getYaw(target_pose.pose.orientation)*(180/M_PI),tf::getYaw(current_pose.pose.orientation)*(180/M_PI));
      
      //std::cout<< "Current error: x: " << errorx << " , y: " << errory << " ,z: " << errorz << " ,yaw: " << erroryaw << std::endl;
      if(errorx< tolx && errory< toly && errorz< tolz && erroryaw < tolyaw)
      {
        at_position_counter++;
        //ROS_INFO("at postioon for the : %i",at_position_counter);
      }
        
      else
      {
          at_position_counter=0;
      }
      if(at_position_counter>=at_position_threshold)
      {
          mode=2;
          at_position_counter=0;
      }
    }


  void recived_command(const setpoint_control_remote::setpoint_control_commandsGoalConstPtr &goal)
  {
    //move to new position
    if(goal->command==1)
    {
        target_pose= goal->pose;
        mode=1;
    }
    else //switch to IDLE (when flying the drone will land)
    {
        mode=0;
    } 

    //got the command
    result_drone_setpoint.confirmation=1;
    ROS_INFO( "%s: Done", action_name_.c_str());
    // set the action state to succeeded
    as_drone_setpoint.setSucceeded(result_drone_setpoint);

  }

  //check if the drone is at the target and should hold positions
  void check_position()
  {
    //compare target with current pos
    //calculate the error
    
    errorx=fabs(target_pose.pose.position.x-current_pose.pose.position.x);
    errory=fabs(target_pose.pose.position.y-current_pose.pose.position.y);
    errorz=fabs(target_pose.pose.position.z-current_pose.pose.position.z); 
    erroryaw = distance(tf::getYaw(target_pose.pose.orientation)*(180/M_PI),tf::getYaw(current_pose.pose.orientation)*(180/M_PI));
    
    if(errorx< tolx && errory< toly && errorz< tolz && erroryaw < tolyaw)
    {
      at_position_counter++;
      //ROS_INFO("at postioon for the : %i",at_position_counter);
    }
      
    else
    {
        at_position_counter=0;
    }
    if(at_position_counter>=at_position_threshold)
    {
        ROS_INFO("Enabeling mode 2.");
        mode=2;
        at_position_counter=0;
    }
  }

  //help function for distance between angle
  double distance(double alpha, double beta) {
        double phi = fmod(abs(beta - alpha), 360.0);       // This is either the distance or 360 - distance
        double distance = phi > 180 ? 360 - phi : phi;
        return distance;
    }

};


//snapshot class
class cloud_snapshoot
{
  protected:
    //node handler
    ros::NodeHandle nh_;
    ros::Subscriber sub_cloud;
    //config action server
    actionlib::SimpleActionClient<setpoint_control_remote::cloud_snapshootAction> ac_cloud_snap;

  public:
    bool save_cloud;
    int snapshot_count;
    string snapshoot_path;

    cloud_snapshoot():ac_cloud_snap("cloud_snapshoot", true),snapshot_count(0),save_cloud(false)
    {
      ac_cloud_snap.waitForServer(); //wait for nbv server

      sub_cloud = nh_.subscribe("/camera/depth/points", 1, &cloud_snapshoot::input_cloud_cb, this);


    }

    ~cloud_snapshoot()
    {

    }


  //action is action
  void send_snapshoot_path()
  {
    // send the path to the cloud
    setpoint_control_remote::cloud_snapshootGoal goal;
    goal.path = snapshoot_path;
    // Need boost::bind to pass in the 'this' pointer
    ac_cloud_snap.sendGoal(goal,
                boost::bind(&cloud_snapshoot::confirmation_snapshoot, this, _1, _2),
                actionlib::SimpleActionClient<setpoint_control_remote::cloud_snapshootAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<setpoint_control_remote::cloud_snapshootAction>::SimpleFeedbackCallback());
  }

  void confirmation_snapshoot(const actionlib::SimpleClientGoalState& state,
              const setpoint_control_remote::cloud_snapshootResultConstPtr& result)
  {
    //ROS_DEBUG_NAMED("game_master", "Finished in state [%s]", state.toString().c_str());
    //ai_move =result->best_move;
  }

  
  //calback for the pointcloud
  void input_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    if(!save_cloud)
      return;

    stringstream ss_num;
    ss_num << snapshot_count;
    snapshoot_path = ros::package::getPath("setpoint_control_remote") + "/clouds/cloud_" + ss_num.str() + ".pcd";   // Read the file
    
    ROS_INFO_STREAM("Received point cloud with " << cloud_msg->height*cloud_msg->width << " points.");


    pcl::PCDWriter writer;
    pcl::PointCloud<pcl::PointXYZRGBNormal> pclCloud;
    fromROSMsg(*cloud_msg, pclCloud);
    pcl::io::savePCDFileASCII (snapshoot_path, pclCloud);
    /*
    // Write .ply file.
    pcl::PointCloud<pcl::PointXYZRGBNormal> pclCloud;
    fromROSMsg(*cloud_msg, pclCloud);

    pcl::PLYWriter writer;
    if (writer.write(snapshoot_path, pclCloud) != 0) {
      ROS_ERROR("Something went wrong when trying to write the point cloud file.");
      return;
    }
    */
    save_cloud=false;
    snapshot_count++;
    send_snapshoot_path();

  }

  
};



//mavros
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

//visu help
geometry_msgs::PoseStamped targetPoint;
void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
      targetPoint.pose=msg->pose;
      targetPoint.header.frame_id="world";
      //ROS_INFO_STREAM("targetPointCallback");

    }


int main(int argc, char **argv)
{

  ros::init(argc, argv, "setpoint_control_remote");
  //node handler
  ros::NodeHandle nh;
  ROS_INFO("Start setpoint_control_remote");
  //start action server
  drone_setpoint ds("setpoint_control_commands");  

  cloud_snapshoot cs;

  //set up drone stuff
  //subscriber mavros
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber sub_target = nh.subscribe("/mavros/setpoint_position/local", 1, &targetCallback);
  //publisher mavros
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::Publisher pub_target = nh.advertise<geometry_msgs::PoseStamped>("/drone_pose_target", 1);
  //mavros services
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected");

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(ds.target_pose);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Ready to go to work");
    ros::Duration(1.0).sleep();
    
    //set state to Offboard
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    //arming the drone
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    //save current time
    ros::Time last_request = ros::Time::now();
    ROS_INFO("Sarting main loop");
    bool new_pose=false;
    while(ros::ok()){
        //if offboard or arming failed try again every second:
        //Offboard
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(1.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(1.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        //performe action depending on flight mode
        if(ds.mode==0)
        {
            //do nothing
            ROS_INFO_THROTTLE(5, "Mode: 0 (IDLE)");
        }
        if(ds.mode==1)
        {
          ROS_INFO_THROTTLE(5, "Mode: 1 (go to target)");
            new_pose=true;
            //get to new target
            local_pos_pub.publish(ds.target_pose);

        }
        if(ds.mode==2)
        {
          ROS_INFO_THROTTLE(5, "Mode: 2 (hold position)");
          if(new_pose)
            cs.save_cloud=true;
          
          new_pose=false;
            //stay in place
            local_pos_pub.publish(ds.target_pose);
        }
        //publish target as pose
        pub_target.publish(targetPoint);
        //ds.check_position();

        ros::spinOnce();
        rate.sleep();
    }
    
 
    return 0;
}
