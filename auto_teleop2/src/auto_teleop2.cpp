#include "ros/ros.h"
#include "std_msgs/String.h"


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
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
    bool got_tele_para;
    int tele_num_vp;
    int tele_start_vp;

    auto_teleop_class():got_tele_para(false)
    {
        sub_targetPoint = node.subscribe("/mavros/setpoint_position/local", 1, &auto_teleop_class::targetPointCallback, this);
        sub_droneposPoint =node.subscribe("/mavros/local_position/pose", 1, &auto_teleop_class::droneposPointCallback, this);
        pub_teleop = node.advertise<std_msgs::String>("/teleop/keyinput", 1000);
        pub_snapshot = node.advertise<std_msgs::String>("/snapshot/pointcloud", 1000);
        pub_goal_point = node.advertise<geometry_msgs::PoseStamped>("/goal", 1);


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


     void get_tele_para(){
      //check if param are ther else use defult of 0.75 and 10
      int i;
      if (node.getParam("/teleop/num_vp", i))
      {
        ROS_INFO_THROTTLE(1 , "Got param num vp: %i", i);
        tele_num_vp= i;
        got_tele_para=true;
      }
      else
      {
        ROS_WARN_THROTTLE(1, "Failed to get param '/teleop/num_vp'");
        //object_hight=0.2;
        got_tele_para=false;
      }

      if (node.getParam("/teleop/start_vp", i))
      {
        ROS_INFO_THROTTLE(1, "Got param start vp: %i", i);
        tele_start_vp= i;
        got_tele_para=true;
      }
      else
      {
        ROS_WARN_THROTTLE(1, "Failed to get param '/teleop/start_vp'");
        //num_viepoints=20;
        got_tele_para=false;
      }


    }




ros::Publisher pub_teleop;
ros::Publisher pub_snapshot;
ros::Publisher pub_goal_point;
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
  
  ros::init(argc, argv, "auto_teleop2");

  auto_teleop_class *at = new auto_teleop_class;
  
  ros::Rate loop_rate(5);

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
  private_node_handle_.param("tol_xy", at->tol_xy, double(0.3));
  private_node_handle_.param("tol_yaw", at->tol_yaw, double(20));


  std_msgs::String msg;

  double errorx=std::numeric_limits<double>::max();
  double errory=std::numeric_limits<double>::max();
  double errorz=std::numeric_limits<double>::max();
  double erroryaw=std::numeric_limits<double>::max();

  

  bool atposition=false;
  bool atperfectpos=false;
  bool helpflag=false;
  bool timestarted=false;
  ros::Time start_time;

  while(at->targetPoint.pose.position.z==0)
  {
    ROS_INFO_THROTTLE(1 , "Waiting for target");
    ros::spinOnce();
    loop_rate.sleep();
  }

  while(!at->got_tele_para){
    at->get_tele_para();
    loop_rate.sleep();
  }


  //set frame id for target (for visualsation only)
  at->targetPoint.header.frame_id="world";
  
  //step 1 get to waypoint
  
  ROS_INFO_STREAM("starting stage 1");
  bool step1=false;
  double flyxyztol=0.3;
  double flyyawtol=50;
  bool atflypos=false;
  int current_point=at->tele_start_vp;

  while(ros::ok() && !step1){

    ros::spinOnce();
    loop_rate.sleep();
    
    errorx=fabs(at->targetPoint.pose.position.x-at->droneposPoint.pose.position.x);
    errory=fabs(at->targetPoint.pose.position.y-at->droneposPoint.pose.position.y);
    errorz=fabs(at->targetPoint.pose.position.z-at->droneposPoint.pose.position.z); 
    erroryaw = distance(tf::getYaw(at->targetPoint.pose.orientation)*(180/M_PI),tf::getYaw(at->droneposPoint.pose.orientation)*(180/M_PI));
    
    //check if error is in margin
    if(errorx< flyxyztol && errory< flyxyztol && errorz< flyxyztol && erroryaw < flyyawtol)
    {
      atposition=true;
    }
    else
    {
      atposition=false;
      timestarted=false;
    }

    //time it
    if(atposition==true && timestarted==false)
    {
      start_time = ros::Time::now();  
      timestarted=true;
    }

    if(ros::Time::now() - start_time > ros::Duration(0.5) && atposition==true)
    {

    //which way is shorter
      if(at->tele_num_vp/2>at->tele_start_vp){
        //left
        msg.data="b";
        ROS_INFO_STREAM("sending b");
        at->pub_teleop.publish(msg);
        timestarted=false;
        current_point=current_point-1;
      }
      else{
        //right
        msg.data="n";
        ROS_INFO_STREAM("sending n");
        at->pub_teleop.publish(msg);
        timestarted=false;
        current_point=current_point+1;
      }


         
    }
    //check if stage 1 complete
      if(current_point<=0 || current_point>=at->tele_num_vp-1)
      {
        step1=true;
        ROS_INFO("current_point: %i ", current_point);
      }
    //publish goal as pose
    at->pub_goal_point.publish(at->targetPoint);
  }


  ROS_INFO_STREAM("starting stage 2");

  bool step2=false;
  atposition=false;
  timestarted=false;
  bool direction;
  int helpcounter=0;
  //set direction
  if(current_point==0)
    direction=true;
  if(current_point==at->tele_num_vp-1)
    direction=false;

  while (ros::ok() && !step2)
  {
    //check if pose is close enogghu to target
    //calc erorrs (only x,y yaw are of interest)
    errorx=fabs(at->targetPoint.pose.position.x-at->droneposPoint.pose.position.x);
    errory=fabs(at->targetPoint.pose.position.y-at->droneposPoint.pose.position.y);
    errorz=fabs(at->targetPoint.pose.position.z-at->droneposPoint.pose.position.z); 
    erroryaw = distance(tf::getYaw(at->targetPoint.pose.orientation)*(180/M_PI),tf::getYaw(at->droneposPoint.pose.orientation)*(180/M_PI));
    
    //check if error is in margin
    if(errorx<at->tol_xy && errory< at->tol_xy && errorz< at->tol_xy && erroryaw < at->tol_yaw)
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
      //left or right?
      if(direction){
        //check if stage 2 is complete
        
         if(current_point>=at->tele_num_vp-1 && helpcounter>0){
          msg.data="n";        
          at->pub_snapshot.publish(msg);
          step2=true;
        }
        else
        {
          msg.data="n";
          ROS_INFO_STREAM("sending n");
          at->pub_teleop.publish(msg);
          at->pub_snapshot.publish(msg);
          timestarted=false;
          helpflag=false;
          current_point=current_point+1;
          helpcounter++;
        }
        
      }
      if(!direction){

         if(current_point<=0 && helpcounter>0){
          step2=true;
          msg.data="b";        
          at->pub_snapshot.publish(msg);
          }
        else{        
          msg.data="b";
          ROS_INFO_STREAM("sending b");
          at->pub_teleop.publish(msg);
          at->pub_snapshot.publish(msg);
          timestarted=false;
          helpflag=false;
          current_point=current_point-1;
          helpcounter++;
        }
      }      
    }

    //check if in perfec pos
    if(errorx<at->tol_xy/5 && errory< at->tol_xy/5 && errorz < at->tol_xy/5 && erroryaw < at->tol_yaw/5)
    {
      atperfectpos=true;
    }
    else
    {
      atperfectpos=false;
    }

    if(atperfectpos && !helpflag)
    {
      if(direction){
        if(current_point>=at->tele_num_vp-1 && helpcounter>0)
        {
        step2=true;
        msg.data="n";        
        at->pub_snapshot.publish(msg);
        }
      else{ 

        msg.data="n";
        ROS_INFO_STREAM("perfect sending n");
        at->pub_teleop.publish(msg);  
        at->pub_snapshot.publish(msg);    
        helpflag=true;
        current_point=current_point+1;
        helpcounter++;
        }
      }
      
      if(!direction){
        if(current_point<=0 && helpcounter>0)
        {
        step2=true;
        msg.data="b";        
        at->pub_snapshot.publish(msg);
        }
      else{ 
        msg.data="b";
        ROS_INFO_STREAM("perfect sending b");
        at->pub_teleop.publish(msg);  
        at->pub_snapshot.publish(msg);    
        helpflag=true;
        current_point=current_point-1;
        helpcounter++;
        }
      }
    }
 
    //publish goal as pose
    at->pub_goal_point.publish(at->targetPoint);

    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO_STREAM("starting stage 3");

  //step 3 return to start vp than land
  bool step3=false;

  while(ros::ok() && !step3)
  {
    ros::spinOnce();
    loop_rate.sleep();
    
    errorx=fabs(at->targetPoint.pose.position.x-at->droneposPoint.pose.position.x);
    errory=fabs(at->targetPoint.pose.position.y-at->droneposPoint.pose.position.y);
    errorz=fabs(at->targetPoint.pose.position.z-at->droneposPoint.pose.position.z); 
    erroryaw = distance(tf::getYaw(at->targetPoint.pose.orientation)*(180/M_PI),tf::getYaw(at->droneposPoint.pose.orientation)*(180/M_PI));
    
    //check if error is in margin
    if(errorx<flyxyztol && errory< flyxyztol && errorz< flyxyztol && erroryaw < flyyawtol)
    {
      atposition=true;
    }
    else
    {
      atposition=false;
      timestarted=false;
    }

    //time it
    if(atposition==true && timestarted==false)
    {
      start_time = ros::Time::now();  
      timestarted=true;
    }

    if(ros::Time::now() - start_time > ros::Duration(0.5) && atposition==true)
    {

    //which way is shorter
      if(at->tele_num_vp/2<=at->tele_start_vp){
        //left
        msg.data="n";
        ROS_INFO_STREAM("sending n");
        at->pub_teleop.publish(msg);
        timestarted=false;
        current_point=current_point+1;
      }
      else{
        //right
        msg.data="b";
        ROS_INFO_STREAM("sending b");
        at->pub_teleop.publish(msg);
        timestarted=false;
        current_point=current_point-1;
      }
    }

      //check if stage 3 complete
      if(current_point==at->tele_start_vp)
        step3=true;

    //publish goal as pose
    at->pub_goal_point.publish(at->targetPoint);

  }

  ROS_INFO_STREAM("starting stage 4 (landing)");

while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();

        msg.data="l";
        //ROS_INFO_STREAM("sending l");
        at->pub_teleop.publish(msg);
    

  }



  return 0;
}
