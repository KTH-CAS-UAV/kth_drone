//generates a circular view point around a tagret given hight and radius of the encupseling cylinder


#include <ros/ros.h>
#include <inttypes.h>

#include <nav_msgs/OccupancyGrid.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>

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

//action
#include <actionlib/client/simple_action_client.h>
#include <circle_vp/circular_view_pointsAction.h> 

//teleop
#include "std_msgs/String.h"
#include <string>
#include <std_msgs/Int8.h>

// mavros stuff
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>



using namespace std;


class circle_view_points
{

  private:   
    tf::TransformBroadcaster br;
    tf::TransformListener listener;
    ros::NodeHandle nh_;
    ros::Subscriber sub_dronepos;
    actionlib::SimpleActionClient<circle_vp::circular_view_pointsAction> ac_circle_vp;
  
  public:
    bool got_params;
    std::vector<geometry_msgs::PoseStamped> circular_view_points;
    std::vector<float> a_target;
    std::vector<float> size;
    //target spezifications
    double object_h;
    double object_r;
    int view_point_number;
    //camera
    double cam_alpha; //field of view in rad
    double cam_tau; // tielt angle camera in rad
    double cam_beta; // magin in rad
    //cage
    double x_dim;
    double y_dim;
    double z_dim;
    //action sending
    bool send_vp_succesfull;
    ros::Publisher vp_pub; 


    circle_view_points():got_params(false),x_dim(2.0),y_dim(2.0),z_dim(2.2),ac_circle_vp("circular_view_points", true),send_vp_succesfull(false)
    {
      //set up subs/pubs
      vp_pub = nh_.advertise<geometry_msgs::PoseArray>( "circular_view_points", 1 );
      ac_circle_vp.waitForServer();
              vp_pub = nh_.advertise<geometry_msgs::PoseArray>( "CCCCCCview_points", 1 );


    }

    ~circle_view_points()
    {

    }

    //action
    void send_circular_view_points()
  {
    //Send the gameboard *******
    circle_vp::circular_view_pointsGoal goal;
    goal.circular_view_points = circular_view_points;
    std::cout << "***********" << a_target.size() << std::endl;
    goal.target =a_target;
    goal.size=size;
    // Need boost::bind to pass in the 'this' pointer
    ac_circle_vp.sendGoal(goal,
                boost::bind(&circle_view_points::confirmation_circular_view_points, this, _1, _2),
                actionlib::SimpleActionClient<circle_vp::circular_view_pointsAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<circle_vp::circular_view_pointsAction>::SimpleFeedbackCallback());
  }

  void confirmation_circular_view_points(const actionlib::SimpleClientGoalState& state,
              const circle_vp::circular_view_pointsResultConstPtr& result)
  {
    ROS_INFO("Got confirmation!");
    send_vp_succesfull=true;
  }


    void generate_circular_vp(std::string target){

      tf::StampedTransform transform;

      // get tf transform from world to target frame
      try{
        listener.lookupTransform("/world", target, ros::Time(0), transform);

        circular_view_points.clear();
        a_target.clear();
        size.clear();

        tf::Vector3 tp(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z()); // target point 3D
          
        a_target.push_back(tp.getX());
        a_target.push_back(tp.getY());
        a_target.push_back(tp.getZ());
        size.push_back(object_r*2*1.5);
        size.push_back(object_r*2*1.5);
        size.push_back(object_h*1.5);

        //calc Z
        double vp_z;
        double vp_r;
        double help_tan_m;
        double help_tan_p;
        help_tan_m=tan(cam_tau-(cam_alpha/2)+cam_beta);
        help_tan_p=tan(cam_tau+(cam_alpha/2)-cam_beta);
        vp_z=((2*object_r+object_h/help_tan_m)*(help_tan_m*help_tan_p))/(help_tan_p- help_tan_m);
        vp_r=(vp_z/help_tan_p)+object_r;
        vp_z=vp_z+tp.getZ();
        ROS_INFO("vp r: %f",vp_r);
        if(vp_z>2.2)
          vp_z=2.2;
        ROS_INFO("vp_z: %f",vp_z);
        double angle = 2*M_PI/view_point_number;

          // generatepoints vp around target given radius and num_vp
          // store points in verctor
        geometry_msgs::PoseStamped temp_vp;
        
        temp_vp.header.frame_id = "/world";
        double temp_angle;
        for(int i=0;i<view_point_number;i++){
          temp_angle=angle*i;
          temp_vp.pose.position.x=(tp.getX()+vp_r*cos(temp_angle));
          temp_vp.pose.position.y=(tp.getY()+vp_r*sin(temp_angle));
          temp_vp.pose.position.z=vp_z;
          temp_vp.pose.orientation = tf::createQuaternionMsgFromYaw(temp_angle+M_PI);
          temp_vp.header.stamp = ros::Time::now();
          circular_view_points.push_back(temp_vp);
        }        
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.1).sleep();
        return;
      }      

      //Check if view points are reachible
      int maker=0;
      int count=0;
      std::vector<int> goodones;
      std::cout << "viewpointsize: " << circular_view_points.size() << std::endl;
      for(int i=0; i<circular_view_points.size();i++){
        //ROS_INFO("for loop %i",i);
        // se if valid in cage
        if(circular_view_points[i].pose.position.x > -x_dim/2 && circular_view_points[i].pose.position.x < x_dim/2 && 
          circular_view_points[i].pose.position.y > -y_dim/2 && circular_view_points[i].pose.position.y < y_dim/2 &&
           circular_view_points[i].pose.position.z < z_dim){
          ROS_INFO("valid %i",i); 
          goodones.push_back(i);  
        }
        else
        { 
          
          
          maker=i;
          count++;
          ROS_INFO("rejected %i",i);   
        }
      }
      //kill the bad ones
      std::vector<geometry_msgs::PoseStamped> temp1;
      for (int i = 0; i < goodones.size(); ++i)
      {
        temp1.push_back(circular_view_points[goodones[i]]);
                
          
      }
      circular_view_points=temp1;
      //std::cout << "maker: " << maker << " , count: " << count << std::endl;
      //make  view point 0t the one after rejection
      std::vector<geometry_msgs::PoseStamped> tempo;
      int cvp_size=circular_view_points.size()-1;
      std::cout << "viewpointsize: " << circular_view_points.size() << std::endl;
      for (int i = 0; i < circular_view_points.size(); ++i)
      {
        std::cout << "viewpointsize: " << circular_view_points.size() << std::endl;

         std::cout << "maker: " << maker << " , count: " << count << " ,i: " << i << std::endl;

        if((i + maker - count) >= cvp_size)
        {
          std::cout << "1 " << (i+maker-count) << "   " << (circular_view_points.size()-1) << std::endl;
          tempo.push_back(circular_view_points[i+maker - count - cvp_size]);
        }
        else
        {
          if((i + maker - count)>=0)
          {
            std::cout << "2" << std::endl;
            tempo.push_back(circular_view_points[i + maker - count]);
          }
          else
          {
            std::cout << "3" << std::endl;
            tempo.push_back(circular_view_points[i]);
          }
        }
      }
      circular_view_points=tempo;
      std::cout << "we have #: " << circular_view_points.size() << " ,number of viewpoints "  << std::endl;

      //publish view points
      geometry_msgs::PoseArray poseArray; 
      poseArray.header.stamp = ros::Time::now();
      poseArray.header.frame_id = "/world";
      for (int i = 0; i < circular_view_points.size(); ++i)
     {      
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.pose.position.x = circular_view_points[i].pose.position.x;
        temp_pose.pose.position.y = circular_view_points[i].pose.position.y;
        temp_pose.pose.position.z = circular_view_points[i].pose.position.z;

        temp_pose.pose.orientation.x = circular_view_points[i].pose.orientation.x;
        temp_pose.pose.orientation.y = circular_view_points[i].pose.orientation.y;
        temp_pose.pose.orientation.z = circular_view_points[i].pose.orientation.z;
        temp_pose.pose.orientation.w = circular_view_points[i].pose.orientation.w;;
        poseArray.poses.push_back(temp_pose.pose);
     }
     vp_pub.publish(poseArray);

    }


    void getcyrcleparam(){
      //check if param are ther else wait and try later
      double d;
      bool got_para_h=false;
      bool got_para_r=false;
      bool got_para_tau=false;
      bool got_para_alpha=false;
      bool got_para_beta=false;
      bool got_para_num_vp=false;

      //object hight
      if (nh_.getParam("/object_height", d))
      {
        ROS_INFO_THROTTLE(1 , "Got param /object_height : %f", d);
        object_h= d;
        got_para_h=true;
      }
      else
      {
        ROS_WARN_THROTTLE(1, "Failed to get param '/object_hight'");
        got_para_h=false;
      }

      //object radius
      if (nh_.getParam("/object_radius", d))
      {
        ROS_INFO_THROTTLE(1, "Got param /object_radius : %f", d);
        object_r= d;
        got_para_r=true;
      }
      else
      {
        ROS_WARN_THROTTLE(1, "Failed to get param '/object_radius'");
        //num_viepoints=20;
        got_para_r=false;
      }

      //cam alpha
      if (nh_.getParam("/cam_alpha", d))
      {
        ROS_INFO_THROTTLE(1, "Got param /cam_alpha : %f", d);
        cam_alpha= d*M_PI/180;
        got_para_alpha=true;
      }
      else
      {
        ROS_WARN_THROTTLE(1, "Failed to get param '/cam_alpha'");
        //num_viepoints=20;
        got_para_alpha=false;
      }

      //cam beta
      if (nh_.getParam("/cam_beta", d))
      {
        ROS_INFO_THROTTLE(1, "Got param /cam_beta : %f", d);
        cam_beta= d*M_PI/180;
        got_para_beta=true;
      }
      else
      {
        ROS_WARN_THROTTLE(1, "Failed to get param '/cam_beta'");
        //num_viepoints=20;
        got_para_beta=false;
      }

      //cam tau
      if (nh_.getParam("/cam_tau", d))
      {
        ROS_INFO_THROTTLE(1, "Got param /cam_tau : %f", d);
        cam_tau= d*M_PI/180;
        got_para_tau=true;
      }
      else
      {
        ROS_WARN_THROTTLE(1, "Failed to get param '/cam_tau'");
        //num_viepoints=20;
        got_para_tau=false;
      }

      //view_point_number
      if (nh_.getParam("/view_point_number", d))
      {
        ROS_INFO_THROTTLE(1, "Got param view_point_number : %f", d);
        view_point_number= (int)d;
        got_para_num_vp=true;
      }
      else
      {
        ROS_WARN_THROTTLE(1, "Failed to get param '/object_radius'");
        //num_viepoints=20;
        got_para_num_vp=false;
      }

      //check if we have all of them
      if(got_para_r && got_para_h && got_para_tau && got_para_beta && got_para_alpha && got_para_num_vp){
        got_params=true;
        ROS_INFO_THROTTLE(1, "Got all the Params.");
      }
      else{
        got_params=false;
        ROS_WARN_THROTTLE(1, "Failed to get all params");
      }

    }

};

int main(int argc, char **argv){

  
  ros::init(argc, argv, "circular_vp"); 

  //ros::Rate rate(10.0);

  circle_view_points cvp;

  //grap the params from the server
  while(ros::ok() && !cvp.got_params)
  {
    ros::spinOnce();
    cvp.getcyrcleparam();
    ros::Duration(1.0).sleep(); // sleep a little 
  } 
  
  ROS_INFO("Got Parameters");


  //calculate the circular view points
  cvp.generate_circular_vp("/target");

  ROS_INFO("Generatet circular viewpoints");

  //send view points to nvb node
  ROS_INFO("Sending them to NVB");
  cvp.send_circular_view_points();

  // wait for confirmation,
  while(ros::ok() && !cvp.send_vp_succesfull)
  {
    ros::Duration(1.0).sleep(); // sleep a little 
    ROS_INFO_THROTTLE(5,"Waiting for confirmation");
  }

  ros::shutdown();
  return 0;
};
