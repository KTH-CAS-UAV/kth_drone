/**
 * Runs the NBV algorithm on th drone

 **************************************'.
 *
 * Run this with a single command line argument:
 *
 * nbv_pcds path_to_yaml
 *
 * Where the YAML file supplied give the location of the pointclouds, target volume and sensor model. See test_files/out.yaml
 * for an example YAML file.
 *
 * The pointclouds supplied need to contain there position within the VIEWPOINT field. Compatible point clouds can be
 * captured using scripts/capture_some_clouds.py.
 *
 * The output of the program will be the view scores and the selected view. To visualise the progress in RViz, subscribe
 * to /nbv_planner/views, /nbv_planner/octomap, and /nbv_planner/volume.
 */
#include <ros/ros.h>
#include <nbv_planning/NBVFinderROS.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl/common/eigen.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>


#include <boost/filesystem.hpp>

 //TF stuff
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
 #include <eigen_conversions/eigen_msg.h>
 //TF stuff
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>


 #include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

 #include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

 //action
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <nbv_planning/setpoint_control_commandsAction.h>
#include <nbv_planning/cloud_snapshootAction.h> 
#include <nbv_planning/circular_view_pointsAction.h> 

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

#include <string>

/**
 * Helper class to wait for a single message on a given ROS topic, and return that message and unsubscribe
 */

 using namespace std;
template<class MessageType>
class WaitForMessage {
public:
    static MessageType get(const std::string &topic_name) {
        WaitForMessage waiter(topic_name);
//        ROS_INFO_STREAM("Waiting for message on " << topic_name);
        while (!waiter.got_it_) {
            ros::spinOnce();
        }
        ROS_INFO("Got it.");
        return waiter.message_;
    }

private:
    WaitForMessage(const std::string &topic_name) : got_it_(false) {
        subscriber_ = node_handle_.subscribe(topic_name, 1, &WaitForMessage::callback, this);
    }

    ros::NodeHandle node_handle_;
    ros::Subscriber subscriber_;
    MessageType message_;
    bool got_it_;

    void callback(MessageType message) {
        subscriber_.shutdown();
        message_ = message;
        got_it_ = true;
    }
};


class nbv_drone_boss
{
  protected:
    //node handler
    ros::NodeHandle nh_;
    //config action server
    actionlib::SimpleActionClient<nbv_planning::setpoint_control_commandsAction> ac_drone_setpoint; 
    actionlib::SimpleActionServer<nbv_planning::cloud_snapshootAction> as_cloud_snapshoot;
    actionlib::SimpleActionServer<nbv_planning::circular_view_pointsAction> as_circular_view_points;
    nbv_planning::cloud_snapshootFeedback feedback_cloud_snapshoot; // create messages that are used to published feedback
    nbv_planning::cloud_snapshootResult result_cloud_snapshoot;    // create messages that are used to published result
    nbv_planning::circular_view_pointsFeedback feedback_circular_view_points; // create messages that are used to published feedback
    nbv_planning::circular_view_pointsResult result_circular_view_points;    // create messages that are used to published result
    tf::TransformListener listener;

  public:
    geometry_msgs::PoseStamped nbv_pose;
    std::vector<geometry_msgs::PoseStamped> vec_circular_vp;
    string path_to_cloud;
    std::vector<Eigen::Affine3d> view_poses;
    std::vector<Eigen::Affine3d> view_poses_it;
    unsigned int nbv_index;
    bool stage1;
    bool stage2;
    bool got_tf;
    std::vector<double> tf_xyz_rpy;
    double stepsize;
    double camera_skew;

    // Create the planner
    nbv_planning::NBVFinderROS::Ptr m_planner;

    nbv_drone_boss():as_cloud_snapshoot(nh_, "cloud_snapshoot", boost::bind(&nbv_drone_boss::cloud_snapshoot, this, _1), false),
      as_circular_view_points(nh_, "circular_view_points", boost::bind(&nbv_drone_boss::circular_view_points, this, _1), false),
      ac_drone_setpoint("setpoint_control_commands", true),stage1(true),stage2(false),stepsize(0.15),camera_skew(0.649),got_tf(false)
    {
        ROS_INFO("NBV: starting cloud snap server");
        as_cloud_snapshoot.start();
        ROS_INFO("NBV: starting circular server");
        as_circular_view_points.start();
        ROS_INFO("NBV: waiting for drone setpoint server");
        ac_drone_setpoint.waitForServer();

        ROS_INFO("NBV: Boss is ready");


    }

    ~nbv_drone_boss()
    {

    }

    void get_static_tf_transform()
    {
        tf::StampedTransform transform;

      // get tf transform from world to target frame
      try{
        listener.lookupTransform("/drone_base", "/d_camera::camera_link",ros::Time(0), transform);
        tf_xyz_rpy.push_back(transform.getOrigin().x());
        tf_xyz_rpy.push_back(transform.getOrigin().y());
        tf_xyz_rpy.push_back(transform.getOrigin().z());
        
        tf::Quaternion q(transform.getRotation());
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        tf_xyz_rpy.push_back(roll);
        tf_xyz_rpy.push_back(pitch);
        tf_xyz_rpy.push_back(yaw);
        got_tf=true;        
        ROS_INFO("NBV: Got static tf");
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.1).sleep();
        return;
        }
    }

    //send drone setpoint
    void send_drone_setpoint(geometry_msgs::PoseStamped best_view)
  {
    nbv_pose=best_view;
    //Send drone setpoint
    nbv_planning::setpoint_control_commandsGoal goal;
    goal.command = 1;
    goal.pose = nbv_pose;
    // Need boost::bind to pass in the 'this' pointer
    ac_drone_setpoint.sendGoal(goal,
                boost::bind(&nbv_drone_boss::confirmation_drone_setpoint, this, _1, _2),
                actionlib::SimpleActionClient<nbv_planning::setpoint_control_commandsAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<nbv_planning::setpoint_control_commandsAction>::SimpleFeedbackCallback());
  }

  void confirmation_drone_setpoint(const actionlib::SimpleClientGoalState& state,
              const nbv_planning::setpoint_control_commandsResultConstPtr& result)
  {
    //ROS_DEBUG_NAMED("game_master", "Finished in state [%s]", state.toString().c_str());
    //ai_move =result->best_move;
  }

  //recive cloud_snapshoot
  void cloud_snapshoot(const nbv_planning::cloud_snapshootGoalConstPtr &goal)
  {
    //recived the path to the saved point cloud
    path_to_cloud=goal->cloud_path;
    Eigen::Affine3d cloud_pose = drone_p_to_view_p(goal->cloud_pose);


    //we get the cloudpath and load it from the disk
    ROS_INFO("Loading clouds from  disk...");
    nbv_planning::NBVFinder::CloudPtr cloud(new nbv_planning::NBVFinder::Cloud);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(path_to_cloud,*cloud) == -1) {
            PCL_ERROR ("Couldn't read file pcd file\n");
            return ;
    }
            

    ROS_INFO("NBV: Updating map");
    m_planner->update_current_volume(cloud, cloud_pose);
    m_planner->publish_octomap();
    std::cout << "NBV: Number of unobserved cells=";
    std::flush(std::cout);
    std::cout << m_planner->count_unobserved_cells() << std::endl;

    //select the next vie point from the circular
    m_planner->set_candidate_views(view_poses); 

    m_planner->publish_volume_marker();
    m_planner->publish_octomap_unobserved_cells();
      ros::spinOnce();
      ros::Duration(0.2).sleep();
      m_planner->publish_views();

ros::Duration(5.2).sleep();
    double score;
    unsigned int view;
    bool got_view;
    got_view = m_planner->choose_next_view(true, view, score,false);
    std::cout << "NBV: next best view is: " << view << std::endl;

    iterative_view_points(view,score);

    //TODO: should not be neccesary ...
    //view_poses.erase(view_poses.begin() + view);
    //nbv_index=view;
    //send_drone_setpoint(view);

  
    result_cloud_snapshoot.confirmation = 1;
    //ROS_DEBUG_NAMED("game_master", "%s: Done", action_name_.c_str());
    // set the action state to succeeded
    as_cloud_snapshoot.setSucceeded(result_cloud_snapshoot);    
  }

  //recive circular view points
  void circular_view_points(const nbv_planning::circular_view_pointsGoalConstPtr &goal)
  {
    ROS_INFO("NBV: Got circular view points");
    std::cout << "target size:" << goal->target.size() << std::endl;
    //recived a target:
    Eigen::Vector3f origin(goal->target[0],goal->target[1],goal->target[2]);
    Eigen::Vector3f extents(goal->size[0],goal->size[1],goal->size[2]);
    nbv_planning::TargetVolume volume(0.05, origin, extents);
    ROS_INFO("NBV: updating volume");
    m_planner->set_target_volume(volume);

    //recived the circular vew points
    vec_circular_vp=goal->circular_view_points;

    //fix the orientation so it fits the camera
    std::vector<Eigen::Affine3d> next_view_poses;
    next_view_poses = generate_all_vp(drone_p_to_view_p(vec_circular_vp[23]),10);
    m_planner->set_candidate_views(next_view_poses);


/*
    //Eigen::Affine3d originvp;
    //geometry_msgs::Pose temp_vp2;
    //temp_vp2.orientation=tf::createQuaternionMsgFromRollPitchYaw(-M_PI/2,0,M_PI/2);
    for(int i=0;i< vec_circular_vp.size();i++)
    {
        view_poses.push_back(drone_p_to_view_p(vec_circular_vp[i]));

    }
    
    

    //set the planner
    ROS_INFO("NBV: Setting poses");
    m_planner->set_candidate_views(view_poses);  

*/
    ros::Duration(0.2).sleep();
    m_planner->publish_views();
    m_planner->publish_volume_marker();
    ros::Duration(1.2).sleep();

    

  
    result_circular_view_points.confirmation = 1;
    //ROS_DEBUG_NAMED("game_master", "%s: Done", action_name_.c_str());
    // set the action state to succeeded
    ROS_INFO("NBV: sending succes");
    as_circular_view_points.setSucceeded(result_circular_view_points);  

    stage1=false;  
  }

  
  void camera_config_from_topic(){


    //read in the sensor model
    
    std::string camera_info_topic, camera_topic;

    nh_.param("camera_info_topic", camera_info_topic, std::string("/camera/depth/camera_info"));
    nh_.param("camera_topic", camera_topic, std::string("/camera/depth/camera_info"));

    // Get a sensor_msgs::CameraInfo message and use it to construct the sensor model
    ROS_INFO_STREAM("Waiting for camera info on " << camera_info_topic );
    sensor_msgs::CameraInfo camera_info = WaitForMessage<sensor_msgs::CameraInfo>::get(camera_info_topic);
    nbv_planning::SensorModel::ProjectionMatrix P(camera_info.P.data());
    nbv_planning::SensorModel sensor_model(camera_info.height, camera_info.width, P, 4, 0.3);

    m_planner = nbv_planning::NBVFinderROS::Ptr(new nbv_planning::NBVFinderROS(sensor_model, nh_));

  }

  void iterative_view_points(int view, double score)
  {
    ROS_INFO("NBV:Iterative********************************''");
    double ig_curr=score;
    double ig_next=score+1;
    Eigen::Affine3d curr_view_pose=view_poses[view];
    std::vector<Eigen::Affine3d> next_view_poses;
    unsigned int t_view;
    while(ig_curr<=ig_next)
    {
        ROS_INFO("NBV: iterative whilewith ig_curr");
      next_view_poses = generate_next_vp(curr_view_pose);
      m_planner->set_candidate_views(next_view_poses); 
      m_planner->publish_volume_marker();
      ros::spinOnce();
      ros::Duration(0.2).sleep();
      m_planner->publish_views();
      //evaluate
      
      bool got_view;
      got_view = m_planner->choose_next_view(false, t_view, ig_next,false);
      if(ig_next>ig_curr)
      {
        curr_view_pose=next_view_poses[t_view];
        ig_curr=ig_next;
      }
      else
      {
        break;
      }
        

    }
    nbv_index=t_view;
    view_poses_it=next_view_poses;
    //send the drone to the bestone
    send_drone_setpoint(view_p_to_drone_p(curr_view_pose));

  }


  std::vector<Eigen::Affine3d> generate_next_vp(Eigen::Affine3d start_vp)
  {
    std::vector<Eigen::Affine3d> next_vp;
    Eigen::Affine3d temp_vp;

    for(int i=0;i<6;i++)
    {
        temp_vp=start_vp;
        switch(i){
            case 0: temp_vp.translate(Eigen::Vector3d(0,0.0,-stepsize));
                    next_vp.push_back(temp_vp);
                    break;
            case 1: temp_vp.translate(Eigen::Vector3d(0,+stepsize,0));
                    next_vp.push_back(temp_vp);
                    break;
            case 2: temp_vp.translate(Eigen::Vector3d(-stepsize,0.0,0));
                    next_vp.push_back(temp_vp);
                    break;
            case 3: temp_vp.translate(Eigen::Vector3d(+stepsize,0.0,0));
                    next_vp.push_back(temp_vp);
                    break;
            case 4: temp_vp.translate(Eigen::Vector3d(0,-stepsize,0));
                    next_vp.push_back(temp_vp);
                    break;
            case 5: temp_vp.translate(Eigen::Vector3d(0,0.0,+stepsize));
                    next_vp.push_back(temp_vp);
                    break;

        }
    }

    return next_vp;
  }


  std::vector<Eigen::Affine3d> generate_all_vp(Eigen::Affine3d start_vp, int dim)
  {
    std::vector<Eigen::Affine3d> next_vp;
    Eigen::Affine3d temp_vp;
    //find outer corner vp
    temp_vp=start_vp;
    temp_vp.translate(Eigen::Vector3d(-stepsize*dim,0.0,0));
    temp_vp.translate(Eigen::Vector3d(0,-stepsize*dim,0));
    temp_vp.translate(Eigen::Vector3d(0,0.0,-stepsize*dim));
    
    for(int i=0;i<dim*2;i++)
    {
        

        for(int j=0;j<dim*2;j++)
        {
            for(int k=0;k<dim*2;k++)
            {
                next_vp.push_back(temp_vp);
                temp_vp.translate(Eigen::Vector3d(0,0.0,+stepsize));
            }
            temp_vp.translate(Eigen::Vector3d(0,0.0,-stepsize*dim*2));
            temp_vp.translate(Eigen::Vector3d(0,+stepsize,0));
        }
        temp_vp.translate(Eigen::Vector3d(0,-stepsize*dim*2,0));
        temp_vp.translate(Eigen::Vector3d(+stepsize,0,0));
    }   

    return next_vp;
  }


  Eigen::Affine3d drone_p_to_view_p(geometry_msgs::PoseStamped drone_p)
  {
    Eigen::Affine3d view_p;

    geometry_msgs::Pose temp =drone_p.pose;
    temp.position.x+=tf_xyz_rpy[0];
    temp.position.y+=tf_xyz_rpy[1];
    temp.position.z+=tf_xyz_rpy[2];
    
    tf::Quaternion q(temp.orientation.x, temp.orientation.y, temp.orientation.z, temp.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    /*
    roll+=M_PI/2-camera_skew;
    pitch+=M_PI;
    yaw+=M_PI/2;
    */
    roll+= tf_xyz_rpy[3];
    pitch+= tf_xyz_rpy[4];
    yaw+= tf_xyz_rpy[5];
    temp.orientation=tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
      
    tf::poseMsgToEigen  (temp,view_p);

    return view_p;
  }



  geometry_msgs::PoseStamped view_p_to_drone_p(Eigen::Affine3d view_p)
  {
    geometry_msgs::PoseStamped drone_p;
    geometry_msgs::Pose temp;

    tf::poseEigenToMsg(view_p,temp);
    tf::Quaternion q(temp.orientation.x, temp.orientation.y, temp.orientation.z, temp.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    /*
    roll-=M_PI/2-camera_skew;
    pitch-=M_PI;
    yaw-=M_PI/2;
    
    */
    roll-=tf_xyz_rpy[3];
    pitch-= tf_xyz_rpy[4];
    yaw-= tf_xyz_rpy[5];
    
    temp.orientation=tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
    temp.position.x-=tf_xyz_rpy[0];
    temp.position.y-=tf_xyz_rpy[1];
    temp.position.z-=tf_xyz_rpy[2];
    drone_p.pose=temp;
    drone_p.header.frame_id = "/world";
    drone_p.header.stamp = ros::Time::now();

    return drone_p;
  }

};



int main(int argc, char **argv) {
    ROS_INFO("NBV");
    ros::init(argc, argv, "nbv_planner");

    //generate the boss
    nbv_drone_boss nbv_db;

    //set up sensor model
    ROS_INFO("NBV: get sensor model");
    nbv_db.camera_config_from_topic();
    //get static transform
     while(ros::ok() && !nbv_db.got_tf)
    {
        ROS_INFO("NBV: Waiting tf transform");
        nbv_db.get_static_tf_transform();
        ros::spinOnce();
        ros::Duration(0.2).sleep();
    }
    ROS_INFO("NBV: preparations complete");
    //we wait for the circular view points and the target
    while(ros::ok() && nbv_db.stage1)
    {
        ROS_INFO("NBV: Waiting for circular viewpoints and a target man ...");
        ros::spinOnce();
        ros::Duration(0.2).sleep();
    }

    //we have he target and our initial vies. time for some informationgain
    double score;
    unsigned int view;
    bool got_view;

    //research mode:
    got_view = nbv_db.m_planner->choose_next_view(false, view, score,true);

    //normal
    //got_view = nbv_db.m_planner->choose_next_view(false, view, score,false);

    //refine view with iterative aproche
    nbv_db.iterative_view_points(view,score);

    //delete the view pose
    //nbv_db.view_poses.erase(nbv_db.view_poses.begin() + view);


    //we send the best view of to the drone and weit for the pointcloud to be send back
    //nbv_db.nbv_index=view;
    //nbv_db.send_drone_setpoint(view);

    //we are in the main stage
    ROS_INFO("Starting main stage");
    while(ros::ok())
    {
        nbv_db.m_planner->publish_volume_marker();
        ros::spinOnce();
        ros::Duration(0.2).sleep();
        nbv_db.m_planner->publish_views();
        //TODO: exit condition?????
    }



    std::cout << "Done, now spinning..." << std::endl;

    try {
        ros::spin();
    } catch (std::runtime_error &e) {
        ROS_ERROR("nbv_planner_server exception: %s", e.what());
        return -1;
    }

    return 0;
}
