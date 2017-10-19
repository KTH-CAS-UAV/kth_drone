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


/**
 * Helper class to wait for a single message on a given ROS topic, and return that message and unsubscribe
 */
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



int main(int argc, char **argv) {
    ros::init(argc, argv, "nbv_planner");
    ros::NodeHandle n;


    //current pose of drone


    //read in the sensor model
    /*
    std::string camera_info_topic, camera_topic;

    n.param("camera_info_topic", camera_info_topic, std::string("/head_xtion/depth/camera_info"));
    n.param("camera_topic", camera_topic, std::string("/points/depth/camera_info"));

    // Get a sensor_msgs::CameraInfo message and use it to construct the sensor model
    ROS_INFO_STREAM("Waiting for camera info on " << camera_info_topic );
    sensor_msgs::CameraInfo camera_info = WaitForMessage<sensor_msgs::CameraInfo>::get(camera_info_topic);
    nbv_planning::SensorModel::ProjectionMatrix P(camera_info.P.data());
    nbv_planning::SensorModel sensor_model(camera_info.height, camera_info.width, P, 4, 0.3);
*/


//*****************temp
    YAML::Node config;
    boost::filesystem::path config_path;

    if (argc!=2){
        std::cout << "Must provide just a single argument, the path to the yaml file describing the PCDs.." << std::endl;
        return 1;
    } else {
        try {
            boost::filesystem::path p(argv[1]);
            if (!boost::filesystem::is_regular(p)) {
                std::cout << "Argument supplied is not a regular file!\n";
            }
            config_path = p.parent_path();
            config = YAML::LoadFile(argv[1]);
            // Do some checks here for the contents of the YAML :-)
        } catch (YAML::BadFile e) {
            std::cout << "Can't load specified yaml file, make sure first parameter is YAML description of PCDs." << std::endl;
            return 1;
        }
    }

    std::cout << "Loaded configuration from " << argv[1] << std::endl;
    std::cout << "Config path: " << config_path << std::endl;

    // Set up the camera parameters
    nbv_planning::SensorModel::ProjectionMatrix P;
    for(int i = 0; i < P.size(); ++i){
        P(i) = config["sensor_model"]["projection_matrix"][i].as<double>();
    }
    nbv_planning::SensorModel sensor_model(config["sensor_model"]["height"].as<int>(),
                                           config["sensor_model"]["width"].as<int>(),
                                           P,
                                           config["sensor_model"]["max_range"].as<float>(),
                                           config["sensor_model"]["min_range"].as<float>(),
                                           config["sensor_model"]["sub_sample"].as<int>());
    std::cout << "Loaded sensor model: " << sensor_model << std::endl;




    // Create the planner
    nbv_planning::NBVFinderROS::Ptr m_planner;
    m_planner = nbv_planning::NBVFinderROS::Ptr(new nbv_planning::NBVFinderROS(sensor_model, n));



    // Set the target volume
    //TODO: make it depenten on target tf
    /*scale: 0.05
  origin: [-2.8, -3.75, 1.0]
  extents: [0.5,0.5,0.3]
*/

    Eigen::Vector3f origin(-2.8,-3.75,1.0);
    Eigen::Vector3f extents(0.5,0.5,0.3);
    nbv_planning::TargetVolume volume(0.05,
                                      origin, extents);

    m_planner->set_target_volume(volume);
    m_planner->publish_volume_marker();
    ros::Publisher vp_pub;  
    
    vp_pub = n.advertise<geometry_msgs::PoseArray>( "view_points", 1 );
    std::vector<geometry_msgs::Pose> vp_pubvec;

    //use cycular view point calculation from ecrm2017 paper
    ROS_INFO("calculating cyrcular viewpoints");
    std::vector<Eigen::Affine3d> view_poses;
    //std::vector<nbv_planning::NBVFinder::CloudPtr> view_clouds;
    tf::Vector3 tp(-2.8,-3.75,1.5);
    double vp_z=1.5;
    double vp_r=1.5;
    int num_viepoints=6;
    geometry_msgs::Pose temp_vp;
    geometry_msgs::Pose temp_vp_drone;
    double angle = 2*M_PI/num_viepoints;
    double temp_angle;
    Eigen::Affine3d originvp;
    for(int i=0;i<num_viepoints;i++){
        temp_angle=angle*i;
        temp_vp.position.x=(tp.getX()+vp_r*cos(temp_angle));
        temp_vp.position.y=(tp.getY()+vp_r*sin(temp_angle));
        temp_vp.position.z=vp_z;
        temp_vp_drone.position=temp_vp.position;
        temp_vp_drone.orientation = tf::createQuaternionMsgFromYaw(temp_angle+M_PI);
        temp_vp.orientation = tf::createQuaternionMsgFromRollPitchYaw(-M_PI/2,0,temp_angle+M_PI/2);
        
              //temp_vp.pose.orientation.z=temp_angle;
        tf::poseMsgToEigen  (temp_vp,originvp);
        //originvp= Eigen::Translation3d(temp_vp.position.x, temp_vp.position.y, temp_vp.position.z) *
          //                   Eigen::Quaterniond(temp_vp.orientation.w, temp_vp.orientation.x, temp_vp.orientation.y, temp_vp.orientation.z);
        vp_pubvec.push_back(temp_vp_drone);
        view_poses.push_back(originvp);
        
    } 

    int vec_length=vp_pubvec.size();
      geometry_msgs::PoseArray poseArray; 
      poseArray.header.stamp = ros::Time::now();
      poseArray.header.frame_id = "/map";
      for (int i = 0; i < vec_length; ++i)
     {      
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.pose.position.x = vp_pubvec[i].position.x;
        temp_pose.pose.position.y = vp_pubvec[i].position.y;
        temp_pose.pose.position.z = vp_pubvec[i].position.z;

        temp_pose.pose.orientation.x = vp_pubvec[i].orientation.x;
        temp_pose.pose.orientation.y = vp_pubvec[i].orientation.y;
        temp_pose.pose.orientation.z = vp_pubvec[i].orientation.z;
        temp_pose.pose.orientation.w = vp_pubvec[i].orientation.w;;
        poseArray.poses.push_back(temp_pose.pose);
     }

     for(int i=0;i<20;i++)
     {
        vp_pub.publish(poseArray);
        ros::Duration(0.1).sleep();

     }
     
    

    // Setup planner and go through each view selecting the best each time
    m_planner->set_candidate_views(view_poses);
    m_planner->publish_views();
    
    bool got_view = true;
    double score;
//    unsigned int view =0;
    while (got_view) {
        unsigned int view;
        got_view = m_planner->choose_next_view(true, view, score);

        //we get thw best starting viewpint and use our gradient decent thingy
        double max_score=1000;
        double threshold_score=1;
        double stepsize=0.15; //15 cm step size
        bool decent=true;
        while(decent)
        {
            std::cout << "********** starting while **************** "  << std::endl;
            //get current best pose
            geometry_msgs::Pose current_best_pose=vp_pubvec[view];
            geometry_msgs::Pose current_best_view;
            tf::poseEigenToMsg  (view_poses[view],current_best_view);

            //generate new views around the current best view
            vp_pubvec.clear();
            view_poses.clear();
            geometry_msgs::Pose temp_vp_2;
            geometry_msgs::Pose temp_vp_drone_2;
            for(int i=0;i<7;i++)
            {
                temp_vp_drone_2=current_best_pose;
                switch (i) 
                {
                    case 0: 
                    {
                        temp_vp_drone_2.position.z-=stepsize;
                        break;
                    }
                    case 1: 
                    {
                        temp_vp_drone_2.position.y+=stepsize;
                        break;
                    }
                    case 2: 
                    {
                        temp_vp_drone_2.position.x-=stepsize;
                        break;
                    }
                    case 3: 
                    {                        
                        break;
                    }
                    case 4: 
                    {
                        temp_vp_drone_2.position.x+=stepsize;
                        break;
                    }
                    case 5: 
                    {
                        temp_vp_drone_2.position.y-=stepsize;
                        break;
                    }
                    case 6: 
                    {
                        temp_vp_drone_2.position.z+=stepsize;
                        break;
                    }
                }
                //adjust the view
                temp_vp_2=temp_vp_drone_2;
                temp_vp_2.orientation=current_best_view.orientation;
                vp_pubvec.push_back(temp_vp_drone_2);
                Eigen::Affine3d originvp_2;
                tf::poseMsgToEigen  (temp_vp_2,originvp_2);
                view_poses.push_back(originvp_2);

            }

            int vec_length=vp_pubvec.size();
              //geometry_msgs::PoseArray poseArray; 
                //poseArray.clear();
              poseArray.header.stamp = ros::Time::now();
              poseArray.header.frame_id = "/map";
              for (int i = 0; i < vec_length; ++i)
             {      
                geometry_msgs::PoseStamped temp_pose;
                temp_pose.pose.position.x = vp_pubvec[i].position.x;
                temp_pose.pose.position.y = vp_pubvec[i].position.y;
                temp_pose.pose.position.z = vp_pubvec[i].position.z;

                temp_pose.pose.orientation.x = vp_pubvec[i].orientation.x;
                temp_pose.pose.orientation.y = vp_pubvec[i].orientation.y;
                temp_pose.pose.orientation.z = vp_pubvec[i].orientation.z;
                temp_pose.pose.orientation.w = vp_pubvec[i].orientation.w;;
                poseArray.poses.push_back(temp_pose.pose);
             }

             for(int i=0;i<20;i++)
             {
                vp_pub.publish(poseArray);
                ros::Duration(0.1).sleep();

             }

             m_planner->set_candidate_views(view_poses);
            m_planner->publish_views();
            got_view = m_planner->choose_next_view(true, view, score);
            std::cout << "********** got view num: " << view << std::endl;
            if(view==3)
            {
                decent=false;
                std::cout << " canceling while " << std::endl;
            }
                

            max_score=score;


        }
        
        if (got_view) {
            ROS_INFO("Updating map");
            //planner.update_current_volume(view_clouds[view], view_poses[view]);
            //planner.publish_octomap();
            std::cout << "Number of unobserved cells=";
            std::flush(std::cout);
            //std::cout << planner.count_unobserved_cells() << std::endl;
            got_view=false;
        }
            
            
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
