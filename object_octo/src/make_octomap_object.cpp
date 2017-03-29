//read in point cloud and make color octo map around target

#include <iostream>
#include <stdlib.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>  // allows subscribing/publishing PCL types as ROS msgs

// PCL
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>

// Octomap
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

// Iterator stuff
#include <iterator>

//TF stuff
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

//#include <tf/Vector3.h>


using namespace pcl;
using namespace std;
using namespace octomap;



namespace cloud_assembler
{

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud2;


class CloudAssembler
{


public:

octomap::ColorOcTree color_tree ;
tf::Vector3 target;
double targetsize;
  CloudAssembler():color_tree(0.005),target(1.0,0.0,1.6),targetsize(0.5)
{
  ros::NodeHandle private_nh("~");

  output_pub_ = node_.advertise<octomap_msgs::Octomap> ("/assembled_cloud", 1);

  cloud_sub_ = node_.subscribe("/cloud_in", 1, &CloudAssembler::cloudCallback, this);


}

void cloudCallback(const sensor_msgs::PointCloud2& cloud_in)
{
  
    
ROS_INFO_STREAM("Callback recived");

tf::StampedTransform transform;

    try{
      listener.lookupTransform("/world", cloud_in.header.frame_id,  
                               cloud_in.header.stamp, transform);
    
     

    //transform point cloud
    Eigen::Matrix4f   transform_Matrix;
    pcl_ros::transformAsMatrix(transform, transform_Matrix);
    sensor_msgs::PointCloud2 transformed_cloud;
    pcl_ros::transformPointCloud(transform_Matrix, cloud_in, transformed_cloud ); 

    //filter cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_unfiltered(new pcl::PointCloud<pcl::PointXYZRGBA>);
    // Create the filtering object
    PointCloud2 temp_cloud2;
    pcl::fromROSMsg(transformed_cloud, temp_cloud2);
    *cloud_unfiltered=temp_cloud2;
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    pass.setInputCloud (cloud_unfiltered);
    //pass.setFilterLimitsNegative (true);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (target.getX()-targetsize/2, target.getX()+targetsize/2);
    pass.filter (*cloud_filtered);
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (target.getY()-targetsize/2, target.getY()+targetsize/2);
    pass.filter (*cloud_filtered);
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (target.getZ()-targetsize/2, target.getZ()+targetsize/2);
    pass.filter (*cloud_filtered);
    
    
    //check if cloud is empty:
    ROS_INFO_STREAM(cloud_filtered->points.size());
    if( cloud_filtered->points.size()>0){
      
    

      //Map pointcloud to color oct tree  
      octomap::Pointcloud octomap_cloud;
      sensor_msgs::PointCloud2 temp_octo;
      pcl::toROSMsg(*cloud_filtered, temp_octo);
      octomap::pointCloud2ToOctomap(temp_octo, octomap_cloud);


      point3d origin(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
    
      //new clouds for ittaration ... shoudent be necceary...
      PointCloud2 temp_cloud;
      pcl::fromROSMsg(temp_octo, temp_cloud);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Pcloud_in(new pcl::PointCloud<pcl::PointXYZRGBA>);
      *Pcloud_in=temp_cloud;

      //reed in point cloud  
      color_tree.insertPointCloud(octomap_cloud,origin);
      
      // Add in the colors
      pcl::PointCloud<pcl::PointXYZRGBA>::const_iterator it;
      for (it = Pcloud_in->begin(); it != Pcloud_in->end(); ++it) 
          {
            if (!isnan (it->x) && !isnan (it->y) && !isnan (it->z))  // Check if the point is invalid
            color_tree.averageNodeColor(it->x, it->y, it->z, it->r, it->g, it->b);
          }


      color_tree.updateInnerOccupancy();  // updates inner node colors, too

      color_tree.prune(); //pruning tree

      // set massage components
      octomap.binary = 1 ;
      octomap.id = 2 ;
      octomap.resolution =0.005 ;
      octomap.header.frame_id = "/world";
      octomap.header.stamp = ros::Time::now();
      bool res = octomap_msgs::fullMapToMsg(color_tree, octomap);
      output_pub_.publish(octomap) ; // publish cloud

      }

    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(0.1).sleep();
    }
  
}


private:
  ros::NodeHandle node_;

  ros::Publisher output_pub_;
  ros::Subscriber cloud_sub_;

  tf::TransformListener tf_;
  
  tf::TransformListener listener;
  octomap_msgs::Octomap octomap ;



};





}; // namespace
octomap::ColorOcTree color_tree (0.05);
int main(int argc, char ** argv)
{
  //ros::Duration(0.5).sleep();
  ROS_INFO_STREAM("starting");
  ros::init(argc, argv, "cloud_assembler");
  
  cloud_assembler::CloudAssembler cloud_assembler;
  ros::spin();
/*
  os::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() ){
        ros::spinOnce();
        //cloud_assembler.publishocto();
        rate.sleep();
    }
*/
  return 0;
}

