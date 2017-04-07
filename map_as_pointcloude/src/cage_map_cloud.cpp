#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */


ros::Publisher pub;

sensor_msgs::PointCloud2 planare_cloud(double x1,double y1, double z1, double x2,double y2, double z2, double x3,double y3, double z3, int density, double noiselevel, std::string framename)
{
   /* initialize random seed: */
  srand (time(NULL));
 
  float cloudwidth=sqrt(pow((x1-x2),2)+pow((y1-y2),2)+pow((z1-z2),2))*density;
  float cloudhigth=sqrt(pow((x1-x3),2)+pow((y1-y3),2)+pow((z1-z3),2))*density;

  // build cloud
  sensor_msgs::PointCloud2 pointcloud_;
  // Prepare constant parts of the pointcloud message to be  published
  pointcloud_.header.frame_id = framename;
  pointcloud_.width  = static_cast<int>(cloudwidth);
  pointcloud_.height = static_cast<int>(cloudhigth);
  pointcloud_.fields.resize(3);

  // Set x/y/z as the only fields
  pointcloud_.fields[0].name = "x";
  pointcloud_.fields[1].name = "y";
  pointcloud_.fields[2].name = "z";

  int offset = 0;
  // All offsets are *4, as all field data types are float32
  for (size_t d = 0; d < pointcloud_.fields.size(); ++d, offset += 4)
  {
    pointcloud_.fields[d].count    = 1;
    pointcloud_.fields[d].offset   = offset;
    pointcloud_.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
  }

  pointcloud_.point_step = offset;
  pointcloud_.row_step   = pointcloud_.point_step * pointcloud_.width;

  pointcloud_.data.resize(pointcloud_.width*pointcloud_.height* pointcloud_.point_step);
  pointcloud_.is_bigendian = false;
  pointcloud_.is_dense     = true;

  int arrayPosition=0;
  int arrayPosX=0;
  int arrayPosY=0;
  int arrayPosZ=0;
  float X = 0;
  float Y = 0;
  float Z = 0;

  std::vector<double> range_W, range_H, range_ZH, range_ZW;
  double delta = sqrt(pow((x1-x2),2)+pow((y1-y2),2)) /double(pointcloud_.width);
  for(int i=0; i<pointcloud_.width; i++) {
      range_W.push_back(i*delta);
  }
  double phi_W=atan2(x1-x2,y1-y2);
  if(phi_W<0){
    phi_W+M_PI;
  }
  delta = sqrt(pow((x1-x3),2)+pow((y1-y3),2)) /double(pointcloud_.height);
  for(int i=0; i<pointcloud_.height; i++) {
      range_H.push_back(i*delta);
  }
  double phi_H=atan2(x1-x3,y1-y3);
  if(phi_H<0){
    phi_H+M_PI;
  }
  delta = std::abs(double(z1-z2))/double(pointcloud_.width);//sqrt(pow((z1-z2),2)+pow((y1-y3),2)) /double(density);   
  for(int i=0; i<pointcloud_.width; i++) {
      range_ZW.push_back(i*delta);
  }  
  delta = std::abs(double(z1-z3))/double(pointcloud_.height);//sqrt(pow((z1-z2),2)+pow((y1-y3),2)) /double(density);   
  for(int i=0; i<pointcloud_.height; i++) {
      range_ZH.push_back(i*delta);
  }

  for(int v=0;v<pointcloud_.height ;v++)
  {

    for(int u=0;u<pointcloud_.width;u++)
    {
      arrayPosition = v*pointcloud_.row_step + u*pointcloud_.point_step;

      // compute position in array where x,y,z data start
      arrayPosX = arrayPosition + pointcloud_.fields[0].offset; // X has an offset of 0
      arrayPosY = arrayPosition + pointcloud_.fields[1].offset; // Y has an offset of 4
      arrayPosZ = arrayPosition + pointcloud_.fields[2].offset; // Z has an offset of 8
      
      X = x1-range_W[u]*sin(phi_W)+((double) rand() / (RAND_MAX))*noiselevel;
      Y = y1-range_H[v]*cos(phi_H)+((double) rand() / (RAND_MAX))*noiselevel;
      if(z1<=z2 && z1<=z3){
        Z = z1+range_ZW[u]+range_ZH[v];
      } 
      if(z1>=z2 && z1>=z3){
        Z = z1-range_ZW[u]-range_ZH[v];
      }
      if(z1>z2 && z1<z3){
        Z = z1-range_ZW[u]+range_ZH[v];
      }
      if(z1<z2 && z1>z3){
        Z = z1+range_ZW[u]-range_ZH[v];
      }
      Z=Z+((double) rand() / (RAND_MAX))*noiselevel;

      memcpy(&pointcloud_.data[arrayPosX], &X, sizeof(float));
      memcpy(&pointcloud_.data[arrayPosY], &Y, sizeof(float));
      memcpy(&pointcloud_.data[arrayPosZ], &Z, sizeof(float));
    }
  }
  return pointcloud_;

}


int main (int argc, char** argv)
{
  ROS_INFO_STREAM("Start");
  // Initialize ROS
  ros::init (argc, argv, "map_cage_cloud");
  ros::NodeHandle nh;
  ros::Rate loopRate(20);

    // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/map/cage_cloud", 1);
  
  // build clouds
  double cagecubesize=2.8/2.0;
 sensor_msgs::PointCloud2 pointcloud_floor = planare_cloud(-cagecubesize,-cagecubesize,0.0,cagecubesize,-cagecubesize,0.0,-cagecubesize,cagecubesize,0.0,10,0.1,"world");
 sensor_msgs::PointCloud2 pointcloud_wall1 = planare_cloud(-cagecubesize,-cagecubesize,0.0,cagecubesize,-cagecubesize,0.0,-cagecubesize,-cagecubesize,cagecubesize*2,10,0.1,"world");
 sensor_msgs::PointCloud2 pointcloud_wall2 = planare_cloud(-cagecubesize,-cagecubesize,0.0,-cagecubesize,-cagecubesize,cagecubesize*2,-cagecubesize,cagecubesize,0.0,10,0.1,"world");
 sensor_msgs::PointCloud2 pointcloud_wall3 = planare_cloud(cagecubesize,-cagecubesize,0.0,cagecubesize,-cagecubesize,cagecubesize*2,cagecubesize,cagecubesize,0.0,10,0.1,"world");
 sensor_msgs::PointCloud2 pointcloud_wall4 = planare_cloud(-cagecubesize,cagecubesize,0.0,cagecubesize,cagecubesize,0.0,-cagecubesize,cagecubesize,cagecubesize*2,10,0.1,"world");
 sensor_msgs::PointCloud2 pointcloud_roof = planare_cloud(-cagecubesize,-cagecubesize,cagecubesize*2,cagecubesize,-cagecubesize,cagecubesize*2,-cagecubesize,cagecubesize,cagecubesize*2,10,0.1,"world");
 
 //add clouds together
 sensor_msgs::PointCloud2 pointcloud_;
 pcl::concatenatePointCloud(pointcloud_floor,pointcloud_wall1,pointcloud_);
 pcl::concatenatePointCloud(pointcloud_,pointcloud_wall2,pointcloud_);
 pcl::concatenatePointCloud(pointcloud_,pointcloud_wall3,pointcloud_);
 pcl::concatenatePointCloud(pointcloud_,pointcloud_wall4,pointcloud_);
 pcl::concatenatePointCloud(pointcloud_,pointcloud_roof,pointcloud_);

  while (ros::ok()) {

    ros::spinOnce();
    loopRate.sleep();
    pub.publish(pointcloud_);
    
  }


}