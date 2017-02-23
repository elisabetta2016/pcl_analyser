#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include<laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "costmap.h"
#include <tf/transform_listener.h>
//Messages
 #include <nav_msgs/OccupancyGrid.h>
 #include<geometry_msgs/PoseStamped.h>
 #include<nav_msgs/Path.h>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
class laserToPC
{
  public:

    laserToPC(ros::NodeHandle& node)
    {
       n_=node;
       scansub_ = n_.subscribe("/scan",1,&laserToPC::scan_cb,this);
       posesub_ = n_.subscribe("/drone",1,&laserToPC::pose_cb,this);
       mapsub_ = n_.subscribe("/map",1,&laserToPC::map_cb,this);
       mappub_ = n_.advertise<nav_msgs::OccupancyGrid>("/map_d",1);
       pcpub_   = n_.advertise<sensor_msgs::PointCloud>("/obstacle_pc", 10);
       PointCloudPtr pcl_scan_raw_ (new pcl::PointCloud<pcl::PointXYZ>);
       PointCloudPtr pcl_drone_ (new pcl::PointCloud<pcl::PointXYZ>);
       PointCloudPtr pcl_cloud_ (new pcl::PointCloud<pcl::PointXYZ>);

       pcl_scan_raw =  pcl_scan_raw_;
       pcl_drone = pcl_drone_;
       pcl_cloud = pcl_cloud_;
       trans_map_laser.setIdentity();
       cmp_ptr = 0;

    }

    void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
      if(cmp_ptr == 0)
      {
        cmp_ptr = new costmap(*msg,false);
      }
      else
      {
        cmp_ptr->UpdateFromMap(*msg);
        update_map_with_drones();
        mappub_.publish(cmp_ptr->getROSmsg());
      }
    }

    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        tf::Vector3 map_drone(msg->pose.position.x,msg->pose.position.y,0.0);
        tf::Vector3 drone_laser = trans_map_laser * map_drone;
        pcl::PointXYZ point;
        point.x = drone_laser.getX();
        point.y = drone_laser.getY();
        point.z = 0.0;
        pcl_drone->push_back(point);
        if(cmp_ptr != 0)
        {
           drones.poses.push_back(*msg);

        }
        else ROS_ERROR("costmap yet to be init!!!!");

    }

    void update_map_with_drones()
    {
      if (drones.poses.size() == 0) return;
      for(int i = 0;i<drones.poses.size();i++)
      {
        cmp_ptr->setCost_WC(drones.poses[i].pose.position.x,drones.poses[i].pose.position.y,100);
      }
    }

    void scan_cb(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
       projector_.projectLaser(*scan_in, cloud);
       sensor_msgs::PointCloud2 cloud2;
       sensor_msgs::convertPointCloudToPointCloud2(cloud,cloud2);
       pcl::fromROSMsg(cloud2,*pcl_scan_raw);
       // calculations
       pcl_cloud->clear();
       *pcl_cloud = *pcl_scan_raw + *pcl_drone;
       pcl::toROSMsg(*pcl_cloud,cloud2);
       sensor_msgs::convertPointCloud2ToPointCloud(cloud2,cloud);
       cloud.header.stamp = ros::Time::now();
       cloud.header.frame_id = scan_in->header.frame_id;
       pcpub_.publish(cloud);

    }

    void handle()
    {
      tf::TransformListener listener;
      ROS_INFO("waiting for base_laser map transform");
      listener.waitForTransform("/base_laser","/map", ros::Time(0), ros::Duration(3));
      ROS_INFO("base_laser map transform found");

      while(ros::ok)
      {
        try{
          listener.lookupTransform("/base_laser", "/map", ros::Time(0), trans_map_laser);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
        ros::Rate(10).sleep();
        ros::spinOnce();
      }
    }

  protected:
  /*state here*/
  ros::NodeHandle n_;
  sensor_msgs::LaserScan scan;
  laser_geometry::LaserProjection projector_;
  sensor_msgs::PointCloud cloud;
  //sensor_msgs::PointCloud cloud2;
  PointCloudPtr pcl_scan_raw;
  PointCloudPtr pcl_drone;
  PointCloudPtr pcl_cloud;
  // Subscriber
  ros::Subscriber scansub_;
  ros::Subscriber posesub_;
  ros::Subscriber mapsub_;
  // Publishers
  ros::Publisher mappub_;
  ros::Publisher pcpub_;
  costmap* cmp_ptr;
  tf::StampedTransform trans_map_laser;
  nav_msgs::Path drones;
  double rate;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "LaserToPc");
  ros::NodeHandle node;

  laserToPC tt(node);
  tt.handle();
  return 0;
}
