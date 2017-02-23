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

//Messages
 #include <nav_msgs/OccupancyGrid.h>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
class laserToPC
{
  public:

    laserToPC(ros::NodeHandle& node)
    {
       n_=node;
       scansub_ = n_.subscribe("/scan",1,&laserToPC::scan_cb,this);
       pcpub_   = n_.advertise<sensor_msgs::PointCloud2>("/obstacle_pc", 10);
       PointCloudPtr pcl_scan_raw_ (new pcl::PointCloud<pcl::PointXYZ>);
       PointCloudPtr pcl_drone_ (new pcl::PointCloud<pcl::PointXYZ>);
       PointCloudPtr pcl_cloud_ (new pcl::PointCloud<pcl::PointXYZ>);

       pcl_scan_raw =  pcl_scan_raw_;
       pcl_drone = pcl_drone_;
       pcl_cloud = pcl_cloud_;

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

       cloud.header.stamp = ros::Time::now();
       cloud.header.frame_id = scan_in->header.frame_id;
       pcpub_.publish(cloud2);

    }

    void handle()
    {
      ros::spin();
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

  // Publishers
  ros::Publisher pcpub_;
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
