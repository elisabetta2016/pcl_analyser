#include <ros/ros.h>
#include <ros/timer.h>
#include <math.h>
#include <iostream>
#include <string>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>

// Messages
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>

#include <Eigen/Dense>

class FakeObstacleCloud
{
	public:
		
	FakeObstacleCloud(ros::NodeHandle& node)
	{			
		n_=node;
		
		obstcle_pub_ 	= n_.advertise<sensor_msgs::PointCloud2> ("RL_cloud", 1);
		
		// Initializers
		costmap_res = 0.6;
	}
	
	
	
	void run()
	{
		pcl::PointCloud<pcl::PointXYZ> fake_obs;
		
		float z_inc = 1.5;
		
		float X_obs = 1.2;
		float Y_obs = 0.4;
		
		pcl::PointXYZ point;
		point.x = X_obs;
		point.y = Y_obs;
		point.z = -0.1;
		
		
		for(int i=0;i<30; i++)
		{
			point.x = point.x + costmap_res/30;
			point.y = Y_obs;
			for(int j=0;j < 30;j++)
			{	
				
				point.y = point.y - costmap_res/30;
				
				fake_obs.points.push_back(point);
				
			}
			point.z = point.z + z_inc/30;
		}
		
		X_obs = 1.8;
		Y_obs = 1.8;
		point.x = X_obs;
		point.y = Y_obs;
		point.z = -0.1;
		
		for(int i=0;i<30; i++)
		{
			point.x = point.x + costmap_res/30;
			point.y = Y_obs;
			for(int j=0;j < 30;j++)
			{	
				
				point.y = point.y - costmap_res/30;
				
				fake_obs.points.push_back(point);
				
			}
			point.z = point.z + z_inc/30;
		}
		
		
    		
    		
    		
    		ros::Rate rate(50);
    		
    		tf::TransformListener listener;
    		bool first_loop = true;
		bool transform_present = true; 
		float curr_x;
		float curr_y;
		float curr_yaw;
		float last_x;
		float last_y;
		float last_yaw;		
		
		

		Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
		
		
    		
    		while(ros::ok())
    		{
    			tf::StampedTransform transform_odom_laser;
    		    	try{
      				listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform_odom_laser);
      				transform_present = true;
    		    	}
    			catch (tf::TransformException ex)
    			{
      				ROS_ERROR("%s",ex.what());
      				ros::Duration(0.05).sleep();
      				transform_present = false;
    			}
			//Reading x and y
			curr_x =transform_odom_laser.getOrigin().x();
			curr_y =transform_odom_laser.getOrigin().y();
			
			tfScalar roll,pitch,yaw;
			
			tf::Matrix3x3 M(transform_odom_laser.getRotation());
			M.getRPY(roll,pitch,yaw,(unsigned int) 1);
			curr_yaw = (float) yaw - M_PI/2; //odom orientation is 90 degree rotated with respect to laser
			
			
			if (!first_loop)
			{	
				
				float delta_x   =  (curr_x - last_x) * cos(curr_yaw) + (curr_y - last_y)* sin(curr_yaw);
				float delta_y   = -(curr_x - last_x) * sin(curr_yaw) + (curr_y - last_y)* cos(curr_yaw);;
				float delta_yaw = (curr_yaw - last_yaw);
				
				
				

				/*
				| cos(yaw)  sin(yaw) 0  x|
				|-sin(yaw)  cos(yaw) 0  y|
				|     0        0     1  0|
				|     0        0     0  1|
				*/
				
				transform_1 (0,0) =  cos (delta_yaw);  transform_1 (0,1) =  sin (delta_yaw);
  				
  				transform_1 (1,0) = -sin (delta_yaw);  transform_1 (1,1) =  cos (delta_yaw);
  				
  				transform_1 (0,3) = -delta_y; 
  				transform_1 (1,3) = -delta_x;
  				transform_1 (2,3) = 0.0;
  				
  				pcl::transformPointCloud (fake_obs, fake_obs, transform_1);
  			
			}
						
			last_x = curr_x;
			last_y = curr_y;
			last_yaw = curr_yaw;
			if (first_loop && transform_present) 
			{
				first_loop = false;
				
			}
			
			sensor_msgs::PointCloud2 cloud;
		
			pcl::toROSMsg(fake_obs,cloud);
		
			cloud.header.frame_id = "base_link";
    			cloud.header.stamp = ros::Time::now();	
    			obstcle_pub_.publish(cloud);
    			
			
			rate.sleep();
			ros::spinOnce();
		}
	}
	
	
	protected:
	ros::NodeHandle n_;
	float costmap_res;
		
	ros::Publisher obstcle_pub_;
	
};
	
int main(int argc, char **argv)
{
	ros::init(argc, argv, "fake_obs_publisher");
	ros::NodeHandle node;
	
	

	FakeObstacleCloud object(node);
	
	object.run();
	
	return 0;
}	
	
