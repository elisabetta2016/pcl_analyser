//#includes 
 //Standard
  #include <ros/ros.h>
  #include <ros/timer.h>
  #include <math.h>
  #include <iostream>
  #include <string>

 // PCL specific includes
 #include <pcl_conversions/pcl_conversions.h>
 #include <pcl/point_cloud.h>
 #include <pcl/point_types.h>

 #include <pcl/filters/voxel_grid.h>
 #include <pcl/io/pcd_io.h>
 #include <pcl/point_types.h>
 #include <pcl/features/normal_3d.h>
 #include <pcl/features/feature.h>
 #include <pcl/keypoints/sift_keypoint.h>
 #include <pcl/common/projection_matrix.h>
 #include <pcl/features/pfhrgb.h>
 #include <pcl/filters/filter.h> 

 #include <pcl/PCLPointCloud2.h>
 #include <pcl/conversions.h>
 #include <pcl_ros/transforms.h>
 #include <pcl/filters/conditional_removal.h>
 #include <pcl/filters/statistical_outlier_removal.h>
 #include <pcl/ModelCoefficients.h>
 #include <pcl/filters/project_inliers.h>

 // Messages
  #include <sensor_msgs/PointCloud2.h>
  #include "donkey_rover/Rover_Scanner.h"
  #include "donkey_rover/Rover_Track_Speed.h"
  #include <geometry_msgs/Vector3.h>
  #include <nav_msgs/Path.h>
  #include <sensor_msgs/point_cloud_conversion.h>

 #include <pcl/keypoints/iss_3d.h>
 #include <pcl/common/transforms.h>
 //Costmap
 #include <costmap_2d/static_layer.h>
 #include <costmap_2d/layer.h>
 #include <costmap_2d/costmap_2d_ros.h>
 #include <costmap_2d/costmap_2d_publisher.h>
 #include <list>
 // TF
 #include <tf/transform_listener.h>
 #include <Eigen/Dense> 

 #include "RoverPath.h"
 #include "costmap.h"
 #include "pathsolver.h"
 #include <vector>

//#defs
 #define INFLATED_OBSTACLE 90
 #define LETHAL_OBSTACLE 100
 #define WIDTH 255 //default

//typedefs
  typedef pcl::PointXYZ PointXYZ;
  typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloudVar;
//Name Spaces
 using std::vector;
 using namespace Eigen;

struct PointIndex {
	int i;
	int j;
};

//Global Variables
 double laser_dist = 0.493;  // distance between laser and center of gravity
 double lethal_rad = 0.1;
 double inf_rad = 0.3;
 double Travel_cost_inc = 0.0;
 double Lethal_cost_inc = 10.0;
 double Inf_cost_inc = 8.0;
 double b = 0.8;
 int sample = 15;
 bool demo_;
 bool pso_analyse = false;
 double pso_inertia;
 double c_1;
 double c_2;
 double Goal_gain;
 double Cost_gain;
 double Speed_gain;
 double omega_x;
 double Dist_passed_threshold = 0.8;
 int path_piece = 1;
 double Watching_hor = 3.0; //Default value


 nav_msgs::Path MatToPath(MatrixXf output_tra,std::string frame_id)
 {
    nav_msgs::Path robot_opt_path;
    int sample = output_tra.cols();
    robot_opt_path.header.stamp = ros::Time::now();
    robot_opt_path.header.frame_id = frame_id;
    robot_opt_path.poses = std::vector<geometry_msgs::PoseStamped> (sample);
    for(size_t i=0; i < sample; i++)
    {
        robot_opt_path.poses[i].pose.position.x = output_tra(0,i);
        robot_opt_path.poses[i].pose.position.y = output_tra(1,i);
        robot_opt_path.poses[i].pose.position.z = 0.0;
    }
    return robot_opt_path;
 }

 class ObstacleDetectorClass
{
	public:
		
	ObstacleDetectorClass(ros::NodeHandle& node)
	{
			n_=node;
      ros::NodeHandle n_pr_temp("~");
      n_pr = n_pr_temp;
			//subscribers
			SubFromCloud_		 = n_.subscribe("/RL_cloud", 1, &ObstacleDetectorClass::cloud_call_back,this);
			subFromElevationCostmap_ = n_.subscribe("/elevation_costmap", 1, &ObstacleDetectorClass::ElevationCallback,this);
			subFromElevationCostmapMeta_ = n_.subscribe("/elevation_costmap_MetaData", 1, &ObstacleDetectorClass::EleMetaCallback,this);
			// publishers
      obstcle_pub_		    = n_.advertise<sensor_msgs::PointCloud2> ("obstacle_cloud", 1);
			obstcle_proj_pub_	  = n_.advertise<sensor_msgs::PointCloud2> ("obstacle_proj_cloud", 1);
			cost_map_cl_pub_	  = n_.advertise<sensor_msgs::PointCloud2> ("costmap_cloud", 1);
      Obstacle_pub_		    = n_.advertise<nav_msgs::OccupancyGrid> ("global_costmap", 1);
      //
      master_grid_ = 0;
      elevation_grid_ = 0;

			//Initializer
			repulsive_force.x = 0.0;
			repulsive_force.y = 0.0;
			repulsive_force.z = 0.0;
			first_elevation_received = true;
			
			
			//obstacle avoidance params
			goal_present = false;

			//Global costmap params
			global_frame = "base_link";
			topic_name = "/global_costmap";
			costmap_x_size = 6.0; // meters
			costmap_y_size = 6.0; // meters
			origin_x = -1.0;
			origin_y = -3.0;
			
			//Elevation costmap params
      elevation_resolution_xy = 0.05;
      elevation_origin_x = 0.493;
      elevation_origin_y = -1.5;
      elevation_costmap_x_size = 3.0;
      elevation_costmap_y_size = 3.0;
			
    			
	}
		
	void obstacle_find_Publish(const sensor_msgs::PointCloud2ConstPtr& cloud)
	{
      ROS_INFO("Obstacle detection Starts!");
  		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl (new pcl::PointCloud<pcl::PointXYZ>);
  		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_pcl (new pcl::PointCloud<pcl::PointXYZ>);
  		pcl::fromROSMsg(*cloud, *cloud_pcl);

      *cloud_filtered_pcl = *cloud_pcl;
  
      // Create the normal estimation class, and pass the input dataset to it
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  		ne.setInputCloud (cloud_pcl);
  		ne.setViewPoint(0,0,0.3);

  		// Create an empty kdtree representation, and pass it to the normal estimation object.
  		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  		ne.setSearchMethod (tree);

  		// normal datasets
  		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  		// Use all neighbors in a sphere of radius 10cm
  		ne.setRadiusSearch (0.10);

  		// Compute the features
  		ne.compute (*cloud_normals);
  		//std::cout << cloud_normals->points.size() << "\n";
  
  
  		//Defining the Kdtree for finding the indicies
  		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  		kdtree.setInputCloud (cloud_pcl);
  		// Defining  the search point
  		pcl::PointXYZ searchPoint;

      // Starting to search and insert
      pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_pcl (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ> obstacle_nomal_pcl;
      pcl::PointCloud<pcl::PointXYZ>::Ptr obs_projected (new pcl::PointCloud<pcl::PointXYZ>);
      int K = 1;
      std::vector<int> pointIdxNKNSearch(K);
      std::vector<float> pointNKNSquaredDistance(K);
	
      obstacle_nomal_pcl = *cloud_filtered_pcl;

      for (size_t i = 0; i < cloud_filtered_pcl->points.size (); ++i)
      {
        searchPoint.x = cloud_filtered_pcl->points[i].x;
        searchPoint.y = cloud_filtered_pcl->points[i].y;
        searchPoint.z = cloud_filtered_pcl->points[i].z;
		
     		if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
          {
  	 	
          for (size_t j = 0; j < pointIdxNKNSearch.size (); ++j)
          {

            //obstacle_nomal_pcl.points[pointIdxNKNSearch[j]].z = cloud_normals->points[pointIdxNKNSearch[j]].normal_z;
            if(cloud_normals->points[pointIdxNKNSearch[j]].normal_z < normal_threshold &&
                cloud_pcl->points[pointIdxNKNSearch[j]].z        > height_threshold &&
			       		cloud_pcl->points[pointIdxNKNSearch[j]].z        < height_max)
            {
							obstacle_pcl->points.push_back (cloud_pcl->points[ pointIdxNKNSearch[j] ]);
						}
          }

        }
	
       }
	
       obstacle_pcl->width = int (obstacle_pcl->points.size ());
       obstacle_pcl->height = 1;
       obstacle_pcl->is_dense = true;
	
       //cloud_outlier_removal(obstacle_pcl,obstacle_pcl);
	
       pcl::toROSMsg(*obstacle_pcl,output_cloud);
       output_cloud.header.frame_id = "laser";
       output_cloud.header.stamp = ros::Time::now();
	
       obstcle_pub_.publish(output_cloud);
       //ROS_WARN("Obstacle cloud published");
       cloud_2D_projection(obstacle_pcl,obs_projected);
       pcl::toROSMsg(*obs_projected,cloud_obstacle_projected);
       cloud_obstacle_projected.header.frame_id = "laser";
       cloud_obstacle_projected.header.stamp = ros::Time::now();
       obstcle_proj_pub_.publish(cloud_obstacle_projected);
       cloud_to_costmap(obstacle_pcl);
       costmap_to_cloud();
//       repuslive_force_pub_.publish(repulsive_force);
       repulsive_force.x = 0.0;
       repulsive_force.y = 0.0;
       repulsive_force.z = 0.0;
	}
	 	  	
	void GoalCallback(const geometry_msgs::Vector3::ConstPtr& msg)
	{
		nav_goal(0,0) = msg->x;
		nav_goal(0,1) = msg->y;
		nav_goal(0,2) = 0.0; 
		goal_present = true; 
	}

  void EleMetaCallback(const hector_elevation_visualization::EcostmapMetaData::Ptr msg)
	{
		ecostmap_meta = *msg;
	}

	void ElevationCallback(const nav_msgs::OccupancyGrid::ConstPtr& new_map)
	{	
		unsigned int size_x = new_map->info.width, size_y = new_map->info.height;
    if(first_elevation_received)
		{
			
			double orig_x = new_map->info.origin.position.x;
			double orig_y = new_map->info.origin.position.y;

			elevation_grid_ = new costmap(orig_x,orig_y,size_x,size_y,new_map->info.resolution,"base_link",false);

			first_elevation_received = false;			
		}

		elevation_grid_->resetMap();
		elevation_grid_->setCost_v(new_map->data,new_map->info.width);
//		Elevation_pub_.publish(elevation_grid_->getROSmsg());
		
	}
	
  void run_pub_once()
  {
    double normal_threshold_default = 0.7;
    double height_threshold_default = -0.1;
    double height_max_default = 2.0;


    //ros::NodeHandle n_pr("~");

    n_pr.param("normal_threshold", normal_threshold, normal_threshold_default);
    n_pr.param("height_threshold", height_threshold, height_threshold_default);
    n_pr.param("height_max", height_max,height_max_default);

    if (normal_threshold != normal_threshold_default) ROS_INFO_ONCE("normal threshold is changed to %f", normal_threshold);
    if (height_threshold != height_threshold_default) ROS_INFO_ONCE("height threshold is changed to %f", height_threshold);
    if (height_max       != height_max_default)       ROS_INFO_ONCE("height threshold is changed to %f", height_max);

    n_pr.param("costmap_res", costmap_res, 0.2);
    n_pr.param("LETHAL_radius", lethal_rad, 0.1);
    n_pr.param("INFLATION_radius", inf_rad, 0.3);

    n_pr.param("pso_inertia", pso_inertia, 0.1);
    n_pr.param("pso_c1", c_1, 0.45);
    n_pr.param("pso_c2", c_2, 0.45);
    n_pr.param("pso_goal_gain", Goal_gain, 30.0);
    n_pr.param("pso_cost_gain", Cost_gain, 1.0);
    n_pr.param("pso_speed_gain", Speed_gain, 0.0);
    n_pr.param("pso_particle_no", particle_no, 10);
    n_pr.param("pso_iteration", iteration, 5);
    n_pr.param("path_piece_no", path_piece, 1);
    n_pr.param("omega_x", omega_x, 0.3);

    n_pr.param("Travel_cost_inc", Travel_cost_inc, 0.0);
    n_pr.param("Lethal_cost_inc", Lethal_cost_inc, 10.0);
    n_pr.param("Inflation_cost_inc", Inf_cost_inc, 3.0);
    n_pr.param("b", b, 0.4);
    n_pr.param("Watching_horizon", Watching_hor, 3.0);
    n_pr.param("sample", sample, 15);
    n_pr.param("demo_mode", demo_, false);

    ROS_INFO_ONCE("PSO Params: pso_inertia:%f, c1:%f, c2:%f, Number of Particle:%d, Iteration:%d",pso_inertia,c_1,c_2,particle_no,iteration);
    ROS_INFO_ONCE("PSO cost function Params: Goal_gain:%f, path_cost_gain:%f, speed_gain:%f",Goal_gain,Cost_gain,Speed_gain);
    ros::Duration(0.4).sleep();
    ros::Rate rate(2.0);

    //costmap params
    cell_x = (unsigned int) floor(abs(costmap_x_size/costmap_res)); //	#cell_x
    cell_y = (unsigned int) floor(abs(costmap_y_size/costmap_res)); //	#cell_y
    elevation_cell_x = floor(abs(elevation_costmap_x_size/elevation_resolution_xy));
    elevation_cell_y = floor(abs(elevation_costmap_y_size/elevation_resolution_xy));

    master_grid_ = new costmap(origin_x,origin_y,cell_x,cell_y,costmap_res,"base_link",false);

    while(ros::ok())
    {
      cloud_to_costmap(cost_map_cloud);
      rate.sleep();
      ros::spinOnce();
    }

  }


	void run()
	{
	
		double normal_threshold_default = 0.7;
		double height_threshold_default = -0.1;
		double height_max_default = 2.0;
		
		
    //ros::NodeHandle n_pr("~");
	
		n_pr.param("normal_threshold", normal_threshold, normal_threshold_default);
		n_pr.param("height_threshold", height_threshold, height_threshold_default);
		n_pr.param("height_max", height_max,height_max_default);
		
		if (normal_threshold != normal_threshold_default) ROS_INFO_ONCE("normal threshold is changed to %f", normal_threshold);
		if (height_threshold != height_threshold_default) ROS_INFO_ONCE("height threshold is changed to %f", height_threshold);
		if (height_max       != height_max_default)       ROS_INFO_ONCE("height threshold is changed to %f", height_max);
		
		n_pr.param("costmap_res", costmap_res, 0.2);
		n_pr.param("LETHAL_radius", lethal_rad, 0.1);
		n_pr.param("INFLATION_radius", inf_rad, 0.3);
		
		n_pr.param("pso_inertia", pso_inertia, 0.1);
		n_pr.param("pso_c1", c_1, 0.45);
		n_pr.param("pso_c2", c_2, 0.45);
		n_pr.param("pso_goal_gain", Goal_gain, 30.0);
		n_pr.param("pso_cost_gain", Cost_gain, 1.0);
		n_pr.param("pso_speed_gain", Speed_gain, 0.0);
		n_pr.param("pso_particle_no", particle_no, 10);
		n_pr.param("pso_iteration", iteration, 5);
		n_pr.param("path_piece_no", path_piece, 1);
		n_pr.param("omega_x", omega_x, 0.3);
		
		n_pr.param("Travel_cost_inc", Travel_cost_inc, 0.0);
		n_pr.param("Lethal_cost_inc", Lethal_cost_inc, 10.0);
		n_pr.param("Inflation_cost_inc", Inf_cost_inc, 3.0);
		n_pr.param("b", b, 0.4);
		n_pr.param("Watching_horizon", Watching_hor, 3.0);
		n_pr.param("sample", sample, 15);
		n_pr.param("demo_mode", demo_, false);
		
		ROS_INFO_ONCE("PSO Params: pso_inertia:%f, c1:%f, c2:%f, Number of Particle:%d, Iteration:%d",pso_inertia,c_1,c_2,particle_no,iteration);
		ROS_INFO_ONCE("PSO cost function Params: Goal_gain:%f, path_cost_gain:%f, speed_gain:%f",Goal_gain,Cost_gain,Speed_gain);
	
		ros::Rate rate(10.0);
		tf::TransformListener listener;
		
		
		//costmap params
		cell_x = (unsigned int) floor(abs(costmap_x_size/costmap_res)); //	#cell_x
		cell_y = (unsigned int) floor(abs(costmap_y_size/costmap_res)); //	#cell_y
		elevation_cell_x = floor(abs(elevation_costmap_x_size/elevation_resolution_xy));
		elevation_cell_y = floor(abs(elevation_costmap_y_size/elevation_resolution_xy));

		master_grid_ = new costmap(origin_x,origin_y,cell_x,cell_y,costmap_res,"base_link",false);
			
		bool first_loop = true;
		bool transform_present = true; 
		float curr_x;
		float curr_y;
		float curr_yaw;
		float last_x;
		float last_y;
		float last_yaw;		
		
		

		Matrix4f transform_1 = Matrix4f::Identity();
		
		float distance_passed = 0.0;
		
		ros::Duration(0.4).sleep();
    ros::Time curr_time, last_time;
    last_time = ros::Time::now();
    bool run = true;
		while(ros::ok())
		{
			
			
      tf::StampedTransform transform_odom_laser;
      try{
          listener.lookupTransform("/map", "/base_link", ros::Time(0), transform_odom_laser);
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
			curr_yaw = (float) yaw - M_PI/2;	//odom orientation is 90 degree rotated with respect to laser
						
			if (!first_loop)
			{	
				
				float delta_x   =  (curr_x - last_x) * cos(curr_yaw) + (curr_y - last_y)* sin(curr_yaw);
				float delta_y   = -(curr_x - last_x) * sin(curr_yaw) + (curr_y - last_y)* cos(curr_yaw);
				float delta_yaw = (curr_yaw - last_yaw);
				
				distance_passed += sqrt(pow(delta_x,2)+pow(delta_y,2));
				if (distance_passed > Dist_passed_threshold)
				{
					distance_passed = 0.0;
					//pso_analyse = false;
				}

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
        transform_1 = transform_1;
        pcl::transformPointCloud (cost_map_cloud, cost_map_cloud, transform_1);
  			
        cloud_to_costmap(cost_map_cloud);
			}
						
			last_x = curr_x;
			last_y = curr_y;
			last_yaw = curr_yaw;
			if (first_loop && transform_present) 
			{
				first_loop = false;
					
			}

      //Calling test functions
      curr_time = ros::Time::now();

			pcl::toROSMsg(cost_map_cloud,costmap_cl);
      costmap_cl.header.frame_id = "base_link";
      costmap_cl.header.stamp = ros::Time::now();
			cost_map_cl_pub_.publish(costmap_cl);			
      rate.sleep();
			ros::spinOnce ();
						
		}
	}
	
	
	protected:
	
		// Node Handler
		ros::NodeHandle n_;
		ros::NodeHandle* n;
    ros::NodeHandle n_pr;
		// Subscribers
		ros::Subscriber SubFromCloud_;
		ros::Subscriber subFromTrackSpeed_;
		ros::Subscriber subFromGoal_;
		ros::Subscriber subFromElevationCostmap_;
		ros::Subscriber subFromElevationCostmapMeta_;
		
		// Publishers
		ros::Publisher obstcle_pub_;
		ros::Publisher obstcle_proj_pub_;
		ros::Publisher cost_map_cl_pub_;
		ros::Publisher repuslive_force_pub_;
		ros::Publisher path_pub_;
		ros::Publisher path_solution_pub_;
    //ros::Publisher* path_trace_pub_ptr;
		ros::Publisher path_colision_pub_;
		ros::Publisher lookuppath_pub_;
		ros::Publisher ChassisPose_pub_;
		ros::Publisher Elevation_pub_;
		ros::Publisher Obstacle_pub_;
    boost::shared_ptr <ros::Publisher> path_trace_pub_ptr;
    ros::Publisher trace_pub;

		geometry_msgs::Vector3 repulsive_force;
		
		//Class Global Variables

		double normal_threshold;
		double height_threshold;
		double height_max;
		sensor_msgs::PointCloud2 output_cloud; 
		sensor_msgs::PointCloud2 cloud_obstacle_projected; 
		sensor_msgs::PointCloud2 costmap_cl;
		
		//costmap variables
		costmap* master_grid_;
		costmap* elevation_grid_;
    pathsolver* ps_ptr;
		double costmap_x_size;
		double costmap_y_size;
		double origin_x;
		double origin_y;
		double costmap_res;
		pcl::PointCloud<pcl::PointXYZI> cost_map_cloud;
		unsigned int cell_x;
		unsigned int cell_y;
		unsigned int cell_elevation_x;
		unsigned int cell_elevation_y;
		
		std::string global_frame, topic_name;
    double elevation_resolution_xy;
    double elevation_origin_x;
    double elevation_origin_y;
    double elevation_costmap_x_size;
    double elevation_costmap_y_size;
		unsigned int elevation_cell_x;
		unsigned int elevation_cell_y;
		
		float angular_resolution;
		float support_size;
		bool setUnseenToMaxRange;
		bool first_elevation_received;

		//Obstacle avoidance variables
		Vector3f nav_goal;
		bool goal_present;
		
		hector_elevation_visualization::EcostmapMetaData ecostmap_meta;
		//Path finder
		pcl::PointCloud<pcl::PointXYZ> path_trace_pcl;
		float path_z_inc;
		int particle_no;
    int iteration;

    //Lookup table
    std::vector<nav_msgs::Path> path_vector;

private:

    nav_msgs::Path PathFromEigenMat(MatrixXf in, std::string frame_id)
    {
        nav_msgs::Path out;
        int length = in.cols();
        out.header.stamp = ros::Time::now();
        out.header.frame_id = frame_id;
        out.poses = std::vector<geometry_msgs::PoseStamped> (length);
        for(int i=0;i< length;i++)
        {
          out.poses[i].pose.position.x = in(0,i);
          out.poses[i].pose.position.y = in(1,i);
          out.poses[i].pose.position.z = 0.0;
        }
        return out;
    }

    VectorXf FromSTDvector(std::vector<double> input)
    {
      VectorXf out((int)input.size());
      for (size_t i = 0; i < input.size(); i++)
      {
        out(i) = input[i];
      }
      return out;
    }


    void cloud_2D_projection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
    {

      // Create a set of planar coefficients with X=Y=0,Z=1
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
      coefficients->values.resize (4);
      coefficients->values[0] = coefficients->values[1] = 0;
      coefficients->values[2] = 1.0;
      coefficients->values[3] = 0;

      // Create the filtering object
      pcl::ProjectInliers<pcl::PointXYZ> proj;
      proj.setModelType (pcl::SACMODEL_PLANE);
      proj.setInputCloud (cloud_in);
      proj.setModelCoefficients (coefficients);
      proj.filter (*cloud_out);

    }

    void cloud_call_back(const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
      obstacle_find_Publish(cloud);


    }


    bool is_on_costmap_margin(int mx,int my)
    {
      bool out = true;
      int margin = 4;
      int x_Unorm = (int) cell_x - margin;
      int y_Unorm = (int) cell_y - margin;
      if(mx < x_Unorm && mx > margin && my < y_Unorm && my > margin) out = false;
      return out;
    }

    void cloud_to_costmap(pcl::PointCloud<pcl::PointXYZ>::Ptr obs_2d)
    {

      int mx;
      int my;

        master_grid_->resetMap(0,0,cell_x-1,cell_y-1);

      for (size_t i = 0; i < obs_2d->points.size (); ++i)
      {
          //if ( is_in_costmap(obs_2d->points[i].x, obs_2d->points[i].y) )

        master_grid_->worldToMapEnforceBounds((double) obs_2d->points[i].x + laser_dist,(double) obs_2d->points[i].y,mx,my); // was enforced
        if( !is_on_costmap_margin(mx,my) )
          master_grid_->setCost(mx,my, LETHAL_OBSTACLE);

      }
      master_grid_->inflate(0.3,0.1);
      //lethal_inflation();
      // Costmap2DPublisher
      Obstacle_pub_.publish(master_grid_->getROSmsg());

    }

    void cloud_to_costmap(pcl::PointCloud<pcl::PointXYZI> obs_2d)
    {

      int mx;
      int my;

        master_grid_->resetMap(0,0,cell_x-1,cell_y-1);
        unsigned char cost;
      for (size_t i = 0; i < obs_2d.points.size (); ++i)
      {

        master_grid_->worldToMapEnforceBounds((double) obs_2d.points[i].x,(double) obs_2d.points[i].y,mx,my); // was enforced
        //master_grid_ros->updateBounds(0,cell_x-1,0,cell_y-1);

        if (obs_2d.points[i].intensity == INFLATED_OBSTACLE ) cost = INFLATED_OBSTACLE;
        else cost = LETHAL_OBSTACLE;
        if( !is_on_costmap_margin(mx,my) )
        master_grid_->setCost(mx,my, cost);
      }

        Obstacle_pub_.publish(master_grid_->getROSmsg());
    }

    void lethal_inflation() //fill the cell around the lethal obstacle
      {
          /* vector< vector<int> > matrix

          0 1 2 3 4 5 6 7 8 ...
          0	x
          1	y

        */
        vector< vector<int> > matrix;
        matrix.resize(WIDTH);
        int index = -1;

        for (unsigned int i=0; i < cell_x ; i++) //loop in x
        {
          for (unsigned int j=0; j < cell_y ; j++) //loop in y
          {
            if(master_grid_->getCost(i,j) == LETHAL_OBSTACLE) //find the lethal obstacle
            {
              index++;
              matrix[index].resize(2);
              matrix[index][0] = i;
              matrix[index][1] = j;
            }
          }
        }

        //loop in the list of lethal obstacles
        for (int k=0; k < index; k++)
        {
          int cell_around = (int) floor(fabs(lethal_rad/costmap_res))+1;
          int i = matrix[k][0]; //x
          int j = matrix[k][1]; //y

          for (int ii=-cell_around; ii<cell_around+1; ii++) //loop around the obstacle
          {
            for (int jj=-cell_around; jj<cell_around+1; jj++)
            {

              if ((i+ii) < 0) ii = std::min(-i,ii);
                    if ((j+jj) < 0) jj = std::min(-j,jj);
              try

              {
              if( !is_on_costmap_margin(i+ii,j+jj) )
              master_grid_->setCost(i+ii, j+jj, LETHAL_OBSTACLE); //fill the cell around the obstacle
              inf_inflation(i+ii,j+jj, master_grid_);

              }

              //continue even if exit from the grid

              catch(int e)

              {
              //do nothing
              }
            }
          }



        }
      }

    void inf_inflation(int i,int j, costmap* grid)
    {
      int cell_around = (int) floor(fabs(inf_rad/costmap_res))+1;
      unsigned char INFLATION_OBSTACLE = INFLATED_OBSTACLE;
        for (int k=-cell_around; k<cell_around+1; k++) //loop around the lethal obstacle
        {

          for (int l=-cell_around; l<cell_around+1; l++)
          {

            if ((i+k) < 0) k = std::min(-i,k);
            if ((i+l) < 0) l = std::min(-j,l);
            unsigned char cost = 0;
            if( !is_on_costmap_margin(k+i,l+j) )
              cost = grid->getCost((unsigned int)k+i,(unsigned int)l+j);


            if(cost != LETHAL_OBSTACLE) //if is a lethal do nothing
            {
              try {
              if( !is_on_costmap_margin(i+k,j+l) )
                   grid->setCost((unsigned int)i+k, (unsigned int)j+l, INFLATION_OBSTACLE); //fill the cell around lethal_obs
              }
              catch(int e)
              {
              //do nothing
              }
            }
          }
        }
    }

    void costmap_to_cloud()
    {
      double temp_x;
      double temp_y;
      cost_map_cloud.clear();
      for (size_t i=0; i < cell_x + 1; i++)
        {
      for (size_t j=0; j < cell_y + 1; j++)
      {
        unsigned char cost = master_grid_->getCost(i,j);
        if(cost == LETHAL_OBSTACLE || cost == INFLATED_OBSTACLE)
        {
          master_grid_->mapToWorld(i,j,temp_x,temp_y);

          for (int k = -1; k < 2; k++) // -1;2
          {
            for (int l = -1; l < 2; l++) // -1;2
            {
              pcl::PointXYZI point;
              point.z = 0.0;
              point.x = temp_x + (float) k*costmap_res/2*0.9;
              point.y = temp_y + (float) l*costmap_res/2*0.9;
              point.intensity = (float) cost;
              cost_map_cloud.points.push_back(point);
            }
          }
        }

      }
       }

    }
		
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcl_analyser");
	ros::NodeHandle node;
	
	

	ObstacleDetectorClass Obstacle_rec(node);
	
  Obstacle_rec.run_pub_once();
	
	return 0;
}
