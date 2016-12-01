#ifndef ROVERPATH_H
#define ROVERPATH_H

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//TF
#include <tf/transform_datatypes.h>

//Messages
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
// Eigen
#include <Eigen/Dense> 

// Costmap_2d
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include "costmap.h"

#include<hector_elevation_visualization/EcostmapMetaData.h>


//typedefs
  typedef pcl::PointXYZ PointXYZ;
  typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloudVar;

using namespace Eigen;

struct PATH_COST{
	float Lethal_cost;
	float Travel_cost;
	float Inf_cost;
	bool collision;	
};
struct CELL{
	unsigned int x;
	unsigned int y;
	signed char c;
};



//define global variables
 extern double omega_x;
 extern double laser_dist;
 extern bool demo_;
 extern double b;


class RoverPathClass
{
public:
		
	RoverPathClass(double b_,int sample_, costmap* grid_);
	RoverPathClass(double b_,int sample_, costmap* obs_grid_, costmap* e_grid_);

	void set_path_params(double Travel_cost_inc_,double Lethal_cost_inc_,double Inf_cost_inc_);
	
	void set_pso_params(double pso_inertia_in,double c_1_in,double c_2_in,double Goal_gain_in,double Cost_gain_in,double Speed_gain_in,int particle_no_,int iteration_);
	
	void set_pso_params_default();
	
	void update_costmap(costmap* grid_);
	
	void traj_to_cloud(MatrixXf tra);

  void path_lookup_table(ros::Publisher* PCpubPtr, ros::NodeHandle* nPtr);

  void LTcleanup(geometry_msgs::Pose Goal);

  bool is_occluded_point(geometry_msgs::Pose Pose,geometry_msgs::Pose Goal);

	void Chassis_simulator(MatrixXf Path, costmap* grid, double map_scale, VectorXf& Poses,geometry_msgs::PoseArray& msg, hector_elevation_visualization::EcostmapMetaData ecostmap_meta);

	void Rover_parts(MatrixXf trajectory, MatrixXf& FrontRightTrack, MatrixXf& FrontLeftTrack, MatrixXf& RearRightTrack, MatrixXf& RearLeftTrack, MatrixXf& Arm);

	Matrix3f TranMatrix2D(float theta, float t_x, float t_y);
	
	pcl::PointCloud<pcl::PointXYZ> get_path_trace_cloud();
	
	PATH_COST Cost_of_path(MatrixXf path, costmap* grid);
	
	MatrixXf Rover_vw(VectorXf V_input, VectorXf Omega_input, double b, double Ts,Vector3f x_0,Vector3f x_dot_0 , int sample, Vector3f& x_dot_f);
	
	MatrixXf PSO_path_finder(Vector3f Goal,Vector2f V_curr_c,double Ts,int particle_no,int iteration,int piece_no,VectorXf& output, bool& solution_found);
protected:
	
	costmap* master_grid_;
	costmap* elevation_grid_;
	//RoverSim params
	double Rover_b;
	int sample;
	
	pcl::PointCloud<pcl::PointXYZ> path_trace_pcl;	
	
	//PSO params
	double pso_inertia;
	double c_1;
	double c_2;
	double Goal_gain;
	double Cost_gain;
	double Speed_gain;
	int particle_no;
  int iteration;
  double Travel_cost_inc;
	double Lethal_cost_inc;
	double Inf_cost_inc;
  float path_z_inc;

  //lookupTable
  std::vector<nav_msgs::Path> path_vector;

	private:
	bool is_in_costmap(float x, float y, costmap* grid);
  bool is_occluded_path(geometry_msgs::Pose Pose,geometry_msgs::Pose Goal);
  nav_msgs::Path PathFromEigenMat(MatrixXf in, std::string frame_id);
  VectorXf EigenVecFromSTDvec(std::vector<double> input);
  nav_msgs::Path find_init_candidate(geometry_msgs::Pose Goal);
  float pose_distance_2d(geometry_msgs::Pose a,geometry_msgs::Pose b);
  void shortenLTpaths(geometry_msgs::Pose Goal);
};
#endif 
