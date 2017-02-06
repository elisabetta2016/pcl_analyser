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

struct PathAdr{
  size_t path_it;
  size_t sample_it;
  std::vector<size_t> bad_it;
};

//define global variables
 extern double omega_x;
 extern double laser_dist;
 extern bool demo_;
 extern double b;


class RoverPathClass
{
public:
		
  RoverPathClass(double b_,int sample_, ros::NodeHandle* nPtr,costmap* grid_);
  RoverPathClass(double b_,int sample_, ros::NodeHandle* nPtr,costmap* obs_grid_, costmap* e_grid_);

  void get_global_attributes(boost::shared_ptr <ros::Publisher> path_trace_pub_);

	void set_path_params(double Travel_cost_inc_,double Lethal_cost_inc_,double Inf_cost_inc_);
	
	void set_pso_params(double pso_inertia_in,double c_1_in,double c_2_in,double Goal_gain_in,double Cost_gain_in,double Speed_gain_in,int particle_no_,int iteration_);
	
	void set_pso_params_default();
	
	void update_costmap(costmap* grid_);
	
	void traj_to_cloud(MatrixXf tra);

  void path_lookup_table(ros::Publisher* PCpubPtr,geometry_msgs::Pose goal);

  void LTcleanup(geometry_msgs::Pose Goal);

  bool is_occluded_point(geometry_msgs::Pose Pose,geometry_msgs::Pose Goal);

  float Chassis_simulator(MatrixXf Path, costmap* grid, double map_scale, VectorXf& Poses,geometry_msgs::PoseArray& msg, hector_elevation_visualization::EcostmapMetaData ecostmap_meta);

	void Rover_parts(MatrixXf trajectory, MatrixXf& FrontRightTrack, MatrixXf& FrontLeftTrack, MatrixXf& RearRightTrack, MatrixXf& RearLeftTrack, MatrixXf& Arm);

	Matrix3f TranMatrix2D(float theta, float t_x, float t_y);
	
	pcl::PointCloud<pcl::PointXYZ> get_path_trace_cloud();
	
	PATH_COST Cost_of_path(MatrixXf path, costmap* grid);
	
	MatrixXf Rover_vw(VectorXf V_input, VectorXf Omega_input, double b, double Ts,Vector3f x_0,Vector3f x_dot_0 , int sample, Vector3f& x_dot_f);
	
  MatrixXf PSO_path_finder(Vector3f Goal,Vector3f Goal_arm,Vector2f V_curr_c,double Ts,int particle_no,int iteration,int piece_no_,VectorXf& output, bool& solution_found);

 bool is_occluded_path(geometry_msgs::Pose Pose,geometry_msgs::Pose Goal);
protected:
  ros::NodeHandle* node_Ptr;
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
  double Ts_global;
  PathAdr CPinfo; //Candidate path information
  boost::shared_ptr <ros::Publisher> path_trace_pub;
  bool path_trace_pub_exist;
  //lookupTable
  std::vector<nav_msgs::Path> path_vector;

	private:
  void init_();
	bool is_in_costmap(float x, float y, costmap* grid);
  void path_lookup_table(geometry_msgs::Pose goal);


  nav_msgs::Path PathFromEigenMat(MatrixXf in, std::string frame_id);

  VectorXf EigenVecFromSTDvec(std::vector<double> input);

  nav_msgs::Path find_init_candidate(geometry_msgs::Pose Goal);

  float pose_distance_2d(geometry_msgs::Pose a,geometry_msgs::Pose b);
  void find_init_control(Vector3f Goal, int particle_no, int& piece_no,MatrixXf& x);
  void shortenLTpaths(geometry_msgs::Pose Goal);
  geometry_msgs::Pose EigVecToPose(Vector3f in);
  float Arm_energy(MatrixXf Path, Vector3f goal);
};
#endif 
