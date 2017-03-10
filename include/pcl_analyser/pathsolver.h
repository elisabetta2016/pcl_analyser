#ifndef PATHSOLVER_H
#define PATHSOLVER_H

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

#include "RoverPath.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_analyser/Lpath.h>
#include <pcl_analyser/Lookuptbl.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/package.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseStamped.h>

using namespace Eigen;
using namespace std;

float Dx(Vector3f a,Vector3f b)
{
  return sqrtf((a-b).coeff(0)*(a-b).coeff(0) + (a-b).coeff(1)*(a-b).coeff(1));
}

class LT_key
{
  public:
   LT_key(float w_)
   {
      w = w_;
      m = (int)((w - 0.0) / resolution);
   }
   float w;
   static const float resolution = 0.2; // meter/cell
   int m;
   inline bool operator<(const LT_key& other) const
   {
     return( m < other.m);
   }
};

struct ctrlparam
{
  double a;
  double b;
  double c;
  double d;
  double v;
  void fill_in(double a_, double b_,double c_, double d_,double v_)
  {
    this->a = a_;
    this->b = b_;
    this->c = c_;
    this->d = d_;
    this->v = v_;
  }
};
class pathsolver
{
public:

  pathsolver(ros::NodeHandle* nPtr_,std::string costmap_topic, std::string emap_topic,double b, float Ts_, int sample_,std::string param_ns_ = "pcl_analyser_node");
  pathsolver(ros::NodeHandle* nPtr_,costmap* obs_grid_, costmap* e_grid_,double b, float Ts_, int sample_);
  ~pathsolver();
  void handle(ros::Publisher* path_pub,ros::Publisher* pathtrace_pub,geometry_msgs::Pose goal_pose);
  void handle(ros::NodeHandle* n_ptr, geometry_msgs::Pose goal_pose);
  nav_msgs::Path get_path();
  void loadLUT();
  pcl_analyser::Lookuptbl readLUT(float wx, float wy);
  void get_publishers(ros::Publisher* temp_pub_ptr);
  pcl_analyser::Lookuptbl searchLUT(float wx,float wy,int desired_path_no);
  void test();
protected:
  RoverPathClass *rov;
  int sample;
  float lookahead;
  float Ts;
  costmap *master_grid_ptr;
  costmap *elevation_grid_ptr;
  bool demo_;
  bool show_;
  ros::NodeHandle* nPtr;
  PointCloudPtr pathtrace_ptr;
  nav_msgs::Path* resultpathptr;
  //map <LT_key,pcl_analyser::Lpath>* LUTmapPtr;
  pcl_analyser::Lookuptbl::ConstPtr LUTmsgPtr;
  multimap<LT_key,multimap<LT_key,pcl_analyser::Lpath> >* LUTmapPtr;
  //sensor_msgs::PointCloud2* pathtrace_pc_msg_ptr;
  ros::Publisher* tmppubptr;
  // for test
  ros::Subscriber pose_sub;
  ros::Subscriber costmap_sub;
  ros::Subscriber emap_sub;
  ros::Publisher path_result_pub;
  ros::Publisher path_LUT_pub_;\
  ros::Publisher Chassis_pub;
  std::string param_ns;
private:
  bool contains_NAN(geometry_msgs::Pose m);
  PATH_COST Cost_of_path(MatrixXf path, costmap *grid, float Lethal_cost_inc = 5.0,float Inf_cost_inc = 2.0,float Travel_cost_inc = 0.1);
  void Chassis_sim_pub(MatrixXf Path, double map_scale = 3.5);
  float Chassis_simulator(MatrixXf Path, MatrixXf& Arm, VectorXf& Poses, geometry_msgs::PoseArray& msg, double map_scale = 3.5);
  void emap_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void costmap_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void build_rov_if_not_exist();
  void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
  hector_elevation_visualization::EcostmapMetaData::Ptr ecostmap_meta_ptr;
  pcl_analyser::Lookuptbl LUTcleanup(geometry_msgs::Pose Goal,pcl_analyser::Lookuptbl lut);
  nav_msgs::Path solve(Vector3f goal);
  MatrixXf rover_tra(ctrlparam Q, float s_max, geometry_msgs::Pose& tail, double& cost);
  MatrixXf compute_tra(float a,float b,float c,float d,float v,float s_max);
  void init_x(MatrixXf *xptr,Vector3f goal,int particle_no);
  float compute_J(MatrixXf *traptr, Vector3f arm_goal, float travelcost, Vector3f Goal, bool& solution_found);
  void init_pso_param(int& particle_no, int& iteration, double& pso_inertia,double& c_1 , double& c_2);
  void EleMetaCallback(const hector_elevation_visualization::EcostmapMetaData::Ptr msg);
};

#endif // PATHSOLVER_H
