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
#define KLGRY "\x1b[90m"
#define KLRED "\x1b[91m"
#define KLGRN "\x1b[92m"
#define KLYEL "\x1b[93m"
#define KLBLU "\x1b[94m"
#define KLMAG "\x1b[95m"
#define KLCYN "\x1b[96m"
#define KLWHT "\x1b[97m"
#define RESET "\[<n>A\33[2K\r"

#include "RoverPath.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_analyser/Lpath.h>
#include <pcl_analyser/Lookuptbl.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/package.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseStamped.h>

//DriveTo stuff
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rover_actions/DriveToAction.h>
#include <rover_actions/DriveToOAAction.h>

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

class pathPR
{
public:
//  pathPR();
//  pathPR(float J_ch_,float J_inf_, float J_arm_, float TravelCost_, float h_goal_, float h_obs_)
//  {
//    J_ch = J_ch_;
//    J_inf = J_inf_;
//    J_arm = J_arm_;
//    TravelCost = TravelCost_;
//    h_goal = h_goal_;
//    h_obs = h_obs_;
//  }
  void fill(float J_ch_,float J_inf_, float J_arm_, float TravelCost_, float h_goal_, float h_obs_)
  {
    J_ch = J_ch_;
    J_inf = J_inf_;
    J_arm = J_arm_;
    TravelCost = TravelCost_;
    h_goal = h_goal_;
    h_obs = h_obs_;
  }
  void save(ros::NodeHandle* nptr,std::string ns = "PathProperties/")
  {
    nptr->setParam(ns + "J_ch",J_ch);
    nptr->setParam(ns + "J_inf",J_inf);
    nptr->setParam(ns + "J_arm",J_arm);
    nptr->setParam(ns + "travelcost",TravelCost);
    nptr->setParam(ns + "h_goal",h_goal);
    nptr->setParam(ns + "h_obs",h_obs);
  }
  float J_ch;
  float J_inf;
  float J_arm;
  float TravelCost;
  float h_goal;
  float h_obs;

};\

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

float Var(std::vector<float> vec)
{
   if (vec.empty()) return 0;
   //Mean Val
   float M = 0.0;
   for(int i = 0;i<vec.size();i++)
   {
     M += vec[i];
   }
   M = M / vec.size();
   //Var
   float var = 0.0;
   for(int i = 0;i<vec.size();i++)
   {
     var += (vec[i]-M)*(vec[i]-M);
   }
   return var;
}

class pathsolver
{
public:

  pathsolver(ros::NodeHandle* nPtr_, std::string costmap_topic, std::string emap_topic, double b, float Ts_, int sample_, std::string param_ns_ = "pcl_analyser_node", string arm_goal_topic = "/uav_pose");
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
  void drone_approach();
  nav_msgs::Path action_solve(float x, float y);
protected:
  bool scan360;
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
  Vector3f Arm_goal;
  //map <LT_key,pcl_analyser::Lpath>* LUTmapPtr;
  pcl_analyser::Lookuptbl::ConstPtr LUTmsgPtr;
  multimap<LT_key,multimap<LT_key,pcl_analyser::Lpath> >* LUTmapPtr;
  //sensor_msgs::PointCloud2* pathtrace_pc_msg_ptr;
  ros::Publisher* tmppubptr;
  // for test
  ros::Subscriber pose_sub;
  ros::Subscriber costmap_sub;
  ros::Subscriber emap_sub;
  ros::Subscriber arm_goal_sub;
  ros::Publisher path_result_pub;
  ros::Publisher path_LUT_pub_;\
  ros::Publisher Chassis_pub;
  std::string param_ns;
  bool arm_goal_exist;
  bool path_solution_exist;
private:
  bool ExecutePath(geometry_msgs::Pose goal, nav_msgs::Path path);
  nav_msgs::Path scanAndsolve(Vector3f goal);
  bool uavpose_in_bodyframe(geometry_msgs::Pose uav_pose_IF,std::string IF_frame,std::string Body_frame, tf::Pose& uav_Body);
  void uavpose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
  float Arm_energy(MatrixXf Path, Vector3f goal);
  bool contains_NAN(geometry_msgs::Pose m);
  PATH_COST Cost_of_path(MatrixXf path, costmap *grid, float Lethal_cost_inc = 5.0,float Inf_cost_inc = 2.0,float Travel_cost_inc = 0.1);
  void Chassis_sim_pub(MatrixXf Path, double map_scale = 3.5);
  float Chassis_simulator(MatrixXf Path, MatrixXf& Arm, VectorXf& Poses, geometry_msgs::PoseArray& msg, double map_scale = 3.5);
  void arm_goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
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
  float compute_J(MatrixXf *traptr, float travelcost, Vector3f Goal, bool& solution_found,pathPR& PR);
  void init_pso_param(int& particle_no, int& iteration, double& pso_inertia,double& c_1 , double& c_2);
  void EleMetaCallback(const hector_elevation_visualization::EcostmapMetaData::Ptr msg);
};

#endif // PATHSOLVER_H
