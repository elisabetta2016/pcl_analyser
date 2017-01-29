#ifndef PATHSOLVER_H
#define PATHSOLVER_H
#include "RoverPath.h"
#include <sensor_msgs/PointCloud2.h>
using namespace Eigen;
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
  pathsolver(ros::NodeHandle* nPtr_,costmap* obs_grid_, costmap* e_grid_,double b, float Ts_, int sample_);
  ~pathsolver();
  void handle(ros::Publisher* path_pub,ros::Publisher* pathtrace_pub,geometry_msgs::Pose goal_pose);
  nav_msgs::Path get_path();
protected:
  RoverPathClass *rov;
  int sample;
  float lookahead;
  float Ts;
  costmap *master_grid_ptr;
  costmap *elevation_grid_ptr;
  bool demo_;
  ros::NodeHandle* nPtr;
  PointCloudPtr pathtrace_ptr;
  nav_msgs::Path* resultpathptr;
  //sensor_msgs::PointCloud2* pathtrace_pc_msg_ptr;
private:
  nav_msgs::Path solve(Vector3f goal);
  MatrixXf rover_tra(ctrlparam Q, float s_max, geometry_msgs::Pose& tail, double& cost);
  MatrixXf compute_tra(float a,float b,float c,float d,float v,float s_max);
  void init_x(MatrixXf *xptr);
  float compute_J(MatrixXf *traptr,float travelcost,VectorXf Goal,bool& solution_found);
  void init_pso_param(int& particle_no, int& iteration, double& pso_inertia,double& c_1 , double& c_2);
};

#endif // PATHSOLVER_H
