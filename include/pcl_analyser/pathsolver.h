#ifndef PATHSOLVER_H
#define PATHSOLVER_H
#include "RoverPath.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_analyser/Lpath.h>
#include <pcl_analyser/Lookuptbl.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/package.h>
#include <boost/foreach.hpp>
using namespace Eigen;
using namespace std;

class LT_key
{
  public:
   LT_key(float wx_,float wy_, float precision_)
   {
      wx = wx_;
      wy = wy_;
      precision = fabs(precision_);
      hasCost = false;
      range = sqrtf(pow(wx,2)+pow(wy,2));
   }
   LT_key(float wx_,float wy_, float cost_, float precision_)
   {
      wx = wx_;
      wy = wy_;
      precision = fabs(precision_);
      hasCost = true;
      Tcost = cost_;
      range = sqrtf(pow(wx,2)+pow(wy,2));
   }


   string Better_than(LT_key& other)
   {
     if (!(hasCost && other.hasCost))
     {
       ROS_ERROR("LT_key: one of the key has no cost to compare!");
       return "DontKnow";
     }
     if(other.Tcost > Tcost)
       return "Yes";
     if(other.Tcost < Tcost)
       return "No";
     return "Boh";
   }

   /*inline bool operator<(const LT_key& other) const
   {
       return (other.wx - wx > precision) && (other.wx - wy > precision);
   }*/
   float wx,wy,precision,range;
   float Tcost;
   bool hasCost;
private:

};
struct Key_compare {
    bool operator()(const LT_key& ky1, const LT_key& ky2) const {
        return (fabs(ky1.wx - ky2.wx)< ky2.precision) && (fabs(ky1.wy - ky2.wy)< ky2.precision);
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
  pathsolver(ros::NodeHandle* nPtr_,costmap* obs_grid_, costmap* e_grid_,double b, float Ts_, int sample_);
  ~pathsolver();
  void handle(ros::Publisher* path_pub,ros::Publisher* pathtrace_pub,geometry_msgs::Pose goal_pose);
  nav_msgs::Path get_path();
  void loadLUT();
  pcl_analyser::Lookuptbl readLUT(float wx, float wy, float precision);
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
  //map <LT_key,pcl_analyser::Lpath>* LUTmapPtr;
  pcl_analyser::Lookuptbl::ConstPtr LUTmsgPtr;
  multimap<LT_key,pcl_analyser::Lpath,Key_compare>* LUTmapPtr;
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
