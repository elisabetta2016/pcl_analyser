#include "pathsolver.h"
#define foreach BOOST_FOREACH
using namespace Eigen;

//global functions
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
void Tra_to_cloud(MatrixXf tra,PointCloudPtr cloud_ptr)
{
  for(size_t i = 0; i < tra.cols(); i++)
  {
    PointXYZ point;
    point.x = tra(0,i);
    point.y = tra(1,i);
    point.z = 0.0;

    cloud_ptr->points.push_back(point);
  }

}

VectorXf linspace(float min, float max, unsigned int item)
{
  VectorXf V(item);
  float d = (max-min)/(item-1);
  for(int i=0; i< item; i++)
  {
    V(i) = min + i*d;
  }
  return V;
}

// Class definition
pathsolver::pathsolver(ros::NodeHandle* nPtr_,costmap* obs_grid_, costmap* e_grid_,double b, float Ts_, int sample_)
{
  //rov = rov_;
  sample = sample_;
  lookahead = b;
  Ts = Ts_;
  master_grid_ptr = obs_grid_;
  elevation_grid_ptr = e_grid_;
  nPtr = nPtr_;
  rov = new RoverPathClass(lookahead,sample,nPtr,master_grid_ptr,elevation_grid_ptr);
  PointCloudPtr temp_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pathtrace_ptr = temp_ptr;
  LUTmapPtr =  new std::multimap <LT_key,pcl_analyser::Lpath,Key_compare>();
}

pathsolver::~pathsolver()
{
  delete rov;
  delete LUTmapPtr;
  ROS_WARN("pathsolver instant destructed!");
}

pcl_analyser::Lookuptbl pathsolver::readLUT(float wx, float wy, float precision)
{
  typedef std::multimap <LT_key,pcl_analyser::Lpath,Key_compare>::iterator Iter;
  LT_key key(wx,wy,precision);
  pcl_analyser::Lookuptbl table;

  std::pair<Iter,Iter> range = LUTmapPtr->equal_range(key);
  for(Iter it = range.first; it != range.second;++it)
  {
    table.pathes.push_back(it->second);
    table.quantity++;
  }
  return table;
}


void pathsolver::loadLUT()
{
  std::string path = ros::package::getPath("pcl_analyser") + "/config/lookuptable.bag";
  rosbag::Bag bag;
  bag.open(path, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string("lookuptable"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  foreach(rosbag::MessageInstance const m, view)
  {
      LUTmsgPtr = m.instantiate<pcl_analyser::Lookuptbl>();
  }
  bag.close();
  if(LUTmsgPtr->quantity < 1)
  {
    ROS_ERROR("pathsolver: Error loading LookUp table!");
    return;
  }
  int count = 0;
  for(size_t i=0; i<LUTmsgPtr->quantity; i++)
  {
    pcl_analyser::Lpath lp;
    lp = LUTmsgPtr->pathes[i];
    LT_key key(lp.tail.position.x,lp.tail.position.y,lp.cost,1.0);
    //std::map <LT_key,pcl_analyser::Lpath> test;
    //test[key] = lp;
    count++;
    //LUTmapPtr->operator[](key) = lp;
    LUTmapPtr->insert(std::make_pair(key,lp));
  }
  ROS_INFO("%d path from the lookup table loaded Successfully",count);
}

void pathsolver::handle(ros::Publisher* path_pub,ros::Publisher* pathtrace_pub,geometry_msgs::Pose goal_pose)
{
  loadLUT();
  sensor_msgs::PointCloud2 pc_msg;
  Vector3f goal;
  goal(0) = goal_pose.position.x;
  goal(1) = goal_pose.position.y;
  goal(2) = 0; // orientation later
  nav_msgs::Path path_msg = solve(goal);
  path_pub->publish(path_msg);

  pcl::toROSMsg(*pathtrace_ptr,pc_msg);
  pc_msg.header.frame_id = "base_link";
  pc_msg.header.stamp = ros::Time::now();
  pathtrace_pub->publish(pc_msg);
  resultpathptr = &path_msg;
}
nav_msgs::Path pathsolver::get_path()
{
  nav_msgs::Path msg = *resultpathptr;
  return msg;
}
MatrixXf pathsolver::compute_tra(float a,float b,float c,float d,float v,float s_max)
{
    // omega = v*(a+b.s+c.s^2+d.s^3)
    VectorXf omega,v_in;
    omega.setZero(sample);
    v_in.setOnes(sample);
    v_in = v*v_in;
    float s = 0;
    float ds = s_max/sample;
    for(int i=0;i<sample;i++)
    {
       omega(i) = v*(a+b*s+c*pow(s,2)+d*pow(s,3));
       s = s+ds;
    }
    Vector3f x_0,x_dot_0,x_dot_f;
    x_0 << 0.0,0.0,0.0;
    x_dot_0<< 0.0,0.0,0.0;

    MatrixXf tra = rov->Rover_vw(v_in, omega, lookahead, Ts,x_0,x_dot_0 , sample, x_dot_f);
    return tra;
}

MatrixXf pathsolver::rover_tra(ctrlparam Q, float s_max, geometry_msgs::Pose& tail, double& cost)
{
  MatrixXf x;
  x.setZero(3,sample);
  VectorXf s = linspace(0.0,s_max,sample);
  float range = Q.v;
  float dt = range/sample;
  float theta = 0;
  float dx = 0;
  float dy = 0;
  x(0,0) = 0.0;
  x(0,1) = 0.0;
  x(0,2) = 0.0;
  cost = 0;
  for(int i = 1; i<sample; i++)
  {
    theta = Q.a*s(i) + Q.b*pow(s(i),2)/2 + Q.c*pow(s(i),3)/3 +Q.d*pow(s(i),4)/4;
    x(0,i) = x(0,i-1)+dx;
    x(1,i) = x(1,i-1)+dy;
    x(2,i) = theta;
    dx = cos(theta)*dt;//V = 1 m/s
    dy = sin(theta)*dt;//V = 1 m/s
    cost = fabs(dx) + fabs(dy);
    if(i == sample-1)
    {
      tail.position.x = x(0,i);
      tail.position.y = x(1,i);
      tail.orientation = tf::createQuaternionMsgFromYaw(x(2,i));
    }
  }

  return x;
}

float pathsolver::compute_J(MatrixXf *traptr,float travelcost,VectorXf Goal,bool& solution_found)
{
  float J = 0.0;
  Vector3f tra_tail;
  tra_tail(0) = traptr->coeffRef(0,traptr->cols()-1);
  tra_tail(1) = traptr->coeffRef(1,traptr->cols()-1);
  PATH_COST cost = rov->Cost_of_path(*traptr, master_grid_ptr);

  float J_0 = sqrtf( pow((tra_tail(0)-Goal(0)), 2) + pow((tra_tail(1)-Goal(1)), 2) );    //effect of distance from the goal
  float J_1 = (cost.Lethal_cost + cost.Inf_cost);				     // path cost
  //float Ob_func_3 = fabs(x(0,0) - prop_speed);			      		     // speed effect
  // Goal distance cost can be evaluated using cnmap now
  // Chassis cost term to be added
  // Arm energy consumption based on path to be calculated and added
  J = J_0 + J_1 + travelcost;
  ROS_INFO("cost of current path is %f",J);
  if (cost.Lethal_cost < 1) solution_found = true;
  return J;
}
void pathsolver::init_x(MatrixXf *xptr)
{
  float a = 0.0;
  float b = 10.0;
  float c = -10.0;
  float d = 1.0;
  float v = 1.0;
  // more suffisticated initialization would be implemented :)
  xptr->coeffRef(0,0) = a;
  xptr->coeffRef(1,0) = b;
  xptr->coeffRef(2,0) = c;
  xptr->coeffRef(3,0) = d;
  xptr->coeffRef(4,0) = v;
}
void pathsolver::init_pso_param(int& particle_no, int& iteration, double& pso_inertia,double& c_1 , double& c_2)
{
   if(!nPtr->getParam("pso_particle_no",particle_no))
   {
      particle_no = 30;
      ROS_WARN("pso particle_no is missing, it is set to the default value of %d",particle_no);
   }
   if(!nPtr->getParam("pso_iteration",iteration))
   {
      int iteration = 20;
      ROS_WARN("pso iteration is missing, it is set to the default value of %d",iteration);
   }
   if(!nPtr->getParam("pso_inertia",pso_inertia))
   {
      pso_inertia = 0.6;
      ROS_WARN("pso pso_inertia is missing, it is set to the default value of %f",pso_inertia);
   }
   if(!nPtr->getParam("pso_c1",c_1))
   {
      c_1 = 0.6;
      ROS_WARN("pso pso_c_1 is missing, it is set to the default value of %f",c_1);
   }
   if(!nPtr->getParam("pso_c2",c_2))
   {
      c_2 = 0.1;
      ROS_WARN("pso pso_c_2 is missing, it is set to the default value of %f",c_2);
   }
}

nav_msgs::Path pathsolver::solve(Vector3f goal)
{
   /*particle structure:
         n Particle
  | a1  ...particle N.O. ... an|
  | b1  ...particle N.O. ... bn|
  | c1  ...particle N.O. ... cn|
  | d1  ...particle N.O. ... dn|
  | v1  ...particle N.O. ... vn|

   */
  int param_no = 5;
  int particle_no,iteration;
  double pso_inertia,c_1,c_2;
  init_pso_param(particle_no, iteration,pso_inertia,c_1,c_2);
  MatrixXf x,v;  			//patricle
  x.setZero(param_no,particle_no);
  v.setZero(param_no,particle_no);

  //find_init_control(Goal_arm, particle_no, piece_no,x);
  init_x(&x);
  VectorXf x_best(param_no);
  VectorXf G(param_no);
  MatrixXf output_tra;
  output_tra.setZero(3,sample);
  float G_cost = 1.0/0.0;
  float x_best_cost = 1.0/0.0;
  bool solution_found = false;
  pathtrace_ptr->clear();
  //init G and x_best
  for(size_t i=0;i< param_no; i++)
  {
     G(i) = x(i,0);
  }
  x_best = G;
  float s_max = 1.5;
  float J = 0;
  for (size_t k = 0; k < iteration; k++)
  {
    MatrixXf tra;
    for(size_t i=0; i < particle_no; i++)
    {
      //Defining random coeffs of the PSO
      float r_1  = ((float) (rand() % 200))/100 -1.0;
      float r_2  = ((float) (rand() % 200))/100 -1.0;
      ctrlparam Q;
      geometry_msgs::Pose tail;
      double travelcost;
      Q.fill_in(x(0,i),x(1,i),x(2,i),x(3,i),x(4,i));
      tra = rover_tra(Q,s_max,tail,travelcost);
      //tra = compute_tra(x(0,i),x(1,i),x(2,i),x(3,i),x(4,i),s_max); deprecated
      ROS_WARN_STREAM("current tra:\n" << tra);
      Tra_to_cloud(tra,pathtrace_ptr);
      float Ob_func = compute_J(&tra,travelcost,goal,solution_found);
      //Best particle in the current iteration
      if (Ob_func < x_best_cost)
      {
        x_best_cost = Ob_func;
        for (size_t jj=0; jj < x.rows();jj++) x_best(jj) = x(jj,i);
        if(demo_) ROS_INFO("new value for x_best_cost");
      }
      //Best absolute solution
      if (Ob_func < G_cost)
      {
        G_cost = Ob_func;
        for (size_t jj=0; jj < x.rows();jj++) G(jj) = x(jj,i);
        output_tra = tra;
        if(demo_) ROS_WARN(" ------>  new value for G_cost");
      }
      if(i==0) //Reseting X_best and its cost in each iteration
      {
        for (size_t jj=0; jj < x.rows();jj++) x_best(jj) = x(jj,i);
        x_best_cost = Ob_func;
      }
      for (size_t jj=0; jj < x.rows();jj++)
        v(jj,i) = pso_inertia * v(jj,i) + c_1 * r_1 * (x_best(jj) - x(jj,i)) + c_2 * r_2 * (G(jj) - x(jj,i));
    }
    x = x+v;
  }
  nav_msgs::Path path_msg = MatToPath(output_tra,"base_link");
  return path_msg;
}

