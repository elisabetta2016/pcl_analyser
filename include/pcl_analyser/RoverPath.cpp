#define INFLATED_OBSTACLE 200

#include "RoverPath.h"
using namespace Eigen;
using costmap_2d::LETHAL_OBSTACLE;


		
RoverPathClass::RoverPathClass(double b_,int sample_,ros::NodeHandle* nPtr, costmap *grid_)
{
	Rover_b = b_;
	sample = sample_;
	master_grid_ = grid_;
  node_Ptr = nPtr;
  init_();
}

RoverPathClass::RoverPathClass(double b_,int sample_,ros::NodeHandle* nPtr, costmap *obs_grid_, costmap *e_grid_)
{
	Rover_b = b_;
	sample = sample_;
	master_grid_ = obs_grid_;
	elevation_grid_ = e_grid_;
  node_Ptr = nPtr;
  init_();
}

void RoverPathClass::init_()
{
  set_pso_params_default();
  Travel_cost_inc = 0.0;
  Lethal_cost_inc = 10.0;
  Inf_cost_inc = 3.0;
  CPinfo.sample_it = 0;
  CPinfo.path_it = 0;
  Ts_global = 3.00;
  path_trace_pub_exist = false;
  //CPinfo.bad_it = {};

}

/*
RoverPathClass::~RoverPathClass()	
{

}*/
VectorXf RoverPathClass::EigenVecFromSTDvec(std::vector<double> input)
{
  VectorXf out((int)input.size());
  for (size_t i = 0; i < input.size(); i++)
  {
    out(i) = input[i];
  }
  return out;
}

nav_msgs::Path RoverPathClass::PathFromEigenMat(MatrixXf in, std::string frame_id)
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

void RoverPathClass::path_lookup_table(ros::Publisher* PCpubPtr,geometry_msgs::Pose goal)
{
  double Ts_L = Ts_global;
  int path_number;
  PointCloudVar PC;

  sensor_msgs::PointCloud2 PC_msg;
  MatrixXf tra;

  tra.setZero( 3, sample);
  VectorXf V_in(sample);
  VectorXf Omega_in(sample);
  Vector3f x_0,x_dot_0,x_dot_f;

  x_0 << 0.0,0.0,0.0;
  x_dot_0 << 0.0,0.0,0.0;

  //RoverPathClass Rov(0.0, sample_L,master_grid_);
  std::vector<double> v_vector;
  std::vector<double> o_vector;
  node_Ptr->getParam("V_1", v_vector);
  node_Ptr->getParam("path_number", path_number);

  V_in = EigenVecFromSTDvec(v_vector);
  std::ostringstream j_no;
  std::string temp;

  nav_msgs::Path temp_path;
  for(int j=0;j < path_number;j++)
  {
    j_no.str("");
    j_no.clear();
    j_no << j;
    temp = "O_" + j_no.str();
    //ROS_WARN("String is %s", temp);
    //std::cout << temp << "\n";

    node_Ptr->getParam("O_"+j_no.str(), o_vector);

    Omega_in = EigenVecFromSTDvec(o_vector);

    tra = Rover_vw(V_in, Omega_in, 0.0, Ts_L, x_0, x_dot_0 , sample, x_dot_f);
    temp_path = PathFromEigenMat(tra, "base_link");
    path_vector.push_back(temp_path);


    // -Omega
    //tra.setZero();
    //tra = Rover_vw(V_in,-Omega_in, 0.0, Ts_L, x_0, x_dot_0 , sample, x_dot_f);

    //temp_path = PathFromEigenMat(tra, "base_link");
    //path_vector.push_back(temp_path);


  }


  PointXYZ point,g;
  g.x = goal.position.x;
  g.y = goal.position.y;
  g.z = 0.5;
  shortenLTpaths(goal);
  LTcleanup(goal);

  //  publish cleaned lookup_table
  for (size_t i=0; i<path_vector.size();i++)
  {
    for(size_t j=0; j< path_vector[i].poses.size();j++)
    {
      point.x = path_vector[i].poses[j].pose.position.x;
      point.y = path_vector[i].poses[j].pose.position.y;
      point.z = 0.0;

      PC.push_back(point);
    }
  }
  //publish lookup table candidate
  // /*
  nav_msgs::Path candidate = find_init_candidate(goal);
  for (size_t i=0; i<candidate.poses.size();i++)
  {
     point.x = candidate.poses[i].pose.position.x;
     point.y = candidate.poses[i].pose.position.y;
     point.z = 0.1;
     PC.push_back(point);
  }
  // */
  PC.push_back(g);

  pcl::toROSMsg(PC,PC_msg);
  PC_msg.header.stamp = ros::Time::now();
  PC_msg.header.frame_id = "base_link";
  PCpubPtr->publish(PC_msg);


}

void RoverPathClass::path_lookup_table(geometry_msgs::Pose goal)
{
  double Ts_L = Ts_global;
  int path_number;
  MatrixXf tra;

  tra.setZero( 3, sample);
  VectorXf V_in(sample);
  VectorXf Omega_in(sample);
  Vector3f x_0,x_dot_0,x_dot_f;

  x_0 << 0.0,0.0,0.0;
  x_dot_0 << 0.0,0.0,0.0;

  //RoverPathClass Rov(0.0, sample_L,master_grid_);
  std::vector<double> v_vector;
  std::vector<double> o_vector;
  if(!node_Ptr->getParam("V_1", v_vector))
      ROS_FATAL("Lookup table not found - V_1");
  if(!node_Ptr->getParam("path_number", path_number))
      ROS_FATAL("Lookup table not found");

  V_in = EigenVecFromSTDvec(v_vector);
  std::ostringstream j_no;
  std::string temp;

  nav_msgs::Path temp_path;
  for(int j=0;j < path_number;j++)
  {
    j_no.str("");
    j_no.clear();
    j_no << j;
    temp = "O_" + j_no.str();
    //ROS_WARN("String is %s", temp);
    //std::cout << temp << "\n";

    node_Ptr->getParam("O_"+j_no.str(), o_vector);

    Omega_in = EigenVecFromSTDvec(o_vector);

    tra = Rover_vw(V_in, Omega_in, 0.0, Ts_L, x_0, x_dot_0 , sample, x_dot_f);
    temp_path = PathFromEigenMat(tra, "base_link");
    path_vector.push_back(temp_path);


    // -Omega
    //tra.setZero();
    //tra = Rover_vw(V_in,-Omega_in, 0.0, Ts_L, x_0, x_dot_0 , sample, x_dot_f);

    //temp_path = PathFromEigenMat(tra, "base_link");
    //path_vector.push_back(temp_path);
  }

  shortenLTpaths(goal);
  LTcleanup(goal);
}

void RoverPathClass::shortenLTpaths(geometry_msgs::Pose Goal)
{
  size_t path_no = path_vector.size();

  geometry_msgs::Pose z;
  z.position.x = 0;
  z.position.y = 0;
  float goal_dist = pose_distance_2d(z,Goal);
  for(size_t i=0;i<path_no;i++)
  {//loop all the paths
    for(int j=0; j < path_vector[i].poses.size(); j++)
    {// loop the samples
        if(pose_distance_2d(path_vector[i].poses[j].pose,z) > goal_dist)
        {
          path_vector[i].poses.erase(path_vector[i].poses.begin()+j,path_vector[i].poses.end()); //eliminate samples farther than goal
          break;
        }
    }
  }

}

void RoverPathClass::LTcleanup(geometry_msgs::Pose Goal) //Cleaning path lookuptable
{
  size_t path_no = path_vector.size();
  std::vector <nav_msgs::Path> temp_path_vector;
  bool occluded_path = false;
  for(size_t i=0;i<path_no;i++)
  {//loop all the paths
    for(int j=0; j < path_vector[i].poses.size(); j++)
    {// loop the samples
        if(is_occluded_point(path_vector[i].poses[j].pose,Goal))
        {
          occluded_path = true;
          break;
        }
    }
    if (!occluded_path)
      temp_path_vector.push_back(path_vector[i]);
    if (occluded_path)
      CPinfo.bad_it.push_back(i); // save the address of occluded paths
    occluded_path = false;
  }
  path_vector = temp_path_vector;
}

nav_msgs::Path RoverPathClass::find_init_candidate(geometry_msgs::Pose Goal)
{
  nav_msgs::Path result;
  float best_val = 1000000.0;
  float curr_val;
  size_t best_i,best_j;
  path_lookup_table(Goal); // initialize the lookup table
  size_t path_no = path_vector.size();
  if(path_vector.size() == 0)
  {
    ROS_ERROR("Lookup table is empty");
    return result;
  }
  for(size_t i=0;i<path_no;i++) //finding the sample among the paths that is closest to the goal
  {
    for(size_t j=0;j < path_vector[i].poses.size();j++)
    {
        curr_val = pose_distance_2d(Goal,path_vector[i].poses[j].pose);
        if( curr_val < best_val)
        {
          best_i = i;
          best_j = j;
          best_val = curr_val;
        }
    }
  }

  for(size_t jj =0;jj< best_j; jj++) //result path is a path shortened to the best sample
  {
    result.poses.push_back(path_vector[best_i].poses[jj]);
  }
  CPinfo.sample_it = best_j;
  CPinfo.path_it = best_i;
  for(size_t m=0; m < CPinfo.bad_it.size();m++)
    if (CPinfo.bad_it[m] <= CPinfo.path_it) CPinfo.path_it++; // to be tested

  return result;
}

float RoverPathClass::pose_distance_2d(geometry_msgs::Pose a,geometry_msgs::Pose b)
{
   return sqrt(pow(a.position.x-b.position.x , 2) + pow(a.position.y-b.position.y,2));
}

bool RoverPathClass::is_occluded_point(geometry_msgs::Pose Pose,geometry_msgs::Pose Goal)
{
  double m = (Goal.position.y - Pose.position.y)/(Goal.position.x - Pose.position.x);
  //Special case to be considered m = 0, m = inf
  double x_increment = (Goal.position.x - Pose.position.x)/(double)sample;
  bool out = false;
  MatrixXf path_PG;

  path_PG.setZero(2,sample);
  // y = m*(x - goal.x)+ goal.y
  PointXYZ point;

  for(int i = 0;i< sample;i++)
  {
    path_PG(0,i) = Pose.position.x + x_increment*i;  //x_element
    path_PG(1,i) = m*(path_PG(0,i) - Goal.position.x)+Goal.position.y; //y_element
    point.x = path_PG(0,i);
    point.y = path_PG(1,i);
    point.z = -1.0;

  }
  PATH_COST Path_PG_cost = Cost_of_path(path_PG, master_grid_);
  int a1 = Path_PG_cost.Lethal_cost;
  int a2 = Path_PG_cost.Inf_cost;
  bool a3 = Path_PG_cost.collision;
  if (Path_PG_cost.collision)
  {
    //ROS_INFO("Collision!!!!");
    out = true;
  }
  else
    //ROS_WARN("Safe!!!");
  return out;
}

void RoverPathClass::set_path_params(double Travel_cost_inc_,double Lethal_cost_inc_,double Inf_cost_inc_)
{
	Travel_cost_inc = Travel_cost_inc_;
	Lethal_cost_inc = Lethal_cost_inc_;
	Inf_cost_inc = Inf_cost_inc_;

}

void RoverPathClass::set_pso_params(double pso_inertia_,double c_1_,double c_2_,double Goal_gain_,double Cost_gain_,double Speed_gain_,int particle_no_,int iteration_)
{
	
	pso_inertia = pso_inertia_;
	c_1 = c_1_;
	c_2 = c_2_;
	Goal_gain = Goal_gain_;
	Cost_gain = Cost_gain_;
	Speed_gain = Speed_gain_;
	particle_no = particle_no_;
  	iteration = iteration_;
}
	
void RoverPathClass::set_pso_params_default()
{
	pso_inertia = 0.1;
	c_1 = 0.45;
	c_2 = 0.45;
	Goal_gain = 30.0;
	Cost_gain = 1.0;
	Speed_gain = 0.0;
	particle_no = 15;
  	iteration = 5;
}
	
void RoverPathClass::update_costmap(costmap *grid_)
{
	master_grid_ = grid_;
}
		
pcl::PointCloud<pcl::PointXYZ> RoverPathClass::get_path_trace_cloud()
{
	return path_trace_pcl;
}
	
PATH_COST RoverPathClass::Cost_of_path(MatrixXf path, costmap *grid)
{
  set_path_params();
	CELL prev_cell;
	CELL curr_cell;
	prev_cell.x = 0;
	prev_cell.y = 0;
	PATH_COST cost;
	cost.Lethal_cost = 0.0;
  cost.Travel_cost = 0.0;//Travel_cost_inc;
	cost.Inf_cost = 0.0;
	cost.collision = false;
	
	for(size_t i=0; i < path.cols(); i++)
	{
		grid->worldToMap((double) path(0,i),(double) path(1,i),curr_cell.x,curr_cell.y);
		
		if( (curr_cell.x != prev_cell.x) && (curr_cell.x != prev_cell.x) )
		{
			curr_cell.c = grid->getCost(curr_cell.x,curr_cell.y);
      //ROS_ERROR("current_cell.c %d",curr_cell.c);
      if ((int)curr_cell.c == 100)//LETHAL_OBSTACLE
			{
        //ROS_INFO("LETHAL !");
				cost.Lethal_cost += Lethal_cost_inc;
				cost.collision = true;
			}
      if ((int)curr_cell.c == 90)//INFLATED_OBSTACLE
			{
        //ROS_INFO("INFLATED !");
				cost.Inf_cost += Inf_cost_inc; 
			}
      //cost.Travel_cost +=  Travel_cost_inc;
			prev_cell = curr_cell;
		}
	}
	return cost;
}
		
MatrixXf RoverPathClass::Rover_vw(VectorXf V_input, VectorXf Omega_input, double b, double Ts,Vector3f x_0,Vector3f x_dot_0 , int sample, Vector3f& x_dot_f)
{
        /*
	Structure of x and x_dot
	| x  |
	| y  |
	| th |
	*/
	MatrixXf x;
	x.setZero(3,sample);
    	MatrixXf x_dot;
    	//MatrixXd V_in;
    	//MatrixXd Omega_in;
    	MatrixXf NE_dot_temp;
    	MatrixXf Rot_temp;
    	MatrixXf V_temp;
      
    	double dt = Ts / ((double)sample); 
    	
    	x_dot.setZero(3,sample);
    	
   	x(0,0) = x_0(0);
   	x(1,0) = x_0(1);
   	x(2,0) = x_0(2);

    	NE_dot_temp.setZero(2,sample);
   	NE_dot_temp.col(0) = x_dot_0.topRows(2);
    
    	Rot_temp.setIdentity(2,2);
    	V_temp.setZero(2,1);
    
    	for(size_t i=1; i < sample; i++)
    	  {
     		x(2,i) = x(2,i-1) + Omega_input(i);
     
     		Rot_temp(0,0) =    cos(x(2,i));
     		Rot_temp(0,1) = -b*sin(x(2,i));
     		Rot_temp(1,0) =    sin(x(2,i));
     		Rot_temp(1,1) =  b*cos(x(2,i));
     
     		V_temp(0,0)  = V_input(i);
     		float temp = Omega_input(i);
     		V_temp(0,1)  = temp;
     		NE_dot_temp.col(i) = Rot_temp * V_temp;
     		x_dot(0,i) = NE_dot_temp(0,i);
     		x_dot(1,i) = NE_dot_temp(1,i);
     		x_dot(2,i) = Omega_input(i);
     		x(0,i) = x(0,i-1)+x_dot(0,i)*dt;
     		x(1,i) = x(1,i-1)+x_dot(1,i)*dt;
        
    	   }
    	x_dot_f = x_dot.rightCols(sample-1);   
    	
    	
	return x;  
}
	
void RoverPathClass::traj_to_cloud(MatrixXf tra)
{
		
	for(size_t i = 0; i < tra.cols(); i++)
	{
		pcl::PointXYZ point;
		point.x = tra(0,i);
		point.y = tra(1,i);
		point.z = 0.0;
			
			
		path_trace_pcl.points.push_back(point);
	}
		
}
	
//updated function
float RoverPathClass::Chassis_simulator(MatrixXf Path, costmap *e_grid, double map_scale, MatrixXf& Arm, VectorXf& Poses,geometry_msgs::PoseArray& msg, hector_elevation_visualization::EcostmapMetaData ecostmap_meta)
{
  float cost = 0.0;
	int vector_size = Path.cols();
	VectorXf temp_output (vector_size);
    //geometry_msgs::PoseArray msg;
	geometry_msgs::Pose temp_pose;
	CELL FRT_cell;
	CELL FLT_cell;
	bool unknownCell = false;
    const float RoverWidth = 0.3965;
	float delta_e = 0.0;
	MatrixXf FrontRightTrack; 
	MatrixXf FrontLeftTrack;
	MatrixXf RearRightTrack;
	MatrixXf RearLeftTrack;
  //MatrixXf Arm;
	int mx,my;
	Rover_parts(Path,FrontRightTrack, FrontLeftTrack, RearRightTrack, RearLeftTrack, Arm);
	
	msg.poses = std::vector <geometry_msgs::Pose> (vector_size);
	for (size_t i=0;i < vector_size;i++)
	{
		if(is_in_costmap(FrontRightTrack(0,i),FrontRightTrack(1,i),e_grid) &&
		   is_in_costmap(FrontLeftTrack(0,i),FrontLeftTrack(1,i),e_grid))
		   {
			    //ROS_WARN("1");
				e_grid->worldToMap((double) FrontRightTrack(0,i),(double) FrontRightTrack(1,i), FRT_cell.x, FRT_cell.y);
				//ROS_WARN("2");
				e_grid->worldToMap((double) FrontLeftTrack(0,i),(double) FrontLeftTrack(1,i), FLT_cell.x, FLT_cell.y);  
				//ROS_WARN_STREAM("FRT \n" << FrontRightTrack << "FrontLeftTrack" << FrontLeftTrack );
		   }
		else
		{   // The point is not in the costmap 
		    //ROS_ERROR_STREAM("FRT \n" << FrontRightTrack << "FrontLeftTrack" << FrontLeftTrack );
			temp_pose.position.x = Path(0,i);
			temp_pose.position.y = Path(1,i);
			temp_pose.position.z = 0.0;
			temp_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw (0.0, 0.0, Path(2,i));
			msg.poses[i] = temp_pose;

			temp_output(i) = 0.0;
			continue;
		}   
		//ROS_WARN("FRT 1:%f 2:%f 3:%d 4:%d",FrontRightTrack(0,i),FrontRightTrack(1,i),FRT_cell.x,FRT_cell.y);
		FRT_cell.c = e_grid->getCost(FRT_cell.x,FRT_cell.y);
		FLT_cell.c = e_grid->getCost(FLT_cell.x,FLT_cell.y);

		delta_e = (((float)FLT_cell.c) - ((float) FRT_cell.c))*(map_scale/254.00);
		if (FRT_cell.c == 255 || FLT_cell.c == 255)
		{
			unknownCell = true;
		}
		/*
		else
		{
			ROS_ERROR("FrontRightTrack x:%f, y:%f   FrontLeftTrack x:%f, y:%f ",FrontRightTrack(0,i),FrontRightTrack(1,i),FrontLeftTrack(0,i),FrontLeftTrack(1,i));
			ROS_INFO("FRT_CELL  1:%d, 2:%d    FLT_cell:   1:%d, 2:%d ",FRT_cell.x,FRT_cell.y,FLT_cell.x,FLT_cell.y);
			ROS_WARN_STREAM("FRT_cell.c:   " << FRT_cell.c << "FLT_cell.c:   " <<FLT_cell.c << "delta_e:   "<<delta_e);
		}*/
		
		
		temp_output(i) = asin(delta_e/RoverWidth);
		if (unknownCell)
		{
			if(i==1) temp_output(i) = 0.0;
			else temp_output(i) = temp_output(i-1);
			unknownCell = false;
		}
		temp_pose.position.x = Path(0,i);
		temp_pose.position.y = Path(1,i);
		temp_pose.position.z = (float) FRT_cell.c*(map_scale/254.00) + delta_e/2 - 3.00;
		temp_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw (temp_output(i), 0.0, Path(2,i));
		msg.poses[i] = temp_pose;
	}
  Poses = temp_output; // Poses are rolls
	//ROS_WARN_STREAM(temp_output);
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "base_link";
  float roll_0 = 0.0;
  float COST_MAX = temp_output.cols() * M_PI/2; // The worst case is when there is M_PI/4 diff and M_PI/4 const for each sample
  for(int i = 0;i< temp_output.cols();i++)
  {
    if(temp_output(i) > M_PI/4 || temp_output(i) < -M_PI/4) //hazard
    {
      cost = 1.5*COST_MAX;
      return cost/COST_MAX;
    }
    else
    {
      cost = fabs(temp_output(i)-roll_0)+fabs(temp_output(i))+cost; // 1st term differential term 2nd term current roll
      roll_0 = temp_output(i);
    }
    //normalized output
    return cost/COST_MAX;
  }
	
}

float RoverPathClass::Arm_energy(MatrixXf Path, Vector3f goal)
{
   float cost = 0.0;
   float x,y,theta_0,theta;
   float COST_MAX = M_PI*Path.cols();
   theta_0 = 0; //assuming that initially arm points to the target
   // make sure the goal is being described in the body frame (base_link)
   for (int i = 0; i<Path.cols();i++)
   {
     x = goal(0) - Path(0,i);
     y = goal(1) - Path(1,i);
     theta = atan2(y,x);
     cost = cost + fabs(theta - theta_0);
     theta_0 = theta;
   }
   //normalized output
   return cost/COST_MAX;
}

void RoverPathClass::Rover_parts(MatrixXf trajectory, MatrixXf& FrontRightTrack, MatrixXf& FrontLeftTrack, MatrixXf& RearRightTrack, MatrixXf& RearLeftTrack, MatrixXf& Arm)
{

    Vector3f FRT_vector, FLT_vector, RRT_vector, RLT_vector, Arm_vector, temp;
    MatrixXf temp_mat;
    int length  =  trajectory.cols();

    FrontRightTrack.setZero(3,length);
    FrontLeftTrack.setZero(3,length);
    RearRightTrack.setZero(3,length);
    RearLeftTrack.setZero(3,length);
    Arm.setZero(3,length);


    

    FRT_vector <<  0.265,-0.3965,1.0;
    FLT_vector <<  0.265, 0.3965,1.0;
    RRT_vector << -0.265,-0.3965,1.0;
    RLT_vector << -0.265, 0.3965,1.0;
    Arm_vector << -0.350,    0.0,1.0;
    temp       <<    0.0,    0.0,1.0;
    Matrix3f Trans;

    for(int i=0; i < length; i++ )
    {
      Trans = TranMatrix2D(trajectory(2,i),trajectory(0,i),trajectory(1,i));
      //ROS_ERROR_STREAM(Trans);
      //ROS_WARN_STREAM(FLT_vector);
      //ROS_INFO_STREAM(Trans*FLT_vector);

      temp  = Trans * FRT_vector;

      FrontRightTrack(0,i) = temp(0);
      FrontRightTrack(1,i) = temp(1);
      FrontRightTrack(2,i) = temp(2);


      temp  = Trans * FLT_vector;

      FrontLeftTrack(0,i) = temp(0);
      FrontLeftTrack(1,i) = temp(1);
      FrontLeftTrack(2,i) = temp(2);


      temp  = Trans * RRT_vector;

      RearRightTrack(0,i) = temp(0);
      RearRightTrack(1,i) = temp(1);
      RearRightTrack(2,i) = temp(2);


      temp  = Trans * RLT_vector;

      RearLeftTrack(0,i) = temp(0);
      RearLeftTrack(1,i) = temp(1);
      RearLeftTrack(2,i) = temp(2);


      temp  = Trans * Arm_vector;

      Arm(0,i) = temp(0);
      Arm(1,i) = temp(1);
      Arm(2,i) = temp(2);
    }
}

Matrix3f RoverPathClass::TranMatrix2D(float theta, float t_x, float t_y)
{
	Matrix3f output;
	output.setIdentity(3,3);
	output(0,0) = cos(theta);    output(0,1) = -sin(theta);    output(0,2) = t_x;
	output(1,0) = sin(theta);    output(1,1) =  cos(theta);    output(1,2) = t_y;
	return output;
}

geometry_msgs::Pose RoverPathClass::EigVecToPose(Vector3f in)
{
    geometry_msgs::Pose res;
    res.position.x = in(0);
    res.position.y = in(1);
    res.position.z = in(2);
    return res;
}

void RoverPathClass::find_init_control(Vector3f Goal, int particle_no, int& piece_no,MatrixXf& x)
{

  nav_msgs::Path candidate_path = find_init_candidate(EigVecToPose(Goal));
  //case of failure
  if(CPinfo.sample_it == 0)
  {
    ROS_ERROR("Failed to find a path");
    return;
  }
  piece_no = CPinfo.sample_it;
  std::vector<double> v_vector;
  std::vector<double> o_vector;
  std::ostringstream ctrl_id;
  ctrl_id.str("");
  ctrl_id.clear();
  ctrl_id << CPinfo.path_it;
  if(!node_Ptr->getParam("V_1", v_vector)) ROS_FATAL("Lookup table not found - V_1 is missing");
  if(!node_Ptr->getParam("O_"+ctrl_id.str(), o_vector)) ROS_FATAL_STREAM("Lookup table not found - O_" <<ctrl_id.str()<< "  is missing");

  //Initialize Particle
  x.setOnes(2*piece_no,particle_no);
  size_t j=0;
  for(size_t i=0;i < 2*piece_no ;i++)  // First element set
  {

    x(i,0) = v_vector[j];
    i++;
    x(i,0) = o_vector[j];
    j++;
  }
  // Reset the path address
  CPinfo.path_it = 0;
  CPinfo.sample_it = 0;
  CPinfo.bad_it.clear();
  path_vector.clear();

}

void RoverPathClass::get_global_attributes(boost::shared_ptr <ros::Publisher> path_trace_pub_)
{
  path_trace_pub =path_trace_pub_;
  path_trace_pub_exist = true;
}

MatrixXf RoverPathClass::PSO_path_finder(Vector3f Goal,Vector3f Goal_arm,Vector2f V_curr_c,double Ts,int particle_no,int iteration,int piece_no_,VectorXf& output, bool& solution_found)
{
	ROS_INFO("PSO Starts!... GOAL:");
	if(demo_) std::cout << Goal << std::endl;
	
	/*       particle structure  
	       n Particle and m piece
	| v11 v12 ...particle N.O. ... v1n|
	| w11 w12 ...particle N.O. ... w1n|
  |	        ...	                    |
  |	     Piece N.O.	                |
  |	        ...	                    |
	| vmn vmn ...particle N.O. ... vmn|
	| wm1 wm2 ...particle N.O. ... wmn|
	*/
	
	//Definition
	//int piece_no = 1;
  int piece_no = piece_no_;
  MatrixXf x;  			//patricle
  find_init_control(Goal_arm, particle_no, piece_no,x);

	VectorXf x_best(2*piece_no);
	VectorXf G(2*piece_no);

  MatrixXf output_tra;
  output_tra.setZero(3,sample);

	float G_cost = 1.0/0.0;
	float x_best_cost = 1.0/0.0;
	   
  // Init particles Start

  /*
   * 1- V_curr_c is not needed anymore, instead here the find_init_candidate should be invoked
   * 2- from output of find_init_candidate the control inputs has to be found from the lookup table
   * 3- The pso params have to be set in a way that the search alsways stay close to the initial candidate, opposite of what we were doing up to now
  */

  /*// Failure
  if(CPinfo.sample_it == 0)
  {
    ROS_ERROR("No pass found");
    return output_tra;
  }

  x.setOnes(2*piece_no,particle_no);

	for(size_t i=0;i < 2*piece_no ;i++)  // First element set
	{
	
		x(i,0) = V_curr_c(0);
		i++;
		x(i,0) = V_curr_c(1);
  }*/
		
	//ROS_INFO_STREAM("V_curr_c is -------->  "  << V_curr_c);

	float rand_v;
	float rand_w;
	for(size_t i=1;i < x.cols();i++)
	{
     for(size_t j=0; j< 2*piece_no ; j++)
     {
        rand_v = ((float) (rand() % 40))/100 + 0.8;
        rand_w  = ((float) (rand() % 200))/100 -1.0;
        x(j,i)  = rand_v * x(j,0); //fixed linear speed
        j++;
        x(j,i)  = rand_w * x(j,0);
     }
	}

  MatrixXf v;  			//particle_increment
  v.setZero(2*piece_no,particle_no);


	// Init particle End

	if(demo_) ROS_INFO("Initial particle");
	if(demo_) std::cout << x << "\n";

	        
	solution_found = false;
	
	for(size_t i=0;i< 2*piece_no; i++)
	{
     G(i) = x(i,0);
	}
	
	x_best = G;
  		
  // Assuming zero speed and zero position, being on the origin
  Vector3f x_0;
  x_0 << 0.0, 0.0, 0.0; //laser_dist, 0.0, 0.0;
  Vector3f x_dot_0;
  x_dot_0 << 0.0, 0.0, 0.0;
  	
  //outputs
  Vector3f x_dot_f;
  VectorXf V_in;
  VectorXf Omega_in;
  MatrixXf tra;
  	
  V_in.setOnes(sample);
  Omega_in.setOnes(sample);
  	
  path_z_inc = 0.0;
    	
	for (size_t k = 0; k < iteration; k++)
	{
		
    		
    MatrixXf tra;
		tra.setZero(3,sample);
		for(size_t i=0; i < particle_no; i++)
		{
      //Defining random coeffs of the PSO
			float r_1  = ((float) (rand() % 200))/100 -1.0;
			float r_2  = ((float) (rand() % 200))/100 -1.0;
		
		
			//first part of trajectory: tra_0			
			int sub_sample = floor(V_in.size()/piece_no);
			size_t row_it = 0;
			for(size_t jj=0;jj < V_in.size() ; jj++)
      {       //      first_iteration    in case sample % piece_no is not 0
				if ( (jj%sub_sample) == 0  &&    jj != 0 &&    (sub_sample*piece_no - row_it) > 1 ) row_it = row_it+2;
				V_in(jj)     = x(row_it,i);
				Omega_in(jj) = x(row_it+1,i);
			}
			if(demo_) ROS_INFO_STREAM_ONCE("V_in  "  <<  V_in);
			if(demo_) ROS_INFO_STREAM_ONCE("Omega_in   "  <<  Omega_in);
			
		 	
			//simulating the trajectory
      tra = Rover_vw(V_in, Omega_in, b, piece_no *Ts_global/sample ,x_0,x_dot_0 , sample,x_dot_f);
			traj_to_cloud(tra);
							
			Vector3f tra_tail;
			tra_tail(0) = tra(0,tra.cols()-1);
			tra_tail(1) = tra(1,tra.cols()-1);
			tra_tail(2) = 0;
			if(demo_) ROS_WARN_STREAM_ONCE("tra length  " << tra.cols() << "   tra_tail :  " << tra_tail);
						
			//Calculating the cost of trajectory
			PATH_COST cost = Cost_of_path(tra, master_grid_);
			
			if(demo_) ROS_INFO_ONCE("cost of the path is %f",cost.Lethal_cost);
			
			float prop_speed = fabs(x(0,i));
			
      //Defining the objective function - Objective func needs significant improvement
			float Ob_func_1 = sqrtf( pow((tra_tail(0)-Goal(0)), 2) + pow((tra_tail(1)-Goal(1)), 2) );    //effect of distance from the goal
      float Ob_func_2 = (cost.Lethal_cost + cost.Inf_cost);				     // path cost
      float Ob_func_3 = fabs(x(0,0) - prop_speed);			      		     // speed effect
      // Goal distance cost can be evaluated using cnmap now
      // Chassis cost term to be added
      // Arm energy consumption based on path to be calculated and added
      float Ob_func = Goal_gain *Ob_func_1 + Cost_gain *Ob_func_2 + Speed_gain * Ob_func_3;
				      
			if(demo_) ROS_ERROR("goal distance: %f, path cost: %f, speed different:%f",Ob_func_1,Ob_func_2,Ob_func_3);
			if(demo_) ROS_INFO("LETHAL COST: %f",cost.Lethal_cost);		      
			if(demo_) ROS_INFO("Ob_fun: %f",Ob_func);

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
				if (cost.Lethal_cost < 1) solution_found = true;
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
    	
	output = G;
  //if(demo_) ROS_INFO_STREAM("best particle" << "\n" << G);
  if(path_trace_pub_exist)
  {
    sensor_msgs::PointCloud2 PC;
    PC.header.frame_id = "base_link";
    PC.header.stamp = ros::Time::now();
    pcl::toROSMsg(path_trace_pcl,PC);
    path_trace_pub->publish(PC);
  }
	return output_tra;    
}
//Private member functions
bool RoverPathClass::is_in_costmap(float x, float y, costmap *grid)
{
	bool output = false;
	//ROS_ERROR("1");
	float x_origin = grid->getOriginX();
	float x_size,y_size;
	grid->get_costmap_size_in_meter(x_size,y_size);
	
	
	if( x > x_origin && x < (x_size + x_origin) && fabs(y) < y_size )
	{
		output = true;
		//ROS_INFO("%f > %f && %f < %f && %f < %f ", x, x_origin, x, x_size+x_origin,fabs(y), y_size);
	}
	return output;
}
