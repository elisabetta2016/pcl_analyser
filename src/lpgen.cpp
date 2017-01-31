#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cnmap.h>
#include <costmap.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include"RoverPath.h"
#include"pathsolver.h"
//Messages
 #include <nav_msgs/OccupancyGrid.h>
 #include <nav_msgs/Path.h>
 #include <geometry_msgs/PoseArray.h>
 #include <pcl_analyser/Lpath.h>
 #include <pcl_analyser/Lookuptbl.h>

//read write
#include <rosbag/bag.h>
#include <ros/package.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH



typedef pcl::PointXYZI pointT;
typedef pcl::PointCloud <pointT> PCL;
using namespace Eigen;
float b_max, b_min, c_max, c_min, d_min ,d_max, a_min, a_max;
int step_b, step_c,step_d,step_a;
VectorXf A_vec;
VectorXf B_vec;
VectorXf C_vec;
VectorXf D_vec;
VectorXf V_vec;
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
       robot_opt_path.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw (output_tra(2,i));
   }
   return robot_opt_path;
}

VectorXf linspace(float min, float max, unsigned int item)
{
  if(item<2)
  {
    VectorXf V2;
    V2.resize(1);
    V2(0) = min;
    return V2;
  }
  VectorXf V(item);
  float d = (max-min)/(item-1);
  for(int i=0; i< item; i++)
  {
    V(i) = min + i*d;
  }
  return V;
}


class lpgen
{
  public:

    lpgen(ros::NodeHandle& node)
    {
       n_=node;
       LPTmsgptr = new pcl_analyser::Lookuptbl() ;
       sample = 50;
       S_NORM = 1.5;
       lookahead= 0.0;
       PCL::Ptr tmp_ptr(new PCL);
       traceptr = tmp_ptr;
       show = true;
       cloudpub = n_.advertise<sensor_msgs::PointCloud2>("LookupTableCloud", 10);
       pathpub  = n_.advertise<nav_msgs::Path> ("pathLP",1);
       path = ros::package::getPath("pcl_analyser") + "/config/lookuptable.bag";
       bagptr = new rosbag::Bag();
       debug = false;
    }

    ~lpgen()
    {
      delete LPTmsgptr;
      delete bagptr;
      ROS_WARN("LookUp table generation node Ciao!");
    }

    void writetobag()
    {
      bagptr->open(path, rosbag::bagmode::Write);
      bagptr->write("lookuptable", ros::Time::now(), *LPTmsgptr);
      bagptr->close();
    }

    void test()
    {
      VectorXf V,Om;
      ctrlparam q;
      q.fill_in(0,33,-82,41,1);
      find_vw(q,V,Om);
      //ROS_INFO("testing find_vw");
      //ROS_WARN_STREAM("V:" << V);
      //ROS_WARN_STREAM("Omega:" << Om);
      ROS_INFO("testing Rover_vw");
      Vector3f x_0,x_dot_0;
      x_0 << 0.0,0.0,0.0;
      x_dot_0 <<0.0,0.0,0.0;
      geometry_msgs::Pose tail;
      double cost;
      ROS_INFO_STREAM( "Path is: \n" << Rover_vw(q, lookahead, 2.3, x_0,x_dot_0, tail, cost ));
    }

    void find_vw(ctrlparam Qparam,VectorXf& V_input, VectorXf& Omega_input)
    {
        V_input.setOnes(sample);
        Omega_input.setZero(sample);
        V_input *= Qparam.v;
        double s = 0;
        double ds = S_NORM/sample;
        for(int i=0; i < sample;i++)
        {
          double K = Qparam.a + Qparam.b*s + Qparam.c*pow(s,2)+Qparam.d*pow(s,3);
          Omega_input(i) = K;//V_input(i)
          s += ds;
        }
    }

    nav_msgs::Path Rover_vw(ctrlparam Qparam, double b, double Ts,Vector3f x_0,Vector3f x_dot_0,geometry_msgs::Pose& tail, double& cost )
    {
      /*  // there is a problem here, implement the what we did in matlab i guess the problem is messing with speed and Ts
       *
      Structure of x and x_dot
      | x  |
      | y  |
      | th |
      */
      VectorXf V_input;
      VectorXf Omega_input;
      find_vw(Qparam,V_input,Omega_input);
      MatrixXf x;
      x.setZero(3,sample);
      MatrixXf x_dot;
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
      cost = 0;
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
            cost = fabs(x(0,i)-x(0,i-1)) + fabs(x(1,i)-x(1,i-1));// |xk - xk-1 | + |yk - yk-1|
            if(i == sample-1)
            {
              tail.position.x = x(0,i);
              tail.position.y = x(1,i);
              tail.orientation = tf::createQuaternionMsgFromYaw(x(2,i));
            }

      }


      nav_msgs::Path msg = MatToPath(x,"base_link");
     if(show)
     {
      msg.header.frame_id = "base_link";
      msg.header.stamp = ros::Time::now();
      ros::Rate r(demo_rate);
      pathpub.publish(msg);
      r.sleep();
      if(!n_.ok()) return msg;
    }
      return msg;
    }

    nav_msgs::Path rover_tra(ctrlparam Q, geometry_msgs::Pose& tail, double& cost)
    {
      MatrixXf x;
      x.setZero(3,sample);
      VectorXf s = linspace(0.0,1.0,sample);
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
      nav_msgs::Path msg = MatToPath(x,"base_link");
      if(show)
      {
       msg.header.frame_id = "base_link";
       msg.header.stamp = ros::Time::now();
       ros::Rate r(demo_rate);
       pathpub.publish(msg);

       r.sleep();
       if(!n_.ok()) return msg;
      }
      return msg;
    }

    void fill(double v_norm)
    {
      traceptr->clear();
      //VectorXf V_vec(1);
      //V_vec << 0.1,0.15,0.2,0.3,0.4,0.7,0.9;

      V_vec *= v_norm;
      Vector3f x_0,x_dot_0;
      x_0 << 0.0,0.0,0.0;
      x_dot_0 << 0.0,0.0,0.0;
      int id = 0;
      ROS_WARN_STREAM_COND(debug,"B_vec \n" << B_vec <<"\n V_vec \n" << V_vec);
      ROS_INFO_STREAM_COND(debug,"v cols  " <<V_vec.cols()<<"  b cols  " <<B_vec.cols());
      for (int v = 0;v<V_vec.rows();v++)
      {
         ROS_INFO_COND(debug,"v: %f", V_vec(v));
         for (int a =0; a<A_vec.rows();a++)
         {
           //B_vec = linspace(-0.3*A_vec(a),-2.3*A_vec(a),4);
           for (int b = 0;b<B_vec.rows();b++)
           {
              //C_vec = linspace(-0.3*B_vec(b),-2.3*B_vec(b),4);
              ROS_ERROR_COND(debug,"   b: %f", B_vec(b));
              for (int c = 0;c<C_vec.rows();c++)
              {
                 ROS_WARN_COND(debug,"       c: %f", C_vec(c));
                 //D_vec = linspace(-0.8*C_vec(c),-1.2*C_vec(c),4);
                 for (int d = 0;d<D_vec.rows();d++)
                 {
                  ROS_INFO_COND(debug,"            d: %f", D_vec(d));
                  pcl_analyser::Lpath lp;
                  lp.a = A_vec(a);
                  lp.b = B_vec(b);
                  lp.c = C_vec(c);
                  lp.d = D_vec(d);
                  lp.v = V_vec(v);
                  lp.id = id;
                  id++;
                  double cost_temp;
                  ctrlparam q;
                  q.fill_in(lp.a,lp.b,lp.c,lp.d,lp.v);
                  //ROS_WARN_STREAM(lp);
                  geometry_msgs::Pose temp_tail;
                  lp.path = rover_tra(q,temp_tail, cost_temp);
                  lp.tail = temp_tail;
                  lp.cost = cost_temp;
                  LPTmsgptr->pathes.push_back(lp);
                  LPTmsgptr->quantity ++;
                  pushpath2cloud(lp.id,lp.path);
                 }
              }
            }
         }
      }

    }

    void fill_smart(double v_norm, float min_a,float max_a, unsigned int a_step)
    {
      traceptr->clear();
      //VectorXf V_vec(1);
      //V_vec << 0.1,0.15,0.2,0.3,0.4,0.7,0.9;
      A_vec = linspace(min_a,max_a,a_step);
      V_vec *= v_norm;
      Vector3f x_0,x_dot_0;
      x_0 << 0.0,0.0,0.0;
      x_dot_0 << 0.0,0.0,0.0;
      int id = 0;
      ROS_WARN_STREAM_COND(debug,"B_vec \n" << B_vec <<"\n V_vec \n" << V_vec);
      ROS_INFO_STREAM_COND(debug,"v cols  " <<V_vec.cols()<<"  b cols  " <<B_vec.cols());
      for (int v = 0;v<V_vec.rows();v++)
      {
         ROS_INFO_COND(debug,"v: %f", V_vec(v));
         for (int a =0; a<A_vec.rows();a++)
         {
           B_vec = linspace(-0.3*A_vec(a),-2.3*A_vec(a),4);
           for (int b = 0;b<B_vec.rows();b++)
           {
              C_vec = linspace(-0.3*B_vec(b),-2.3*B_vec(b),4);
              ROS_ERROR_COND(debug,"   b: %f", B_vec(b));
              for (int c = 0;c<C_vec.rows();c++)
              {
                 D_vec = linspace(-0.8*C_vec(c),-1.2*C_vec(c),4);
                 ROS_WARN_COND(debug,"       c: %f", C_vec(c));
                 for (int d = 0;d<D_vec.rows();d++)
                 {
                  ROS_INFO_COND(debug,"            d: %f", D_vec(d));
                  pcl_analyser::Lpath lp;
                  lp.a = A_vec(a);
                  lp.b = B_vec(b);
                  lp.c = C_vec(c);
                  lp.d = D_vec(d);
                  lp.v = V_vec(v);
                  lp.id = id;
                  id++;
                  double cost_temp;
                  ctrlparam q;
                  q.fill_in(lp.a,lp.b,lp.c,lp.d,lp.v);
                  //ROS_WARN_STREAM(lp);
                  geometry_msgs::Pose temp_tail;
                  lp.path = rover_tra(q,temp_tail, cost_temp);
                  lp.tail = temp_tail;
                  lp.cost = cost_temp;
                  LPTmsgptr->pathes.push_back(lp);
                  LPTmsgptr->quantity ++;
                  pushpath2cloud(lp.id,lp.path);
                 }
              }
            }
         }
      }

    }

    void pushpath2cloud(int id, nav_msgs::Path path)
    {
       pointT point;
       for(int i = 0; i < sample; i++)
       {
          point.x = path.poses[i].pose.position.x;
          point.y = path.poses[i].pose.position.y;
          point.z = 0;
          point.intensity = (float) id;
       }
       traceptr->push_back(point);

    }

    void gen()
    {
      demo_rate = 200.0;
      rate = 10.0;
      ros::Rate r(rate);
      sensor_msgs::PointCloud2 msg;
      V_vec = linspace(0.3,3,10);

      A_vec = linspace(-3,-3,1);
      B_vec = linspace(7,2,5);
      C_vec = linspace(3,-3,3);
      D_vec = linspace(-0.2,0.2,4);

      fill(1);
      A_vec = linspace(3,3,1);
      B_vec = linspace(-7,-2,5);
      C_vec = linspace(3,-3,3);
      D_vec = linspace(-0.2,0.2,4);

      fill(1);

      A_vec = linspace(0,0,1);
      B_vec = linspace(7,2,5);
      C_vec = linspace(-7,-5,3);
      D_vec = linspace(-0.2,0.2,4);

      fill(1);

      A_vec = linspace(0,0,1);
      B_vec = linspace(-7,-2,5);
      C_vec = linspace(7,5,3);
      D_vec = linspace(-0.2,0.2,4);

      fill(1);

      fill_smart(1,-3.0,3.0,2);

      pcl::toROSMsg(*traceptr,msg);
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "base_link";
      writetobag();
    }


    void run()
    {
       demo_rate = 1.0;
       rate = 10.0;
       ros::Rate r(rate);
       sensor_msgs::PointCloud2 msg;
       fill(10);
       pcl::toROSMsg(*traceptr,msg);
       msg.header.stamp = ros::Time::now();
       msg.header.frame_id = "base_link";
       while(n_.ok())
       {
         //test();
         //fill(2.3);
         cloudpub.publish(msg);
         r.sleep();
         ros::spinOnce();
       }

    }

  protected:
  /*state here*/
  ros::NodeHandle n_;

  // Subscribers
  ros::Subscriber subFromJoystick_;

  // Publishers
  ros::Publisher cloudpub;
  ros::Publisher pathpub;
  double rate;
  pcl_analyser::Lookuptbl* LPTmsgptr;
  int sample;
  double S_NORM;
  double lookahead;
  pcl::PointCloud<pointT>::Ptr traceptr;
  bool show;
  double demo_rate;
  std::string path;
  rosbag::Bag *bagptr;
  bool debug;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "LPgen");
  ros::NodeHandle node;
  //float b_max, b_min, c_max, c_min, d;
  //int step_b, step_c;
  float d;
  lpgen tt(node);
  switch (argc)
  {
  case 2:
    tt.gen();
    break;
  case 6:
    b_min = atof(argv[1]);
    b_max = atof(argv[2]);
    c_min = atof(argv[3]);
    c_max = atof(argv[4]);
    d =     atof(argv[5]);
    step_b = 8;
    step_c = 8;
    ROS_INFO("b = [%f  %d steps  %f]",b_min,step_b,b_max);
    ROS_INFO("c = [%f  %d steps  %f]",c_min,step_c,c_max);
    A_vec.resize(1);
    A_vec << 0.0;
    B_vec = linspace(b_min,b_max,step_b);
    C_vec = linspace(c_min,c_max,step_c);
    D_vec.resize(1);
    D_vec(0) = d;
    V_vec.resize(1);
    V_vec(0) = 1;
    tt.run();
    break;
  case 8:
    b_min  = atof(argv[1]);
    b_max  = atof(argv[2]);
    step_b = atoi(argv[3]);
    c_min  = atof(argv[4]);
    c_max  = atof(argv[5]);
    step_c = atoi(argv[6]);
    d =      atof(argv[7]);
    ROS_INFO("b = [%f  %d steps  %f]",b_min,step_b,b_max);
    ROS_INFO("c = [%f  %d steps  %f]",c_min,step_c,c_max);
    A_vec.resize(1);
    A_vec << 0.0;
    B_vec = linspace(b_min,b_max,step_b);
    C_vec = linspace(c_min,c_max,step_c);
    D_vec.resize(1);
    D_vec(0) = d;
    V_vec.resize(1);
    V_vec(0) = 1;
    tt.run();
    break;
  case 13:
    a_min  = atof(argv[1]);
    a_max  = atof(argv[2]);
    step_a = atoi(argv[3]);
    b_min  = atof(argv[4]);
    b_max  = atof(argv[5]);
    step_b = atoi(argv[6]);
    c_min  = atof(argv[7]);
    c_max  = atof(argv[8]);
    step_c = atoi(argv[9]);
    d_min  = atof(argv[10]);
    d_max  = atof(argv[11]);
    step_d = atoi(argv[12]);
    A_vec = linspace(a_min,a_max,step_a);
    B_vec = linspace(b_min,b_max,step_b);
    C_vec = linspace(c_min,c_max,step_c);
    D_vec = linspace(d_min,d_max,step_d);
    ROS_INFO("a = [%f  %d steps  %f]",a_min,step_a,a_max);
    ROS_INFO("b = [%f  %d steps  %f]",b_min,step_b,b_max);
    ROS_INFO("c = [%f  %d steps  %f]",c_min,step_c,c_max);
    ROS_INFO("d = [%f  %d steps  %f]",d_min,step_d,d_max);
    V_vec.resize(1);
    V_vec(0) = 1;
    tt.run();
    break;
  default:
    b_min  = -7.0;
    b_max  = 7.0;
    step_b = 8;
    c_min  = 7.0;
    c_max  = -7.0;
    step_c = 8;
    d =      0;
    ROS_WARN("Argument received: %d", argc);
    ROS_INFO("b = [%f  %d steps  %f]",b_min,step_b,b_max);
    ROS_INFO("c = [%f  %d steps  %f]",c_min,step_c,c_max);
    A_vec.resize(1);
    A_vec(0) = 0.0;
    B_vec = linspace(b_min,b_max,step_b);
    C_vec = linspace(c_min,c_max,step_c);
    D_vec.resize(1);
    D_vec(0) = d;
    V_vec.resize(1);
    V_vec(0) = 1;
    tt.run();
    break;
  }




  return 0;
}
