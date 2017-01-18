#include "cnmap.h"
using namespace std;
unsigned char const UNKOWN = 255;
unsigned char const FREE = 0;
unsigned char const OCC = 254;
cnmap::cnmap(costmap *base_map_ptr, int scale, bool debug_)
{
   base_ptr = base_map_ptr;
   debug = debug_;
   if(!base_ptr->is_initialized())
   {
     ROS_WARN("cnmap: the costmap instant is not initialized");
     return;
   }
   unsigned int mx_new,my_new;
   base_ptr->get_costmap_size_in_cell(mx_new,my_new);
   mx_new = floor(mx_new/scale);
   my_new = floor(my_new/scale);
   self_ptr = new costmap(base_ptr->getOriginX(),base_ptr->getOriginY(),mx_new,my_new,base_ptr->getResolution()*scale,base_ptr->frame_id,false);
   init_ = true;
   home_ = false;
   DUL_init = false;
}
cnmap::cnmap()
{
  init_ = false;
  home_ = false;
}
cnmap::~cnmap()
{
  ROS_WARN("cnmap instant destructed!");
  delete base_ptr,self_ptr;
}

void cnmap::update(){
  if(!init_)
  {
    ROS_ERROR("cnmap: cnmap instant is yet to be initialized, update failed!");
    return;
  }
  if(!home_)
  {
    ROS_ERROR("cnmap: Update is invoked while home is yet to be known, update failed!");
    return;
  }
  //Finding home cell status
  unsigned char cost = find_cell_status(home_cell.first,home_cell.second);
  if (cost == OCC)
  {
    ROS_ERROR("Home cell is occupied !!!!!");
    return;
  }
  check_surrouding_cells(home_cell.first,home_cell.second);
  int watch = 0;
  while(!DUL.empty() && DUL_init) //do it until the vector is empty
  {
     ROS_INFO_STREAM("DUL SIZE is : "<<DUL.size());
     pair <unsigned int,unsigned int> temp_pair;
     temp_pair = DUL [DUL.size()-1 ];  //accessing last value of the vector
     DUL.pop_back(); // eliminating the last value of the vector
     check_surrouding_cells(temp_pair.first,temp_pair.second); //check the surrounding cells of this cell
     DCL.push_back(temp_pair); //save it in the DCL
     watch++;
     if (watch > 1000)
     {
       ROS_WARN("INF Loop detected !!!!!");
       break;
     }
  }
}

void cnmap::set_home(float wx,float wy){
  if(!init_)
  {
    ROS_ERROR("cnmap: cnmap instant is yet to be initialized, set home failed!");
    return;
  }
  unsigned int a,b;
  self_ptr->worldToMap(wx,wy,a,b);
  home_cell = make_pair(a,b);
  home_ = true;
}

bool cnmap::is_initialized()
{
  bool res = false;
  if (init_) res = true;
  return res;
}

bool cnmap::is_home_set()
{
  return home_;
}

void cnmap::publish_ROS(ros::Publisher *pubPtr_)
{
  if(!base_ptr->is_initialized()) return;
  nav_msgs::OccupancyGrid msg = base_ptr->getROSmsg();
  pubPtr_->publish(msg);
}
void cnmap::check_surrouding_cells(int mx,int my)
{
  unsigned int a,b;
  if(a<0 || b<0)
  {
    a = max(0,mx);
    b = max(0,my);
  }
  unsigned char cost;
  for(int i=-1;i<2;i++)
    for(int j=-1;j<2;j++)
    {
       cost = find_cell_status(a+i,b+j);
       if(cost == FREE && !is_in_vector(make_pair(a,b),DCL))
       {
          DUL.push_back(make_pair(abs(a+i),abs(b+j)));
          DUL_init = true;
       }
    }
}

unsigned char cnmap::find_cell_status(int a, int b)
{
  unsigned char result = UNKOWN; //by default unknown
  // check it is a valid request
  if(!self_ptr->Is_in_map(a,b))
  {
     ROS_ERROR_COND_NAMED(debug, "cnmap","invalid cell request!!");
     return result;
  }
  int sub_res = self_ptr->getResolution()/base_ptr->getResolution();
  double wx_self,wy_self,wx_base_0,wy_base_0;
  unsigned int unkown = 0;
  unsigned int free = 0;
  unsigned int occ = 0;
  self_ptr->mapToWorld(a,b,wx_self,wy_self);
  unsigned int mx_base_0,my_base_0;
  wx_base_0 = wx_self-self_ptr->getResolution()/2+base_ptr->getResolution()/2;
  wy_base_0 = wy_self-self_ptr->getResolution()/2+base_ptr->getResolution()/2;
  base_ptr->worldToMap(wx_base_0,wy_base_0,mx_base_0,my_base_0);
  ROS_INFO_COND_NAMED(debug,"cnmap","a:%d, b:%d, mx_base: %d,my_base: %d",a,b,mx_base_0,my_base_0);
  ROS_INFO_COND(debug,"wx_self: %f,wy_self: %f, wx_b:%f, wy_b:%f",wx_self,wy_self,wx_base_0,wy_base_0);
  unsigned char curr_cost;
  for(unsigned int i =0; i < sub_res; i++)
  {
    for(unsigned int j =0; j < sub_res; j++)
    {
      curr_cost =base_ptr->getCost(mx_base_0+i,my_base_0+j);
      //debug
      ROS_ERROR_COND(debug && curr_cost!=UNKOWN,"cost: %d",curr_cost);
      if(curr_cost == UNKOWN) //unkown
        unkown++;
      if(curr_cost == OCC)
      {
        occ++;
        ROS_INFO("Occupied cell found: %d",occ);
      }
      if(curr_cost == FREE)
      {
        free++;
        ROS_INFO("Free cell found: %d",free);
      }

    }
  }
  int sub_cell_no = (int) floor(1/sub_res/sub_res);
  if(free > (int)round(sub_cell_no * 0.8) )
    result = FREE; //Free
  if(occ > (int)round(sub_cell_no * 0.8))
    result = OCC;//Occupoied
  return result;
}

bool cnmap::is_in_vector(pair <unsigned int,unsigned int> e,vector <pair<unsigned int,unsigned int> > ls)
{
   bool res = true;
   vector <pair <unsigned int,unsigned int> >::iterator it = find(ls.begin(),ls.end(),e);
   if( it == ls.end() ) res = false; // returning end value implies not being found
   return res;
}
