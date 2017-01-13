#include "cnmap.h"
using namespace std;
unsigned char const UNKOWN = 255;
unsigned char const FREE = 0;
unsigned char const OCC = 254;
cnmap::cnmap(costmap base_map, int scale, bool debug_)
{
   base = base_map;
   debug = debug_;
   unsigned int mx_new,my_new;
   base.get_costmap_size_in_cell(mx_new,my_new);
   mx_new = floor(mx_new/scale);
   my_new = floor(my_new/scale);
   self = costmap(base.getOriginX(),base.getOriginY(),mx_new,my_new,base.getResolution()*scale,base.frame_id,false);
}
void cnmap::update(){
  //Finding home cell status
  unsigned char cost = find_cell_status(home_cell.first,home_cell.second);
  if (cost == OCC)
  {
    ROS_ERROR("Home cell is occupied !!!!!");
    return;
  }
  check_surrouding_cells(home_cell.first,home_cell.second);
  int watch = 0;
  while(DUL.size() > 0) //do it until the list is empty
  {
     pair <size_t,size_t> temp_pair = DUL.back();  //accessing last value of the list
     DUL.pop_back(); // eliminating the last value of the list
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

void cnmap::set_home(size_t a,size_t b){
  home_cell = make_pair(a,b);
}

void cnmap::check_surrouding_cells(int a,int b)
{
  unsigned char cost;
  for(int i=-1;i<2;i++)
    for(int j=-1;j<2;j++)
    {
       cost = find_cell_status(a+i,b+j);
       if(cost == FREE && !is_in_list(make_pair(a,b),DCL))
       {
          DUL.push_back(make_pair(abs(a+i),abs(b+j)));
       }
    }
}

unsigned char cnmap::find_cell_status(int a, int b)
{
  unsigned char result = UNKOWN; //by default unknown
  // check it is a valid request
  if(!self.Is_in_map(a,b))
  {
     if(debug) ROS_ERROR("invalid cell request!!");
     return result;
  }
  int sub_res = self.getResolution()/base.getResolution();
  double wx_self,wy_self,wx_base_0,wy_base_0;
  size_t unkown = 0;size_t free = 0;size_t occ = 0;
  self.mapToWorld(a,b,wx_self,wy_self);
  unsigned int mx_base_0,my_base_0;
  wx_base_0 = wx_self-self.getResolution()/2+base.getResolution()/2;
  wy_base_0 = wy_self-self.getResolution()/2+base.getResolution()/2;
  base.worldToMap(wx_base_0,wy_base_0,mx_base_0,my_base_0);
  unsigned char curr_cost;
  for(size_t i =0; i < sub_res; i++)
  {
    for(size_t j =0; j < sub_res; j++)
    {
      curr_cost =base.getCost(mx_base_0+i,my_base_0+j);
      if(curr_cost == UNKOWN) //unkown
        unkown++;
      if(curr_cost == OCC)
        occ++;
      if(curr_cost == FREE)
        free++;
    }
  }
  int sub_cell_no = (int) floor(1/sub_res/sub_res);
  if(free > (int)round(sub_cell_no * 0.8) )
    result = FREE; //Free
  if(occ > (int)round(sub_cell_no * 0.8))
    result = OCC;//Occupoied
  return result;
}

bool cnmap::is_in_list(pair <size_t,size_t> e,list <pair<size_t,size_t> > ls)
{
   bool res = true;
   list<pair <size_t,size_t> >::iterator it = find(ls.begin(),ls.end(),e);
   if( it == ls.end() ) res = false; // returning end value implies not being found
   return res;
}
