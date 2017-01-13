#include "costmap.h"


costmap::costmap(double x_orig_,double y_orig_,unsigned int cell_x_size_,unsigned int cell_y_size_,float resolution_,std::string frame_id_,bool show_debug_)
{
    default_cost = -1; //unknown cell
    resolution = resolution_;
    x_orig = x_orig_;
    y_orig = y_orig_;
    x_size = cell_x_size*resolution;
    y_size = cell_y_size*resolution;
    cell_x_size = cell_x_size_;
    cell_y_size = cell_y_size_;
    //ROS_INFO("cell_x_size: %d   and    cell_y_size:   %d   ",cell_x_size,cell_y_size);
    mat.resize(cell_x_size, std::vector<signed char> (cell_y_size, default_cost));
    frame_id = frame_id_;
    show_debug = show_debug_;
    if (show_debug_) ROS_INFO("costmap prints debug msgs");
    init = true;
}

costmap::costmap()
{
   init = false;
}

bool costmap::is_initialized()
{
  return init;
}

float costmap::getOriginX()
{
    float out = x_orig;
    return out;
}
float costmap::getOriginY()
{
    float out = y_orig;
    return out;
}
float costmap::getResolution()
{
    float out = resolution;
    return out;
}
void costmap::get_costmap_size_in_meter(float& x_size_,float& y_size_)
{
    x_size_ = x_size;
    y_size_ = y_size;
}
void costmap::get_costmap_size_in_cell(unsigned int& mx, unsigned int& my)
{
    mx = cell_x_size;
    my = cell_y_size;
}

bool costmap::worldToMap (float wx, float wy,unsigned int& mx, unsigned int& my)
{
    bool is_valid = true;
	 mx = (int)((wx - x_orig) / resolution);
     my = (int)((wy - y_orig) / resolution);
     if(mx > cell_x_size || my > cell_y_size ) is_valid = false;
     return is_valid;
}

void costmap::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy)
{
    wx = x_orig + (mx + 0.5) * resolution;
    wy = y_orig + (my + 0.5) * resolution;
}

void costmap::worldToMapEnforceBounds (double wx, double wy, int &mx, int &my)
{
	 mx = (int)((wx - x_orig) / resolution);
     my = (int)((wy - y_orig) / resolution);
     mx = std::min(mx,(int) cell_x_size);
     mx = std::max(mx,0);
     my = std::min(my,(int) cell_x_size);
     my = std::max(my,0);

}
signed char costmap::getCost(unsigned int mx,unsigned int my)
{
    
    signed char cost;
    if(mx < cell_x_size && my < cell_y_size)
       cost = mat[my][mx];
    else
    {
        cost = default_cost;
        if(show_debug) ROS_ERROR("Out of range cost request");
    }
    return cost;
}
signed char costmap::getCost_WC(float wx,float wy) //cost from world coordinates
{
    signed char cost;
    unsigned int mx,my;
    if(!worldToMap(wx,wy,mx,my))
    {
        cost = default_cost;
        if(show_debug) ROS_ERROR("Out of range cost request");
    }
    else cost = getCost(mx,my);
    return cost;
}
nav_msgs::OccupancyGrid costmap::getROSmsg()
{
    nav_msgs::OccupancyGrid msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = ros::Time::now(); 
    msg.info.resolution = resolution;
    msg.info.width = cell_x_size;
    msg.info.height = cell_y_size;
    msg.info.origin.position.x = x_orig;
    msg.info.origin.position.y = y_orig;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = 0.0;
    msg.info.origin.orientation.w = 1.0;
    msg.data = MatrixToVector();
    return msg;
}
void costmap::setCost(unsigned int mx,unsigned int my,signed char cost)
{
    if(mx < cell_x_size && my < cell_y_size)
        mat[my][mx] = cost;
    else
        if(show_debug) ROS_ERROR("Out of range cell request");
}
void costmap::setCost_WC(float wx,float wy, signed char cost)
{
     unsigned mx,my;
     bool k = worldToMap(wx,wy,mx,my);
     setCost(mx,my,cost);
}
void costmap::setCost_v(std::vector<signed char> vector,int cols)
{
    //ROS_ERROR_STREAM("vector size is   "<<vector.size());
    
    if (vector.size()%cols != 0)
    {
        ROS_ERROR("Number of columns:%d is incompatible with vector size:%d",cols,(int) vector.size());
        return;
    }
    for(int i = 0;i< vector.size();i++)
    {
        mat[floor(i/cols)][i%cols] = vector[i];
        //ROS_WARN_STREAM("cost:  " << (int) vector[i] << "is set to i and j:  " <<floor(i/cols) <<"   and   "<< i%cols );
    }
}
void costmap::resetMap() 
{
    int rows = mat.size();
    int cols = mat[0].size();
    for(int i = 0;i< rows; i++)
    {
        for(int j = 0; j< cols; j++)
        {
            mat[i][j] = 0;
        }
    }
    
}
void costmap::resetMap (unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn)
{
    for(int i = x0;i< xn; i++)
    {
        for(int j = y0; j< yn; j++)
        {
            mat[i][j] = 0;
        }
    }    
}
std::vector<signed char> costmap::MatrixToVector()
{
    int rows = mat.size();
    int cols = mat[0].size();
    std::vector<signed char> Vector = std::vector<signed char> (rows*cols);
    //ROS_INFO("rows:%d     cols:%d   size:%d",rows,cols,rows*cols);
    for(int i = 0;i< rows; i++)
    {
        for(int j = 0; j< cols; j++)
        {
            Vector[j + i*cols ] = mat[i][j];
            //ROS_WARN_STREAM("vector[ " <<j + i*cols<<" ] = Matrix["<<i<<"] ["<<j<<"]  =  "<< (int) Vector[j + i*cols ]);
            
        }
    }
    return Vector;

}
bool costmap::Is_in_map(int mx,int my){
  if (mx > cell_x_size || mx < 0)
    return false;
  if (my > cell_y_size || my < 0)
    return false;
  return true;
}
