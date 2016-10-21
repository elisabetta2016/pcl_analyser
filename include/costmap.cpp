#include "costmap.h"

costmap:costmap(double x_orig_,double y_orig_,float x_size_,float y_size_,float resolution_)
{
    default_cost = -1; //unknown cell
    float resolution = resolution_;
    x_orig = x_orig_;
    y_orig = y_orig_;
    x_size = x_size_;
    y_size = y_size_;
    cell_x_size = x_size/resolution;
    cell_y_size = y_size/resolution;
    mat.resize(cell_x_size, std::vector<int8_t>(cell_y_size, default_cost));
}

float costmap::get_x_origin()
{
    float out = x_orig;
    return out;
}
float costmap::get_y_origin()
{
    float out = y_orig;
    return out;
}
float costmap::get_resolution()
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

bool costmap::worldtomap (float wx, float wy,unsigned int& mx, unsigned int& my)
{
    bool is_valid = true;
	 mx = (int)((wx - x_orig) / resolution);
     my = (int)((wy - y_orig) / resolution);
     if(mx > cell_x_size || my > cell_y_size ) is_valid = false;
     return is_valid;
}

void costmap::maptoworld(unsigned int mx, unsigned int my, double& wx, double& wy)
{
    wx = x_orig + (mx + 0.5) * resolution;
    wy = y_orig + (my + 0.5) * resolution;
}

int8_t getcost(int mx,int my)
{
    int8_t cost = mat[mx][my];
    return cost;
}
int8_t costmap::getcost_WC(float wx,float wy)
{
    int8_t cost;
    unsigned int mx,my;
    if(!worldtomap(wx,wy,mx,my))
    {
        cost = default_cost;
        ROS_ERROR();
    }

}
