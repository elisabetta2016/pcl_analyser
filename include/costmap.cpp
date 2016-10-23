#include "costmap.h"


costmap::costmap(double x_orig_,double y_orig_,float x_size_,float y_size_,float resolution_,std::string frame_id_)
{
    default_cost = -1; //unknown cell
    float resolution = resolution_;
    x_orig = x_orig_;
    y_orig = y_orig_;
    x_size = x_size_;
    y_size = y_size_;
    cell_x_size = x_size/resolution;
    cell_y_size = y_size/resolution;
    mat.resize(cell_x_size, std::vector<signed char> (cell_y_size, default_cost));
    frame_id = frame_id_;
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

signed char costmap::getcost(unsigned int mx,unsigned int my)
{
    
    signed char cost;
    if(mx < cell_x_size && my < cell_y_size)
       cost = mat[mx][my];
    else
    {
        cost = default_cost;
        ROS_ERROR("Out of range cost request");
    }
    return cost;
}
signed char costmap::getcost_WC(float wx,float wy) //cost from world coordinates
{
    signed char cost;
    unsigned int mx,my;
    if(!worldtomap(wx,wy,mx,my))
    {
        cost = default_cost;
        ROS_ERROR("Out of range cost request");
    }
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
    msg.data = MatrixToVector(mat);
    return msg;
}
void costmap::setcost(unsigned int mx,unsigned int my,signed char cost)
{
    if(mx < cell_x_size && my < cell_y_size)
        mat[mx][my] = cost;
    else
        ROS_ERROR("Out of range cell request");
}
void costmap::setcost_WC(float wx,float wy, signed char cost)
{
     unsigned mx,my;
     bool k = worldtomap(wx,wy,mx,my);
     setcost(mx,my,cost);
}
std::vector<signed char> costmap::MatrixToVector( std::vector< std::vector<signed char> > Matrix)
{
    int rows = Matrix.size();
    int cols = Matrix[0].size();
    std::vector<signed char> Vector = std::vector<signed char> (rows*cols);
    for(int i = 0;i< rows; i++)
    {
        for(int j = 0; j< cols; j++)
        {
            Vector[j + i*cols ] = Matrix[i][j];
        }
    }
    return Vector;

}
