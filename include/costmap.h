#ifndef COSTMAP_H
#define COSTMAP_H

//#includes
 //Standard
  #include <ros/ros.h>
  #include <ros/timer.h>
  #include <math.h>
  #include <iostream>
  #include <string>
 //Messages
  #include <nav_msgs/OccupancyGrid.h>

class costmap{
    public:
        costmap(double x_orig_,double y_orig_,unsigned int cell_x_size_,unsigned int cell_y_size_,float resolution_,std::string frame_id_,bool show_debug_);
        float getOriginX();
        float getOriginY();
        float getResolution();
        void  get_costmap_size_in_meter(float& x_size_,float& y_size_);
        void  get_costmap_size_in_cell(unsigned int& mx, unsigned int& my);
        bool  worldToMap (float wx, float wy,unsigned int& mx, unsigned int& my);
        void  worldToMapEnforceBounds (double wx, double wy, int &mx, int &my);
        void  mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy);
        signed char getCost(unsigned int mx,unsigned int my);
        signed char getCost_WC(float wx,float wy); //cost from world coordinates
        nav_msgs::OccupancyGrid getROSmsg();

        void  setCost(unsigned int mx,unsigned int my,signed char cost);

        void setCost_WC(float wx,float wy, signed char cost);
        void resetMap();
        void resetMap (unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn);
        void setCost_v(std::vector<signed char> vector,int cols);

    protected:
        float resolution; // meters/cell
        float x_orig;
        float y_orig;
        float x_size;
        float y_size;
        unsigned int cell_x_size;
        unsigned int cell_y_size;
        std::vector< std::vector<signed char> > mat;
        std::string frame_id;
        bool show_debug; 

    private:
        signed char default_cost;
        std::vector<signed char> MatrixToVector();
};
#endif
