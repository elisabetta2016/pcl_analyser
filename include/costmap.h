//#includes

class costmap{
    public:
        costmap(double x_orig_,double y_orig_,float x_size_,float y_size_,float resolution_,std::string frame_id_);
        float get_x_origin();
        float get_y_origin();
        float get_resolution();
        void  get_costmap_size_in_meter(float& x_size_,float& y_size_);
        void  get_costmap_size_in_cell(unsigned int& mx, unsigned int& my);
        bool  worldtomap (float wx, float wy,unsigned int& mx, unsigned int& my);
        void  maptoworld(unsigned int mx, unsigned int my, double& wx, double& wy);
        int8_t getcost(unsigned int mx,unsigned int my);
        int8_t getcost_WC(float wx,float wy); //cost from world coordinates
        nav_msgs::OccupancyGrid getROSmsg();

        void  setcost(unsigned int mx,unsigned int my,int8_t cost);

        void setcost_WC(float wx,float wy, int8_t cost);

    protected:
        float resolution; // meters/cell
        float x_orig;
        float y_orig;
        float x_size;
        float y_size;
        unsigned int cell_x_size;
        unsigned int cell_y_size;
        std::vector<std::vector<int8_t>> mat;
        std::string frame_id; 

    private:
        int8_t default_cost;
};
#endif