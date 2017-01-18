#ifndef CNMAP_H
#define CNMAP_H
#include "costmap.h"


class cnmap
{
public:
  cnmap(costmap *base_map_ptr, int scale, bool debug_);
  cnmap();
  ~cnmap();
  void update();
  void set_home(float wx,float wy);
  void publish_ROS(ros::Publisher *pubPtr_);
  bool is_initialized();
  bool is_home_set();


protected:
  float resolution; // meters/cell
  std::vector < std::pair<unsigned int,unsigned int> > DUL;  //Discovered Unchecked Cells
  std::vector < std::pair<unsigned int,unsigned int> > DCL;  //Discovered Checkd    Cells
  std::pair <unsigned int,unsigned int> home_cell;
  bool debug;
  costmap *base_ptr;
  costmap *self_ptr;
  bool init_;
  bool home_;
  bool DUL_init;

private:
  unsigned char find_cell_status(int a,int b);
  void check_surrouding_cells(int a,int b);
  bool is_in_vector(std::pair <unsigned int,unsigned int> e,std::vector < std::pair <unsigned int,unsigned int> > ls);

};

#endif // CNMAP_H
