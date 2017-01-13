#ifndef CNMAP_H
#define CNMAP_H
#include "costmap.h"


class cnmap
{
public:
  cnmap(costmap base_map, int scale, bool debug_);
  void update();
  void set_home(size_t a,size_t b);

protected:
  float resolution; // meters/cell
  std::list < std::pair<size_t,size_t> > DUL;  //Discovered Unchecked Cells
  std::list < std::pair<size_t,size_t> > DCL;  //Discovered Checkd    Cells
  std::pair <size_t,size_t> home_cell;
  bool debug;
  costmap base;
  costmap self;

private:
  unsigned char find_cell_status(int a,int b);
  void check_surrouding_cells(int a,int b);
  bool is_in_list(std::pair <size_t,size_t> e,std::list < std::pair <size_t,size_t> > ls);

};

#endif // CNMAP_H
