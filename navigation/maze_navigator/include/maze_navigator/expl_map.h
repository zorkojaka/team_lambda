#ifndef EXPL_MAP_H
#define EXPL_MAP_H

#include <vector>

#include <maze_navigator/costmap.h> // MAX_SIZE constant

// map constants
const int LAYER_ROBOT_REACHABLE = 2;
const int LAYER_COND_REACHABLE = 4;

const int LAYER_TARGET_CELL = 8;
const int LAYER_TARGET_REACHED = 32;

const int LAYER_VIS = 1;
const int LAYER_PROCESSED = 64;

const int LAYER_NORTH_SOUTH = 128;
const int LAYER_SOUTH_NORTH = 256;
const int LAYER_EAST_WEST = 512;
const int LAYER_WEST_EAST = 1024;
const int LAYER_SIMPLE = 2048;

const int LAYER_ALL_TARGETS = LAYER_NORTH_SOUTH | LAYER_SOUTH_NORTH | 
  LAYER_EAST_WEST | LAYER_WEST_EAST | LAYER_SIMPLE;

/*
    Target cell.
*/
class TargetCell
{
public:
  int y_, x_;
  int layer_flag_;
  TargetCell(int y, int x, int layer_flag) : y_(y), x_(x), layer_flag_(layer_flag) {}

  bool isSimpleCell();
  int getId();
};

/*
    Exploration map.
*/
class Explmap
{

public:
  int cell_idx_[MAX_SIZE][MAX_SIZE];
  int aux_map_[MAX_SIZE][MAX_SIZE];

  int width_, height_;
  std::vector<TargetCell> t_cells_;

  Explmap()
  {}
  Explmap(int height, int width): height_(height), width_(width)
  {}

  int getLabel(int y, int x);
  TargetCell getTargetCell(int y, int x);

  void setLabel(int y, int x, int val);

  // helper
  bool isVisited(int y, int x);
  bool isVisitedTargetCell(int y, int x);
  bool isRobotReachable(int y, int x);
  bool isTargetCell(int y, int x);

  void clearVisited();
  void markVisited(int y, int x);
  void markVisitedTargetCell(int y, int x);

  void addLayer(std::vector< std::vector<int> > layer_map, int height, int width, int layer_flag);

private:
  void addTargetLayer(std::vector< std::vector<int> > layer_map, int layer_flag);
};

#endif