#ifndef COSTMAP_H
#define COSTMAP_H

#include <nav_msgs/GetMap.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


const int MAX_SIZE = 600;

// costmap constants
const int FREE_CELL = 0;
const int OCC_CELL = 255;

const int FREE_THRESH = 50;
const int OCC_THRESH = 100;
const int LETHAL_THRESH = 100;

class Costmap
{
  public:
    float map_resolution_;

    int width_, height_;
    int map_[MAX_SIZE][MAX_SIZE];
    int simple_map_[MAX_SIZE][MAX_SIZE];

    bool init_, simplemap_stored_, costmap_stored_;
    Costmap()
    {
        init_ = false;
        simplemap_stored_ = false;
        costmap_stored_ = false;
    }

    void fromPixel(int py, int px, double &wy, double &wx);
    void fromWorld(double wy, double wx, int &py, int &px);
    
    void store(const nav_msgs::OccupancyGridConstPtr &msg_map);
    void storeSimple(const nav_msgs::OccupancyGridConstPtr &msg_map);
    void layerMaps();

    bool isFree(int y, int x);
    bool inBounds(int y, int x);
    bool isLethal(int y, int x);
    bool isObstacle(int y, int x);

  private:
    tf::Transform map_transform_;
};

#endif