#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <maze_navigator/costmap.h>

void Costmap::fromPixel(int py, int px, double &wy, double &wx)
{
    tf::Point pt((float)px * map_resolution_, (float)py * map_resolution_, 0.0);
    tf::Point trans = map_transform_ * pt;
    wy = trans.getY();
    wx = trans.getX();
}

void Costmap::fromWorld(double wy, double wx, int &py, int &px)
{
    tf::Point pt(wx, wy, 0);
    tf::Point trans = map_transform_.inverse() * pt;
    trans *= 1.0 / map_resolution_;
    py = (int)round(trans.getY());
    px = (int)round(trans.getX());
}

void Costmap::store(const nav_msgs::OccupancyGridConstPtr &msg_map)
{
    if (costmap_stored_) // no updates
        return;

    width_ = msg_map->info.width;
    height_ = msg_map->info.height;
    map_resolution_ = msg_map->info.resolution;

    // transformation
    tf::poseMsgToTF(msg_map->info.origin, map_transform_);

    const std::vector<int8_t> &map_msg_data(msg_map->data);
    for (int y = 0; y < height_; y++)
        for (int x = 0; x < width_; x++)
            map_[y][x] = map_msg_data[y * width_ + x];
    costmap_stored_ = true;
}

void Costmap::storeSimple(const nav_msgs::OccupancyGridConstPtr &msg_map)
{
    if (simplemap_stored_) // no updates
        return;

    int s_width = msg_map->info.width;
    int s_height = msg_map->info.height;

    const std::vector<int8_t> &map_msg_data(msg_map->data);
    for (int y = 0; y < s_height; y++)
        for (int x = 0; x < s_width; x++)
        {
            simple_map_[y][x] = map_msg_data[y * s_width + x];
        }
    simplemap_stored_ = true;
}

void Costmap::layerMaps()
{
    if (!(simplemap_stored_ && costmap_stored_))
        return;

    ROS_INFO("MAP LAYERING\n");
    /*
            costmap is changing based of robot laser scan but
            lethal obstacles are fixed. 
        */
    for (int y = 0; y < height_; y++)
        for (int x = 0; x < width_; x++)
        {
            if (map_[y][x] >= LETHAL_THRESH && simple_map_[y][x] < LETHAL_THRESH)
            {
                map_[y][x] = LETHAL_THRESH - 1; // this is not a constant obstacle
            }

            if (simple_map_[y][x] == -1)
                map_[y][x] = -1;
            else if (simple_map_[y][x] == 100)
                map_[y][x] = 100;
        }
    init_ = true;
}

bool Costmap::isFree(int y, int x)
{
    return inBounds(y, x) && (map_[y][x] >= 0 && map_[y][x] <= FREE_THRESH);
}

bool Costmap::inBounds(int y, int x)
{
    return (y >= 0 && y < height_ && x >= 0 && x < width_);
}

bool Costmap::isLethal(int y, int x)
{
    return inBounds(y, x) && (map_[y][x] >= LETHAL_THRESH);
}

bool Costmap::isObstacle(int y, int x)
{
    return inBounds(y, x) && (map_[y][x] < 0 || map_[y][x] >= OCC_THRESH);
}
