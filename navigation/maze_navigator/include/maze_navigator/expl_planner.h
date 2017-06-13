#ifndef EXPL_PLANNER_H
#define EXPLORATION_PLANNER_H

#include <vector>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <maze_navigator/costmap.h>
#include <maze_navigator/expl_map.h>
#include <maze_navigator/robot_pose.h>

// field of view constants
const double VIEW_ANGLE = 0.5;
const int ANGLE_PARTS = 50;
const double VIEW_RANGE = 0.7;

// field of view calculator
struct FoV
{
    double wy_;
    double wx_;
    std::vector<tf::Vector3> vecs_;
    std::vector<double> mag_;
    std::vector<std::pair<int, int> > pts_;
};

// planner constants
const int ROBOT_ANGLE_GRANULARITY = 4;

// Exploration planner
class ExplPlanner
{
  public:
    Explmap expl_map_;

    bool init_;
    ExplPlanner() {
        init_ = false; 
        y_axis_ = tf::Vector3(0, 1, 0);
        z_axis_ = tf::Vector3(0, 0, 1);
    }

    void init(const RobotPose &r_pos, Costmap &Costmap);

    // callbacks
    RobotPose getNextGoal(const RobotPose &r_pos, Costmap &costmap, bool &goal_found);
    void goalReachedCb(const RobotPose &r_pos, Costmap &costmap);

    void addLayer(int **layer_map, int height, int width,
                  int layer_flag);

  private:
    std::deque<std::pair<int, int> > dq_;

    tf::Vector3 y_axis_;
    tf::Vector3 z_axis_;

    std::vector<TargetCell> getVisibleTargetCells(const RobotPose &r_pos, Costmap &costmap);
    void constructExplorationMap(const RobotPose &r_pos, Costmap &costmap);

    /*void constructLayer(const RobotPose &r_pos, int layer_flag)
    {
        dq_.pb(mp(r_pos.y_, r_pos.x_));
        while (!dq_.empty())
        {
            pair<int, int> top = dq_.front();
            dq_.pop_front();

            int y = top.first;
            int x = top.second;

            if (layer_flag == ROBOT_REACHABLE && !Costmap.isFree(y, x))
                continue;
            if (layer_flag == COND_REACHABLE && Costmap.isObstacle(y, x))
                continue;
            if (aux_map_[y][x] & layer_flag)
                continue;
            aux_map_[y][x] |= layer_flag;

            // 8 - connect
            for (int dx = -1; dx <= 1; dx++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    int ay = y + dy;
                    int ax = x + dx;

                    if (!Costmap.inBounds(ay, ax))
                        continue;

                    if (layer_flag == ROBOT_REACHABLE && !Costmap.isFree(ay, ax))
                        continue;
                    if (layer_flag == COND_REACHABLE && Costmap.isObstacle(ay, ax))
                        continue;
                    if (aux_map_[ay][ax] & layer_flag)
                        continue;

                    dq_.pb(mp(ay, ax));
                }
            }
        }
    }*/
};

#endif