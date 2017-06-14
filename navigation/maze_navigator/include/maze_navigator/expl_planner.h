#ifndef EXPL_PLANNER_H
#define EXPL_PLANNER_H

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

// cell visibility constants
const int CHG_THRESH_IN_WORLD = 0.2;

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
        expl_map_ = Explmap(255, 255);
    }

    void init(const RobotPose &r_pos, Costmap &costmap);

    // callbacks
    RobotPose getNextGoal(const RobotPose &r_pos, Costmap &costmap, bool &goal_found);
    RobotPose getApproachGoal(const RobotPose &r_pos, const RobotPose &goalCell, Costmap &costmap, bool &goal_found);
    RobotPose findClosestReachableCell(const RobotPose &r_pos, Costmap &costmap, bool &goal_found);
    
    void goalReachedCb(const RobotPose &r_pos, Costmap &costmap);

    void addLayer(std::vector< std::vector<int> > layer_map, int height, int width,
                  int layer_flag);

  private:
    std::deque<std::pair<int, int> > dq_;
    bool expl_used_goals_[MAX_SIZE][MAX_SIZE][ROBOT_ANGLE_GRANULARITY + 2];


    tf::Vector3 y_axis_;
    tf::Vector3 z_axis_;

    std::vector<TargetCell> getVisibleTargetCells(const RobotPose &r_pos, Costmap &costmap);
    void constructExplorationMap(const RobotPose &r_pos, Costmap &costmap);

    void constructLayer(const RobotPose &r_pos, Costmap& costmap, int layer_flag);

    bool detCellCondVis(const RobotPose &r_pos, Costmap &costmap, TargetCell &t_cell);

};

#endif