#ifndef EXPL_PLANNER_H
#define EXPLORATION_PLANNER_H

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
    vector<tf::Vector3> vecs_;
    vector<double> mag_;
    vector<pair<int, int> > pts_;
};


// Exploration planner
class ExplPlanner
{
  public:
    Explmap expl_map_;

    bool init_;
    Planner() { init_ = false; }

    void init(const RobotPose &r_pos, CostMap &costmap);

    // callbacks
    RobotPose getNextGoal(const RobotPose &r_pos, CostMap &costmap, bool &goal_found);
    void goalReachedCallback(const RobotPose &r_pos, CostMap &costmap);
    
  private:
    deque<pair<int, int> > dq_;
    set<pair<int, int> > proc_points_;

    tf::Vector3 y_axis_(0, 1, 0);
    tf::Vector3 z_axis_(0, 0, 1); 
    
    //int pix_2_comp_[MAX_SIZE][MAX_SIZE];
    //int pix_2_cnt_[MAX_SIZE][MAX_SIZE];

    vector<TargetPosition> target_positions;
    void constructLayer(const RobotPose &r_pos, int layer_flag)
    {
        dq_.pb(mp(r_pos.y_, r_pos.x_));
        while (!dq_.empty())
        {
            pair<int, int> top = dq_.front();
            dq_.pop_front();

            int y = top.first;
            int x = top.second;

            if (layer_flag == ROBOT_REACHABLE && !costmap.isFree(y, x))
                continue;
            if (layer_flag == COND_REACHABLE && costmap.isObstacle(y, x))
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

                    if (!costmap.inBounds(ay, ax))
                        continue;

                    if (layer_flag == ROBOT_REACHABLE && !costmap.isFree(ay, ax))
                        continue;
                    if (layer_flag == COND_REACHABLE && costmap.isObstacle(ay, ax))
                        continue;
                    if (aux_map_[ay][ax] & layer_flag)
                        continue;

                    dq_.pb(mp(ay, ax));
                }
            }
        }
    }
};

#endif