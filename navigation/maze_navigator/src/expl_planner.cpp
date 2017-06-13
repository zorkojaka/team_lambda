#include <set>
#include <algorithm>
#include <cmath>

#include <maze_navigator/robot_pose.h>
#include <maze_navigator/expl_planner.h>

#define mp make_pair
#define pb push_back

using namespace std;

vector<TargetCell> ExplPlanner::getVisibleTargetCells(const RobotPose &r_pos, Costmap &costmap)
{

    double step = 2.0 * VIEW_ANGLE / ANGLE_PARTS;
    set<int> pts_unique;

    vector<TargetCell> t_result;
    for (double rot = -VIEW_ANGLE; rot <= VIEW_ANGLE; rot += step)
    {
        tf::Vector3 curr_rot = r_pos.rot_.rotate(z_axis_, rot);

        int py, px;
        for (double mult = 0.01; mult <= VIEW_RANGE; mult += 0.01)
        {
            costmap.fromWorld(r_pos.wy_ + curr_rot.y() * mult, r_pos.wx_ + curr_rot.x() * mult, py, px);

            // obstacle can't a be target cell!!!
            if (costmap.isObstacle(py, px))
                break;
            if (expl_map_.isTargetCell(py, px))
            {
                TargetCell t_cell = expl_map_.getTargetCell(py, px);

                if (t_cell.isSimpleCell()) // cell is visible
                {
                    if (pts_unique.find(t_cell.getId()) == pts_unique.end())
                    {
                        pts_unique.insert(t_cell.getId());
                        t_result.pb(t_cell);
                    }
                }
            }
        }
    }
    return t_result;
}

// initialization
void ExplPlanner::init(const RobotPose &r_pos, Costmap &costmap)
{
    constructExplorationMap(r_pos, costmap);
   // constructPlan(costmap);
    init_ = true;
}

void ExplPlanner::addLayer(std::vector< std::vector<int> > layer_map, int height, int width,
                           int layer_flag)
{
    expl_map_.addLayer(layer_map, height, width, layer_flag);
}

void ExplPlanner::constructExplorationMap(const RobotPose &r_pos, Costmap &costmap)
{

    // constructLayer(r_pos, ROBOT_REACHABLE); // floodfill robot reachable
    // constructLayer(r_pos, COND_REACHABLE); // floodfill conditionali reachable
}

// map enrichment
RobotPose ExplPlanner::getNextGoal(const RobotPose &r_pos,
                      Costmap &costmap, bool &goal_found)
{
    RobotPose r_goal;
    dq_.clear();

    expl_map_.clearVisited();

    int startY = r_pos.y_;
    int startX = r_pos.x_;

    // start new search
    dq_.pb(mp(r_pos.y_, r_pos.x_));
    while (!dq_.empty())
    {
        pair<int, int> top = dq_.front();
        dq_.pop_front();

        int y = top.first;
        int x = top.second;

        if (!expl_map_.isRobotReachable(y, x)) // only reachable cells
        {
            ROS_INFO("Robot is out of map range!\n");
        }
        if (expl_map_.isVisited(y, x))
            continue;
        expl_map_.markVisited(y, x);

        bool has_new_points = false;

        double angle_step = (2 * M_PI) / ROBOT_ANGLE_GRANULARITY;
        double full_circle = 2 * M_PI;
        for (double angle = 0.0; angle < full_circle; angle += angle_step)
        {
            tf::Vector3 curr_rot = y_axis_.rotate(z_axis_, angle);

            vector<TargetCell> t_cells_visible = getVisibleTargetCells(
                RobotPose(y, x, curr_rot, costmap), costmap);

            vector<TargetCell> t_cells_new;
            for (int i = 0; i < t_cells_visible.size(); i++)
            {
                if (expl_map_.isVisitedTargetCell(t_cells_visible[i].y_, t_cells_visible[i].x_))
                    continue;
                t_cells_new.pb(t_cells_visible[i]);
            }

            if (t_cells_new.size() > 0)
            {
                goal_found = true;
                r_goal = RobotPose(y, x, curr_rot, angle, costmap);
                break;
            }
        }

        if (goal_found)
        {
            ROS_INFO("Planner: NEW GOAL FOUND\n");
            dq_.clear();
            return r_goal;
        }

        // 8 - connect
        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                int ay = y + dy;
                int ax = x + dx;

                if (!costmap.inBounds(ay, ax))
                    continue;
                if (!expl_map_.isRobotReachable(ay, ax))
                    continue;
                if (expl_map_.isVisited(ay, ax))
                    continue;
                dq_.pb(mp(ay, ax));
            }
        }
    }

    goal_found = false;
    ROS_WARN("Planner: NO GOAL FOUND\n");
    return r_pos;
}

/*
        Goal reached callback.
*/
void ExplPlanner::goalReachedCb(const RobotPose &r_pos, Costmap &costmap)
{
    vector<TargetCell> t_cells_visible = getVisibleTargetCells(
        r_pos, costmap);

    for (int i = 0; i < t_cells_visible.size(); i++)
    {
        expl_map_.markVisitedTargetCell(t_cells_visible[i].y_, t_cells_visible[i].x_);
    }
}
