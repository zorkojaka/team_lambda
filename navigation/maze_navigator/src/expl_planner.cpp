#include <set>
#include <algorithm>
#include <cmath>

#include <maze_navigator/robot_pose.h>
#include <maze_navigator/expl_planner.h>

#define mp make_pair
#define pb push_back

using namespace std;

/*
    Take cell visibility conditions into account.
    @returns true if cell is visible from r_pos
*/
bool ExplPlanner::detCellCondVis(const RobotPose &r_pos, Costmap &costmap, TargetCell &t_cell)
{
    if (t_cell.layer_flag_ == LAYER_SIMPLE)
        return true;

    double wx, wy;
    costmap.fromPixel(t_cell.y_, t_cell.x_, wy, wx);

    tf::Vector3 vis_vec(wx - r_pos.wx_, wy - r_pos.wy_, 0);
    if (t_cell.layer_flag_ == LAYER_NORTH_SOUTH)
        return (vis_vec.y() < 0) && (abs(vis_vec.x()) < CHG_THRESH_IN_WORLD);
    else if (t_cell.layer_flag_ == LAYER_SOUTH_NORTH)
        return (vis_vec.y() > 0) && (abs(vis_vec.x()) < CHG_THRESH_IN_WORLD);
    else if (t_cell.layer_flag_ == LAYER_EAST_WEST)
        return (vis_vec.x() < 0) && (abs(vis_vec.y()) < CHG_THRESH_IN_WORLD);
    else if (t_cell.layer_flag_ == LAYER_WEST_EAST)
        return (vis_vec.x() > 0) && (abs(vis_vec.y()) < CHG_THRESH_IN_WORLD);
    return false;
}

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

                if (detCellCondVis(r_pos, costmap, t_cell)) // cell is visible
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
    expl_map_ = Explmap(costmap.height_, costmap.width_);
    constructExplorationMap(r_pos, costmap);
    // constructPlan(costmap);
    init_ = true;
}

void ExplPlanner::addLayer(std::vector<std::vector<int> > layer_map, int height, int width,
                           int layer_flag)
{
    expl_map_.addLayer(layer_map, height, width, layer_flag);
}

void ExplPlanner::constructExplorationMap(const RobotPose &r_pos, Costmap &costmap)
{

    constructLayer(r_pos, costmap, LAYER_ROBOT_REACHABLE); // floodfill robot reachable
    // constructLayer(r_pos, COND_REACHABLE); // floodfill conditionali reachable
}

void ExplPlanner::constructLayer(const RobotPose &r_pos, Costmap &costmap, int layer_flag)
{
    int n_layer_cells = 0;

    ROS_INFO("Constructing layer %d", layer_flag);
    dq_.clear();

    dq_.pb(mp(r_pos.y_, r_pos.x_));
    while (!dq_.empty())
    {
        pair<int, int> top = dq_.front();
        dq_.pop_front();

        int y = top.first;
        int x = top.second;

        if (layer_flag == LAYER_ROBOT_REACHABLE && !costmap.isFree(y, x))
            continue;

        //if (layer_flag == COND_REACHABLE && Costmap.isObstacle(y, x))
        //    continue;
        if (expl_map_.getLabel(y, x) & layer_flag)
            continue;

        n_layer_cells++;
        expl_map_.setLabel(y, x, expl_map_.getLabel(y, x) | layer_flag);

        // 8 - connect
        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                int ay = y + dy;
                int ax = x + dx;

                if (!costmap.inBounds(ay, ax))
                    continue;

                if (layer_flag == LAYER_ROBOT_REACHABLE && !costmap.isFree(ay, ax))
                    continue;
                //                if (layer_flag == LAYER_ROBOT_REACHABLE && costmap.isObstacle(ay, ax))
                //                    continue;
                if (expl_map_.getLabel(ay, ax) & layer_flag)
                    continue;
                dq_.pb(mp(ay, ax));
            }
        }
    }

    ROS_INFO("Layer has %d cells", n_layer_cells);
}

/*
    @returns New exploration goal.
*/
RobotPose ExplPlanner::getNextGoal(const RobotPose &r_pos,
                                   Costmap &costmap, bool &goal_found)
{
    RobotPose r_goal;
    dq_.clear();

    expl_map_.clearVisited();

    int startY = r_pos.y_;
    int startX = r_pos.x_;

    /*
        If robot falls out of "comfort zone" bring it back as soon as possible.
    */
    if (!expl_map_.isRobotReachable(startY, startX)) // only reachable cells
    {
        ROS_INFO("Robot is out of map range - rescue\n");
        return findClosestReachableCell(r_pos, costmap, goal_found);
    }

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
            ROS_ERROR("Robot is out of map range!\n");
        }
        
        if (expl_map_.isVisited(y, x))
            continue;
        expl_map_.markVisited(y, x);

        bool has_new_points = false;

        double angle_step = (2 * M_PI) / ROBOT_ANGLE_GRANULARITY;
        double full_circle = 2 * M_PI;
        int angle_it = 0;
        for (double angle = 0.0; angle < full_circle; angle += angle_step, angle_it++)
        {
            tf::Vector3 curr_rot = y_axis_.rotate(z_axis_, angle);

            if (expl_used_goals_[y][x][angle_it]) // avoid reusing same goal twice
                continue;

            vector<TargetCell> t_cells_visible = getVisibleTargetCells(
                RobotPose(y, x, curr_rot, costmap), costmap);

            vector<TargetCell> t_cells_new;
            for (int i = 0; i < t_cells_visible.size(); i++)
            {
                if (expl_map_.isVisitedTargetCell(t_cells_visible[i].y_, t_cells_visible[i].x_))
                    continue;
                t_cells_new.pb(t_cells_visible[i]);
            }

            if (t_cells_new.size() > 10)
            {
                ROS_INFO("New goal %d %d %d with %d cells", y, x, angle_it, (int)t_cells_new.size() );
                goal_found = true;
                expl_used_goals_[y][x][angle_it] = true; // potential place for mistakes
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
    Finds closest cell from LAYER_ROBOT_REACHABLE.
*/
RobotPose ExplPlanner::findClosestReachableCell(const RobotPose &r_pos, Costmap &costmap, 
    bool &goal_found){
    
    RobotPose r_goal;

    dq_.clear();
    expl_map_.clearVisited();

    dq_.pb(mp(r_pos.y_, r_pos.x_));
    while (!dq_.empty())
    {
        pair<int, int> top = dq_.front();
        dq_.pop_front();

        int y = top.first;
        int x = top.second;

        if (expl_map_.isRobotReachable(y, x))
        {
            r_goal = RobotPose(y, x, costmap);

            dq_.clear();
            goal_found = true;
            return r_goal;
        }

        if (expl_map_.isVisited(y, x))
            continue;
        expl_map_.markVisited(y, x);

        // 8 - connect
        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                int ay = y + dy;
                int ax = x + dx;

                if (!costmap.inBounds(ay, ax))
                    continue;
                if (expl_map_.isVisited(ay, ax))
                    continue;
                dq_.pb(mp(ay, ax));
            }
        }
    }

    goal_found = false;
    return r_pos;
}

/*
    Get object approach goal.
*/
RobotPose ExplPlanner::getApproachGoal(const RobotPose &r_pos, const RobotPose &goal_cell,
                          Costmap &costmap, bool &goal_found)
{
    RobotPose r_goal;
    dq_.clear();

    expl_map_.clearVisited();

    int startY = goal_cell.y_;
    int startX = goal_cell.x_;

    /*
        Find closest ROBOT_REACHABLE cell and right orientation vector.
    */
    dq_.pb(mp(goal_cell.y_, goal_cell.x_));
    while (!dq_.empty())
    {
        pair<int, int> top = dq_.front();
        dq_.pop_front();

        int y = top.first;
        int x = top.second;

        if (expl_map_.isRobotReachable(y, x))
        {
            // goal found
            double wy, wx;
            costmap.fromPixel(y, x, wy, wx);

            tf::Vector3 rot_vec(goal_cell.wx_ - wx, goal_cell.wy_ - wy, 0);
            //double angle_2d = rotVecToAngle(rot_vec);
            double angle_2d = 0.0;
            r_goal = RobotPose(y, x, rot_vec, angle_2d, costmap);

            dq_.clear();
            goal_found = true;
            return r_goal;
        }

        if (expl_map_.isVisited(y, x))
            continue;
        expl_map_.markVisited(y, x);

        // 8 - connect
        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                int ay = y + dy;
                int ax = x + dx;

                if (!costmap.inBounds(ay, ax))
                    continue;
                if (expl_map_.isVisited(ay, ax))
                    continue;
                dq_.pb(mp(ay, ax));
            }
        }
    }

    goal_found = false;
    ROS_WARN("Planner: NO APPROACH GOAL FOUND\n");
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
