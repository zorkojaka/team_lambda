#include <vector>
#include <set>
#include <algorithm>
#include <cmath>

#include <maze_navigator/robot_pose.h>
#include <maze_navigator/expl_planner.h>


#define mp make_pair
#define pb push_back


void ExplMap::addLayer(int** layer_map, int height, int width,
    int layer_flag){




    }


const int VISIBLE_TARGET_CELL = 128;
const int ROBOT_ANGLE_PARTS = 4;
const int PLANNER_NEW_POINTS = 2;

vector<TargetCell> ExplPlanner::getVisibleTargetCells(const RobotPose &r_pos, Costmap &costmap){
    
    tf::Vector3 z_axis(0, 0, 1); // z-axis rotation
    double step = 2.0 * VIEW_ANGLE / ANGLE_PARTS;

    set< pair<int,int> > pts_unique;
    for (double rot = -VIEW_ANGLE; rot <= VIEW_ANGLE; rot += step)
    {
        tf::Vector3 curr_rot = r_pos.rot_.rotate(z_axis, rot);

        int py, px;
        for (double mult = 0.01; mult <= VIEW_RANGE; mult += 0.01)
        {
            costmap.fromWorld(r_pos.wy_ + curr_rot.y() * mult, r_pos.wx_ + curr_rot.x() * mult, py, px);
            
            // obstacle is can't be target cell!!! 
            if (costmap.isObstacle(py, px))
                    break;
            
            if(expl_map.isTargetCell(py, px)){

                // is it visible or not???

            }


            if (costmap.isObstacle(py, px))
            {
                field_of_view.mag_[field_of_view.mag_.size() - 1] = mult;
                if(pts_unique.find(mp(py, px)) == pts_unique.end()){
                    field_of_view.pts_.pb(mp(py, px));
                    pts_unique.insert(mp(py, px));
                }
                break;
            }
        }
    }
    return field_of_view;
}


// initialization
void ExplPlanner::init(const RobotPose &r_pos, Costmap &costmap)
{
    constructExplorationMap(r_pos, costmap);
    constructPlan(costmap);
    init_ = true;
}

void ExplPlanner::addLayer(int** layer_map, int height, int width,
    int layer_flag){
        expl_map_.addLayer(layer_map, height, width, layer_flag);
}

void ExplPlanner::constructExplorationMap(const RobotPose &r_pos, Costmap &costmap){
   
   // constructLayer(r_pos, ROBOT_REACHABLE); // floodfill robot reachable
   // constructLayer(r_pos, COND_REACHABLE); // floodfill conditionali reachable


}

// map enrichment
RobotPose getNextGoal(const RobotPose &r_pos,
    Costmap& costmap, bool &goal_found)
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

        if (!expl_map_.isRobotReachable(y,x)) // only reachable cells
        {
            ROS_INFO("Robot is out of map range!\n");
        }
        if (expl_map_.isVisited(y,x))
            continue;
        expl_map_.markVisited(y,x);
        
        bool has_new_points = false;
        for (double angle = 0.0; angle < 2 * M_PI; angle += (2 * M_PI) / ROBOT_ANGLE_PARTS)
        {
            tf::Vector3 curr_rot = y_axis.rotate(z_axis, angle);
            vecotor<TargetCell> visible_cells = getVisibleTargetCells(
                RobotPose(y, x, curr_rot, costmap), costmap);

            for(int i = 0;i < visible_cells.size(); i++){
                // if already visited then do not count...
            }

            if (n_new > PLANNER_NEW_POINTS)
            {
                r_goal = RobotPose(y, x, curr_rot, angle, costmap);
                break;
            }
        }

        int dist = abs(startY - y) + abs(startX - x);
        if (n_new > PLANNER_NEW_POINTS && dist > 4)
        {
            dq_.clear();

            goal_found = true;
            ROS_INFO("Planner: NEW GOAL FOUND\n");

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
                if (!(aux_map_[ay][ax] & ROBOT_REACHABLE))
                    continue;
                if (aux_map_[ay][ax] & VIS)
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
void goalReached(const RobotPose &r_pos, CostMap &costmap)
{
    vector<pair<int, int>> pts = calcVisibleTargetCells(r_pos, costmap);
    for (int i = 0; i < pts.size(); i++)
    {
        proc_points_.insert(my_view.pts_[i]);
        aux_map_[my_view.pts_[i].first][my_view.pts_[i].second] |= TARGET_REACHED;
    }
}

