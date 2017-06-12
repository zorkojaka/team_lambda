#ifndef EXPL_PLANNER_H
#define EXPLORATION_PLANNER_H

const int MAX_SIZE = 600;

class ExplPlanner
{
  public:
    int aux_map_[MAX_SIZE][MAX_SIZE];
    bool init_;

    Planner() { init_ = false; }

    void init(const RobotPose &r_pos, CostMap &costmap);
    
    // callbacks
    RobotPose getNextGoal(const RobotPose &r_pos, CostMap &costmap, bool &goal_found);
    void goalReachedCallback(const RobotPose &r_pos, CostMap &costmap);
    
  private:
    deque<pair<int, int> > dq_;
    set<pair<int, int> > proc_points_;
    //int pix_2_comp_[MAX_SIZE][MAX_SIZE];
    //int pix_2_cnt_[MAX_SIZE][MAX_SIZE];

    vector<TargetPosition> target_positions;

   // set<int> active_targets;
   // set<int> passive_targets;

  //  int active_target;

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

    void getTargetCells(const RobotPose &r_pos)
    {
        for (int y = 0; y < costmap.height_; y++)
            for (int x = 0; x < costmap.width_; x++)
            {
                if (!(aux_map_[y][x] & COND_REACHABLE))
                    continue;
                for (int dx = -1; dx <= 1; dx++)
                    for (int dy = -1; dy <= 1; dy++){
                        // do a 4 - connect
                        if(abs(dx) + abs(dy) != 1)
                            continue;

                        if (costmap.inBounds(y + dy, x + dx) &&
                            costmap.isLethalObstacle(y + dy, x + dx))
                        {
                            aux_map_[y][x] |= TARGET_CELL;
                        }
                    }
            }
    }

    void getVisibleTargetCells(CostMap& costmap)
    {
        ROS_INFO("CONSTRUCTING VISIBLE TARGET CELLS...");

        tf::Vector3 y_axis(0, 1, 0);
        tf::Vector3 z_axis(0, 0, 1); // z-axis rotation

        for (int y = 0; y < costmap.height_; y++){
            for (int x = 0; x < costmap.width_; x++)
            {
                if (!(aux_map_[y][x] & ROBOT_REACHABLE)) // only reachable cells
                    continue;
                
                vector<pair<int,int> > pts;
                for (double angle = 0.0; angle < 2 * M_PI; angle += (2 * M_PI) / ROBOT_ANGLE_PARTS)
                {
                   // Dynamic rotation
                    tf::Vector3 curr_rot = y_axis.rotate(z_axis, angle);
                    pts = calcVisibleTargetCells(RobotPose(y, x, curr_rot, costmap), costmap);
                    for (int i = 0; i < pts.size(); i++)
                        aux_map_[pts[i].first][pts[i].second] |= VISIBLE_TARGET_CELL; 
                }
            }
        }

        // remove all not directly visible target cells
        for (int y = 0; y < costmap.height_; y++)
            for (int x = 0; x < costmap.width_; x++){
                if((aux_map_[y][x] & TARGET_CELL) && ((aux_map_[y][x] & VISIBLE_TARGET_CELL) == 0)){
                    aux_map_[y][x] ^= TARGET_CELL;
                }
            }
         ROS_INFO("CONSTRUCTION FINISHED...");
       
    }

    /*
    // create connected components
    void numberThem(int y, int x, vector< pair< int,int >& comp, CostMap& costmap){
        if(!costmap.inBounds(y, x) ||
            costmap.isLethalObstacle(y, x) || 
            aux_map_[y][x] & VIS)){
                return;
            }
        if(aux_map_[y][x] & TARGET_CELL){
            comp.push_back(mp(y,x));
            for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++){
                    numberThem(y + dy, x + dx, comp, costmap);
                }
        }
    }

    vector<int> getContComponents(vector< pair<int,int> > &pts){
        set<int> cmp;
        for(int j = 0;j < pts.size();j++){
            int comp_idx_curr = pix_2_comp_[pts[j].first][pts[j].second];
            cmp.insert(comp_idx_curr);
        }

        vector<int> ans;
        for(set<int>::iterator itr = cmp.begin(); itr != cmp.end(); itr++){
            ans.push_back(*itr);
        }
        return ans;
    }

    bool areContinious(vector< pair<int,int> > &pts, int comp_idx){
        int pts_cnt = 0;
        int min_pos = 1000000000;
        int max_pos = -1;
        for(int j = 0;j < pts.size();j++){
            int comp_idx_curr = pix_2_comp_[pts[j].first][pts[j].second];
            if(comp_idx == comp_idx_curr){
                min_pos = min(min_pos, pix_2_cnt_[pts[j].first][pts[j].second]);
                max_pos = max(max_pos, pix_2_cnt_[pts[j].first][pts[j].second]);
                pts_cnt++;
            }
        }
        return (max_pos - min_pos + 1 == pts_cnt);
    }

    void constructPlan(CostMap& costmap){
         // clear visited cells
        for (int y = 0; y < costmap.height_; y++)
            for (int x = 0; x < costmap.width_; x++){
                if(aux_map_[y][x] & VIS)
                    aux_map_[y][x] ^= VIS;
            }

        int n_comp = 0; // number of components
        vector< vector<pair<int,int> > > comp;
        for (int y = 0; y < costmap.height_; y++)
            for (int x = 0; x < costmap.width_; x++){
                if((aux_map_[y][x] & TARGET_CELL) && (aux_map_[y][x] & VIS == 0)){
                    comp.push_back(vector<pair<int,int> >());
                    numberThem(y, x, comp[n_comp], costmap);
                    n_comp++;
                }
            }
        
        for(int i = 0;i < n_comp;i++){
            for(int j = 0;j < comp[i].size();i++){
                pix_2_comp_[comp[i][j].first][comp[i][j].second] = i;
                pix_2_cnt_[comp[i][j].first][comp[i][j].second] = j;
            }
        }

        ROS_INFO("Constructed %d components", n_comp);    
        ROS_INFO("Constructing planner...");     
        tf::Vector3 y_axis(0, 1, 0);
        tf::Vector3 z_axis(0, 0, 1); // z-axis rotation


        target_positions.clear();
        for (int y = 0; y < costmap.height_; y++){
            for (int x = 0; x < costmap.width_; x++)
            {
                if (!(aux_map_[y][x] & ROBOT_REACHABLE)) // only reachable cells
                    continue;
                
                vector<pair<int,int> > pts;
                vector<int> ccomp;
                for (double angle = 0.0; angle < 2 * M_PI; angle += (2 * M_PI) / ROBOT_ANGLE_PARTS)
                {
                    // Dynamic rotation
                    tf::Vector3 curr_rot = y_axis.rotate(z_axis, angle);
                    pts = calcVisibleTargetCells(RobotPose(y, x, curr_rot, costmap), costmap);
                    ccomp = getContComponents(pts);
                    
                    bool flag = false;

                    TargetPosition t_pos;
                    t_pos.py_ = y;
                    t_pos.px_ = x;
                    t_pos.angle_ = angle;
                    t_pos.pts_ = pts;
                    for(int i = 0;i < ccomp.size();i++){
                        if(areContinious(pts, ccomp[i]))
                            t_pos.ccomp_.push_back(ccomp[i]);
                    }
                    if(t_pos.ccomp_.size() > 0) // sees at least one continious inteval
                        target_positions.push_back(t_pos);
                }
            }
        }

        for(int comp_idx = 0;comp_idx < n_comp;comp_idx++){
            // get me all target positions for this component
            vector<int> t_idx;
            for(int i = 0;i < target_positions.size();i++){
                for(int j = 0;j < target_positions[i].ccomp_.size();j++)
                    if(target_positions[i].ccomp_[j] == comp_idx)
                    {
                        t_idx.pb(i);
                        break;
                    }
            }

        }
        ROS_INFO("Planner constructed, %d target positions!", (int)target_positions.size());     
    }
    */

    vector<pair<int, int> > calcVisibleTargetCells(const RobotPose &r_pos, CostMap &costmap)
    {
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
            
                if (costmap.isObstacle(py, px))
                    break;
                else if (aux_map_[py][px] & TARGET_CELL){
                    if(pts_unique.find(mp(py, px)) == pts_unique.end()){
                        pts_unique.insert(mp(py, px));
                    }
                }
            }
        }

        vector<pair<int,int> > ans;
        for(set<pair<int,int> >::iterator itr = pts_unique.begin(); itr != pts_unique.end(); itr++){
            ans.pb(mp(itr->first, itr->second));
        }
        return ans;
    }
};

#endif