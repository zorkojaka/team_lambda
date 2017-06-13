#include <maze_navigator/visualizer.h>

visualization_msgs::Marker createPointCloud(int id, float r, float g, float b)
{
    visualization_msgs::Marker point_cloud;

    point_cloud.header.stamp = ros::Time::now();
    point_cloud.header.frame_id = "/map";
    point_cloud.id = id;

    point_cloud.type = visualization_msgs::Marker::POINTS;
    point_cloud.action = visualization_msgs::Marker::ADD;

    point_cloud.scale.x = 0.02; // point width
    point_cloud.scale.y = 0.02; // point height

    // violet?
    point_cloud.color.r = r;
    point_cloud.color.g = g;
    point_cloud.color.b = b;

    point_cloud.color.a = 1.0;

    return point_cloud;
}

geometry_msgs::Point createPoint(double x, double y, double z)
{
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

/*
    Visualize exploration map layer.
*/
void visualizeLayer(const ros::Publisher &vis_pub, const Explmap &expl_map, Costmap& costmap, int layer_flag,
                    visualization_msgs::Marker point_cloud)
{
    for (int y = 0; y < expl_map.height_; y++)
    {
        for (int x = 0; x < expl_map.width_; x++)
        {
            if (expl_map.aux_map_[y][x] & layer_flag)
            {
                double wx, wy;
                costmap.fromPixel(y, x, wy, wx);

                geometry_msgs::Point p;
                p.x = wx;
                p.y = wy;
                p.z = 0;
                point_cloud.points.push_back(p);
            }
        }
    }
    vis_pub.publish(point_cloud);
}

/*
    Visualize given field of view.
*/
/*
void visualizeFoV(const ros::Publisher &vis_pub, FoV &fov)
{
    // visualize view rays
    visualization_msgs::Marker line_list;
    line_list.header.stamp = ros::Time::now();
    line_list.header.frame_id = "/map";
    line_list.id = 1;

    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.scale.x = 0.005;

    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    for (int i = 0; i < fov.vecs_.size(); i++)
    {
        line_list.points.push_back(createPoint(fov.wx_, fov.wy_, 0.0));
        line_list.points.push_back(createPoint(fov.wx_ + fov.vecs_[i].x() * fov.mag_[i],
                                               fov.wy_ + fov.vecs_[i].y() * fov.mag_[i], 0.0));
    }
    vis_pub.publish(line_list);
    

    // visualize "seen" points
    visualization_msgs::Marker point_cloud = createPointCloud(7, 1.0, 1.0, 0.0);
    for (int i = 0; i < fov.pts_.size(); i++)
    {
        double wx, wy;
        costmap.fromPixel(fov.pts_[i].first, fov.pts_[i].second, wy, wx);
        point_cloud.points.push_back(createPoint(wx, wy, 0.0));
    }
    vis_pub.publish(point_cloud);
}

void debug(const ros::Publisher &vis_pub,
           visualization_msgs::Marker point_cloud)
{
    for (int y = 0; y < costmap.height_; y++)
    {
        for (int x = 0; x < costmap.width_; x++)
        {
            if (costmap.simple_map_[y][x] == 0)
            {
                double wx, wy;
                costmap.fromPixel(y, x, wy, wx);

                geometry_msgs::Point p;
                p.x = wx;
                p.y = wy;
                p.z = 0;
                point_cloud.points.push_back(p);
            }
        }
    }
    vis_pub.publish(point_cloud);
}
*/

void visualizeMultipleLayers(const ros::Publisher &vis_pub, const Explmap &expl_map, Costmap& costmap)
{
    visualizeLayer(vis_pub, expl_map, costmap,
        LAYER_TARGET_CELL, createPointCloud(0,1.0,0.0,0.0));  
    visualizeLayer(vis_pub, expl_map, costmap,
        LAYER_TARGET_REACHED, createPointCloud(1,0.0,1.0,0.0));
}


void visualizeSinglePoint(const ros::Publisher &vis_pub, RobotPose& r_pos){
    visualization_msgs::Marker point_cloud = createPointCloud(11, 0.0, 0.0, 1.0);
    point_cloud.scale.x = 0.1; // point width
    point_cloud.scale.y = 0.1; // point height

    geometry_msgs::Point p;
    p.x = r_pos.wx_;
    p.y = r_pos.wy_;
    p.z = 0;
    point_cloud.points.push_back(p);
    vis_pub.publish(point_cloud);
}

/*
visualization_msgs::Marker createArrow(int id, double r, double g, double b){
    visualization_msgs::Marker arrow;
    arrow.header.stamp = ros::Time::now();
    arrow.header.frame_id = "/map";
    arrow.id = id;

    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.scale.x = 1.0;
    arrow.scale.y = 0.02;

    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    
    arrow.color.a = 1.0;

    return arrow;
}

/*
float rgb[4][3] = {{1,0,0}, {0,1,0}, {0,0,1}, {1,1,1}};
void visualizeRobotOrientation(const ros::Publisher &vis_pub, RobotPose& r_pos, RobotPose& r_goal){
    ROS_INFO("VISUALIZE ROBOT ORIENTATION");
    int arrow_cnt = 0;
    visualization_msgs::Marker arrow;
    for (double angle = 0.0; angle < 2 * M_PI && arrow_cnt < 4; angle += (2 * M_PI) / 4.0)
    {
        arrow = createArrow(arrow_cnt, rgb[arrow_cnt][0],
            rgb[arrow_cnt][1],rgb[arrow_cnt][2]);
        arrow.pose.position = createPoint(r_pos.wx_, r_pos.wy_, 0.0);

        tf::Quaternion quat = constructRotation(angle);
        arrow.pose.orientation.x = quat.x();
        arrow.pose.orientation.y = quat.y();
        arrow.pose.orientation.z = quat.z();
        arrow.pose.orientation.w = quat.w();


        vis_pub.publish(arrow);

        arrow_cnt++;
    }

    arrow = createArrow(arrow_cnt, 0.7,
            0.0, 0.4);
    arrow.pose.position = createPoint(r_goal.wx_, r_goal.wy_, 0.0);

    arrow.pose.orientation.x = r_goal.rot_quat_.x();
    arrow.pose.orientation.y = r_goal.rot_quat_.y();
    arrow.pose.orientation.z = r_goal.rot_quat_.z();
    arrow.pose.orientation.w = r_goal.rot_quat_.w();
    vis_pub.publish(arrow);
    arrow_cnt++;


    arrow = createArrow(arrow_cnt, 0.0,
            0.0, 0.0);
    arrow.pose.position = createPoint(r_pos.wx_, r_pos.wy_, 0.0);

    arrow.pose.orientation.x = r_pos.rot_quat_.x();
    arrow.pose.orientation.y = r_pos.rot_quat_.y();
    arrow.pose.orientation.z = r_pos.rot_quat_.z();
    arrow.pose.orientation.w = r_pos.rot_quat_.w();
    vis_pub.publish(arrow);
    arrow_cnt++;
}
*/