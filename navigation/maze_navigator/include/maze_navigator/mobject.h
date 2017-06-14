#ifndef MOBJECT_H
#define MOBJECT_H

#include <algorithm>
#include <vector>

#include <geometry_msgs/PoseArray.h>

class MObject{
    public:
        
        MObject(){ last_call_size_ = 0; }

        bool hasNewFace();
        std::pair<double,double> getLastFacePos();
        void facePositionCallback(const geometry_msgs::PoseArray::ConstPtr& msg);

    private:
        std::vector< std::pair<double, double> > face_pos_;
        int last_call_size_;
};

#endif