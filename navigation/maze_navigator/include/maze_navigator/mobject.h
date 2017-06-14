#ifndef MOBJECT_H
#define MOBJECT_H

#include <geometry_msgs/PoseArray.h>

class MObject{
    public:

        void facePositionCallback(const geometry_msgs::PoseArray::ConstPtr& msg);

    private:
        
};

#endif