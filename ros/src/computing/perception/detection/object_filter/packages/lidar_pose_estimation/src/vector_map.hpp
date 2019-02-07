#ifndef VECTOR_MAP_HPP
#define VECTOR_MAP_HPP

#include <vector>

struct LanePoint
{
    double tx;  // translation of X axis
    double ty;  // translation of Y axis
    double rz;  // rotation of Z axis (yaw)
    double sr;  // squared radius
};

class VectorMap
{
    public:

        bool load();
        bool getyaw(double x, double y, double& yaw);

    private:

        std::vector<LanePoint> lane_points_;
};

#endif
