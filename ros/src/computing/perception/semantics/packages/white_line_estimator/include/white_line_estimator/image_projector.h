#ifndef IMAGE_PROJECTOR_H_INCLUDED
#define IMAGE_PROJECTOR_H_INCLUDED

//headers in ROS
#include <std_msgs/Header.h>

//headers in STL
#include <map>

struct ProjectedImage
{
    std_msgs::Header header;
    int height;
    int width;
    std::vector<double> ranges;
    std::vector<double> distances;
};

class ImageProjector
{
public:
    ImageProjector();
    ~ImageProjector();
    ProjectedImage reproject();
private:
};
#endif  //IMAGE_PROJECTOR_H_INCLUDED