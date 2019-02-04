#include <autoware2vectormap_converter/autoware2vectormap_converter.h>

using vector_map::VectorMap;
using autoware_map::AutowareMap;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "awm_vmap_converter");


    Converter awmap2vmap_converter;
    ros::spin();
}
