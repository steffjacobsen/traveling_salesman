#include "traveling_salesman/route_generator.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "route_generator_node");
    ros::NodeHandle nh("~");
    RouteGenerator rg = RouteGenerator(&nh);
    ros::spin();
    return 0;
}