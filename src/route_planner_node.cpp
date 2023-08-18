#include "traveling_salesman/route_planner.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "route_planner_node");
    ros::NodeHandle nh("~");
    RoutePlanner rg = RoutePlanner(&nh);
    ros::spin();
    return 0;
}