#include<ros/ros.h>
#include<random>
#include<vector>
#include<algorithm>
#include<std_srvs/Trigger.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#include<nav_msgs/Path.h>
#include<jsk_rviz_plugins/OverlayText.h>
#include<traveling_salesman/PlanRoute.h>

using namespace std;

struct Point{
    double x;
    double y;
};

class RoutePlanner{
private:
    vector<Point> loaded_points;
    ros::Subscriber points_sub;
    ros::Publisher route_pub;
    ros::Publisher overlay_pub;
    ros::ServiceServer plan_srv;
    void pts_callback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    bool plan_callback(traveling_salesman::PlanRoute::Request& req, traveling_salesman::PlanRoute::Response& res);
    vector<Point> bruteforce(const vector<Point>& points);
    vector<Point> twooptswap(const vector<Point>& points, int v1, int v2);
    vector<Point> twooptsearch(const vector<Point>& points);
    vector<Point> threeoptsearch(const vector<Point>& points);
    vector<Point> threeoptswap(const vector<Point>& points, int v1, int v2, int v3);
    vector<Point> branchandbound(const vector<Point>& points);
    vector<Point> nearestneighbor(const vector<Point>& points);
    double routecost(const vector<Point>& points);
    void publish_route(const vector<Point>& route);
    void publish_overlay(const string& text);
public:
    RoutePlanner(ros::NodeHandle *nh);
};