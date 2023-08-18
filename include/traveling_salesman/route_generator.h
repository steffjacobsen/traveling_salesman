
#include<ros/ros.h>
#include<random>
#include<vector>
#include<std_srvs/Trigger.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#include<traveling_salesman/GeneratePoints.h>

using namespace std;

struct Point{
    double x;
    double y;
};

class RouteGenerator{
private:
    std::string topic;
    ros::ServiceServer generate_points_srv;
    ros::Publisher points_pub;
    ros::Publisher ids_pub;
public:
    RouteGenerator(ros::NodeHandle *nh);
    void publish_points(const vector<Point>& points);
    void publish_ids(const vector<Point>& points);
    bool generate_pts_callback(traveling_salesman::GeneratePoints::Request &req, traveling_salesman::GeneratePoints::Response &res);
    vector<Point> generate_random_points(double min_x, double max_x, double min_y, double max_y, int num_points);
};