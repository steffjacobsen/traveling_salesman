#include"traveling_salesman/route_planner.h"

RoutePlanner::RoutePlanner(ros::NodeHandle *nh){
    route_pub = nh->advertise<nav_msgs::Path>("route", 10);
    points_sub = nh->subscribe("points", 10, &RoutePlanner::pts_callback, this);
    overlay_pub = nh->advertise<jsk_rviz_plugins::OverlayText>("overlay", 10);
    plan_srv = nh->advertiseService("plan", &RoutePlanner::plan_callback, this);
}

bool RoutePlanner::plan_callback(traveling_salesman::PlanRoute::Request& req, traveling_salesman::PlanRoute::Response& res){
    
    // Check if there are points:
    if(loaded_points.empty()){
        res.success = false;
        res.message = "No points";
        return false;
    }

    uint8_t algorithm  = req.algorithm;

    // Generate a route:
    vector<Point> route;
    route.reserve(loaded_points.size());
    ros::Time begin = ros::Time::now();
    string method = "";
    if (algorithm == traveling_salesman::PlanRoute::Request::BRUTEFORCE) {
        ROS_INFO("Using brute force algorithm");
        route = bruteforce(loaded_points);
        method = "brute force";
    } else if (algorithm == traveling_salesman::PlanRoute::Request::TWOOPTSEARCH) {
        ROS_INFO("Using 2-opt algorithm");
        route = twooptsearch(loaded_points);
        method = "2-opt";
    } else if (algorithm == traveling_salesman::PlanRoute::Request::THREEOPTSEARCH) {
        ROS_INFO("Using 3-opt algorithm");
        route = threeoptsearch(loaded_points);
        method = "3-opt";
    } else if (algorithm == traveling_salesman::PlanRoute::Request::NEARESTNEIGHBOR){
        ROS_INFO("Using nearest neighbor algorithm");
        route = nearestneighbor(loaded_points);
        method = "nearest neighbor";
    }
    else {
        ROS_ERROR("Unknown algorithm");
        res.success = false;
        res.message = "Unknown algorithm";
        return false;
    }
    ROS_INFO("Route planned");
    ros::Duration duration = ros::Time::now() - begin;

    // Publish the route:
    publish_route(route);

    // Publish statistics:
    double min_cost = routecost(route);
    publish_overlay("Method: " + method + "\nThe min cost is " + to_string(min_cost) + "\nThe time cost is " + to_string(duration.toSec()) + " s");

    res.success = true;
    res.message = "Route planned";

    return true;
}

void RoutePlanner::publish_route(const vector<Point>& route){
    nav_msgs::Path path;
    path.header.frame_id = "world";
    path.header.stamp = ros::Time::now();
    for (const auto& point : route) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }
    route_pub.publish(path);
}

void RoutePlanner::publish_overlay(const string& text){
    jsk_rviz_plugins::OverlayText overlay;
    overlay.action = jsk_rviz_plugins::OverlayText::ADD;
    overlay.width = 400;
    overlay.height = 400;
    overlay.left = 10;
    overlay.top = 10;
    overlay.text_size = 24;
    overlay.line_width = 2;
    overlay.font = "DejaVu Sans Mono";
    overlay.text = text;
    overlay_pub.publish(overlay);
}

void RoutePlanner::pts_callback(const visualization_msgs::MarkerArray::ConstPtr& msg){
    loaded_points.clear();
    for (const auto& marker : msg->markers) {
        Point point;
        point.x = marker.pose.position.x;
        point.y = marker.pose.position.y;
        loaded_points.push_back(point);
    }
    publish_route(vector<Point>());
}

double RoutePlanner::routecost(const vector<Point>& points){
    double cost = 0.0;
    for (int i = 0; i < points.size() - 1; ++i) {
        const Point& point1 = points[i];
        const Point& point2 = points[i + 1];
        cost += sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
    }
    return cost;
}

vector<Point> RoutePlanner::twooptswap(const vector<Point>& points, int v1, int v2){
    assert(v1 < v2 && v2 < points.size());
    vector<Point> route;
    route.reserve(points.size());
    for (int i = 0; i < v1; ++i) {
        route.push_back(points[i]);
    }
    for (int i = v2; i >= v1; --i) {
        route.push_back(points[i]);
    }
    for (int i = v2 + 1; i < points.size(); ++i) {
        route.push_back(points[i]);
    }
    return route;
}

vector<Point> RoutePlanner::threeoptswap(const vector<Point>& points, int v1, int v2, int v3){
    assert(v1 < v2 && v2 < v3 && v3 < points.size());
    vector<Point> route;
    route.reserve(points.size());
    for (int i = 0; i < v1; ++i) {
        route.push_back(points[i]);
    }
    for (int i = v2; i >= v1; --i) {
        route.push_back(points[i]);
    }
    for (int i = v3; i >= v2 + 1; --i) {
        route.push_back(points[i]);
    }
    for (int i = v3 + 1; i < points.size(); ++i) {
        route.push_back(points[i]);
    }
    return route;
}

vector<Point> RoutePlanner::twooptsearch(const vector<Point>& points){
    // The 2-opt algorithm is a local search algorithm that tries to improve
    // an existing route by swapping two edges. It is very fast and should be
    // used instead of the brute force algorithm. However, it does not always
    // find the optimal solution.
    vector<Point> route = points;
    bool improved = true;
    double best_cost = routecost(route);
    while (improved) {
        improved = false;
        for (int i = 1; i < route.size() - 1; ++i) {
            for (int k = i + 1; k < route.size(); ++k) {
                vector<Point> new_route = twooptswap(route, i, k);
                double new_cost = routecost(new_route);
                if (new_cost < best_cost) {
                    best_cost = new_cost;
                    route = new_route;
                    improved = true;
                }
            }
        }
    }
    return route;
}

vector<Point> RoutePlanner::bruteforce(const vector<Point>& points){
    // The brute force algorithm is a naive algorithm that tries all possible
    // routes and returns the one with the lowest cost. It is very slow and
    // should only be used for small numbers of points. The time complexity is
    // O(n!). For 10 points, there are 1814400 possible routes. For 20 points,
    // there are 2432902008176640000 possible routes. It will always find the 
    // optimal solution.
    vector<Point> route;
    route.reserve(points.size());
    vector<int> vertex;
    for (int i = 0; i < points.size(); i++)
        vertex.push_back(i);
    double min_cost = numeric_limits<double>::max();
    do {
        double cost = 0;
        for (int i = 0; i < vertex.size() - 1; ++i) {
            const Point& point1 = points[vertex[i]];
            const Point& point2 = points[vertex[i + 1]];
            cost += sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
        }
        stringstream ss;
        for (auto& i : vertex) {
            ss << i << " ";
        }
        if (cost < min_cost) {
            min_cost = cost;
            route.clear();
            route.push_back(points[0]);
            for (auto& index : vertex) {
                route.push_back(points[index]);
            }
        }
    } while (next_permutation(vertex.begin(), vertex.end()));
    return route;
}

vector<Point> RoutePlanner::threeoptsearch(const vector<Point>& points){
    // The 3-opt algorithm is a local search algorithm that tries to improve
    // an existing route by swapping three edges. It is an improvement over
    // the 2-opt algorithm. The algorithm can be computationally expensive compared
    // the 2-opt algorithm, but it is still much faster than the brute force and 
    // can give better solutions than the 2-opt algorithm. It will not always find
    // the optimal solution.
    bool improved = true;
    vector<Point> route = points;
    double best_cost = routecost(route);
    while (improved) {
        improved = false;
        for (int i = 1; i < route.size() - 2; ++i) {
            for (int k = i + 1; k < route.size() - 1; ++k) {
                for (int m = k + 1; m < route.size(); ++m) {
                    vector<Point> new_route = threeoptswap(route, i, k, m);
                    double new_cost = routecost(new_route);
                    if (new_cost < best_cost) {
                        best_cost = new_cost;
                        route = new_route;
                        improved = true;
                    }
                }
            }
        }
    }
    return route;
}

vector<Point> RoutePlanner::nearestneighbor(const vector<Point>& points){
    // The nearest neighbor algorithm is a greedy algorithm that starts at a
    // random point and then goes to the nearest unvisited point. It is fast
    // and should be used for large numbers of points. However, it does not
    // always find the optimal solution.
    vector<Point> route;
    route.reserve(points.size());
    vector<int> vertex;
    for (int i = 0; i < points.size(); i++)
        if (i != 0) vertex.push_back(i);
    route.push_back(points[0]);
    while (!vertex.empty()) {
        double min_dist = numeric_limits<double>::max();
        int min_index = -1;
        for (int i = 0; i < vertex.size(); ++i) {
            const Point& point1 = route.back();
            const Point& point2 = points[vertex[i]];
            double dist = sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
            if (dist < min_dist) {
                min_dist = dist;
                min_index = i;
            }
        }
        route.push_back(points[vertex[min_index]]);
        vertex.erase(vertex.begin() + min_index);
    }
    return route;
}
