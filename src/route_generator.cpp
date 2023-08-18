#include"traveling_salesman/route_generator.h"

RouteGenerator::RouteGenerator(ros::NodeHandle *nh){
    generate_points_srv = nh->advertiseService("generate_points", &RouteGenerator::generate_pts_callback, this);
    points_pub = nh->advertise<visualization_msgs::MarkerArray>("points", 10);
    ids_pub = nh->advertise<visualization_msgs::MarkerArray>("points_ids", 10);
}

bool RouteGenerator::generate_pts_callback(traveling_salesman::GeneratePoints::Request &req, traveling_salesman::GeneratePoints::Response &res) {
  if(req.number_of_points < 3){
    res.success = false;
    res.message = "Number of points must be greater than 3";
    return true; 
  }
  res.success = true;
  res.message = "Points generated";
  vector<Point> points = generate_random_points(-1, 1, -1, 1, req.number_of_points);
  publish_points(points);
  publish_ids(points);
  return true;
}

void RouteGenerator::publish_points(const vector<Point>& points) {
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
      const Point& point = points[i];
      visualization_msgs::Marker& marker = marker_array.markers[i];
      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time::now();
      marker.ns = "random_points";
      marker.id = i;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = point.x;
      marker.pose.position.y = point.y;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
  }
  points_pub.publish(marker_array);
}

void RouteGenerator::publish_ids(const vector<Point>& points) {
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
      const Point& point = points[i];
      visualization_msgs::Marker& marker = marker_array.markers[i];
      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time::now();
      marker.ns = "ids";
      marker.id = i;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = point.x;
      marker.pose.position.y = point.y;
      marker.pose.orientation.w = 1.0;
      marker.scale.z = 0.3;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker.color.a = 1.0;
      stringstream ss;
      ss << i;
      string text = ss.str();
      marker.text = text;
  }
  ids_pub.publish(marker_array);
}

vector<Point> RouteGenerator::generate_random_points(double min_x, double max_x, double min_y, double max_y, int num_points) {
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<double> distX(min_x, max_x);
    uniform_real_distribution<double> distY(min_y, max_y);
    vector<Point> points;
    points.reserve(num_points);
    for (int i = 0; i < num_points; ++i) {
        Point point;
        point.x = distX(gen);
        point.y = distY(gen);
        points.push_back(point);
    }
    return points;
}