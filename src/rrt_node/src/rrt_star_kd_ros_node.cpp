#include <chrono>
#include <filesystem>
#include <fstream>
#include <regex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

class RrtStarKdNode : public rclcpp::Node
{
public:
  RrtStarKdNode() : Node("rrt_star_kd_node")
  {
    map_yaml_ = declare_parameter<std::string>("map_yaml", "");
    planner_executable_ = declare_parameter<std::string>(
      "planner_executable",
      "/home/john-rizkallah/thesis_ros2_public/src/rrt_node/standalone_build/build/rrt_star_kd_planner");
    output_directory_ = declare_parameter<std::string>(
      "output_directory",
      "/home/john-rizkallah/thesis_ros2_public/output/rrt_star_kd");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    start_option_ = declare_parameter<int>("start_option", 0);
    goal_option_ = declare_parameter<int>("goal_option", 2);

    if (map_yaml_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'map_yaml' is required.");
      return;
    }

    path_pub_ = create_publisher<nav_msgs::msg::Path>("rrt_star_kd/path", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("rrt_star_kd/markers", 10);
    plan_service_ = create_service<std_srvs::srv::Trigger>(
      "rrt_star_kd/plan",
      std::bind(&RrtStarKdNode::planCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "RRT*KD ROS 2 node ready. Call /rrt_star_kd/plan.");
  }

private:
  void planCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    fs::create_directories(output_directory_);
    const std::string path_file = output_directory_ + "/rrt_path_ros2.txt";

    const std::string command =
      planner_executable_ +
      " --map " + map_yaml_ +
      " --start-option " + std::to_string(start_option_) +
      " --goal-option " + std::to_string(goal_option_) +
      " --output " + path_file;

    RCLCPP_INFO(get_logger(), "Running planner: %s", command.c_str());

    const int return_code = std::system(command.c_str());
    if (return_code != 0) {
      response->success = false;
      response->message = "Planner failed with return code " + std::to_string(return_code);
      return;
    }

    std::vector<std::pair<double, double>> points;
    if (!readPathFile(path_file, points)) {
      response->success = false;
      response->message = "Could not read planner path: " + path_file;
      return;
    }

    if (points.size() < 2) {
      response->success = false;
      response->message = "Planner returned fewer than two path points";
      return;
    }

    publishPath(points);
    publishStartGoalMarkers(points.front(), points.back());

    response->success = true;
    response->message = "Published RRT*KD path with " +
                        std::to_string(points.size()) + " poses";
  }

  bool readPathFile(
    const std::string & file_path,
    std::vector<std::pair<double, double>> & points)
  {
    std::ifstream file(file_path);
    if (!file.is_open()) {
      return false;
    }

    const std::regex number_pattern(R"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)");
    std::string line;

    while (std::getline(file, line)) {
      std::sregex_iterator it(line.begin(), line.end(), number_pattern);
      std::sregex_iterator end;

      std::vector<double> values;
      while (it != end && values.size() < 4) {
        values.push_back(std::stod(it->str()));
        ++it;
      }

      if (values.size() == 4) {
        points.emplace_back(values[0], values[1]);
        points.emplace_back(values[2], values[3]);
      }
    }

    // Remove consecutive duplicate points created by segment endpoints.
    std::vector<std::pair<double, double>> unique_points;
    for (const auto & point : points) {
      if (unique_points.empty() ||
          std::abs(unique_points.back().first - point.first) > 1e-9 ||
          std::abs(unique_points.back().second - point.second) > 1e-9) {
        unique_points.push_back(point);
      }
    }

    points = unique_points;
    return !points.empty();
  }

  void publishPath(
    const std::vector<std::pair<double, double>> & points)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = map_frame_;

    for (const auto & point : points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = path.header.stamp;
      pose.header.frame_id = map_frame_;
      pose.pose.position.x = point.first;
      pose.pose.position.y = point.second;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }

    path_pub_->publish(path);
    RCLCPP_INFO(get_logger(), "Published path with %zu poses", path.poses.size());
  }

    void publishStartGoalMarkers(
    const std::pair<double, double> & start,
    const std::pair<double, double> & goal)
  {
    visualization_msgs::msg::Marker start_marker;
    start_marker.header.stamp = now();
    start_marker.header.frame_id = map_frame_;
    start_marker.ns = "rrt_star_kd_start";
    start_marker.id = 0;
    start_marker.type = visualization_msgs::msg::Marker::SPHERE;
    start_marker.action = visualization_msgs::msg::Marker::ADD;
    start_marker.pose.position.x = start.first;
    start_marker.pose.position.y = start.second;
    start_marker.pose.position.z = 0.0;
    start_marker.pose.orientation.w = 1.0;
    start_marker.scale.x = 1.0;
    start_marker.scale.y = 1.0;
    start_marker.scale.z = 1.0;
    start_marker.color.r = 0.0;
    start_marker.color.g = 1.0;
    start_marker.color.b = 0.0;
    start_marker.color.a = 1.0;

    visualization_msgs::msg::Marker goal_marker = start_marker;
    goal_marker.ns = "rrt_star_kd_goal";
    goal_marker.id = 1;
    goal_marker.pose.position.x = goal.first;
    goal_marker.pose.position.y = goal.second;
    goal_marker.color.r = 1.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 0.0;

    marker_pub_->publish(start_marker);
    marker_pub_->publish(goal_marker);
  }

  std::string map_yaml_;
  std::string planner_executable_;
  std::string output_directory_;
  std::string map_frame_;
  int start_option_;
  int goal_option_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RrtStarKdNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
