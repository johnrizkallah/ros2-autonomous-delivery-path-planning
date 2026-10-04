#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include <json/json.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

class RrtStarKdNode : public rclcpp::Node
{
public:
  RrtStarKdNode()
  : Node("rrt_star_kd_node")
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
    goal_option_ = declare_parameter<int>("goal_option", 3);

    animate_tree_ = declare_parameter<bool>("animate_tree", true);
    edges_per_batch_ = declare_parameter<int>("edges_per_batch", 250);
    animation_period_ms_ = declare_parameter<int>("animation_period_ms", 30);

    if (map_yaml_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'map_yaml' is required.");
      return;
    }

    path_pub_ = create_publisher<nav_msgs::msg::Path>("rrt_star_kd/path", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "rrt_star_kd/markers", 10);

    plan_service_ = create_service<std_srvs::srv::Trigger>(
      "rrt_star_kd/plan",
      std::bind(
        &RrtStarKdNode::planCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "RRT*KD ROS 2 node ready. Call /rrt_star_kd/plan to plan and animate.");
  }

private:
  using Point2D = std::pair<double, double>;
  using Edge = std::pair<Point2D, Point2D>;

  void planCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (animation_timer_) {
      animation_timer_->cancel();
      animation_timer_.reset();
    }

    fs::create_directories(output_directory_);

    const std::string path_file = output_directory_ + "/rrt_path_ros2.txt";
    const std::string json_file = output_directory_ + "/rrt_star_kd_data.json";

    std::error_code ec;
    fs::remove(path_file, ec);
    fs::remove(json_file, ec);

    const std::string command =
      "cd \"" + output_directory_ + "\" && " +
      "\"" + planner_executable_ + "\"" +
      " --map \"" + map_yaml_ + "\"" +
      " --start-option " + std::to_string(start_option_) +
      " --goal-option " + std::to_string(goal_option_) +
      " --output \"" + path_file + "\"";

    RCLCPP_INFO(get_logger(), "Running planner: %s", command.c_str());

    const int return_code = std::system(command.c_str());
    if (return_code != 0) {
      response->success = false;
      response->message = "Planner failed with return code " + std::to_string(return_code);
      return;
    }

    if (!readPathFile(path_file, final_path_points_)) {
      response->success = false;
      response->message = "Could not read planner path: " + path_file;
      return;
    }

    if (final_path_points_.size() < 2) {
      response->success = false;
      response->message = "Planner returned fewer than two path points";
      return;
    }

    if (!readTreeJson(json_file, tree_edges_)) {
      response->success = false;
      response->message = "Could not read tree JSON: " + json_file;
      return;
    }

    publishDeleteMarker();
    publishStartGoalMarkers(final_path_points_.front(), final_path_points_.back());

    if (animate_tree_ && !tree_edges_.empty()) {
      animation_index_ = 0;
      animation_timer_ = create_wall_timer(
        std::chrono::milliseconds(std::max(1, animation_period_ms_)),
        std::bind(&RrtStarKdNode::publishNextTreeBatch, this));

      response->success = true;
      response->message =
        "Planning complete: animating " + std::to_string(tree_edges_.size()) +
        " RRT*KD tree edges before publishing the final path";
    } else {
      publishTree(tree_edges_.size());
      publishFinalPath();
      response->success = true;
      response->message =
        "Published RRT*KD tree and path with " +
        std::to_string(final_path_points_.size()) + " poses";
    }
  }

  bool readPathFile(const std::string & file_path, std::vector<Point2D> & points)
  {
    std::ifstream file(file_path);
    if (!file.is_open()) {
      return false;
    }

    const std::regex number_pattern(R"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)");
    std::string line;
    points.clear();

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

    std::vector<Point2D> unique_points;
    for (const auto & point : points) {
      if (
        unique_points.empty() ||
        std::abs(unique_points.back().first - point.first) > 1e-9 ||
        std::abs(unique_points.back().second - point.second) > 1e-9)
      {
        unique_points.push_back(point);
      }
    }

    points = unique_points;
    return !points.empty();
  }

  bool readTreeJson(const std::string & file_path, std::vector<Edge> & edges)
  {
    std::ifstream file(file_path);
    if (!file.is_open()) {
      return false;
    }

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errors;

    if (!Json::parseFromStream(builder, file, &root, &errors)) {
      RCLCPP_ERROR(get_logger(), "JSON parse error: %s", errors.c_str());
      return false;
    }

    if (!root.isMember("tree") || !root.isMember("parents")) {
      RCLCPP_ERROR(get_logger(), "Tree JSON does not contain 'tree' and 'parents'.");
      return false;
    }

    const auto & tree = root["tree"];
    const auto & parents = root["parents"];

    if (!tree.isArray() || !parents.isArray() || tree.size() != parents.size()) {
      RCLCPP_ERROR(get_logger(), "Invalid tree/parents JSON dimensions.");
      return false;
    }

    edges.clear();
    for (Json::ArrayIndex i = 0; i < tree.size(); ++i) {
      const int parent_index = parents[i].asInt();

      if (parent_index < 0 || parent_index >= static_cast<int>(tree.size())) {
        continue;
      }

      if (
        !tree[i].isArray() || tree[i].size() < 2 ||
        !tree[parent_index].isArray() || tree[parent_index].size() < 2)
      {
        continue;
      }

      const Point2D parent{
        tree[parent_index][0].asDouble(),
        tree[parent_index][1].asDouble()
      };
      const Point2D child{
        tree[i][0].asDouble(),
        tree[i][1].asDouble()
      };

      edges.emplace_back(parent, child);
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu RRT*KD tree edges", edges.size());
    return !edges.empty();
  }

  void publishNextTreeBatch()
  {
    const std::size_t batch = static_cast<std::size_t>(std::max(1, edges_per_batch_));
    animation_index_ = std::min(animation_index_ + batch, tree_edges_.size());

    publishTree(animation_index_);

    if (animation_index_ >= tree_edges_.size()) {
      animation_timer_->cancel();
      animation_timer_.reset();
      publishFinalPath();

      RCLCPP_INFO(
        get_logger(),
        "Tree animation complete; published final path with %zu poses",
        final_path_points_.size());
    }
  }

  void publishTree(std::size_t edge_count)
  {
    visualization_msgs::msg::Marker tree_marker;
    tree_marker.header.stamp = now();
    tree_marker.header.frame_id = map_frame_;
    tree_marker.ns = "rrt_star_kd_tree";
    tree_marker.id = 0;
    tree_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    tree_marker.action = visualization_msgs::msg::Marker::ADD;
    tree_marker.pose.orientation.w = 1.0;
    tree_marker.scale.x = 0.06;
    tree_marker.color.r = 0.05;
    tree_marker.color.g = 0.25;
    tree_marker.color.b = 0.95;
    tree_marker.color.a = 0.75;

    tree_marker.points.reserve(edge_count * 2);

    for (std::size_t i = 0; i < edge_count; ++i) {
      geometry_msgs::msg::Point parent;
      parent.x = tree_edges_[i].first.first;
      parent.y = tree_edges_[i].first.second;
      parent.z = 0.03;

      geometry_msgs::msg::Point child;
      child.x = tree_edges_[i].second.first;
      child.y = tree_edges_[i].second.second;
      child.z = 0.03;

      tree_marker.points.push_back(parent);
      tree_marker.points.push_back(child);
    }

    marker_pub_->publish(tree_marker);
  }

  void publishFinalPath()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = map_frame_;

    for (const auto & point : final_path_points_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.first;
      pose.pose.position.y = point.second;
      pose.pose.position.z = 0.10;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }

    path_pub_->publish(path);
    publishFinalPathMarker();
  }

  void publishFinalPathMarker()
  {
    visualization_msgs::msg::Marker path_marker;
    path_marker.header.stamp = now();
    path_marker.header.frame_id = map_frame_;
    path_marker.ns = "rrt_star_kd_final_path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.40;
    path_marker.color.r = 0.0;
    path_marker.color.g = 1.0;
    path_marker.color.b = 0.10;
    path_marker.color.a = 1.0;

    for (const auto & point : final_path_points_) {
      geometry_msgs::msg::Point p;
      p.x = point.first;
      p.y = point.second;
      p.z = 0.12;
      path_marker.points.push_back(p);
    }

    marker_pub_->publish(path_marker);
  }

  void publishStartGoalMarkers(const Point2D & start, const Point2D & goal)
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
    start_marker.pose.position.z = 0.20;
    start_marker.pose.orientation.w = 1.0;
    start_marker.scale.x = 2.0;
    start_marker.scale.y = 2.0;
    start_marker.scale.z = 2.0;
    start_marker.color.r = 0.0;
    start_marker.color.g = 1.0;
    start_marker.color.b = 0.0;
    start_marker.color.a = 1.0;

    visualization_msgs::msg::Marker goal_marker = start_marker;
    goal_marker.ns = "rrt_star_kd_goal";
    goal_marker.id = 0;
    goal_marker.pose.position.x = goal.first;
    goal_marker.pose.position.y = goal.second;
    goal_marker.color.r = 1.0;
    goal_marker.color.g = 0.10;
    goal_marker.color.b = 0.0;

    marker_pub_->publish(start_marker);
    marker_pub_->publish(goal_marker);
  }

  void publishDeleteMarker()
  {
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.stamp = now();
    delete_marker.header.frame_id = map_frame_;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_pub_->publish(delete_marker);
  }

  std::string map_yaml_;
  std::string planner_executable_;
  std::string output_directory_;
  std::string map_frame_;

  int start_option_;
  int goal_option_;
  bool animate_tree_;
  int edges_per_batch_;
  int animation_period_ms_;

  std::vector<Point2D> final_path_points_;
  std::vector<Edge> tree_edges_;
  std::size_t animation_index_{0};

  rclcpp::TimerBase::SharedPtr animation_timer_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RrtStarKdNode>());
  rclcpp::shutdown();
  return 0;
}
