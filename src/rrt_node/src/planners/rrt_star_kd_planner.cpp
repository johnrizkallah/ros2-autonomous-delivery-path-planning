// IMPORTANT: Disable FLANN serialization before including FLANN headers.
#define FLANN_NO_SERIALIZATION

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <tuple>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <random>
#include <algorithm>
#include <json/json.h>  // For JSON output if available

#include <yaml-cpp/yaml.h>

// Include FLANN from its own installation.
#include <flann/flann.hpp>

// Include OpenCV headers (which may include cv::flann).
#include <opencv2/opencv.hpp>
#include <opencv2/flann/miniflann.hpp>


namespace flann_lib = ::flann;
using namespace std;
using namespace cv;
using Clock = chrono::steady_clock;

// Conversion helper functions
//---------------------------------------------------------------------

// Convert cv::Mat to a 2D vector of ints.
std::vector<std::vector<int>> convertMatToVector(const cv::Mat &mat) {
    std::vector<std::vector<int>> vec(mat.rows, std::vector<int>(mat.cols));
    for (int y = 0; y < mat.rows; y++) {
        for (int x = 0; x < mat.cols; x++) {
            vec[y][x] = static_cast<int>(mat.at<uchar>(y, x));
        }
    }
    return vec;
}

// Convert cv::Point2d to std::pair<double, double>
std::pair<double, double> convertPoint2d(const cv::Point2d &pt) {
    return {pt.x, pt.y};
}

// Convert a vector of cv::Point2d to vector of std::pair<double, double>
std::vector<std::pair<double, double>> convertPoints2d(const std::vector<cv::Point2d> &pts) {
    std::vector<std::pair<double, double>> pairs;
    for (const auto &pt : pts) {
        pairs.push_back(convertPoint2d(pt));
    }
    return pairs;
}

// Convert a path segment from cv::Point2d pair to std::pair of pairs.
std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> 
convertPath(const std::vector<std::pair<cv::Point2d, cv::Point2d>> &path) {
    std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> converted;
    for (const auto &seg : path) {
        auto startPair = convertPoint2d(seg.first);
        auto endPair = convertPoint2d(seg.second);
        converted.push_back({startPair, endPair});
    }
    return converted;
}

// Convert cv::Point (used in footprints) to std::pair<double, double>
std::pair<double, double> convertPoint(const cv::Point &pt) {
    return {static_cast<double>(pt.x), static_cast<double>(pt.y)};
}

// Convert footprints (vector of vector<cv::Point>) to vector of vector of std::pair<double, double>
std::vector<std::vector<std::pair<double, double>>> 
convertFootprints(const std::vector<std::vector<cv::Point>> &fps) {
    std::vector<std::vector<std::pair<double, double>>> conv;
    for (const auto &fp : fps) {
        std::vector<std::pair<double, double>> temp;
        for (const auto &pt : fp) {
            temp.push_back(convertPoint(pt));
        }
        conv.push_back(temp);
    }
    return conv;
}

//---------------------------------------------------------------------
// JSON saving function (unchanged from your original, except that it now
// expects the converted data types)
//---------------------------------------------------------------------
void saveData(
    std::vector<std::vector<int>> &BW,  // Binary map
    std::vector<std::pair<double, double>> &tree,  // RRT tree
    std::pair<double, double> &start_pos,  // Start position
    std::vector<std::pair<double, double>> &goal_pos,  // Goal positions
    std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> &path,  // Path (edges)
    std::vector<std::vector<std::pair<double, double>>> &footprints,  // Footprints (optional)
    std::vector<double> &map_origin,  // Map origin (x, y)
    double map_resolution  // Resolution
) {
    Json::Value root;

    // Convert the map (BW) to JSON
    Json::Value mapJson;
    for (const auto &row : BW) {
        Json::Value jsonRow;
        for (int cell : row) {
            jsonRow.append(cell);
        }
        mapJson.append(jsonRow);
    }
    root["map"] = mapJson;

    // Convert the tree nodes
    Json::Value treeJson;
    for (const auto &node : tree) {
        Json::Value jsonNode;
        jsonNode.append(node.first);   // x-coordinate
        jsonNode.append(node.second);  // y-coordinate
        treeJson.append(jsonNode);
    }
    root["tree"] = treeJson;

    // Start position
    root["start"].append(start_pos.first);
    root["start"].append(start_pos.second);

    // Goal positions
    Json::Value goalJson;
    for (const auto &goal : goal_pos) {
        Json::Value jsonGoal;
        jsonGoal.append(goal.first);
        jsonGoal.append(goal.second);
        goalJson.append(jsonGoal);
    }
    root["goal"] = goalJson;

    // Path (edges between nodes)
    Json::Value pathJson;
    for (const auto &edge : path) {
        Json::Value jsonEdge;
        Json::Value start, end;
        start.append(edge.first.first);
        start.append(edge.first.second);
        end.append(edge.second.first);
        end.append(edge.second.second);
        jsonEdge["start"] = start;
        jsonEdge["end"] = end;
        pathJson.append(jsonEdge);
    }
    root["path"] = pathJson;

    // Footprints
    Json::Value footprintsJson;
    for (const auto &footprint : footprints) {
        Json::Value footprintJson;
        for (const auto &point : footprint) {
            Json::Value jsonPoint;
            jsonPoint.append(point.first);
            jsonPoint.append(point.second);
            footprintJson.append(jsonPoint);
        }
        footprintsJson.append(footprintJson);
    }
    root["footprints"] = footprintsJson;

    // Map origin
    root["origin"].append(map_origin[0]);
    root["origin"].append(map_origin[1]);

    // Map resolution
    root["resolution"] = map_resolution;

    // Write to file
    std::ofstream file("rrt_star_kd_data.json");
    if (!file) {
        std::cerr << "Error opening file for writing\n";
        return;
    }
    file << root;
    file.close();

    std::cout << "Data saved to rrt_star_kd_data.json\n";
}




// -----------------------------------------------------------------------------
// Dummy plotting function (disregard plotting)
void plot_map(const Mat &BW, int vehicle_size,
              const vector<Point2d> &tree_world,
              const Point2d &start_pos,
              const vector<Point2d> &goal_world_positions,
              const vector<pair<Point2d, Point2d>> &path,
              const vector< vector<Point> > &footprints,
              const vector<double> &origin,
              double resolution) {
    // Plotting function ignored.
}

// -----------------------------------------------------------------------------
// is_within_bounds
bool is_within_bounds(const Point &point, const Mat &BW) {
    return (point.x >= 0 && point.x < BW.cols && point.y >= 0 && point.y < BW.rows);
}

// -----------------------------------------------------------------------------
// is_valid_node
bool is_valid_node(const vector<Point> &footprint, const Mat &BW) {
    // Check if all points in the footprint are within the map boundaries.
    for (const auto &pt : footprint) {
        if (pt.x < 0 || pt.x >= BW.cols || pt.y < 0 || pt.y >= BW.rows)
            return false;
    }
    // Check occupancy: Ensure all footprint points are in free space.
    // (In our binary map free cells have value 0; obstacles are 255.)
    // (This mimics the Python code that returns np.all(occupancy != 0).)
    for (const auto &pt : footprint) {
        if (BW.at<uchar>(pt.y, pt.x) == 0)
            return false;
    }
    return true;
}

// -----------------------------------------------------------------------------
// load_map_yaml
tuple<string, double, vector<double>> load_map_yaml(const string &yaml_file) {
    YAML::Node map_data = YAML::LoadFile(yaml_file);
    string map_image_file = map_data["image"].as<string>();
    double map_resolution = map_data["resolution"].as<double>();
    vector<double> map_origin = map_data["origin"].as<vector<double>>();
    return make_tuple(map_image_file, map_resolution, map_origin);
}

// -----------------------------------------------------------------------------
// load_map_image
Mat load_map_image(const string &map_image_file) {
    Mat map_image = imread(map_image_file, IMREAD_UNCHANGED);
    if (map_image.empty()) {
        cout << "Error loading map image" << endl;
    }
    return map_image;
}

// -----------------------------------------------------------------------------
// convert_map_to_binary
Mat convert_map_to_binary(const Mat &map_image) {
    // Initialize the binary map with zeros (obstacles)
    Mat BW = Mat::zeros(map_image.size(), CV_8UC1);
    // Convert the specific values to binary
    for (int y = 0; y < map_image.rows; y++) {
        for (int x = 0; x < map_image.cols; x++) {
            uchar pixel = map_image.at<uchar>(y, x);
            if (pixel == 205)
                BW.at<uchar>(y, x) = 0;   // Free cells
            else if (pixel == 254)
                BW.at<uchar>(y, x) = 255; // Occupied cells (obstacles)
            else
                BW.at<uchar>(y, x) = 255; // Unknown cells treated as obstacles
        }
    }
    return BW;
}

// -----------------------------------------------------------------------------
// map_to_world
Point2d map_to_world(const Point &map_point, double resolution, const vector<double> &origin, int map_height) {
    double world_x = origin[0] + map_point.x * resolution;
    double world_y = origin[1] + (map_height - map_point.y) * resolution;
    return Point2d(world_x, world_y);
}

// -----------------------------------------------------------------------------
// world_to_map
Point world_to_map(const Point2d &world_point, double resolution, const vector<double> &origin, int map_height) {
    int map_x = static_cast<int>((world_point.x - origin[0]) / resolution);
    int map_y = static_cast<int>(map_height - (world_point.y - origin[1]) / resolution);
    return Point(map_x, map_y);
}

// -----------------------------------------------------------------------------
// euclidean_distance (for Point)
double euclidean_distance(const Point &p1, const Point &p2) {
    double dx = p1.x - p2.x, dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}

// Overload for Point2d
double euclidean_distance(const Point2d &p1, const Point2d &p2) {
    double dx = p1.x - p2.x, dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}

// -----------------------------------------------------------------------------
// calculate_angle_between_points
double calculate_angle_between_points(const Point &p1, const Point &p2) {
    Point delta = p2 - p1;
    return atan2(delta.y, delta.x);
}

// -----------------------------------------------------------------------------
// create_vehicle_footprint
vector<Point> create_vehicle_footprint(const Point &center, int radius) {
    int num_points = 36;
    vector<Point> perimeter_points;
    // Generate perimeter points
    for (int i = 0; i < num_points; i++) {
        double angle = 2 * M_PI * i / num_points;
        perimeter_points.push_back(Point(center.x + static_cast<int>(radius * cos(angle)),
                                           center.y + static_cast<int>(radius * sin(angle))));
    }
    // Generate interior points
    vector<double> interior_fraction = {0.25, 0.5, 0.75};
    vector<Point> interior_points;
    for (double frac : interior_fraction) {
        for (int i = 0; i < num_points; i++) {
            double angle = 2 * M_PI * i / num_points;
            interior_points.push_back(Point(center.x + static_cast<int>(frac * radius * cos(angle)),
                                              center.y + static_cast<int>(frac * radius * sin(angle))));
        }
    }
    // Combine perimeter and interior points
    vector<Point> footprint_points;
    footprint_points.insert(footprint_points.end(), perimeter_points.begin(), perimeter_points.end());
    footprint_points.insert(footprint_points.end(), interior_points.begin(), interior_points.end());
    return footprint_points;
}

// -----------------------------------------------------------------------------
// get_goal_positions
vector<Point> get_goal_positions(int option) {
    switch (option) {
        case 0: return {Point(792, 1620)};
        case 1: return {Point(680, 1434)};
        case 2: return {Point(2214, 1051)};
        case 3: return {Point(2088, 374)};
        case 4: return {Point(1238, 718)};
        default: return {};
    }
}

// -----------------------------------------------------------------------------
// get_start_position
vector<Point> get_start_position(int option2) {
    switch (option2) {
        case 0: return {Point(792, 1620)};
        case 1: return {Point(680, 1434)};
        case 2: return {Point(2214, 1051)};
        case 3: return {Point(2088, 374)};
        case 4: return {Point(1238, 718)};
        default: return {};
    }
}

// -----------------------------------------------------------------------------
// gaussian_filter1d helper function
vector<double> gaussian_filter1d(const vector<double> &input, double sigma) {
    int n = static_cast<int>(2 * ceil(3 * sigma) + 1);
    if (n % 2 == 0)
        n++; // ensure odd kernel size
    int half = n / 2;
    vector<double> kernel(n);
    double sum = 0.0;
    for (int i = 0; i < n; i++) {
        int x = i - half;
        kernel[i] = exp(-(x * x) / (2 * sigma * sigma));
        sum += kernel[i];
    }
    for (int i = 0; i < n; i++) {
        kernel[i] /= sum;
    }
    int len = input.size();
    vector<double> output(len, 0.0);
    for (int i = 0; i < len; i++) {
        double acc = 0.0;
        for (int j = 0; j < n; j++) {
            int idx = i + j - half;
            if (idx < 0)
                idx = 0;
            if (idx >= len)
                idx = len - 1;
            acc += input[idx] * kernel[j];
        }
        output[i] = acc;
    }
    return output;
}

// -----------------------------------------------------------------------------
// smooth_path
vector< pair<Point, Point> > smooth_path(const vector< pair<Point, Point> > &path, double sigma = 0.01) {
    vector<double> x, y;
    for (const auto &segment : path) {
        x.push_back(segment.first.x);
        y.push_back(segment.first.y);
    }
    // Append the end of the last segment.
    if (!path.empty()) {
        x.push_back(path.back().second.x);
        y.push_back(path.back().second.y);
    }
    vector<double> smooth_x = gaussian_filter1d(x, sigma);
    vector<double> smooth_y = gaussian_filter1d(y, sigma);
    vector< pair<Point, Point> > smoothed_segments;
    for (size_t i = 0; i < smooth_x.size() - 1; i++) {
        Point start(static_cast<int>(round(smooth_x[i])), static_cast<int>(round(smooth_y[i])));
        Point end(static_cast<int>(round(smooth_x[i + 1])), static_cast<int>(round(smooth_y[i + 1])));
        smoothed_segments.push_back(make_pair(start, end));
    }
    return smoothed_segments;
}


// -----------------------------------------------------------------------------
// main
int main(int argc, char *argv[]) {
    // Default configuration reproduces the original validated map8 run.
    string yaml_file = "/home/john-rizkallah/thesis_ros2_public/src/rrt_node/maps/map8.yaml";
    string output_file = "rrt_path_star_kd.txt";
    int start_x = -1, start_y = -1;
    int goal_x = -1, goal_y = -1;
    int start_option = 0;
    int goal_option = 2;

    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        auto next = [&]() -> string {
            if (i + 1 >= argc) {
                cerr << "Missing value for " << arg << endl;
                exit(EXIT_FAILURE);
            }
            return argv[++i];
        };

        if (arg == "--map") yaml_file = next();
        else if (arg == "--output") output_file = next();
        else if (arg == "--start-x") start_x = stoi(next());
        else if (arg == "--start-y") start_y = stoi(next());
        else if (arg == "--goal-x") goal_x = stoi(next());
        else if (arg == "--goal-y") goal_y = stoi(next());
        else if (arg == "--start-option") start_option = stoi(next());
        else if (arg == "--goal-option") goal_option = stoi(next());
        else if (arg == "--help") {
            cout << "Usage: rrt_star_kd_planner [options]\n"
                 << "  --map PATH           Occupancy-grid YAML file\n"
                 << "  --output PATH        Output path file\n"
                 << "  --start-x INT        Start x in map pixels\n"
                 << "  --start-y INT        Start y in map pixels\n"
                 << "  --goal-x INT         Goal x in map pixels\n"
                 << "  --goal-y INT         Goal y in map pixels\n"
                 << "  --start-option INT   Legacy preset start index\n"
                 << "  --goal-option INT    Legacy preset goal index\n";
            return 0;
        } else {
            cerr << "Unknown argument: " << arg << endl;
            return EXIT_FAILURE;
        }
    }

    // 1. Measure time for loading map YAML
    auto start_time = Clock::now();
    string map_image_file;
    double map_resolution;
    vector<double> map_origin;
    tie(map_image_file, map_resolution, map_origin) = load_map_yaml(yaml_file);
    double load_yaml_time = chrono::duration_cast<chrono::duration<double>>(Clock::now() - start_time).count();
    cout << "Time to load map YAML: " << load_yaml_time << " seconds" << endl;

    // 2. Measure time for loading map image
    start_time = Clock::now();
    Mat map_image = load_map_image(map_image_file);
    if (map_image.empty()) {
        cout << "Error loading map image" << endl;
        return -1;
    }
    double load_image_time = chrono::duration_cast<chrono::duration<double>>(Clock::now() - start_time).count();
    cout << "Time to load map image: " << load_image_time << " seconds" << endl;

    // Goal position: use explicit pixel coordinates if provided,
    // otherwise fall back to the original legacy preset.
    vector<Point> goal_position;
    if (goal_x >= 0 && goal_y >= 0) {
        goal_position = {Point(goal_x, goal_y)};
    } else {
        goal_position = get_goal_positions(goal_option);
        if (goal_position.empty()) {
            cout << "No goal positions found for option: " << goal_option << endl;
            return -1;
        }
    }

    // 3. Measure time for converting map to binary
    start_time = Clock::now();
    Mat BW = convert_map_to_binary(map_image);
    double convert_map_time = chrono::duration_cast<chrono::duration<double>>(Clock::now() - start_time).count();
    cout << "Time to convert map to binary: " << convert_map_time << " seconds" << endl;

    // Get free cells from BW (free cells are those with value 255)
    vector<Point> BW_free;
    for (int y = 0; y < BW.rows; y++) {
        for (int x = 0; x < BW.cols; x++) {
            if (BW.at<uchar>(y, x) == 255)
                BW_free.push_back(Point(x, y));
        }
    }

    // Vehicle parameters
    int radius = 25;      // Vehicle radius in map units
    int radius_goal = 13;

    Point start_pos;
    if (start_x >= 0 && start_y >= 0) {
        start_pos = Point(start_x, start_y);
    } else {
        vector<Point> start_pos_vec = get_start_position(start_option);
        if (start_pos_vec.empty()) {
            cout << "No start position found for option: " << start_option << endl;
            return -1;
        }
        start_pos = start_pos_vec[0];
    }

    Point2d start_world = map_to_world(start_pos, map_resolution, map_origin, BW.rows);
    vector<Point2d> goal_world_positions;
    for (const auto &goal : goal_position) {
        goal_world_positions.push_back(map_to_world(goal, map_resolution, map_origin, BW.rows));
    }

    // ===== RRT* DATA STRUCTURES =====
    vector<Point> tree = {start_pos};              // list of map-coordinate nodes
    vector<Point2d> tree_world = {start_world};      // corresponding world coordinates for plotting
    vector< vector<Point> > footprints = {create_vehicle_footprint(start_pos, radius)};
    vector<double> costs = {0.0};                    // cost-to-come for each node; start has cost 0
    vector<int> parents = {-1};                      // parent index for each node (-1 for the root)
    // ---------------------------------

    int max_iterations = 300000;
    // Reserve enough space if possible (here we assume max_iterations points)
    std::vector<double> dataset; 
    dataset.reserve(max_iterations * 2); 

    // Add the start node to the dataset (each point has 2 coordinates)
    dataset.push_back(static_cast<double>(start_pos.x));
    dataset.push_back(static_cast<double>(start_pos.y));

    // Create a FLANN matrix wrapping our dataset.
    // Note: The matrix dimensions are: (number of points) x (2)
    flann_lib::Matrix<double> dataset_mat(dataset.data(), tree.size(), 2);

    // Create the FLANN index using L2 (Euclidean) distance and KDTree index parameters.
    flann_lib::Index<flann_lib::L2<double>> index(dataset_mat, flann_lib::KDTreeIndexParams(4));
    index.buildIndex();

    int iteration = 0;

    bool goal_reached = false;
    int step = 2;
    double rewire_radius = 40.0;  // Radius within which to consider rewiring neighbors

    auto total_rrt_start = Clock::now();
    int goal_index = -1;  // Will hold the index of the node that reached the goal
    double update_tree_time = 0.0;
    double footprint_time = 0.0;
    double find_nearest_time = 0.0;
    double rewire_time_total = 0.0;

    int new_node_index = 0;
    int update_threshold = 5;
    std::vector<std::array<double, 2>> new_points;
    // Random number generation for free cell sampling.
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dis(0, BW_free.size() - 1);

    while (!goal_reached && iteration < max_iterations) {
        iteration++;
        auto iteration_start = Clock::now();

        Point nearest_node;
        size_t nearest_idx;

        // Random Sampling (sample only free cells)
        auto start_sampling = Clock::now();
        Point random_point = BW_free[dis(gen)];
        double random_sampling_time = chrono::duration_cast<chrono::duration<double>>(Clock::now() - start_sampling).count();

        // Find the nearest node using knnSearch (k=1)
        auto start_nearest = Clock::now();

            std::vector<double> queryNN(2);
        queryNN[0] = static_cast<double>(random_point.x);
        queryNN[1] = static_cast<double>(random_point.y);
        flann_lib::Matrix<double> queryNN_mat(queryNN.data(), 1, 2);

        std::vector<int> nn_indices(1);
        std::vector<double> nn_dists(1);
        flann_lib::Matrix<int> indices_mat(nn_indices.data(), 1, 1);
        flann_lib::Matrix<double> dists_mat(nn_dists.data(), 1, 1);

        index.knnSearch(queryNN_mat, indices_mat, dists_mat, 1, flann_lib::SearchParams(128));
        nearest_idx = nn_indices[0];
        nearest_node = tree[nearest_idx];

        find_nearest_time = chrono::duration_cast<chrono::duration<double>>(Clock::now() - start_nearest).count();

        // Steer from nearest node towards random point by step size
        double dx = random_point.x - nearest_node.x;
        double dy = random_point.y - nearest_node.y;
        double norm_val = sqrt(dx * dx + dy * dy);
        
        if (norm_val == 0)
            continue;
        double dir_x = dx / norm_val;
        double dir_y = dy / norm_val;
        Point new_node;
        new_node.x = nearest_node.x + static_cast<int>(round(step * dir_x));
        new_node.y = nearest_node.y + static_cast<int>(round(step * dir_y));

        // Clip new_node to map bounds
        new_node.x = min(max(new_node.x, 0), BW.cols - 1);
        new_node.y = min(max(new_node.y, 0), BW.rows - 1);
        
        // Create footprint for the new node and check validity
        auto start_fp = Clock::now();
        vector<Point> new_footprint = create_vehicle_footprint(new_node, radius);
        if (!is_valid_node(new_footprint, BW))
            continue;
        footprint_time = chrono::duration_cast<chrono::duration<double>>(Clock::now() - start_fp).count();

        // ----- Radius Search using FLANN for rewiring -----
        std::vector<double> queryRadius(2);
        queryRadius[0] = static_cast<double>(new_node.x);
        queryRadius[1] = static_cast<double>(new_node.y);
        flann_lib::Matrix<double> queryRadius_mat(queryRadius.data(), 1, 2);

        // FLANN’s radiusSearch returns results in a vector of vectors.
        std::vector< std::vector<int> > radius_indices;
        std::vector< std::vector<double> > radius_dists;
        // The search radius should be provided as squared distance.
        double search_radius = rewire_radius * rewire_radius;
        index.radiusSearch(queryRadius_mat, radius_indices, radius_dists, search_radius, flann_lib::SearchParams());

        // Extract neighbor indices from the first (and only) query.
        vector<int> neighbor_indices;
        if (!radius_indices.empty()) {
            neighbor_indices = radius_indices[0];
        }

        
        // Choose the best parent for new_node among the neighbors (including nearest_node)
        double min_cost = costs[nearest_idx] + euclidean_distance(nearest_node, new_node);
        int best_parent = nearest_idx;
        for (int idx : neighbor_indices) {
            // Skip the new node itself if it is in the neighbor list.
            if (idx == tree.size() - 1)
                continue;
            double cost_through_neighbor = costs[idx] + euclidean_distance(tree[idx], new_node);
            if (cost_through_neighbor < min_cost) {
                best_parent = idx;
                min_cost = cost_through_neighbor;
            }
        }

        // Add new_node to the tree with the chosen parent
        tree.push_back(new_node);
        tree_world.push_back(map_to_world(new_node, map_resolution, map_origin, BW.rows));
        footprints.push_back(new_footprint);
        costs.push_back(min_cost);
        parents.push_back(best_parent);
    
        
        // ----- Update the FLANN Dataset -----  
        // Append the new node's coordinates to the dataset vector.
        dataset.push_back(static_cast<double>(new_node.x));
        dataset.push_back(static_cast<double>(new_node.y));

        // Create a temporary matrix wrapping the new node's coordinates.
        flann_lib::Matrix<double> new_point_mat(dataset.data() + (tree.size()-1)*2, 1, 2);
        index.addPoints(new_point_mat, 3.0);

        

        //cout << "Node " << tree.size() - 1 << ": (" << new_node.x << ", " << new_node.y << "), Parent: ";
        /*if (parents.back() != -1)
            cout << "(" << tree[parents.back()].x << ", " << tree[parents.back()].y << ")" << endl;
        else
            cout << "None" << endl;*/

        update_tree_time = chrono::duration_cast<chrono::duration<double>>(Clock::now() - iteration_start).count() -
                           (random_sampling_time + find_nearest_time + footprint_time);

        // RRT*: Rewire the tree (try to improve the cost of nearby nodes)
        auto start_rewire = Clock::now();

        new_node_index = tree.size() - 1;
        for (int idx : neighbor_indices) {
            if (idx == new_node_index)
                continue;
            double potential_cost = costs[new_node_index] + euclidean_distance(new_node, tree[idx]);
            if (potential_cost < costs[idx]) {
                costs[idx] = potential_cost;
                parents[idx] = new_node_index;
            }
        }
        double rewire_time = chrono::duration_cast<chrono::duration<double>>(Clock::now() - start_rewire).count();
        rewire_time_total += rewire_time;

        // Check if new_node is close enough to any goal (assuming one goal)
        if (euclidean_distance(new_node, goal_position[0]) <= radius_goal) {
            goal_reached = true;
            cout << "Path cost: " << costs[new_node_index] << endl;
            //cout << "Total Nodes: " << tree.size() << endl;
            goal_index = tree.size() - 1;
            //cout << "Goal index: " << goal_index << endl;
            //cout << "Parent: " << parents[goal_index] << endl;
            //cout << "Coord: (" << tree[goal_index].x << ", " << tree[goal_index].y << ")" << endl;
        }

        double iteration_time = chrono::duration_cast<chrono::duration<double>>(Clock::now() - iteration_start).count();
        /*cout << "Iteration " << iteration << ": random sampling " << random_sampling_time << "s, nearest node " << find_nearest_time
             << "s, update tree " << update_tree_time << "s, footprint " << footprint_time
             << "s, rewire " << rewire_time << "s, total " << iteration_time << "s" << endl;*/

        if (goal_reached)
            break;
    }

    double total_rrt_time = chrono::duration_cast<chrono::duration<double>>(Clock::now() - total_rrt_start).count();
    cout << "Total RRT* time: " << total_rrt_time << " seconds" << endl;
    cout << "Iterations " << iteration << endl;
    

    // Backtracking to extract the path from start to goal
    auto start_backtrack = Clock::now();
    vector< pair<Point, Point> > path;
    vector< pair<Point2d, Point2d> > path_world;
    vector< pair<Point2d, Point2d> > waypoints;
    if (goal_reached && goal_index != -1) {
        int current_index = goal_index;
        int old_index = goal_index;
        while (current_index != -1) {
            //cout << "Path step: (" << tree[current_index].x << ", " << tree[current_index].y << ")" << endl;
            path.push_back(make_pair(tree[current_index], tree[old_index]));
            old_index = current_index;
            current_index = parents[current_index];
        }
        reverse(path.begin(), path.end());
        vector< pair<Point, Point> > better_path = path;
        //cout << "inver path = " << path.size() << " steps" << endl;
        //vector< pair<Point, Point> > better_path = smooth_path(path);
        //cout << "better path = " << better_path.size() << " segments" << endl;

        for (const auto &seg : better_path) {
            Point2d start_path = map_to_world(seg.first, map_resolution, map_origin, BW.rows);
            Point2d end_path = map_to_world(seg.second, map_resolution, map_origin, BW.rows);
            path_world.push_back(make_pair(start_path, end_path));
            waypoints.push_back(make_pair(start_path, end_path));
        }

        // Save the path to a text file
        ofstream file(output_file);
        if (file.is_open()) {
            for (const auto &coord : path_world) {
                file << "(" << coord.first.x << ", " << coord.first.y << ") -> ("
                     << coord.second.x << ", " << coord.second.y << ")" << "\n";
            }
            file.close();
            cout << "Path saved to rrt_path_star_kd.txt" << endl;
        }
    }
    bool success = !waypoints.empty();
    cout << (success ? "True" : "False") << endl;

    // Output JSON result.
    cout << "{\"success\": true}" << endl;
    double backtrack_time = chrono::duration_cast<chrono::duration<double>>(Clock::now() - start_backtrack).count();
    cout << "Time to backtrack and find path: " << backtrack_time << " seconds" << endl;

    /*std::ofstream file("simulations/RRT*kd/B15rrt*kddata.csv", std::ios::app); // Open in append mode

    if (!file) {
        std::cerr << "Error opening file!" << std::endl;
        return 1;
    }

    // Append data as a new row in the CSV file
    file << iteration << "," << costs[new_node_index]/10 << "," << total_rrt_time << std::endl;

    file.close(); // Close the file
    std::cout << "Data appended successfully!" << std::endl;*/

    // --- Plotting and Saving Data ---
    // Prepare the data for saving by converting to the expected types.
    auto BW_vector = convertMatToVector(BW);
    auto tree_vector = convertPoints2d(tree_world);
    auto start_vector = convertPoint2d(start_world);
    auto goal_vector = convertPoints2d(goal_world_positions);
    vector<std::pair<std::pair<double, double>, std::pair<double, double>>> path_vector;
    if (goal_reached) {
        vector<pair<Point2d, Point2d>> path_world;
        vector<pair<Point, Point>> better_path = smooth_path(path);
        for (const auto &seg : better_path) {
            Point2d start_path = map_to_world(seg.first, map_resolution, map_origin, BW.rows);
            Point2d end_path = map_to_world(seg.second, map_resolution, map_origin, BW.rows);
            path_world.push_back({start_path, end_path});
        }
        path_vector = convertPath(path_world);
    }
    auto footprints_vector = convertFootprints(footprints);

    if (goal_reached)
        saveData(BW_vector, tree_vector, start_vector, goal_vector, path_vector, footprints_vector, map_origin, map_resolution);
    else {
        vector<std::pair<std::pair<double, double>, std::pair<double, double>>> empty_path;
        saveData(BW_vector, tree_vector, start_vector, goal_vector, empty_path, footprints_vector, map_origin, map_resolution);
    }
    
    // Call Python script to plot (note the semicolon added)
    // Optional external plotting removed from the clean standalone build.

    return 0;
}
