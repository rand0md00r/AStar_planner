// astar_planner_node.cpp

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <queue>
#include <vector>
#include <cmath>
#include <chrono>

// Define a structure to represent a node in the grid map
struct Node {
    int x;
    int y;
    double f;  // f = g + h, the total cost
    double g;  // cost from start to current node
    double h;  // heuristic cost from current node to goal
    Node* parent;

    Node(int x_, int y_) : x(x_), y(y_), f(0.0), g(0.0), h(0.0), parent(nullptr) {}

    // Overload the "<" operator for priority queue sorting
    bool operator<(const Node& other) const {
        return f > other.f;  // Greater than for min-heap
    }
};

class AStarPlannerNode {
public:
    AStarPlannerNode() : nh("~") {
        // Subscribe to topics
        map_sub = nh.subscribe("/map", 1, &AStarPlannerNode::mapCallback, this);
        current_robot_pose.pose.pose.position.x = map.info.width / 2 + map.info.origin.position.x;      // for debug
        current_robot_pose.pose.pose.position.y = map.info.height / 2 + map.info.origin.position.y;      // for debug
        ROS_INFO("Hand set pose: x=%.2f, y=%.2f", current_robot_pose.pose.pose.position.x, current_robot_pose.pose.pose.position.y);

        robot_pose_sub = nh.subscribe("/robot_pose", 1, &AStarPlannerNode::robotPoseCallback, this);
        goal_sub = nh.subscribe("/move_base_simple/goal", 1, &AStarPlannerNode::goalCallback, this);
        

        // Create a publisher for the path
        path_pub = nh.advertise<nav_msgs::Path>("/AStar_path", 1);
        ori_path_pub = nh.advertise<nav_msgs::Path>("/Origin_Astar_path", 1);
        // Create a publisher for the binary map
        binary_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/binary_map", 1);
        inf_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/inflated_map", 1);


        nh.param<double>("inflation_radius", inflation_radius, 0.5);

        // Read the planning frequency parameter from the parameter server
        nh.param<double>("planning_frequency", planning_frequency, 1.0); // Default: 1.0 Hz
        ROS_INFO("Planning frequency set to: %.1f Hz", planning_frequency);

        ROS_INFO("A* Planner Node Initialized.");
    }

    // Callback function for /robot_pose topic
    void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        // Update robot pose
        current_robot_pose.pose.pose.position.x = msg->pose.pose.position.x;
        current_robot_pose.pose.pose.position.y = msg->pose.pose.position.y;

        ROS_INFO("Received robot pose: x=%.2f, y=%.2f", current_robot_pose.pose.pose.position.x, current_robot_pose.pose.pose.position.y);
    }

    // Callback function for /goal topic
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // Update goal position
        
        goal_position.pose.position.x = msg->pose.position.x;
        goal_position.pose.position.y = msg->pose.position.y;

        ROS_INFO("Received goal position: x=%.2f, y=%.2f", goal_position.pose.position.x, goal_position.pose.position.y);
    }

    // Callback function for /map topic
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        // Update the map data
        map = *msg;
        map_received = true;

        ROS_INFO("Received map data. Map size: %d x %d", map.info.width, map.info.height);

        // Process the map data: Binarize the map (0 for free space, 100 for obstacles)
        for (size_t i = 0; i < map.data.size(); ++i) {
            if (map.data[i] < 0) {
                map.data[i] = 0; // Set unknown space as free space
            } else if (map.data[i] >= 50) {
                map.data[i] = 100; // Set obstacles as obstacle space
            }
            // else, keep the original value (known free space)
        }

        // Inflate the obstacles in the map
        inflateObstacles(0.1, 0.5);

        // Publish the binary map
        binary_map_pub.publish(map);
        inf_map_pub.publish(inflated_map);
    }

    // A* path planning algorithm implementation
    std::vector<geometry_msgs::Point> astarPlanner() {
        // Check if the inflated_map data has been received
        if (!map_received) 
        {
            ROS_WARN("Map data not received yet. Cannot plan the path.");
            return std::vector<geometry_msgs::Point>(); // Return an empty path
        }


        // Convert the start and goal positions from world coordinates to grid coordinates
        int start_grid_x = (current_robot_pose.pose.pose.position.x - inflated_map.info.origin.position.x) / inflated_map.info.resolution;
        int start_grid_y = (current_robot_pose.pose.pose.position.y - inflated_map.info.origin.position.y) / inflated_map.info.resolution;
        int goal_grid_x = (goal_position.pose.position.x - inflated_map.info.origin.position.x) / inflated_map.info.resolution;
        int goal_grid_y = (goal_position.pose.position.y - inflated_map.info.origin.position.y) / inflated_map.info.resolution;


        // Check if the goal position is valid
        if (goal_grid_x < 0 || goal_grid_x >= inflated_map.info.width ||
            goal_grid_y < 0 || goal_grid_y >= inflated_map.info.height) 
        {
            ROS_WARN("Invalid goal position. Goal is outside the inflated_map. Cannot plan the path.");
            return std::vector<geometry_msgs::Point>(); // Return an empty path
        }


        // Initialize the A* algorithm
        std::priority_queue<Node> open_list;
        std::vector<std::vector<bool>> closed_list(inflated_map.info.height, std::vector<bool>(inflated_map.info.width, false));


        // Add the start node to the open list
        Node start_node(start_grid_x, start_grid_y);
        start_node.g = 0.0;
        start_node.h = calculateHeuristic(start_node.x, start_node.y, goal_grid_x, goal_grid_y);
        start_node.f = start_node.g + start_node.h;
        open_list.push(start_node);


        // Define the possible movement directions (8-connectivity)
        std::vector<int> dx = {-1, 0, 1, -1, 1, -1, 0, 1};
        std::vector<int> dy = {-1, -1, -1, 0, 0, 1, 1, 1};
        // A* algorithm main loop
        while (!open_list.empty()) {
            Node current_node = open_list.top();
            open_list.pop();

            // Check if the goal is reached
            if (current_node.x == goal_grid_x && current_node.y == goal_grid_y) {
                // Goal reached, reconstruct the path
                std::vector<geometry_msgs::Point> path;
                while (current_node.parent) {
                    geometry_msgs::Point point;
                    point.x = current_node.x * inflated_map.info.resolution + inflated_map.info.origin.position.x;
                    point.y = current_node.y * inflated_map.info.resolution + inflated_map.info.origin.position.y;
                    path.push_back(point);
                    current_node = *current_node.parent;
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            // Expand the current node's neighbors
            for (size_t i = 0; i < dx.size(); ++i) {
                int next_x = current_node.x + dx[i];
                int next_y = current_node.y + dy[i];

                // Check if the next position is within the map boundaries
                if (next_x >= 0 && next_x < inflated_map.info.width && next_y >= 0 && next_y < inflated_map.info.height) {
                    // Check if the next position is not an obstacle (occupancy > 50)
                    int index = next_y * inflated_map.info.width + next_x;
                    if (inflated_map.data[index] < 5 && !closed_list[next_y][next_x]) {
                        Node neighbor(next_x, next_y);
                        neighbor.g = current_node.g + 1.0; // Assuming the cost of moving to a neighbor is 1
                        neighbor.h = calculateHeuristic(neighbor.x, neighbor.y, goal_grid_x, goal_grid_y);
                        neighbor.f = neighbor.g + neighbor.h;
                        neighbor.parent = new Node(current_node);
                        open_list.push(neighbor);
                        closed_list[next_y][next_x] = true;
                    }
                }
            }
        }

        // Goal not reachable, return an empty path
        ROS_WARN("Goal not reachable. Failed to find a path.");
        return std::vector<geometry_msgs::Point>();
    }

    struct routePoint {
        int pointVal;
        int pointIndex;
        geometry_msgs::Point pointPose;
    };


    // Check if there is an obstacle between two points on the map using Bresenham algorithm
    bool isPathClear(const nav_msgs::OccupancyGrid& map, const geometry_msgs::Point& start, const geometry_msgs::Point& end) {

        int x0 = static_cast<int>((start.x - map.info.origin.position.x) / map.info.resolution);
        int y0 = static_cast<int>((start.y - map.info.origin.position.x) / map.info.resolution);
        int x1 = static_cast<int>((end.x - map.info.origin.position.x) / map.info.resolution);
        int y1 = static_cast<int>((end.y - map.info.origin.position.y) / map.info.resolution);

        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        while (true) {
            // Check if the current point is an occupied cell (obstacle)
            int current_x = x0;
            int current_y = y0;

            int map_index = x0 + map.info.width * y0;
            if (map_index >= 0 && map_index < map.data.size()) {
                if (map.data[map_index] == 100) {
                    return false; // Obstacle found
                }
            }

            if (x0 == x1 && y0 == y1) {
                break; // Reached the end point
            }

            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y0 += sy;
            }
        }

        return true; // No obstacles found along the path
    }


    std::vector<geometry_msgs::Point> prunePath(const nav_msgs::OccupancyGrid& map, const std::vector<geometry_msgs::Point>& path) {

        if(map.data.empty() || path.empty()) return std::vector<geometry_msgs::Point> ();

        ROS_DEBUG("路径点数量: %zu", path.size());

        std::vector<geometry_msgs::Point> prunedPath;

        prunedPath.push_back(path.front()); // Add the start point to the pruned path

        for (size_t i = 0; i < path.size(); ++i) {
            for(size_t j = i + 2; j < path.size() - 1; ++j){

                geometry_msgs::Point start_point = path[i];
                geometry_msgs::Point end_point = path[j];

                geometry_msgs::Point end_pre_point = path[j - 1];
                geometry_msgs::Point end_next_point = path[j + 1];

                // if (!isPathClear(map, start_point, end_point ) && isPathClear(map, start_point, end_pre_point) && !isPathClear(map, start_point, end_next_point)) {      // 路径通过障碍物时,更新
                if (!isPathClear(map, start_point, end_point )) {      // 路径通过障碍物时,更新
                    geometry_msgs::Point cornorPoint = path[j - 1];
                    prunedPath.push_back(cornorPoint);
                    i = j;
                    break;
                }
            }
        }

        prunedPath.push_back(path.back());

        ROS_DEBUG("剪枝后剩余的点的数量为: %zu", prunedPath.size());

        return prunedPath;
    }

    std::vector<geometry_msgs::Point> interpolatePath(const std::vector<geometry_msgs::Point>& input_path, double interpolation_distance)
    {
        std::vector<geometry_msgs::Point> interpolated_path;

        if (input_path.empty())
        {
            ROS_WARN("Input path is empty.");
            return interpolated_path;
        }

        if (interpolation_distance <= 0.0)
        {
            ROS_WARN("Interpolation distance must be greater than zero.");
            return interpolated_path;
        }

        // Add the first point from the input path
        interpolated_path.push_back(input_path.front());

        // Iterate over the input path and interpolate between each consecutive pair of points
        for (size_t i = 1; i < input_path.size(); ++i)
        {
            const geometry_msgs::Point& prev_point = input_path[i - 1];
            const geometry_msgs::Point& current_point = input_path[i];

            // Calculate the distance between the current and previous points
            double distance = std::sqrt(std::pow(current_point.x - prev_point.x, 2) +
                                        std::pow(current_point.y - prev_point.y, 2) +
                                        std::pow(current_point.z - prev_point.z, 2));

            // Determine how many points to add between the current and previous points
            int num_interpolated_points = static_cast<int>(distance / interpolation_distance);
            if (num_interpolated_points <= 0)
            {
                // If distance is smaller than the interpolation_distance, use at least one interpolated point
                num_interpolated_points = 1;
            }

            // Calculate the step size for each interpolation
            double step_size_x = (current_point.x - prev_point.x) / (num_interpolated_points + 1);
            double step_size_y = (current_point.y - prev_point.y) / (num_interpolated_points + 1);
            double step_size_z = (current_point.z - prev_point.z) / (num_interpolated_points + 1);

            // Interpolate and add the points between the current and previous points
            for (int j = 0; j < num_interpolated_points; ++j)
            {
                geometry_msgs::Point interpolated_point;
                interpolated_point.x = prev_point.x + (j + 1) * step_size_x;
                interpolated_point.y = prev_point.y + (j + 1) * step_size_y;
                interpolated_point.z = prev_point.z + (j + 1) * step_size_z;
                interpolated_path.push_back(interpolated_point);
            }

            // Add the current point from the input path
            interpolated_path.push_back(current_point);
        }

        ROS_DEBUG("线性插值后，路径点的数量为: %zu", interpolated_path.size());

        return interpolated_path;
    }






// Catmull-Rom spline interpolation function
std::vector<geometry_msgs::Point> catmullRomInterpolation(const std::vector<geometry_msgs::Point>& input_path, int num_interpolation_points)
{
    std::vector<geometry_msgs::Point> interpolated_path;

    if (input_path.size() < 2 || num_interpolation_points <= 0)
    {
        std::cout << "Input path is too short or number of interpolation points is invalid." << std::endl;
        return interpolated_path;
    }

    int n = input_path.size();

    for (int i = 0; i < n - 1; ++i)
    {
        for (int j = 0; j <= num_interpolation_points; ++j)
        {
            double t = static_cast<double>(j) / num_interpolation_points;
            double t2 = t * t;
            double t3 = t2 * t;

            geometry_msgs::Point interpolated_point;
            interpolated_point.x = 0.5 * ((2 * input_path[i].x) +
                                          (-input_path[i - 1].x + input_path[i + 1].x) * t +
                                          (2 * input_path[i - 1].x - 5 * input_path[i].x + 4 * input_path[i + 1].x - input_path[i + 2].x) * t2 +
                                          (-input_path[i - 1].x + 3 * input_path[i].x - 3 * input_path[i + 1].x + input_path[i + 2].x) * t3);
            interpolated_point.y = 0.5 * ((2 * input_path[i].y) +
                                          (-input_path[i - 1].y + input_path[i + 1].y) * t +
                                          (2 * input_path[i - 1].y - 5 * input_path[i].y + 4 * input_path[i + 1].y - input_path[i + 2].y) * t2 +
                                          (-input_path[i - 1].y + 3 * input_path[i].y - 3 * input_path[i + 1].y + input_path[i + 2].y) * t3);
            interpolated_point.z = 0.5 * ((2 * input_path[i].z) +
                                          (-input_path[i - 1].z + input_path[i + 1].z) * t +
                                          (2 * input_path[i - 1].z - 5 * input_path[i].z + 4 * input_path[i + 1].z - input_path[i + 2].z) * t2 +
                                          (-input_path[i - 1].z + 3 * input_path[i].z - 3 * input_path[i + 1].z + input_path[i + 2].z) * t3);

            interpolated_path.push_back(interpolated_point);
        }
    }

    // Add the last point of the input path to the interpolated path
    interpolated_path.push_back(input_path.back());

    ROS_DEBUG("插值后，路径点的数量为: %zu", interpolated_path.size());

    return interpolated_path;
}




    // Publish the planned path
    void publishPath(const std::vector<geometry_msgs::Point>& path) {
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map"; // Assuming the path is in the map frame

        // Convert the vector of points to the path message
        for (const auto& point : path) {
            geometry_msgs::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.position.z = 0.0; // Assuming the path is in 2D
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }

        // Publish the path message
        path_pub.publish(path_msg);
    }

    // Publish the planned path
    void publistOriginPath(const std::vector<geometry_msgs::Point>& path) {
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map"; // Assuming the path is in the map frame

        // Convert the vector of points to the path message
        for (const auto& point : path) {
            geometry_msgs::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.position.z = 0.0; // Assuming the path is in 2D
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }

        // Publish the path message
        ori_path_pub.publish(path_msg);
    }

    // Main loop of the node
    void run() {
        ros::Rate loop_rate(planning_frequency);

        while (ros::ok()) {

            // Call the A* path planning algorithm
            auto start_time = std::chrono::high_resolution_clock::now();

            std::vector<geometry_msgs::Point> path = astarPlanner();

            std::vector<geometry_msgs::Point> prune_Path = prunePath(inflated_map, path);  // 路径剪枝

            std::vector<geometry_msgs::Point> interpolated_path = interpolatePath(prune_Path, 0.1);          // 线性插值
            // std::vector<geometry_msgs::Point> interpolated_path = catmullRomInterpolation(prune_Path, 100);     // Catmull-Rom 样条插值

            auto after_inter = std::chrono::high_resolution_clock::now();

            if(!path.empty()) {
                auto duration_ms = after_inter - start_time;
                std::cout << "--- Planning Success: Time of Planning: " << duration_ms.count()  / 1000000 << " ms, " << interpolated_path.size() << " Points in final Path." << std::endl;
            }

            // Publish the path (you can convert the vector of points to a ROS message)
            publishPath(interpolated_path);
            publistOriginPath(prune_Path);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }




private:
    ros::NodeHandle nh;
    ros::Subscriber robot_pose_sub;
    ros::Subscriber goal_sub;
    ros::Subscriber map_sub;
    ros::Publisher path_pub;
    ros::Publisher ori_path_pub;
    ros::Publisher binary_map_pub; // Publisher for binary map
    ros::Publisher inf_map_pub;

    // Planning frequency parameter
    double planning_frequency;

    // Current robot pose and goal position
    geometry_msgs::PoseWithCovarianceStamped current_robot_pose;
    geometry_msgs::PoseStamped goal_position;

    double inflation_radius;

    // OccupancyGrid to hold the map data
    nav_msgs::OccupancyGrid map;
    nav_msgs::OccupancyGrid inflated_map;
    bool map_received;


// Helper function to inflate the obstacles in the map
void inflateObstacles(double d_cell, double inflation_r) {
    // Copy the original map data to a temporary map
    inflated_map = map;

    // Calculate the inflation radius in terms of grid cells
    int inflation_radius_in_cells = std::ceil(inflation_r / d_cell);

    // Define the possible movement directions (4-connectivity)
    std::vector<int> dx = {-1, 0, 1, -1, 1, -1, 0, 1};
    std::vector<int> dy = {-1, -1, -1, 0, 0, 1, 1, 1};

    // Traverse the map and inflate obstacles
    for (int y = 0; y < map.info.height; ++y) {
        for (int x = 0; x < map.info.width; ++x) {
            // Check if the cell is an obstacle
            int index = y * map.info.width + x;
            if (map.data[index] == 100) {
                // Inflate the obstacle and the surrounding cells within the 9x9 range
                for (int i = -inflation_radius_in_cells; i <= inflation_radius_in_cells; ++i) {
                    for (int j = -inflation_radius_in_cells; j <= inflation_radius_in_cells; ++j) {
                        int nx = x + j;
                        int ny = y + i;

                        // Check if the neighbor cell is within the map boundaries
                        if (nx >= 0 && nx < map.info.width && ny >= 0 && ny < map.info.height) {
                            int nIndex = ny * map.info.width + nx;
                            inflated_map.data[nIndex] = 100;
                        }
                    }
                }
            }
        }
    }
}


    // Helper function to calculate the Euclidean distance heuristic
    double calculateHeuristic(int x1, int y1, int x2, int y2) {
        return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "astar_planner_node");

    AStarPlannerNode planner_node;
    planner_node.run();

    return 0;
}