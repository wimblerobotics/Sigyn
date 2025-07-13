#include <getopt.h>
#include <sqlite3.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.hpp>

struct WallSegment {
    int id;
    geometry_msgs::msg::Point start_point;
    geometry_msgs::msg::Point end_point;
    double length;
    double angle;
    std::string room_name;
    std::string wall_name;
};

class WallFinder : public rclcpp::Node {
public:
    WallFinder() : Node("wall_finder") {
        RCLCPP_INFO(this->get_logger(), "Wall Finder node starting...");
        
        // Initialize parameters with default values
        loadParameters();
        
        // Initialize database
        initializeDatabase();
        
        // Set up subscriber
        costmap_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            costmap_topic_, 10,
            std::bind(&WallFinder::costmapCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Wall Finder node initialized. Waiting for costmap...");
    }

private:
    void loadParameters() {
        // Load parameters from YAML file if config argument is provided
        if (config_file_path_.empty()) {
            // Use ROS parameters
            this->declare_parameter<std::string>("costmap_topic", "/global_costmap/static_layer");
            this->declare_parameter<int>("costmap_threshold", 50);
            this->declare_parameter<int>("wall_threshold", 95);
            this->declare_parameter<int>("erosion_size", 5);
            this->declare_parameter<int>("erosion_iterations", 2);
            this->declare_parameter<double>("min_wall_length_inches", 5.0);
            this->declare_parameter<double>("min_wall_length_meters", 0.127);
            this->declare_parameter<double>("hough_rho", 1.0);
            this->declare_parameter<double>("hough_theta", 0.017453292519943295);
            this->declare_parameter<int>("hough_threshold", 50);
            this->declare_parameter<int>("hough_min_line_length", 25);
            this->declare_parameter<int>("hough_max_line_gap", 10);
            this->declare_parameter<double>("angle_tolerance", 0.087266462599716477);
            this->declare_parameter<double>("distance_tolerance", 0.1);
            this->declare_parameter<std::string>("database_path", "walls.db");
            this->declare_parameter<std::string>("map_frame", "map");
            
            costmap_topic_ = this->get_parameter("costmap_topic").as_string();
            costmap_threshold_ = this->get_parameter("costmap_threshold").as_int();
            wall_threshold_ = this->get_parameter("wall_threshold").as_int();
            erosion_size_ = this->get_parameter("erosion_size").as_int();
            erosion_iterations_ = this->get_parameter("erosion_iterations").as_int();
            min_wall_length_inches_ = this->get_parameter("min_wall_length_inches").as_double();
            min_wall_length_meters_ = this->get_parameter("min_wall_length_meters").as_double();
            hough_rho_ = this->get_parameter("hough_rho").as_double();
            hough_theta_ = this->get_parameter("hough_theta").as_double();
            hough_threshold_ = this->get_parameter("hough_threshold").as_int();
            hough_min_line_length_ = this->get_parameter("hough_min_line_length").as_int();
            hough_max_line_gap_ = this->get_parameter("hough_max_line_gap").as_int();
            angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();
            distance_tolerance_ = this->get_parameter("distance_tolerance").as_double();
            database_path_ = this->get_parameter("database_path").as_string();
            map_frame_ = this->get_parameter("map_frame").as_string();
        }
    }

    void initializeDatabase() {
        int rc = sqlite3_open(database_path_.c_str(), &db_);
        if (rc) {
            RCLCPP_ERROR(this->get_logger(), "Can't open database: %s", sqlite3_errmsg(db_));
            return;
        }
        
        // Create table if it doesn't exist
        const char* sql = R"(
            CREATE TABLE IF NOT EXISTS walls (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                start_x REAL NOT NULL,
                start_y REAL NOT NULL,
                end_x REAL NOT NULL,
                end_y REAL NOT NULL,
                length REAL NOT NULL,
                angle REAL NOT NULL,
                room_name TEXT DEFAULT 'undefined',
                wall_name TEXT DEFAULT 'undefined'
            );
        )";
        
        char* err_msg = 0;
        rc = sqlite3_exec(db_, sql, 0, 0, &err_msg);
        if (rc != SQLITE_OK) {
            RCLCPP_ERROR(this->get_logger(), "SQL error: %s", err_msg);
            sqlite3_free(err_msg);
        } else {
            RCLCPP_INFO(this->get_logger(), "Database initialized successfully");
        }
    }

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received costmap with size %dx%d", msg->info.width, msg->info.height);
        
        // Process the costmap once and then shutdown
        processOccupancyGrid(msg);
        
        RCLCPP_INFO(this->get_logger(), "Wall detection completed. Shutting down...");
        rclcpp::shutdown();
    }

    void processOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        // Convert occupancy grid to OpenCV image
        cv::Mat occupancy_image = occupancyGridToImage(msg);
        
        // Create a mask for only highly occupied areas (actual walls, not inflated areas)
        cv::Mat wall_mask;
        double high_occupancy_threshold = (wall_threshold_ / 100.0) * 255.0;
        cv::threshold(occupancy_image, wall_mask, high_occupancy_threshold, 255, cv::THRESH_BINARY);
        
        // Apply morphological operations to clean up noise
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(wall_mask, wall_mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(wall_mask, wall_mask, cv::MORPH_OPEN, kernel);
        
        // Apply thinning/skeletonization to find wall centerlines
        cv::Mat skeleton = skeletonize(wall_mask);
        
        // Apply Hough line detection directly on the skeleton
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(skeleton, lines, hough_rho_, hough_theta_, hough_threshold_, 
                       hough_min_line_length_, hough_max_line_gap_);
        
        RCLCPP_INFO(this->get_logger(), "Using wall threshold: %d%% (%.1f in image space) for high occupancy", 
                   wall_threshold_, high_occupancy_threshold);
        RCLCPP_INFO(this->get_logger(), "Detected %zu raw line segments from wall skeletons", lines.size());
        
        // Convert line segments to wall segments
        std::vector<WallSegment> wall_segments = linesToWallSegments(lines, msg);
        
        // Merge similar wall segments
        std::vector<WallSegment> merged_walls = mergeWallSegments(wall_segments);
        
        // Filter walls by minimum length
        std::vector<WallSegment> filtered_walls = filterWallsByLength(merged_walls);
        
        RCLCPP_INFO(this->get_logger(), "Found %zu wall segments after processing", filtered_walls.size());
        
        // Clear database and store new walls
        clearDatabase();
        storeWalls(filtered_walls);
        
        // Close database connection
        sqlite3_close(db_);
    }

    cv::Mat occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        cv::Mat image(msg->info.height, msg->info.width, CV_8UC1);
        
        for (unsigned int y = 0; y < msg->info.height; y++) {
            for (unsigned int x = 0; x < msg->info.width; x++) {
                int index = y * msg->info.width + x;
                int8_t value = msg->data[index];
                
                if (value == -1) {
                    // Unknown space - treat as free space for wall detection
                    image.at<uint8_t>(y, x) = 0;
                } else {
                    // Scale occupancy probability (0-100) to image value (0-255)
                    image.at<uint8_t>(y, x) = static_cast<uint8_t>(value * 255 / 100);
                }
            }
        }
        
        return image;
    }

    std::vector<WallSegment> linesToWallSegments(const std::vector<cv::Vec4i>& lines, 
                                                const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::vector<WallSegment> segments;
        
        for (size_t i = 0; i < lines.size(); i++) {
            const cv::Vec4i& line = lines[i];
            
            WallSegment segment;
            segment.id = static_cast<int>(i);
            
            // Convert pixel coordinates to world coordinates
            segment.start_point = pixelToWorld(line[0], line[1], msg);
            segment.end_point = pixelToWorld(line[2], line[3], msg);
            
            // Calculate length
            double dx = segment.end_point.x - segment.start_point.x;
            double dy = segment.end_point.y - segment.start_point.y;
            segment.length = std::sqrt(dx * dx + dy * dy);
            
            // Calculate angle (relative to x-axis)
            segment.angle = std::atan2(dy, dx);
            
            // Initialize names
            segment.room_name = "undefined";
            segment.wall_name = "undefined";
            
            segments.push_back(segment);
        }
        
        return segments;
    }

    geometry_msgs::msg::Point pixelToWorld(int x, int y, const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        geometry_msgs::msg::Point point;
        
        // Convert pixel coordinates to world coordinates
        point.x = msg->info.origin.position.x + x * msg->info.resolution;
        point.y = msg->info.origin.position.y + y * msg->info.resolution;
        point.z = 0.0;
        
        return point;
    }

    std::vector<WallSegment> mergeWallSegments(const std::vector<WallSegment>& segments) {
        std::vector<WallSegment> merged;
        std::vector<bool> used(segments.size(), false);
        
        for (size_t i = 0; i < segments.size(); i++) {
            if (used[i]) continue;
            
            WallSegment merged_segment = segments[i];
            used[i] = true;
            
            // Find segments that can be merged with this one
            bool found_merge = true;
            while (found_merge) {
                found_merge = false;
                
                for (size_t j = 0; j < segments.size(); j++) {
                    if (used[j]) continue;
                    
                    if (canMergeSegments(merged_segment, segments[j])) {
                        merged_segment = mergeTwoSegments(merged_segment, segments[j]);
                        used[j] = true;
                        found_merge = true;
                    }
                }
            }
            
            merged.push_back(merged_segment);
        }
        
        return merged;
    }

    bool canMergeSegments(const WallSegment& seg1, const WallSegment& seg2) {
        // Check if angles are similar
        double angle_diff = std::abs(seg1.angle - seg2.angle);
        if (angle_diff > M_PI) {
            angle_diff = 2 * M_PI - angle_diff;
        }
        
        if (angle_diff > angle_tolerance_) {
            return false;
        }
        
        // Check if segments are collinear and close enough
        double distance = distanceBetweenSegments(seg1, seg2);
        return distance < distance_tolerance_;
    }

    double distanceBetweenSegments(const WallSegment& seg1, const WallSegment& seg2) {
        // Calculate minimum distance between two line segments
        double d1 = pointToPointDistance(seg1.start_point, seg2.start_point);
        double d2 = pointToPointDistance(seg1.start_point, seg2.end_point);
        double d3 = pointToPointDistance(seg1.end_point, seg2.start_point);
        double d4 = pointToPointDistance(seg1.end_point, seg2.end_point);
        
        return std::min({d1, d2, d3, d4});
    }

    double pointToPointDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    WallSegment mergeTwoSegments(const WallSegment& seg1, const WallSegment& seg2) {
        WallSegment merged;
        
        // Find the endpoints that are farthest apart
        double d1 = pointToPointDistance(seg1.start_point, seg2.start_point);
        double d2 = pointToPointDistance(seg1.start_point, seg2.end_point);
        double d3 = pointToPointDistance(seg1.end_point, seg2.start_point);
        double d4 = pointToPointDistance(seg1.end_point, seg2.end_point);
        
        double max_distance = std::max({d1, d2, d3, d4});
        
        if (max_distance == d1) {
            merged.start_point = seg1.start_point;
            merged.end_point = seg2.start_point;
        } else if (max_distance == d2) {
            merged.start_point = seg1.start_point;
            merged.end_point = seg2.end_point;
        } else if (max_distance == d3) {
            merged.start_point = seg1.end_point;
            merged.end_point = seg2.start_point;
        } else {
            merged.start_point = seg1.end_point;
            merged.end_point = seg2.end_point;
        }
        
        // Recalculate length and angle
        double dx = merged.end_point.x - merged.start_point.x;
        double dy = merged.end_point.y - merged.start_point.y;
        merged.length = std::sqrt(dx * dx + dy * dy);
        merged.angle = std::atan2(dy, dx);
        
        merged.room_name = "undefined";
        merged.wall_name = "undefined";
        
        return merged;
    }

    std::vector<WallSegment> filterWallsByLength(const std::vector<WallSegment>& segments) {
        std::vector<WallSegment> filtered;
        
        for (const auto& segment : segments) {
            if (segment.length >= min_wall_length_meters_) {
                filtered.push_back(segment);
            }
        }
        
        return filtered;
    }

    void clearDatabase() {
        const char* sql = "DELETE FROM walls";
        char* err_msg = 0;
        int rc = sqlite3_exec(db_, sql, 0, 0, &err_msg);
        if (rc != SQLITE_OK) {
            RCLCPP_ERROR(this->get_logger(), "SQL error clearing database: %s", err_msg);
            sqlite3_free(err_msg);
        }
    }

    void storeWalls(const std::vector<WallSegment>& walls) {
        const char* sql = R"(
            INSERT INTO walls (start_x, start_y, end_x, end_y, length, angle, room_name, wall_name)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        )";
        
        sqlite3_stmt* stmt;
        int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, NULL);
        if (rc != SQLITE_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to prepare statement: %s", sqlite3_errmsg(db_));
            return;
        }
        
        for (const auto& wall : walls) {
            sqlite3_bind_double(stmt, 1, wall.start_point.x);
            sqlite3_bind_double(stmt, 2, wall.start_point.y);
            sqlite3_bind_double(stmt, 3, wall.end_point.x);
            sqlite3_bind_double(stmt, 4, wall.end_point.y);
            sqlite3_bind_double(stmt, 5, wall.length);
            sqlite3_bind_double(stmt, 6, wall.angle);
            sqlite3_bind_text(stmt, 7, wall.room_name.c_str(), -1, SQLITE_STATIC);
            sqlite3_bind_text(stmt, 8, wall.wall_name.c_str(), -1, SQLITE_STATIC);
            
            rc = sqlite3_step(stmt);
            if (rc != SQLITE_DONE) {
                RCLCPP_ERROR(this->get_logger(), "Failed to insert wall: %s", sqlite3_errmsg(db_));
            }
            
            sqlite3_reset(stmt);
        }
        
        sqlite3_finalize(stmt);
        
        RCLCPP_INFO(this->get_logger(), "Stored %zu wall segments to database", walls.size());
    }

    // Skeletonization function to find centerlines of thick walls
    cv::Mat skeletonize(const cv::Mat& src) {
        cv::Mat image = src.clone();
        cv::Mat skeleton(image.size(), CV_8UC1, cv::Scalar(0));
        cv::Mat temp;
        cv::Mat eroded;
        
        cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
        
        bool done = false;
        while (!done) {
            cv::erode(image, eroded, element);
            cv::dilate(eroded, temp, element);
            cv::subtract(image, temp, temp);
            cv::bitwise_or(skeleton, temp, skeleton);
            eroded.copyTo(image);
            
            done = (cv::countNonZero(image) == 0);
        }
        
        return skeleton;
    }

    // Member variables
    std::string config_file_path_;
    std::string costmap_topic_;
    int costmap_threshold_;
    int wall_threshold_;
    int erosion_size_;
    int erosion_iterations_;
    double min_wall_length_inches_;
    double min_wall_length_meters_;
    double hough_rho_;
    double hough_theta_;
    int hough_threshold_;
    int hough_min_line_length_;
    int hough_max_line_gap_;
    double angle_tolerance_;
    double distance_tolerance_;
    std::string database_path_;
    std::string map_frame_;
    
    sqlite3* db_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription_;
};

void processConfiguration(int argc, char* argv[], std::string& config_file_path) {
    static const struct option long_options[] = {
        {"config", required_argument, nullptr, 'c'},
        {"ros-args", optional_argument, nullptr, 'r'},
        {nullptr, 0, nullptr, 0}
    };

    int opt = 0;
    int option_index = 0;
    while ((opt = getopt_long_only(argc, argv, "c:r", long_options, &option_index)) != -1) {
        switch (opt) {
            case 'c':
                config_file_path = optarg;
                break;
        }
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    std::string config_file_path;
    processConfiguration(argc, argv, config_file_path);
    
    auto node = std::make_shared<WallFinder>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception caught: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
