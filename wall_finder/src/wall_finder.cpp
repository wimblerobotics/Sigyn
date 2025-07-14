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
        
        // Set up publisher for walls costmap visualization
        if (publish_walls_costmap_) {
            walls_costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
                "/walls_costmap", 10);
            
            // Create timer to publish walls costmap continuously (1 Hz)
            walls_costmap_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000),
                std::bind(&WallFinder::publishStoredWallsCostmap, this));
        }
        
        RCLCPP_INFO(this->get_logger(), "Wall Finder node initialized. Waiting for costmap...");
    }

    ~WallFinder() {
        if (db_) {
            sqlite3_close(db_);
            RCLCPP_INFO(this->get_logger(), "Database connection closed.");
        }
    }

private:
    void loadParameters() {
        // Load parameters from YAML file if config argument is provided
        if (config_file_path_.empty()) {
            // Use ROS parameters
            this->declare_parameter<std::string>("costmap_topic", "/global_costmap/static_layer");
            this->declare_parameter<int>("costmap_threshold", 50);
            this->declare_parameter<int>("erosion_size", 5);
            this->declare_parameter<int>("erosion_iterations", 2);
            this->declare_parameter<double>("min_wall_length_inches", 5.0);
            this->declare_parameter<double>("min_wall_length_meters", 0.127);
            this->declare_parameter<double>("hough_rho", 1.0);
            this->declare_parameter<double>("hough_theta", 0.017453292519943295);
            this->declare_parameter<int>("hough_threshold", 50);
            this->declare_parameter<int>("hough_min_line_length", 25);
            this->declare_parameter<int>("hough_max_line_gap", 10);
            this->declare_parameter<double>("angle_tolerance", 0.174532925);
            this->declare_parameter<double>("distance_tolerance", 0.2);
            this->declare_parameter<double>("proximity_tolerance", 0.15);
            this->declare_parameter<std::string>("database_path", "walls.db");
            this->declare_parameter<std::string>("map_frame", "map");
            this->declare_parameter<bool>("publish_walls_costmap", true);
            
            costmap_topic_ = this->get_parameter("costmap_topic").as_string();
            costmap_threshold_ = this->get_parameter("costmap_threshold").as_int();
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
            proximity_tolerance_ = this->get_parameter("proximity_tolerance").as_double();
            database_path_ = this->get_parameter("database_path").as_string();
            map_frame_ = this->get_parameter("map_frame").as_string();
            publish_walls_costmap_ = this->get_parameter("publish_walls_costmap").as_bool();
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
        
        // Process the costmap once only
        static bool processed = false;
        if (processed) {
            return; // Don't process again
        }
        processed = true;
        
        processOccupancyGrid(msg);
        
        if (!publish_walls_costmap_) {
            RCLCPP_INFO(this->get_logger(), "Wall detection completed. Shutting down...");
            rclcpp::shutdown();
        } else {
            RCLCPP_INFO(this->get_logger(), "Wall detection completed. Continuing to publish walls costmap...");
        }
    }

    void processOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        // Convert occupancy grid to OpenCV image
        cv::Mat occupancy_image = occupancyGridToImage(msg);
        
        // Save debug images to understand what's happening
        cv::imwrite("/tmp/debug_01_original.png", occupancy_image);
        
        // Create a mask for wall detection - use a lower threshold to capture wall edges
        cv::Mat wall_mask;
        double wall_detection_threshold = (costmap_threshold_ / 100.0) * 255.0;
        cv::threshold(occupancy_image, wall_mask, wall_detection_threshold, 255, cv::THRESH_BINARY);
        cv::imwrite("/tmp/debug_02_thresholded.png", wall_mask);
        
        // For perfect hand-drawn maps, let's try a completely different approach
        // Instead of complex morphological operations, let's be more conservative
        
        // First, let's try to identify if this is indeed a clean hand-drawn map
        // by checking for mostly horizontal/vertical edges
        cv::Mat simple_edges;
        cv::Canny(wall_mask, simple_edges, 100, 200, 3);
        cv::imwrite("/tmp/debug_03_simple_edges.png", simple_edges);
        
        // For hand-drawn maps, try using probabilistic Hough with very permissive parameters
        // to capture long lines even if they have small gaps
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(simple_edges, lines, 
                       1.0,                    // rho: 1 pixel resolution
                       CV_PI/180,              // theta: 1 degree resolution
                       10,                     // threshold: very low for clean maps
                       5,                      // min line length: very short
                       50);                    // max gap: large gap tolerance for hand-drawn
        
        RCLCPP_INFO(this->get_logger(), "DEBUG: Original image size: %dx%d", occupancy_image.cols, occupancy_image.rows);
        RCLCPP_INFO(this->get_logger(), "DEBUG: Wall detection threshold: %d%% (%.1f in image space)", 
                   costmap_threshold_, wall_detection_threshold);
        RCLCPP_INFO(this->get_logger(), "DEBUG: Detected %zu raw line segments", lines.size());
        
        // Draw all detected lines on a debug image
        cv::Mat debug_lines = cv::Mat::zeros(occupancy_image.size(), CV_8UC3);
        for (size_t i = 0; i < lines.size(); i++) {
            cv::Vec4i line = lines[i];
            cv::line(debug_lines, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), 
                    cv::Scalar(0, 255, 0), 2);
        }
        cv::imwrite("/tmp/debug_04_detected_lines.png", debug_lines);
        
        // Convert line segments to wall segments
        std::vector<WallSegment> wall_segments = linesToWallSegments(lines, msg);
        
        // Apply very aggressive coalescing for hand-drawn maps
        std::vector<WallSegment> coalesced_walls = coalesceForHandDrawnMaps(wall_segments);
        
        // Filter walls by minimum length
        std::vector<WallSegment> filtered_walls = filterWallsByLength(coalesced_walls);
        
        RCLCPP_INFO(this->get_logger(), "DEBUG: %zu raw segments -> %zu coalesced -> %zu filtered", 
                   wall_segments.size(), coalesced_walls.size(), filtered_walls.size());
        
        // Draw final walls on debug image
        cv::Mat debug_final = cv::Mat::zeros(occupancy_image.size(), CV_8UC3);
        for (const auto& wall : filtered_walls) {
            // Convert back to pixel coordinates for debug
            int start_x = static_cast<int>((wall.start_point.x - msg->info.origin.position.x) / msg->info.resolution);
            int start_y = static_cast<int>((wall.start_point.y - msg->info.origin.position.y) / msg->info.resolution);
            int end_x = static_cast<int>((wall.end_point.x - msg->info.origin.position.x) / msg->info.resolution);
            int end_y = static_cast<int>((wall.end_point.y - msg->info.origin.position.y) / msg->info.resolution);
            
            cv::line(debug_final, cv::Point(start_x, start_y), cv::Point(end_x, end_y), 
                    cv::Scalar(0, 0, 255), 3);
        }
        cv::imwrite("/tmp/debug_05_final_walls.png", debug_final);
        
        RCLCPP_INFO(this->get_logger(), "DEBUG: Saved debug images to /tmp/debug_*.png");
        
        // Publish walls as costmap for visualization
        publishWallsCostmap(filtered_walls, msg);
        
        // Store walls to database (only once)
        clearDatabase();
        storeWalls(filtered_walls);
        
        RCLCPP_INFO(this->get_logger(), "Wall detection completed. Database populated with %zu walls.", filtered_walls.size());
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

    std::vector<WallSegment> coalesceWallSegmentsWallAware(const std::vector<WallSegment>& segments) {
        RCLCPP_INFO(this->get_logger(), "Starting wall-aware coalescing with %zu segments", segments.size());
        
        std::vector<WallSegment> coalesced;
        std::vector<bool> used(segments.size(), false);
        
        // Typical wall thickness in meters (4-8 inches)
        double min_wall_thickness = 0.10; // 4 inches
        double max_wall_thickness = 0.25; // 10 inches
        
        for (size_t i = 0; i < segments.size(); i++) {
            if (used[i]) continue;
            
            std::vector<WallSegment> cluster;
            cluster.push_back(segments[i]);
            used[i] = true;
            
            // Find segments that can be merged with this one (same-side wall segments)
            bool found_merge = true;
            while (found_merge) {
                found_merge = false;
                
                for (size_t j = 0; j < segments.size(); j++) {
                    if (used[j]) continue;
                    
                    // Check if this segment can be merged with any segment in the cluster
                    bool can_merge = false;
                    for (const auto& cluster_seg : cluster) {
                        if (canCoalesceWallAware(cluster_seg, segments[j], min_wall_thickness, max_wall_thickness)) {
                            can_merge = true;
                            break;
                        }
                    }
                    
                    if (can_merge) {
                        cluster.push_back(segments[j]);
                        used[j] = true;
                        found_merge = true;
                    }
                }
            }
            
            // Fit a single line to all segments in the cluster
            if (cluster.size() > 1) {
                WallSegment fitted_line = fitLineToSegmentsRobust(cluster);
                coalesced.push_back(fitted_line);
                RCLCPP_DEBUG(this->get_logger(), "Coalesced %zu segments into single wall line", cluster.size());
            } else {
                coalesced.push_back(cluster[0]);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Wall-aware coalescing: %zu segments -> %zu walls", segments.size(), coalesced.size());
        return coalesced;
    }

    bool canCoalesceWallAware(const WallSegment& seg1, const WallSegment& seg2, 
                              double min_wall_thickness, double max_wall_thickness) {
        // Check if angles are similar (within tolerance)
        double angle_diff = std::abs(seg1.angle - seg2.angle);
        if (angle_diff > M_PI) {
            angle_diff = 2 * M_PI - angle_diff;
        }
        
        if (angle_diff > angle_tolerance_) {
            return false;
        }
        
        // Check if segments are close enough (endpoint proximity)
        double endpoint_distance = minEndpointDistance(seg1, seg2);
        if (endpoint_distance > proximity_tolerance_) {
            return false;
        }
        
        // Check if segments are roughly collinear
        double collinearity_distance = distanceToLine(seg1, seg2);
        if (collinearity_distance > distance_tolerance_) {
            return false;
        }
        
        // CRITICAL: Check if segments are on the same side of a wall
        // Calculate the perpendicular distance between the parallel segments
        double perpendicular_distance = perpendicularDistanceBetweenSegments(seg1, seg2);
        
        // If segments are too far apart perpendicular to their direction,
        // they might be on opposite sides of a thick wall - don't merge
        if (perpendicular_distance > max_wall_thickness) {
            RCLCPP_DEBUG(this->get_logger(), "Rejecting merge: perpendicular distance %.3f > max wall thickness %.3f", 
                        perpendicular_distance, max_wall_thickness);
            return false;
        }
        
        // If segments are very close perpendicular to their direction,
        // they're likely on the same wall edge - merge them
        if (perpendicular_distance < min_wall_thickness) {
            return true;
        }
        
        // For intermediate distances, be more conservative
        // Check if the segments are roughly aligned (not offset significantly)
        double alignment_score = calculateAlignmentScore(seg1, seg2);
        if (alignment_score < 0.7) { // Threshold for good alignment
            RCLCPP_DEBUG(this->get_logger(), "Rejecting merge: poor alignment score %.3f", alignment_score);
            return false;
        }
        
        return true;
    }

    double perpendicularDistanceBetweenSegments(const WallSegment& seg1, const WallSegment& seg2) {
        // Calculate the perpendicular distance between two roughly parallel segments
        
        // Get the direction vector of seg1
        double dx1 = seg1.end_point.x - seg1.start_point.x;
        double dy1 = seg1.end_point.y - seg1.start_point.y;
        double len1 = std::sqrt(dx1*dx1 + dy1*dy1);
        
        if (len1 == 0) return distance_tolerance_ + 1; // Invalid segment
        
        // Normalize direction vector
        dx1 /= len1;
        dy1 /= len1;
        
        // Calculate perpendicular vector
        double perp_x = -dy1;
        double perp_y = dx1;
        
        // Get vector from seg1 start to seg2 start
        double vec_x = seg2.start_point.x - seg1.start_point.x;
        double vec_y = seg2.start_point.y - seg1.start_point.y;
        
        // Project onto perpendicular direction
        double perp_distance = std::abs(vec_x * perp_x + vec_y * perp_y);
        
        return perp_distance;
    }

    double calculateAlignmentScore(const WallSegment& seg1, const WallSegment& seg2) {
        // Calculate how well two segments are aligned (0 = poor, 1 = perfect)
        
        // Check overlap in the direction of the segments
        double overlap_score = calculateOverlapScore(seg1, seg2);
        
        // Check parallelism
        double angle_diff = std::abs(seg1.angle - seg2.angle);
        if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
        double parallelism_score = 1.0 - (angle_diff / angle_tolerance_);
        
        // Check collinearity
        double collinearity_distance = distanceToLine(seg1, seg2);
        double collinearity_score = 1.0 - (collinearity_distance / distance_tolerance_);
        
        // Combined score
        return (overlap_score * 0.4 + parallelism_score * 0.3 + collinearity_score * 0.3);
    }

    double calculateOverlapScore(const WallSegment& seg1, const WallSegment& seg2) {
        // Project both segments onto the direction of seg1 and calculate overlap
        
        double dx = seg1.end_point.x - seg1.start_point.x;
        double dy = seg1.end_point.y - seg1.start_point.y;
        double len = std::sqrt(dx*dx + dy*dy);
        
        if (len == 0) return 0;
        
        // Normalize direction
        dx /= len;
        dy /= len;
        
        // Project all points onto this direction
        double seg1_start_proj = 0; // seg1 start is our reference
        double seg1_end_proj = len;
        
        double seg2_start_proj = (seg2.start_point.x - seg1.start_point.x) * dx + 
                                (seg2.start_point.y - seg1.start_point.y) * dy;
        double seg2_end_proj = (seg2.end_point.x - seg1.start_point.x) * dx + 
                              (seg2.end_point.y - seg1.start_point.y) * dy;
        
        // Ensure seg2 projections are ordered
        if (seg2_start_proj > seg2_end_proj) {
            std::swap(seg2_start_proj, seg2_end_proj);
        }
        
        // Calculate overlap
        double overlap_start = std::max(seg1_start_proj, seg2_start_proj);
        double overlap_end = std::min(seg1_end_proj, seg2_end_proj);
        double overlap_length = std::max(0.0, overlap_end - overlap_start);
        
        // Calculate total span
        double total_start = std::min(seg1_start_proj, seg2_start_proj);
        double total_end = std::max(seg1_end_proj, seg2_end_proj);
        double total_length = total_end - total_start;
        
        if (total_length == 0) return 0;
        
        return overlap_length / total_length;
    }

    WallSegment fitLineToSegmentsRobust(const std::vector<WallSegment>& segments) {
        // More robust line fitting using RANSAC-like approach
        if (segments.empty()) {
            return WallSegment(); // Empty segment
        }
        
        if (segments.size() == 1) {
            return segments[0];
        }
        
        // Collect all endpoints
        std::vector<geometry_msgs::msg::Point> points;
        for (const auto& seg : segments) {
            points.push_back(seg.start_point);
            points.push_back(seg.end_point);
        }
        
        // Find the two points that are farthest apart
        geometry_msgs::msg::Point best_start, best_end;
        double max_distance = 0;
        
        for (size_t i = 0; i < points.size(); i++) {
            for (size_t j = i + 1; j < points.size(); j++) {
                double distance = pointToPointDistance(points[i], points[j]);
                if (distance > max_distance) {
                    max_distance = distance;
                    best_start = points[i];
                    best_end = points[j];
                }
            }
        }
        
        // Create the fitted wall segment
        WallSegment fitted;
        fitted.start_point = best_start;
        fitted.end_point = best_end;
        fitted.length = max_distance;
        fitted.angle = std::atan2(best_end.y - best_start.y, best_end.x - best_start.x);
        fitted.room_name = "undefined";
        fitted.wall_name = "undefined";
        
        return fitted;
    }

    std::vector<WallSegment> coalesceForHandDrawnMaps(const std::vector<WallSegment>& segments) {
        RCLCPP_INFO(this->get_logger(), "Starting hand-drawn map coalescing with %zu segments", segments.size());
        
        std::vector<WallSegment> coalesced;
        std::vector<bool> used(segments.size(), false);
        
        for (size_t i = 0; i < segments.size(); i++) {
            if (used[i]) continue;
            
            std::vector<WallSegment> cluster;
            cluster.push_back(segments[i]);
            used[i] = true;
            
            // For hand-drawn maps, be very aggressive about merging segments
            // that are roughly aligned and close together
            bool found_merge = true;
            while (found_merge) {
                found_merge = false;
                
                for (size_t j = 0; j < segments.size(); j++) {
                    if (used[j]) continue;
                    
                    // Check if this segment can be merged with any segment in the cluster
                    bool can_merge = false;
                    for (const auto& cluster_seg : cluster) {
                        if (canMergeHandDrawn(cluster_seg, segments[j])) {
                            can_merge = true;
                            break;
                        }
                    }
                    
                    if (can_merge) {
                        cluster.push_back(segments[j]);
                        used[j] = true;
                        found_merge = true;
                    }
                }
            }
            
            // Fit a single line to all segments in the cluster
            if (cluster.size() > 1) {
                WallSegment fitted_line = fitLineToCluster(cluster);
                coalesced.push_back(fitted_line);
                RCLCPP_INFO(this->get_logger(), "Merged %zu segments into single wall line (length: %.2fm)", 
                           cluster.size(), fitted_line.length);
            } else {
                coalesced.push_back(cluster[0]);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Hand-drawn coalescing: %zu segments -> %zu walls", segments.size(), coalesced.size());
        return coalesced;
    }

    bool canMergeHandDrawn(const WallSegment& seg1, const WallSegment& seg2) {
        // For hand-drawn maps, be very permissive about angles
        // Check if angles are similar (within 5 degrees for hand-drawn)
        double angle_diff = std::abs(seg1.angle - seg2.angle);
        if (angle_diff > M_PI) {
            angle_diff = 2 * M_PI - angle_diff;
        }
        
        // Allow up to 10 degrees difference for hand-drawn maps
        double max_angle_diff = 0.174533; // 10 degrees in radians
        if (angle_diff > max_angle_diff) {
            return false;
        }
        
        // Check if segments are roughly on the same line (collinear)
        double collinearity_distance = distanceToLine(seg1, seg2);
        if (collinearity_distance > 0.3) { // 30cm tolerance for hand-drawn
            return false;
        }
        
        // Check if segments are close enough (be very permissive)
        double endpoint_distance = minEndpointDistance(seg1, seg2);
        if (endpoint_distance > 1.0) { // 1 meter tolerance for gaps
            return false;
        }
        
        // Check if segments overlap or are close in their direction
        double projection_gap = calculateProjectionGap(seg1, seg2);
        if (projection_gap > 0.5) { // 50cm gap tolerance
            return false;
        }
        
        return true;
    }

    double calculateProjectionGap(const WallSegment& seg1, const WallSegment& seg2) {
        // Project both segments onto the direction of seg1 and find the gap
        double dx = seg1.end_point.x - seg1.start_point.x;
        double dy = seg1.end_point.y - seg1.start_point.y;
        double len = std::sqrt(dx*dx + dy*dy);
        
        if (len == 0) return 1000; // Invalid segment
        
        // Normalize direction
        dx /= len;
        dy /= len;
        
        // Project all points onto this direction
        double seg1_start_proj = 0; // seg1 start is our reference
        double seg1_end_proj = len;
        
        double seg2_start_proj = (seg2.start_point.x - seg1.start_point.x) * dx + 
                                (seg2.start_point.y - seg1.start_point.y) * dy;
        double seg2_end_proj = (seg2.end_point.x - seg1.start_point.x) * dx + 
                              (seg2.end_point.y - seg1.start_point.y) * dy;
        
        // Ensure seg2 projections are ordered
        if (seg2_start_proj > seg2_end_proj) {
            std::swap(seg2_start_proj, seg2_end_proj);
        }
        
        // Calculate gap between the segments
        double gap = 0;
        if (seg2_start_proj > seg1_end_proj) {
            // seg2 is after seg1
            gap = seg2_start_proj - seg1_end_proj;
        } else if (seg1_start_proj > seg2_end_proj) {
            // seg1 is after seg2
            gap = seg1_start_proj - seg2_end_proj;
        }
        // If gap is 0, segments overlap or touch
        
        return gap;
    }

    WallSegment fitLineToCluster(const std::vector<WallSegment>& segments) {
        if (segments.empty()) {
            return WallSegment();
        }
        
        if (segments.size() == 1) {
            return segments[0];
        }
        
        // Collect all endpoints
        std::vector<geometry_msgs::msg::Point> points;
        for (const auto& seg : segments) {
            points.push_back(seg.start_point);
            points.push_back(seg.end_point);
        }
        
        // Find the overall span by projecting all points onto the average direction
        double avg_angle = 0;
        for (const auto& seg : segments) {
            avg_angle += seg.angle;
        }
        avg_angle /= segments.size();
        
        // Project all points onto this average direction
        double min_proj = std::numeric_limits<double>::max();
        double max_proj = std::numeric_limits<double>::lowest();
        geometry_msgs::msg::Point ref_point = points[0];
        
        double cos_angle = std::cos(avg_angle);
        double sin_angle = std::sin(avg_angle);
        
        for (const auto& point : points) {
            double dx = point.x - ref_point.x;
            double dy = point.y - ref_point.y;
            double projection = dx * cos_angle + dy * sin_angle;
            
            min_proj = std::min(min_proj, projection);
            max_proj = std::max(max_proj, projection);
        }
        
        // Create the fitted line endpoints
        WallSegment fitted;
        fitted.start_point.x = ref_point.x + min_proj * cos_angle;
        fitted.start_point.y = ref_point.y + min_proj * sin_angle;
        fitted.start_point.z = 0;
        
        fitted.end_point.x = ref_point.x + max_proj * cos_angle;
        fitted.end_point.y = ref_point.y + max_proj * sin_angle;
        fitted.end_point.z = 0;
        
        fitted.length = max_proj - min_proj;
        fitted.angle = avg_angle;
        fitted.room_name = "undefined";
        fitted.wall_name = "undefined";
        
        return fitted;
    }

    double minEndpointDistance(const WallSegment& seg1, const WallSegment& seg2) {
        // Calculate minimum distance between endpoints of two line segments
        double d1 = pointToPointDistance(seg1.start_point, seg2.start_point);
        double d2 = pointToPointDistance(seg1.start_point, seg2.end_point);
        double d3 = pointToPointDistance(seg1.end_point, seg2.start_point);
        double d4 = pointToPointDistance(seg1.end_point, seg2.end_point);
        
        return std::min({d1, d2, d3, d4});
    }

    double distanceToLine(const WallSegment& seg1, const WallSegment& seg2) {
        // Calculate distance from seg2 endpoints to the line formed by seg1
        double dist1 = pointToLineDistance(seg2.start_point, seg1.start_point, seg1.end_point);
        double dist2 = pointToLineDistance(seg2.end_point, seg1.start_point, seg1.end_point);
        return std::max(dist1, dist2);
    }

    double pointToLineDistance(const geometry_msgs::msg::Point& point, 
                               const geometry_msgs::msg::Point& line_start,
                               const geometry_msgs::msg::Point& line_end) {
        // Calculate distance from point to line segment
        double A = point.x - line_start.x;
        double B = point.y - line_start.y;
        double C = line_end.x - line_start.x;
        double D = line_end.y - line_start.y;
        
        double dot = A * C + B * D;
        double len_sq = C * C + D * D;
        
        if (len_sq == 0) {
            return std::sqrt(A * A + B * B);
        }
        
        double param = dot / len_sq;
        
        double xx, yy;
        if (param < 0) {
            xx = line_start.x;
            yy = line_start.y;
        } else if (param > 1) {
            xx = line_end.x;
            yy = line_end.y;
        } else {
            xx = line_start.x + param * C;
            yy = line_start.y + param * D;
        }
        
        double dx = point.x - xx;
        double dy = point.y - yy;
        return std::sqrt(dx * dx + dy * dy);
    }

    double pointToPointDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return std::sqrt(dx * dx + dy * dy);
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

    void publishWallsCostmap(const std::vector<WallSegment>& walls, 
                             const nav_msgs::msg::OccupancyGrid::SharedPtr original_msg) {
        if (!publish_walls_costmap_ || !walls_costmap_publisher_) {
            return;
        }
        
        // Create a new occupancy grid message
        stored_walls_costmap_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        
        // Copy header and info from original message
        stored_walls_costmap_->header = original_msg->header;
        stored_walls_costmap_->info = original_msg->info;
        
        // Initialize data with free space (0)
        stored_walls_costmap_->data.resize(original_msg->info.width * original_msg->info.height, 0);
        
        // Draw walls as occupied cells (100)
        for (const auto& wall : walls) {
            drawLineOnGrid(stored_walls_costmap_, wall.start_point, wall.end_point);
        }
        
        // Publish the costmap immediately
        walls_costmap_publisher_->publish(*stored_walls_costmap_);
        
        RCLCPP_INFO(this->get_logger(), "Published walls costmap with %zu walls", walls.size());
    }
    
    void publishStoredWallsCostmap() {
        if (!publish_walls_costmap_ || !walls_costmap_publisher_ || !stored_walls_costmap_) {
            return;
        }
        
        // Update timestamp
        stored_walls_costmap_->header.stamp = this->get_clock()->now();
        
        // Publish the stored costmap
        walls_costmap_publisher_->publish(*stored_walls_costmap_);
    }
    
    void drawLineOnGrid(std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid,
                        const geometry_msgs::msg::Point& start,
                        const geometry_msgs::msg::Point& end) {
        // Convert world coordinates to grid coordinates
        int start_x = static_cast<int>((start.x - grid->info.origin.position.x) / grid->info.resolution);
        int start_y = static_cast<int>((start.y - grid->info.origin.position.y) / grid->info.resolution);
        int end_x = static_cast<int>((end.x - grid->info.origin.position.x) / grid->info.resolution);
        int end_y = static_cast<int>((end.y - grid->info.origin.position.y) / grid->info.resolution);
        
        // Use Bresenham's line algorithm to draw the line
        int dx = std::abs(end_x - start_x);
        int dy = std::abs(end_y - start_y);
        int sx = (start_x < end_x) ? 1 : -1;
        int sy = (start_y < end_y) ? 1 : -1;
        int err = dx - dy;
        
        int x = start_x;
        int y = start_y;
        
        while (true) {
            // Check bounds and set pixel
            if (x >= 0 && x < static_cast<int>(grid->info.width) && 
                y >= 0 && y < static_cast<int>(grid->info.height)) {
                int index = y * grid->info.width + x;
                grid->data[index] = 100; // Mark as occupied
            }
            
            if (x == end_x && y == end_y) break;
            
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x += sx;
            }
            if (e2 < dx) {
                err += dx;
                y += sy;
            }
        }
    }

    // Member variables
    std::string config_file_path_;
    std::string costmap_topic_;
    int costmap_threshold_;
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
    double proximity_tolerance_;
    std::string database_path_;
    std::string map_frame_;
    bool publish_walls_costmap_;
    
    sqlite3* db_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr walls_costmap_publisher_;
    rclcpp::TimerBase::SharedPtr walls_costmap_timer_;
    
    // Store the walls costmap for continuous publishing
    nav_msgs::msg::OccupancyGrid::SharedPtr stored_walls_costmap_;
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
