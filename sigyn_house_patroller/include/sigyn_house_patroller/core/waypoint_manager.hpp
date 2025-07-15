#pragma once

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "sigyn_house_patroller/msg/room_identification.hpp"

namespace sigyn_house_patroller {

/**
 * @brief Structure representing a patrol waypoint
 */
struct PatrolWaypoint {
  std::string id;
  geometry_msgs::msg::PoseStamped pose;
  std::string room_name;
  std::string waypoint_type;  // "navigation", "inspection", "sensor_check"
  double priority;
  std::chrono::seconds dwell_time;
  std::vector<std::string> required_sensors;
  std::unordered_map<std::string, std::string> metadata;
  
  PatrolWaypoint() : priority(1.0), dwell_time(std::chrono::seconds(5)) {}
};

/**
 * @brief Room information structure
 */
struct RoomInfo {
  std::string name;
  std::string type;  // "bedroom", "kitchen", "living_room", "bathroom", etc.
  geometry_msgs::msg::Point center;
  double area;
  std::vector<PatrolWaypoint> waypoints;
  std::vector<std::string> typical_threats;
  std::unordered_map<std::string, double> normal_conditions;  // temp, humidity, etc.
  bool is_accessible;
  std::chrono::system_clock::time_point last_visited;
  
  RoomInfo() : area(0.0), is_accessible(true) {}
};

/**
 * @brief Waypoint management and navigation coordination
 */
class WaypointManager {
public:
  explicit WaypointManager(rclcpp::Node::SharedPtr node);
  ~WaypointManager() = default;

  /**
   * @brief Initialize the waypoint manager
   * @return True if initialization successful
   */
  bool Initialize();

  /**
   * @brief Load waypoints from configuration
   * @param config_file Path to waypoint configuration file
   * @return True if loaded successfully
   */
  bool LoadWaypoints(const std::string& config_file);

  /**
   * @brief Save current waypoints to configuration
   * @param config_file Path to save waypoints to
   * @return True if saved successfully
   */
  bool SaveWaypoints(const std::string& config_file);

  /**
   * @brief Get all waypoints
   * @return Vector of all patrol waypoints
   */
  std::vector<PatrolWaypoint> GetAllWaypoints() const;

  /**
   * @brief Get waypoints for a specific room
   * @param room_name Name of the room
   * @return Vector of waypoints in the room
   */
  std::vector<PatrolWaypoint> GetWaypointsForRoom(const std::string& room_name) const;

  /**
   * @brief Get waypoints by type
   * @param waypoint_type Type of waypoint ("navigation", "inspection", etc.)
   * @return Vector of waypoints of the specified type
   */
  std::vector<PatrolWaypoint> GetWaypointsByType(const std::string& waypoint_type) const;

  /**
   * @brief Add a new waypoint
   * @param waypoint Waypoint to add
   * @return True if added successfully
   */
  bool AddWaypoint(const PatrolWaypoint& waypoint);

  /**
   * @brief Update an existing waypoint
   * @param waypoint_id ID of waypoint to update
   * @param waypoint Updated waypoint data
   * @return True if updated successfully
   */
  bool UpdateWaypoint(const std::string& waypoint_id, const PatrolWaypoint& waypoint);

  /**
   * @brief Remove a waypoint
   * @param waypoint_id ID of waypoint to remove
   * @return True if removed successfully
   */
  bool RemoveWaypoint(const std::string& waypoint_id);

  /**
   * @brief Find waypoint by ID
   * @param waypoint_id ID of waypoint to find
   * @return Pointer to waypoint if found, nullptr otherwise
   */
  const PatrolWaypoint* FindWaypoint(const std::string& waypoint_id) const;

  /**
   * @brief Get next waypoint in patrol sequence
   * @param current_waypoint_id ID of current waypoint
   * @return Next waypoint to visit
   */
  PatrolWaypoint GetNextWaypoint(const std::string& current_waypoint_id) const;

  /**
   * @brief Get optimal waypoint sequence for full house patrol
   * @param start_pose Starting position
   * @return Ordered sequence of waypoints
   */
  std::vector<PatrolWaypoint> GetOptimalPatrolSequence(
    const geometry_msgs::msg::PoseStamped& start_pose) const;

  /**
   * @brief Calculate distance between two waypoints
   * @param wp1 First waypoint
   * @param wp2 Second waypoint
   * @return Distance in meters
   */
  double CalculateDistance(const PatrolWaypoint& wp1, const PatrolWaypoint& wp2) const;

  /**
   * @brief Load room information from configuration
   * @param config_file Path to room configuration file
   * @return True if loaded successfully
   */
  bool LoadRoomInfo(const std::string& config_file);

  /**
   * @brief Get information about a specific room
   * @param room_name Name of the room
   * @return Pointer to room info if found, nullptr otherwise
   */
  const RoomInfo* GetRoomInfo(const std::string& room_name) const;

  /**
   * @brief Get all room information
   * @return Map of room names to room info
   */
  std::unordered_map<std::string, RoomInfo> GetAllRooms() const;

  /**
   * @brief Identify current room based on position
   * @param pose Current robot pose
   * @return Room identification result
   */
  msg::RoomIdentification IdentifyCurrentRoom(const geometry_msgs::msg::PoseStamped& pose) const;

  /**
   * @brief Update room visit time
   * @param room_name Name of the room
   * @param visit_time Time of visit
   */
  void UpdateRoomVisitTime(const std::string& room_name, 
                          const std::chrono::system_clock::time_point& visit_time);

  /**
   * @brief Get rooms that haven't been visited recently
   * @param max_age Maximum age since last visit
   * @return List of room names needing visits
   */
  std::vector<std::string> GetRoomsNeedingVisit(const std::chrono::seconds& max_age) const;

  /**
   * @brief Check if a waypoint is reachable
   * @param waypoint Waypoint to check
   * @return True if waypoint is reachable
   */
  bool IsWaypointReachable(const PatrolWaypoint& waypoint) const;

  /**
   * @brief Get current robot pose
   * @return Current pose if available
   */
  geometry_msgs::msg::PoseStamped GetCurrentPose() const;

  /**
   * @brief Transform pose to map frame
   * @param pose Pose to transform
   * @return Transformed pose
   */
  geometry_msgs::msg::PoseStamped TransformToMapFrame(
    const geometry_msgs::msg::PoseStamped& pose) const;

private:
  /**
   * @brief Optimize waypoint sequence using TSP-like algorithm
   * @param waypoints Waypoints to optimize
   * @param start_pose Starting position
   * @return Optimized sequence
   */
  std::vector<PatrolWaypoint> OptimizeWaypointSequence(
    const std::vector<PatrolWaypoint>& waypoints,
    const geometry_msgs::msg::PoseStamped& start_pose) const;

  /**
   * @brief Load waypoints from YAML configuration
   * @param config_file Path to configuration file
   * @return True if loaded successfully
   */
  bool LoadWaypointsFromYAML(const std::string& config_file);

  /**
   * @brief Save waypoints to YAML configuration
   * @param config_file Path to configuration file
   * @return True if saved successfully
   */
  bool SaveWaypointsToYAML(const std::string& config_file) const;

  /**
   * @brief Generate waypoint ID
   * @return Unique waypoint ID
   */
  std::string GenerateWaypointId() const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  std::vector<PatrolWaypoint> waypoints_;
  std::unordered_map<std::string, RoomInfo> rooms_;
  
  mutable std::mutex waypoints_mutex_;
  mutable std::mutex rooms_mutex_;
  
  // Configuration parameters
  std::string map_frame_;
  std::string robot_frame_;
  double waypoint_tolerance_;
  double room_boundary_tolerance_;
};

}  // namespace sigyn_house_patroller
