#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "sigyn_house_patroller/msg/threat_alert.hpp"
#include "sigyn_house_patroller/msg/system_health.hpp"
#include "sigyn_house_patroller/msg/room_identification.hpp"

namespace sigyn_house_patroller {

class ChangeDetectorNode : public rclcpp::Node {
public:
  explicit ChangeDetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("change_detector", options),
        last_pointcloud_update_(std::chrono::steady_clock::now()),
        last_image_update_(std::chrono::steady_clock::now()) {
    
    // Declare parameters
    declare_parameter("monitoring_frequency", 0.5);
    declare_parameter("pointcloud_timeout", 60.0);
    declare_parameter("image_timeout", 60.0);
    declare_parameter("pointcloud_topic", "/camera/depth/points");
    declare_parameter("image_topic", "/camera/color/image_raw");
    declare_parameter("change_threshold", 0.3);
    declare_parameter("alert_cooldown", 180.0);  // 3 minutes
    declare_parameter("enable_visual_diff", true);
    declare_parameter("enable_3d_diff", true);
    declare_parameter("reference_update_interval", 600.0);  // 10 minutes
    declare_parameter("min_change_area", 0.01);  // 1% of image
    declare_parameter("max_change_area", 0.5);   // 50% of image
    declare_parameter("depth_change_threshold", 0.05);  // 5cm
    declare_parameter("point_density_threshold", 0.1);
    
    // Get parameters
    monitoring_frequency_ = get_parameter("monitoring_frequency").as_double();
    pointcloud_timeout_ = get_parameter("pointcloud_timeout").as_double();
    image_timeout_ = get_parameter("image_timeout").as_double();
    pointcloud_topic_ = get_parameter("pointcloud_topic").as_string();
    image_topic_ = get_parameter("image_topic").as_string();
    change_threshold_ = get_parameter("change_threshold").as_double();
    alert_cooldown_ = get_parameter("alert_cooldown").as_double();
    enable_visual_diff_ = get_parameter("enable_visual_diff").as_bool();
    enable_3d_diff_ = get_parameter("enable_3d_diff").as_bool();
    reference_update_interval_ = get_parameter("reference_update_interval").as_double();
    min_change_area_ = get_parameter("min_change_area").as_double();
    max_change_area_ = get_parameter("max_change_area").as_double();
    depth_change_threshold_ = get_parameter("depth_change_threshold").as_double();
    point_density_threshold_ = get_parameter("point_density_threshold").as_double();
    
    // Initialize state
    pointcloud_healthy_ = true;
    image_healthy_ = true;
    current_room_ = "unknown";
    
    // Publishers
    threat_alert_pub_ = create_publisher<msg::ThreatAlert>(
      "~/threat_alerts", rclcpp::QoS(10).reliable());
    
    health_pub_ = create_publisher<msg::SystemHealth>(
      "~/health", rclcpp::QoS(10).reliable());
    
    change_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "~/change_visualization", rclcpp::QoS(10).reliable());
    
    // Subscribers
    if (enable_3d_diff_) {
      pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_, rclcpp::QoS(10).reliable(),
        std::bind(&ChangeDetectorNode::PointCloudCallback, this, std::placeholders::_1));
    }
    
    if (enable_visual_diff_) {
      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, rclcpp::QoS(10).reliable(),
        std::bind(&ChangeDetectorNode::ImageCallback, this, std::placeholders::_1));
    }
    
    room_id_sub_ = create_subscription<msg::RoomIdentification>(
      "/sigyn_house_patroller/room_identification", rclcpp::QoS(10).reliable(),
      std::bind(&ChangeDetectorNode::RoomIdentificationCallback, this, std::placeholders::_1));
    
    // Timers
    monitoring_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / monitoring_frequency_)),
      std::bind(&ChangeDetectorNode::MonitoringTimerCallback, this));
    
    health_timer_ = create_wall_timer(
      std::chrono::seconds(10),
      std::bind(&ChangeDetectorNode::HealthTimerCallback, this));
    
    reference_update_timer_ = create_wall_timer(
      std::chrono::seconds(static_cast<int>(reference_update_interval_)),
      std::bind(&ChangeDetectorNode::UpdateReferenceModels, this));
    
    RCLCPP_INFO(get_logger(), "Change detector started - Visual: %s, 3D: %s", 
                enable_visual_diff_ ? "enabled" : "disabled",
                enable_3d_diff_ ? "enabled" : "disabled");
  }

private:
  void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    current_pointcloud_ = msg;
    last_pointcloud_update_ = std::chrono::steady_clock::now();
    
    // Check for 3D changes
    if (enable_3d_diff_) {
      CheckPointCloudChanges();
    }
  }
  
  void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    current_image_ = msg;
    last_image_update_ = std::chrono::steady_clock::now();
    
    // Check for visual changes
    if (enable_visual_diff_) {
      CheckImageChanges();
    }
  }
  
  void RoomIdentificationCallback(const msg::RoomIdentification::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (msg->confidence > 0.7) {
      std::string new_room = msg->room_name;
      
      // If room changed, update reference models
      if (new_room != current_room_) {
        current_room_ = new_room;
        RCLCPP_INFO(get_logger(), "Room changed to: %s", current_room_.c_str());
        
        // Reset references for new room
        LoadReferenceModels(current_room_);
      }
    }
  }
  
  void MonitoringTimerCallback() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    auto now = std::chrono::steady_clock::now();
    
    // Check pointcloud health
    if (enable_3d_diff_) {
      auto pc_time_since_update = std::chrono::duration_cast<std::chrono::seconds>(
        now - last_pointcloud_update_).count();
      
      if (pc_time_since_update > pointcloud_timeout_) {
        pointcloud_healthy_ = false;
        
        if (pc_time_since_update > pointcloud_timeout_ * 2) {  // Only alert after 2x timeout
          SendCommunicationAlert("pointcloud", pc_time_since_update);
        }
      } else {
        pointcloud_healthy_ = true;
      }
    }
    
    // Check image health
    if (enable_visual_diff_) {
      auto img_time_since_update = std::chrono::duration_cast<std::chrono::seconds>(
        now - last_image_update_).count();
      
      if (img_time_since_update > image_timeout_) {
        image_healthy_ = false;
        
        if (img_time_since_update > image_timeout_ * 2) {  // Only alert after 2x timeout
          SendCommunicationAlert("image", img_time_since_update);
        }
      } else {
        image_healthy_ = true;
      }
    }
  }
  
  void CheckPointCloudChanges() {
    if (!current_pointcloud_ || current_room_ == "unknown") {
      return;
    }
    
    // Get reference point cloud for current room
    auto ref_it = room_reference_clouds_.find(current_room_);
    if (ref_it == room_reference_clouds_.end()) {
      // No reference model yet, create one
      CreateReferencePointCloud(current_room_);
      return;
    }
    
    // Convert ROS messages to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::fromROSMsg(*current_pointcloud_, *current_cloud);
    pcl::fromROSMsg(ref_it->second, *reference_cloud);
    
    if (current_cloud->empty() || reference_cloud->empty()) {
      return;
    }
    
    // Calculate change metrics
    double change_score = CalculatePointCloudChange(current_cloud, reference_cloud);
    
    if (change_score > change_threshold_) {
      // Check alert cooldown
      auto now = std::chrono::steady_clock::now();
      if (IsAlertCooldownActive(current_room_ + "_3d_change", now)) {
        return;
      }
      
      // Send change alert
      SendChangeAlert("3d_environment_change", change_score, 
                     "Significant 3D environment change detected in " + current_room_,
                     "{\"change_score\": " + std::to_string(change_score) + 
                     ", \"threshold\": " + std::to_string(change_threshold_) + 
                     ", \"room\": \"" + current_room_ + "\"}");
      
      UpdateAlertCooldown(current_room_ + "_3d_change", now);
      
      RCLCPP_WARN(get_logger(), "3D change detected in %s: %.2f", 
                  current_room_.c_str(), change_score);
    }
  }
  
  void CheckImageChanges() {
    if (!current_image_ || current_room_ == "unknown") {
      return;
    }
    
    // Get reference image for current room
    auto ref_it = room_reference_images_.find(current_room_);
    if (ref_it == room_reference_images_.end()) {
      // No reference model yet, create one
      CreateReferenceImage(current_room_);
      return;
    }
    
    // Convert ROS messages to OpenCV
    cv_bridge::CvImagePtr current_cv, reference_cv;
    
    try {
      current_cv = cv_bridge::toCvCopy(current_image_, sensor_msgs::image_encodings::BGR8);
      reference_cv = cv_bridge::toCvCopy(ref_it->second, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "CV bridge exception: %s", e.what());
      return;
    }
    
    // Calculate visual change
    double change_score = CalculateImageChange(current_cv->image, reference_cv->image);
    
    if (change_score > change_threshold_) {
      // Check alert cooldown
      auto now = std::chrono::steady_clock::now();
      if (IsAlertCooldownActive(current_room_ + "_visual_change", now)) {
        return;
      }
      
      // Create change visualization
      cv::Mat change_viz = CreateChangeVisualization(current_cv->image, reference_cv->image);
      PublishChangeVisualization(change_viz);
      
      // Send change alert
      SendChangeAlert("visual_environment_change", change_score,
                     "Significant visual change detected in " + current_room_,
                     "{\"change_score\": " + std::to_string(change_score) + 
                     ", \"threshold\": " + std::to_string(change_threshold_) + 
                     ", \"room\": \"" + current_room_ + "\"}");
      
      UpdateAlertCooldown(current_room_ + "_visual_change", now);
      
      RCLCPP_WARN(get_logger(), "Visual change detected in %s: %.2f", 
                  current_room_.c_str(), change_score);
    }
  }
  
  double CalculatePointCloudChange(const pcl::PointCloud<pcl::PointXYZ>::Ptr& current,
                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr& reference) {
    // Simple approach: calculate average distance between point clouds
    // More sophisticated methods could use ICP, volume differences, etc.
    
    if (current->empty() || reference->empty()) {
      return 0.0;
    }
    
    // Use a subset of points for efficiency
    size_t sample_size = std::min(static_cast<size_t>(1000), current->size());
    double total_distance = 0.0;
    int valid_points = 0;
    
    for (size_t i = 0; i < sample_size; ++i) {
      size_t idx = (i * current->size()) / sample_size;
      const auto& current_point = current->points[idx];
      
      // Find nearest point in reference cloud
      double min_distance = std::numeric_limits<double>::max();
      for (const auto& ref_point : reference->points) {
        double distance = std::sqrt(
          std::pow(current_point.x - ref_point.x, 2) +
          std::pow(current_point.y - ref_point.y, 2) +
          std::pow(current_point.z - ref_point.z, 2)
        );
        
        if (distance < min_distance) {
          min_distance = distance;
        }
      }
      
      if (min_distance < 2.0) {  // Only consider reasonable distances
        total_distance += min_distance;
        valid_points++;
      }
    }
    
    if (valid_points == 0) {
      return 0.0;
    }
    
    double avg_distance = total_distance / valid_points;
    return std::min(1.0, avg_distance / depth_change_threshold_);
  }
  
  double CalculateImageChange(const cv::Mat& current, const cv::Mat& reference) {
    if (current.empty() || reference.empty()) {
      return 0.0;
    }
    
    // Resize images to same size if needed
    cv::Mat current_resized, reference_resized;
    cv::Size target_size(320, 240);  // Downsample for efficiency
    
    cv::resize(current, current_resized, target_size);
    cv::resize(reference, reference_resized, target_size);
    
    // Convert to grayscale
    cv::Mat current_gray, reference_gray;
    cv::cvtColor(current_resized, current_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(reference_resized, reference_gray, cv::COLOR_BGR2GRAY);
    
    // Calculate absolute difference
    cv::Mat diff;
    cv::absdiff(current_gray, reference_gray, diff);
    
    // Apply threshold to get binary difference
    cv::Mat binary_diff;
    cv::threshold(diff, binary_diff, 30, 255, cv::THRESH_BINARY);
    
    // Calculate change percentage
    int total_pixels = binary_diff.rows * binary_diff.cols;
    int changed_pixels = cv::countNonZero(binary_diff);
    
    double change_percentage = static_cast<double>(changed_pixels) / total_pixels;
    
    // Filter out noise and major lighting changes
    if (change_percentage < min_change_area_ || change_percentage > max_change_area_) {
      return 0.0;
    }
    
    return change_percentage;
  }
  
  void UpdateReferenceModels() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (current_room_ != "unknown") {
      RCLCPP_INFO(get_logger(), "Updating reference models for room: %s", current_room_.c_str());
      if (enable_3d_diff_) {
        CreateReferencePointCloud(current_room_);
      }
      if (enable_visual_diff_) {
        CreateReferenceImage(current_room_);
      }
    }
  }

  bool IsAlertCooldownActive(const std::string& alert_key, const std::chrono::steady_clock::time_point& now) {
    auto it = alert_cooldowns_.find(alert_key);
    if (it != alert_cooldowns_.end()) {
      auto time_since_last_alert = std::chrono::duration_cast<std::chrono::seconds>(now - it->second).count();
      if (time_since_last_alert < alert_cooldown_) {
        return true;
      }
    }
    return false;
  }

  void UpdateAlertCooldown(const std::string& alert_key, const std::chrono::steady_clock::time_point& now) {
    alert_cooldowns_[alert_key] = now;
  }

  cv::Mat CreateChangeVisualization(const cv::Mat& current, const cv::Mat& reference) {
    cv::Mat visualization;
    
    // Create side-by-side comparison
    cv::Mat current_resized, reference_resized;
    cv::Size viz_size(160, 120);
    
    cv::resize(current, current_resized, viz_size);
    cv::resize(reference, reference_resized, viz_size);
    
    // Create difference image
    cv::Mat diff;
    cv::absdiff(current_resized, reference_resized, diff);
    
    // Combine images horizontally
    cv::hconcat(current_resized, reference_resized, visualization);
    cv::hconcat(visualization, diff, visualization);
    
    // Add labels
    cv::putText(visualization, "Current", cv::Point(10, 20), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    cv::putText(visualization, "Reference", cv::Point(170, 20), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    cv::putText(visualization, "Difference", cv::Point(330, 20), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    
    return visualization;
  }
  
  void PublishChangeVisualization(const cv::Mat& visualization) {
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", visualization).toImageMsg();
    msg->header.stamp = this->now();
    msg->header.frame_id = "camera_link";
    
    change_image_pub_->publish(*msg);
  }
  
  void CreateReferencePointCloud(const std::string& room) {
    if (current_pointcloud_) {
      room_reference_clouds_[room] = *current_pointcloud_;
      RCLCPP_INFO(get_logger(), "Created reference point cloud for room: %s", room.c_str());
    }
  }
  
  void CreateReferenceImage(const std::string& room) {
    if (current_image_) {
      room_reference_images_[room] = *current_image_;
      RCLCPP_INFO(get_logger(), "Created reference image for room: %s", room.c_str());
    }
  }
  
  void LoadReferenceModels(const std::string& room) {
    // In a real implementation, this would load saved reference models
    // For now, we'll create new ones from current sensor data
    
    auto delay = std::chrono::seconds(2);  // Wait a bit for sensor data
    
    auto timer = create_wall_timer(delay, [this, room]() {
      std::lock_guard<std::mutex> lock(data_mutex_);
      
      if (enable_3d_diff_ && current_pointcloud_) {
        CreateReferencePointCloud(room);
      }
      
      if (enable_visual_diff_ && current_image_) {
        CreateReferenceImage(room);
      }
    });
    
    // Cancel the timer after it fires once
    timer->cancel();
  }
  
  void SendChangeAlert(const std::string& threat_type, double change_score,
                      const std::string& description, const std::string& sensor_data) {
    msg::ThreatAlert alert;
    alert.header.stamp = this->now();
    alert.header.frame_id = "camera_link";
    alert.threat_id = threat_type + "_" + current_room_;
    alert.threat_type = threat_type;
    alert.severity_level = (change_score > change_threshold_ * 2) ? 2 : 1; // 2=WARNING, 1=INFO
    alert.description = description;
    alert.confidence = std::min(1.0, change_score / change_threshold_);
    alert.sensor_data_json = sensor_data;
    
    threat_alert_pub_->publish(alert);
  }

  void SendCommunicationAlert(const std::string& sensor_type, double time_since_update) {
    auto now = std::chrono::steady_clock::now();
    auto& last_alert_time = last_comm_alerts_[sensor_type];
    auto time_since_last_alert = std::chrono::duration_cast<std::chrono::seconds>(now - last_alert_time).count();

    if (time_since_last_alert > alert_cooldown_) {
        msg::ThreatAlert alert;
        alert.header.stamp = this->now();
        alert.threat_id = "comm_loss_" + sensor_type;
        alert.threat_type = "communication_failure";
        alert.severity_level = 3; // CRITICAL
        alert.description = "Communication lost with " + sensor_type + " sensor. No data for " + 
                           std::to_string(time_since_update) + " seconds.";
        alert.confidence = 1.0;
        alert.sensor_data_json = "{\"sensor\": \"" + sensor_type + "\"}";
        
        threat_alert_pub_->publish(alert);
        last_alert_time = now;
    }
  }

  void HealthTimerCallback() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    msg::SystemHealth health_msg;
    health_msg.header.stamp = this->now();
    
    bool is_healthy = pointcloud_healthy_ && image_healthy_;
    
    health_msg.overall_health = is_healthy ? 1 : 2; // 1=HEALTHY, 2=DEGRADED
    health_msg.system_status_description = is_healthy ? "System nominal." : "System degraded.";

    health_msg.component_names.push_back(this->get_name());
    health_msg.component_health_status.push_back(is_healthy ? 1 : 3); // 1=HEALTHY, 3=UNHEALTHY
    health_msg.component_descriptions.push_back(
        "Pointcloud: " + std::string(pointcloud_healthy_ ? "OK" : "FAIL") +
        ", Image: " + std::string(image_healthy_ ? "OK" : "FAIL"));
    health_msg.last_update_times.push_back(this->now());
    
    health_pub_->publish(health_msg);
  }
  
  // Parameters
  double monitoring_frequency_;
  double pointcloud_timeout_;
  double image_timeout_;
  std::string pointcloud_topic_;
  std::string image_topic_;
  double change_threshold_;
  double alert_cooldown_;
  bool enable_visual_diff_;
  bool enable_3d_diff_;
  double reference_update_interval_;
  double min_change_area_;
  double max_change_area_;
  double depth_change_threshold_;
  double point_density_threshold_;
  
  // State
  bool pointcloud_healthy_;
  bool image_healthy_;
  std::string current_room_;
  std::chrono::steady_clock::time_point last_pointcloud_update_;
  std::chrono::steady_clock::time_point last_image_update_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_comm_alerts_;

  // Data
  sensor_msgs::msg::PointCloud2::SharedPtr current_pointcloud_;
  sensor_msgs::msg::Image::SharedPtr current_image_;
  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2> room_reference_clouds_;
  std::unordered_map<std::string, sensor_msgs::msg::Image> room_reference_images_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> alert_cooldowns_;
  
  // ROS2 interfaces
  rclcpp::Publisher<msg::ThreatAlert>::SharedPtr threat_alert_pub_;
  rclcpp::Publisher<msg::SystemHealth>::SharedPtr health_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr change_image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<msg::RoomIdentification>::SharedPtr room_id_sub_;
  rclcpp::TimerBase::SharedPtr monitoring_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  rclcpp::TimerBase::SharedPtr reference_update_timer_;
  
  std::mutex data_mutex_;
};

}  // namespace sigyn_house_patroller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<sigyn_house_patroller::ChangeDetectorNode>();
  
  RCLCPP_INFO(rclcpp::get_logger("change_detector"), "Starting change detector node");
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
