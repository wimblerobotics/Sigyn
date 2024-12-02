#include <ldlidar_node.h>

LD06::LD06()
  : Node("ld06_node")
{
  std::string topic_name = this->declare_parameter("topic_name", "scan");
  std::string port_name = this->declare_parameter("serial_port", ""); //TODO: Figure out what's the real port name
  std::string lidar_frame = this->declare_parameter("lidar_frame", "laser");
  float range_threshold = this->declare_parameter("range_threshold", 0.2);

  lidar_ = new LiPkg;
  lidar_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, 10);

  lidar_->SetLidarFrame(lidar_frame);
  lidar_->SetRangeThreshold(range_threshold);
  if (port_name.empty())
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Autodetecting serial port");
    std::vector<std::pair<std::string, std::string>> device_list;
    cmd_port_.GetCmdDevices(device_list);
    auto found = std::find_if(
      device_list.begin(),
      device_list.end(),
      [](std::pair<std::string, std::string> n)
      { return strstr(n.second.c_str(), "CP2102"); }
    );

    if (found != device_list.end())
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s %s", found->first.c_str(), found->second.c_str());
      port_name = found->first;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Can't find LiDAR LD06");
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Using port %s", port_name.c_str());

	cmd_port_.SetReadCallback([this](const char *byte, size_t len) {
		if(lidar_->Parse((uint8_t*)byte, len))
		{
			lidar_->AssemblePacket();
		}
	});

  if(cmd_port_.Open(port_name))
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LiDAR_LD06 started successfully");
  } 
  else 
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Can't open the serial port");
  }

  loop_timer_ = this->create_wall_timer(
    100ms, 
    std::bind(&LD06::publishLoop, this)
  );
}

void LD06::publishLoop()
{
  if (lidar_->IsFrameReady())
  {
    lidar_pub_->publish(lidar_->GetLaserScan());
    lidar_->ResetFrameReady();
  }
}