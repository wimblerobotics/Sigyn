// Copyright 2024 Wimble Robotics
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef PERIMETER_ROAMER_V3__BT_NODES_HPP_
#define PERIMETER_ROAMER_V3__BT_NODES_HPP_

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

namespace perimeter_roamer_v3
{

/**
 * @brief Custom behavior tree node to check battery state
 */
class CheckBatteryState : public BT::SyncActionNode
{
public:
  CheckBatteryState(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    // For now, simulate a simple battery check
    // In a real implementation, this would check actual battery status
    
    // Simulate battery level (you would replace this with actual battery reading)
    static int battery_level = 100;
    battery_level -= 1; // Simulate battery drain
    
    if (battery_level <= 20) {
      std::cout << "Battery low (" << battery_level << "%), need to charge!" << std::endl;
      return BT::NodeStatus::FAILURE; // Battery is low, need to charge
    } else {
      std::cout << "Battery OK (" << battery_level << "%)" << std::endl;
      return BT::NodeStatus::SUCCESS; // Battery is fine
    }
  }
};

} // namespace perimeter_roamer_v3

#endif // PERIMETER_ROAMER_V3__BT_NODES_HPP_
