# `this_to_that` ROS 2 Package

This package provides a flexible ROS 2 node (`field_mapper_node`) that subscribes to a topic, extracts data from specified fields of the incoming message, and publishes a new message on a different topic, populating its fields based on the extracted data and static values.

## Requirements

*   **ROS 2 Distribution:** Jazzy Jalisco
*   **Dependencies:**
    *   `rclpy`
    *   `std_msgs`
    *   `sensor_msgs`
    *   `wifi_viz` (External dependency providing `wifi_viz/msg/MinMaxCurr` used in examples)
    *   *(Note: Ensure all dependencies, especially external ones like `wifi_viz`, are properly installed in your workspace or system.)*

## Functionality

The `field_mapper_node` performs the following:

1.  **Subscribes** to an input topic with a specified message type.
2.  **Parses** configuration for:
    *   Field mappings (copying data from an input message field to an output message field).
    *   Static field assignments (setting output message fields to fixed values).
3.  **Creates** an output message of a specified type.
4.  **Populates** the output message based on the configured mappings and static assignments whenever an input message is received.
5.  **Publishes** the populated output message to a specified output topic.

## Configuration Parameters

The node is configured via ROS 2 parameters, typically loaded from a YAML file.

*   `input_topic_name` (string): The topic to subscribe to.
*   `input_message_type` (string): The message type of the input topic (e.g., `sensor_msgs/msg/BatteryState`).
*   `output_topic_name` (string): The topic to publish to.
*   `output_message_type` (string): The message type of the output topic (e.g., `wifi_viz/msg/MinMaxCurr`).
*   `field_mappings_str` (string): A comma-separated string defining direct field copies. Format: `"input_field1:output_field1,input_field2:output_field2,..."`.
*   `static_fields_str` (string): A comma-separated string defining static value assignments. Format: `"output_field1=value1,output_field2=value2,..."`. Values are parsed for basic types (float, int, bool, string).

## Usage

### 1. Creating a Configuration YAML File

You define the specific conversion task by creating a YAML parameter file.

**Template (`my_config.yaml`):**

```yaml
# Use the node name defined in your launch file or code
field_mapper_node:
  ros__parameters:
    input_topic_name: /your/input/topic          # e.g., /imu/data
    input_message_type: package/msg/InputType   # e.g., sensor_msgs/msg/Imu
    output_topic_name: /your/output/topic       # e.g., /simplified_imu
    output_message_type: package/msg/OutputType # e.g., geometry_msgs/msg/Vector3Stamped

    # Comma-separated string: "input_field:output_field,..."
    # Example: Copy linear acceleration x, y, z
    field_mappings_str: "linear_acceleration.x:vector.x, linear_acceleration.y:vector.y, linear_acceleration.z:vector.z, header.stamp:header.stamp"

    # Comma-separated string: "output_field=value,..."
    # Example: Set a static frame_id
    static_fields_str: "header.frame_id=imu_link"
```

Place this file within your package, typically in a `config` directory (e.g., `src/this_to_that/config/my_config.yaml`). Remember to update `setup.py` to install files from this directory if you haven't already.

### 2. Creating a Launch File

To run the node with your configuration, create a Python launch file.

**Template (`my_launch.py`):**

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'this_to_that' # Your package name

    # Construct the full path to your config file
    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'my_config.yaml' # The name of your YAML file
    )

    # Define the node action
    mapper_node = Node(
        package=package_name,
        executable='this_to_that_node.py', # The node script
        name='field_mapper_node', # Must match the top key in your YAML file
        output='screen',
        parameters=[config_file] # Pass the config file path
    )

    return LaunchDescription([
        mapper_node
    ])
```

Place this file within your package, typically in a `launch` directory (e.g., `src/this_to_that/launch/my_launch.py`). Remember to update `setup.py` to install files from this directory.

### 3. Building the Workspace

Navigate to your workspace root (`~/this_to_that_ws`) and build:

```bash
colcon build --packages-select this_to_that
source install/setup.bash
```

### 4. Running the Node

Use `ros2 launch` to execute your launch file:

```bash
ros2 launch this_to_that my_launch.py
```

The node will start, load parameters from `my_config.yaml`, and begin the topic conversion process.

## Included Examples

This package includes example configuration and launch files demonstrating conversions from `sensor_msgs/msg/BatteryState` to `wifi_viz/msg/MinMaxCurr`:

*   `config/battery_voltage.yaml` & `launch/battery_voltage.py`: Maps battery voltage.
*   `config/battery_percentage.yaml` & `launch/battery_percentage.py`: Maps battery percentage.

These are provided as illustrations. You should create your own configuration and launch files for your specific conversion needs following the steps above.

To run an example:

```bash
# Build and source first
ros2 launch this_to_that battery_voltage.py
# or
ros2 launch this_to_that battery_percentage.py
```

