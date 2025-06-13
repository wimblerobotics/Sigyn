# TODO: Add support for nested field access (e.g., header.stamp)
# TODO: Add support for setting complex static types (e.g., ColorRGBA) via params
# TODO: Add option for python expressions for transformation
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rosidl_runtime_py.utilities import get_message as get_message_class
import sys

class FieldMapperNode(Node):
    def __init__(self):
        super().__init__('field_mapper_node')

        # --- Parameter Declarations ---
        self.declare_parameter('input_topic_name', '/input_topic')
        self.declare_parameter('input_message_type', 'std_msgs/msg/String')
        self.declare_parameter('output_topic_name', '/output_topic')
        self.declare_parameter('output_message_type', 'std_msgs/msg/String')

        # Use string parameter for mappings
        self.declare_parameter('field_mappings_str', '')
        self.get_logger().info(f"Declared parameter 'field_mappings_str' as STRING")

        # Use string parameter for static fields
        self.declare_parameter('static_fields_str', '') # Default to empty string
        self.get_logger().info(f"Declared parameter 'static_fields_str' as STRING")


        # --- Get Parameters ---
        self.input_topic_name = self.get_parameter('input_topic_name').get_parameter_value().string_value
        self.input_message_type_str = self.get_parameter('input_message_type').get_parameter_value().string_value
        self.output_topic_name = self.get_parameter('output_topic_name').get_parameter_value().string_value
        self.output_message_type_str = self.get_parameter('output_message_type').get_parameter_value().string_value

        # Get the string parameters
        field_mapping_config_str = self.get_parameter('field_mappings_str').get_parameter_value().string_value
        self.get_logger().info(f"Loaded field_mappings_str: '{field_mapping_config_str}'")

        static_fields_config_str = self.get_parameter('static_fields_str').get_parameter_value().string_value
        self.get_logger().info(f"Loaded static_fields_str: '{static_fields_config_str}'")


        # --- Parse Mappings & Static Fields ---
        # Pass the config strings to the updated parsing functions
        self.parsed_mappings = self._parse_mappings_from_string(field_mapping_config_str)
        self.parsed_static_fields = self._parse_static_fields_from_string(static_fields_config_str) # Use new function

        # --- Initialization State ---
        self.subscription = None
        self.publisher = None
        self.output_msg_type = None
        init_ok = True

        # Check parsing results (allow empty lists, but not None which indicates format errors)
        if self.parsed_mappings is None:
            self.get_logger().error("Failed to parse 'field_mappings' due to format errors.")
            init_ok = False
        if self.parsed_static_fields is None:
            self.get_logger().error("Failed to parse 'static_fields' due to format errors.")
            init_ok = False

        if not init_ok:
            self.get_logger().fatal("Node initialization failed due to parameter parsing errors.")
            return # Stop initialization here

        # --- Setup Subscription ---
        sub_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        try:
            input_msg_type = get_message_class(self.input_message_type_str)
            self.subscription = self.create_subscription(
                input_msg_type,
                self.input_topic_name,
                self.listener_callback,
                sub_qos_profile
            )
            self.get_logger().info(
                f"Subscribed to topic: {self.input_topic_name} "
                f"with type: {self.input_message_type_str}"
            )
        except (ValueError, AttributeError, ModuleNotFoundError, Exception) as e:
            self.get_logger().fatal( # Use fatal as node likely unusable
                f"Failed to import input message type '{self.input_message_type_str}' "
                f"or create subscription for topic '{self.input_topic_name}': {e}"
            )
            init_ok = False # Mark initialization as failed

        # --- Setup Publisher ---
        if init_ok: # Only proceed if subscription setup was okay
            pub_qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=10
            )
            try:
                self.output_msg_type = get_message_class(self.output_message_type_str)
                self.publisher = self.create_publisher(
                    self.output_msg_type,
                    self.output_topic_name,
                    pub_qos_profile
                )
                self.get_logger().info(
                    f"Publishing to topic: {self.output_topic_name} "
                    f"with type: {self.output_message_type_str}"
                )
            except (ValueError, AttributeError, ModuleNotFoundError, Exception) as e:
                self.get_logger().fatal( # Use fatal as node likely unusable
                    f"Failed to import output message type '{self.output_message_type_str}' "
                    f"or create publisher for topic '{self.output_topic_name}': {e}"
                )
                init_ok = False # Mark initialization as failed

        # --- Final Initialization Check ---
        if not init_ok:
             # Clean up subscription if it was created before publisher failed
             if self.subscription:
                 self.destroy_subscription(self.subscription)
                 self.subscription = None
             self.publisher = None # Ensure publisher is None if failed
             # Let main handle the shutdown based on None checks

    def _parse_mappings_from_string(self, mapping_config_str):
        """Parses 'input1:output1,input2:output2' string. Returns dict."""
        mappings = {}
        if not mapping_config_str:
            self.get_logger().info("No field mappings provided via 'field_mappings_str'.")
            return mappings # Empty dict is valid

        # Split the string by commas first
        mapping_pairs = mapping_config_str.split(',')

        for item in mapping_pairs:
            item = item.strip() # Remove leading/trailing whitespace from pair
            if not item:
                continue # Skip empty entries if there are trailing/double commas

            parts = item.split(':')
            if len(parts) == 2:
                input_field = parts[0].strip()
                output_field = parts[1].strip()
                if input_field and output_field:
                    mappings[input_field] = output_field
                else:
                    self.get_logger().error(f"Invalid mapping entry (empty field): '{item}'. Skipping.")
            else:
                self.get_logger().error(f"Invalid mapping format: '{item}'. Use 'input_field:output_field'. Skipping.")

        self.get_logger().info(f"Parsed field mappings from string: {mappings}")
        return mappings

    def _parse_static_fields_from_string(self, static_fields_config_str):
        """Parses 'field1=value1,field2=value2' string. Returns dict."""
        static_fields = {}
        if not static_fields_config_str:
            self.get_logger().info("No static fields provided via 'static_fields_str'.")
            return static_fields # Empty dict is valid

        # Split the string by commas first
        static_field_pairs = static_fields_config_str.split(',')

        for item in static_field_pairs:
            item = item.strip() # Remove leading/trailing whitespace from pair
            if not item:
                continue # Skip empty entries

            parts = item.split('=', 1) # Split only on the first '='
            if len(parts) == 2:
                field_name = parts[0].strip()
                value_str = parts[1].strip()
                if not field_name:
                    self.get_logger().error(f"Invalid static field entry (empty field name): '{item}'. Skipping.")
                    continue

                # Attempt type conversion (float -> int -> bool -> string)
                try:
                    value = float(value_str)
                    if '.' not in value_str and 'e' not in value_str.lower():
                         if value.is_integer():
                             value = int(value)
                except ValueError:
                    if value_str.lower() == 'true':
                        value = True
                    elif value_str.lower() == 'false':
                        value = False
                    else:
                        value = value_str
                        if len(value) >= 2 and value[0] == value[-1] and value[0] in ('"', "'"):
                            value = value[1:-1]

                static_fields[field_name] = value
            else:
                self.get_logger().error(f"Invalid static field format: '{item}'. Use 'output_field=value'. Skipping.")

        self.get_logger().info(f"Parsed static fields from string: {static_fields}")
        return static_fields

    def listener_callback(self, msg):
        """Callback for input topic. Creates, populates, and publishes output message."""
        if not self.publisher or not self.output_msg_type:
            self.get_logger().warn("Publisher or output type not initialized, skipping message.")
            return

        try:
            # Create an instance of the output message type
            output_msg = self.output_msg_type()

            # 1. Apply static field assignments
            if self.parsed_static_fields is not None:
                for field_name, value in self.parsed_static_fields.items():
                    try:
                        setattr(output_msg, field_name, value)
                    except AttributeError:
                        self.get_logger().warn(
                            f"Static field assignment skipped: Output field '{field_name}' "
                            f"not found in message type '{self.output_message_type_str}'."
                        )
                    except TypeError as te:
                         self.get_logger().warn(
                            f"Static field assignment skipped for '{field_name}={value}': Type mismatch. "
                            f"Is the value '{value}' (type: {type(value).__name__}) compatible with the field type? Error: {te}"
                         )

            # 2. Apply field-to-field mappings
            if self.parsed_mappings is not None:
                for input_field, output_field in self.parsed_mappings.items():
                    try:
                        input_value = getattr(msg, input_field)
                        setattr(output_msg, output_field, input_value)
                    except AttributeError as e:
                        self.get_logger().warn(
                            f"Mapping skipped: '{input_field}' -> '{output_field}'. AttributeError: {e}. "
                            f"Check if fields exist in input ('{self.input_message_type_str}') "
                            f"or output ('{self.output_message_type_str}') types."
                        )
                    except TypeError as te:
                         self.get_logger().warn(
                            f"Mapping skipped: '{input_field}' -> '{output_field}'. TypeError: {te}. "
                            f"Are the field types compatible?"
                         )

            # Publish the message
            self.publisher.publish(output_msg)

        except Exception as e:
            self.get_logger().error(f"Unexpected error in listener_callback: {e}", exc_info=True)


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = FieldMapperNode()
        if node.subscription is None or node.publisher is None:
            node.get_logger().error("Initialization failed. Exiting.")
            exit_code = 1
        else:
            rclpy.spin(node)
    except Exception as e:
        if node:
            node.get_logger().fatal(f"Unhandled exception: {e}")
        else:
            print(f"Unhandled exception during node creation: {e}")
        exit_code = 1
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
