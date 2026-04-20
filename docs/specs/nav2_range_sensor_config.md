# Nav2 Range Sensor Layer Configuration

**Applies to:** `wr_ros_teensy` per-sensor proximity topics (B1)  
**Last updated:** 2026-03-08

---

## Topic Naming Convention

Each VL53L0X sensor on Board 1 publishes on its own topic:

```
/sigyn/sensors/range/<common_name>
```

where `<common_name>` comes from `config/sensor_names.json`.  
The current sensor positions and their topics are:

| Instance | Common Name        | Topic                                        | TF Frame                    |
|----------|--------------------|----------------------------------------------|-----------------------------|
| 0        | rear_right_bkwd    | `/sigyn/sensors/range/rear_right_bkwd`       | `vl53l0x_rear_right_bkwd`   |
| 1        | rear_right_side    | `/sigyn/sensors/range/rear_right_side`       | `vl53l0x_rear_right_side`   |
| 2        | front_right_side   | `/sigyn/sensors/range/front_right_side`      | `vl53l0x_front_right_side`  |
| 3        | front_right_fwd    | `/sigyn/sensors/range/front_right_fwd`       | `vl53l0x_front_right_fwd`   |
| 4        | front_left_fwd     | `/sigyn/sensors/range/front_left_fwd`        | `vl53l0x_front_left_fwd`    |
| 5        | front_left_side    | `/sigyn/sensors/range/front_left_side`       | `vl53l0x_front_left_side`   |
| 6        | rear_left_side     | `/sigyn/sensors/range/rear_left_side`        | `vl53l0x_rear_left_side`    |
| 7        | rear_left_bkwd     | `/sigyn/sensors/range/rear_left_bkwd`        | `vl53l0x_rear_left_bkwd`    |

To add or rename a sensor, edit `config/sensor_names.json` only — no C++ changes required.

---

## Message Details

**Type:** `sensor_msgs/msg/Range`

| Field            | Value                     |
|------------------|---------------------------|
| `radiation_type` | `INFRARED` (1)            |
| `field_of_view`  | `0.436` rad (25°)         |
| `min_range`      | `0.03` m                  |
| `max_range`      | `2.0` m                   |
| `header.frame_id`| from `sensor_names.json → tf_frame` |

---

## Nav2 Costmap Layer Configuration

Use the `range_sensor_layer` plugin in your `nav2_params.yaml`.
List all eight sensor topics explicitly.

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: >-
          rear_right_bkwd rear_right_side
          front_right_side front_right_fwd
          front_left_fwd front_left_side
          rear_left_side rear_left_bkwd
        rear_right_bkwd:
          topic: /sigyn/sensors/range/rear_right_bkwd
          sensor_frame: vl53l0x_rear_right_bkwd
          data_type: Range
          marking: true
          clearing: true
          obstacle_max_range: 2.0
          obstacle_min_range: 0.03
        rear_right_side:
          topic: /sigyn/sensors/range/rear_right_side
          sensor_frame: vl53l0x_rear_right_side
          data_type: Range
          marking: true
          clearing: true
          obstacle_max_range: 2.0
          obstacle_min_range: 0.03
        front_right_side:
          topic: /sigyn/sensors/range/front_right_side
          sensor_frame: vl53l0x_front_right_side
          data_type: Range
          marking: true
          clearing: true
          obstacle_max_range: 2.0
          obstacle_min_range: 0.03
        front_right_fwd:
          topic: /sigyn/sensors/range/front_right_fwd
          sensor_frame: vl53l0x_front_right_fwd
          data_type: Range
          marking: true
          clearing: true
          obstacle_max_range: 2.0
          obstacle_min_range: 0.03
        front_left_fwd:
          topic: /sigyn/sensors/range/front_left_fwd
          sensor_frame: vl53l0x_front_left_fwd
          data_type: Range
          marking: true
          clearing: true
          obstacle_max_range: 2.0
          obstacle_min_range: 0.03
        front_left_side:
          topic: /sigyn/sensors/range/front_left_side
          sensor_frame: vl53l0x_front_left_side
          data_type: Range
          marking: true
          clearing: true
          obstacle_max_range: 2.0
          obstacle_min_range: 0.03
        rear_left_side:
          topic: /sigyn/sensors/range/rear_left_side
          sensor_frame: vl53l0x_rear_left_side
          data_type: Range
          marking: true
          clearing: true
          obstacle_max_range: 2.0
          obstacle_min_range: 0.03
        rear_left_bkwd:
          topic: /sigyn/sensors/range/rear_left_bkwd
          sensor_frame: vl53l0x_rear_left_bkwd
          data_type: Range
          marking: true
          clearing: true
          obstacle_max_range: 2.0
          obstacle_min_range: 0.03
```

### Notes

- The `sensor_frame` must match the `tf_frame` in `sensor_names.json` and must
  exist in the URDF/SRDF TF tree.
- URDF link names for the VL53L0X sensors should match the `tf_frame` values
  in the table above (e.g. `vl53l0x_front_right_fwd`).
- The `global_costmap` typically does not need range sensors; obstacle inflation
  on the local costmap is sufficient for close-range avoidance.
- If a sensor is physically absent or disconnected, Nav2 will warn about a stale
  topic but continue operating.  Set appropriate timeout values in the costmap
  plugin configuration if needed.
