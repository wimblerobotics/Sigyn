|topic|hz|
|---|---|
|/amcl/transition_event||
|/amcl_pose||
|/behavior_server/transition_event||
|/behavior_tree_log||
|/bluetoothJoystick||
|/bond||
|/bt_navigator/transition_event||
|/clicked_point||
|/clock||
|/cmd_vel||
|/cmd_vel_joystick||
|/cmd_vel_keyboard||
|/cmd_vel_nav||
|/cmd_vel_smoothed||
|/cmd_vel_teleop||
|/controller_selector||
|/controller_server/transition_event||
|/cost_cloud||
|/diagnostics||
|/evaluation||
|/global_costmap/costmap|0.62|
|/global_costmap/costmap_raw||
|/global_costmap/costmap_raw_updates||
|/global_costmap/costmap_updates||
|/global_costmap/footprint||
|/global_costmap/global_costmap/transition_event||
|/global_costmap/obstacle_layer||
|/global_costmap/obstacle_layer_raw||
|/global_costmap/obstacle_layer_raw_updates||
|/global_costmap/obstacle_layer_updates||
|/global_costmap/published_footprint||
|/global_costmap/static_layer||
|/global_costmap/static_layer_raw||
|/global_costmap/static_layer_raw_updates||
|/global_costmap/static_layer_updates||
|/goal_pose||
|/initialpose||
|/joint_states||
|/local_costmap/costmap|3.28|
|/local_costmap/costmap_raw||
|/local_costmap/costmap_raw_updates||
|/local_costmap/costmap_updates||
|/local_costmap/footprint||
|/local_costmap/lidar_layer||
|/local_costmap/lidar_layer_raw||
|/local_costmap/lidar_layer_raw_updates||
|/local_costmap/lidar_layer_updates||
|/local_costmap/local_costmap/transition_event||
|/local_costmap/published_footprint||
|/local_plan||
|/main_battery||
|/map||
|/map_server/transition_event||
|/marker||
|/move_base_simple/goal||
|/odom||
|/parameter_events||
|/particle_cloud||
|/plan||
|/plan_smoothed||
|/planner_selector||
|/planner_server/transition_event||
|/preempt_teleop||
|/raw_scan||
|/received_global_plan||
|/roboclaw_status|1.0|
|/robot_description||
|/rosout||
|/scan|10|
|/set_pose||
|/smoother_server/transition_event||
|/speed_limit||
|/teensy_diagnostics|30|
|/teensy_stats|1|
|/tf|80|
|/tf_static||
|/transformed_global_plan||
|/velocity_smoother/transition_event||
|/waypoint_follower/transition_event||
|/wheel_odom|30|

[`roboclaw_status`]

```code
{
  "data": "{\"LogicVoltage\":0.0,\"MainVoltage\":0.0,\"Encoder_Left\":26817,\"Encoder_Right\":-26820,\"LeftMotorCurrent\":0.000,\"RightMotorCurrent\":0.000,\"LeftMotorSpeed\":0,\"RightMotorSpeed\":0,\"Error\":0}"
}
```

[`teensy_diagnostics]`]

```code
{
  "data": "{\"Stats\": {\"loops\":73,\"Ms\":1005.2,\"mdls\":[{\"n\":\"Batt\",\"MnMxAv\":[0.0,0.2,0.0]},{\"n\":\"uRos\",\"MnMxAv\":[0.0,44.5,7.0]},{\"n\":\"Rlay\",\"MnMxAv\":[0.0,0.0,0.0]},{\"n\":\"Robo\",\"MnMxAv\":[5.9,7.4,6.7]},{\"n\":\"TSd\",\"MnMxAv\":[0.0,0.0,0.0]}]}}"
}
```

[`/teensy_stats`]

```code
{
  "data": "{\"Stats\": {\"loops\":73,\"Ms\":1005.2,\"mdls\":[{\"n\":\"Batt\",\"MnMxAv\":[0.0,0.2,0.0]},{\"n\":\"uRos\",\"MnMxAv\":[0.0,44.5,7.0]},{\"n\":\"Rlay\",\"MnMxAv\":[0.0,0.0,0.0]},{\"n\":\"Robo\",\"MnMxAv\":[5.9,7.4,6.7]},{\"n\":\"TSd\",\"MnMxAv\":[0.0,0.0,0.0]}]}}"
}
```

[`wheel_odom`]
<details open>
<summary>wheel odometry pose</summary>

```code
{
  "header": {
    "stamp": {
      "sec": 1736020028,
      "nsec": 254000000
    },
    "frame_id": "odom"
  },
  "child_frame_id": "base_link",
  "pose": {
    "pose": {
      "position": {
        "x": -0.0002808550780173391,
        "y": -0.00030423683347180486,
        "z": 0
      },
      "orientation": {
        "x": 0,
        "y": 0,
        "z": 0.20931944251060486,
        "w": -0.9778473377227783
      }
    },
```

</details>
<details>
<summary>wheel odometry pose covariance</summary>

```code
    "covariance": [
      0.001,
      0,
      0,
      0,
      0,
      0,
      0,
      0.001,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0.001
    ]
```

</details>
<details open>
<summary>wheel odometry twist</summary>

```code
  },
  "twist": {
    "twist": {
      "linear": {
        "x": 0,
        "y": 0,
        "z": 0
      },
      "angular": {
        "x": 0,
        "y": 0,
        "z": 0
      }
    },
```

</details>
<details>
<summary>wheel odometry twist covariance</summary>

```code
    "covariance": [
      0.0001,
      0,
      0,
      0,
      0,
      0,
      0,
      0.0001,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0.0001
    ]
```
</details>

```code
  }
}
```