# Commands for SQLit
```bash
sqlite3 /home/ros/sigyn_ws/src/Sigyn/wifi_data.db
.tables
.schema wifi_data
.headers ON
SELECT count(*) from wifi_data;
SELECT * from wifi_data LIMIT 20;
SELECT * from wifi_data where x=0 limit 200;
SELECT min(signal_level) from wifi_data;

min(signal_level)|max(signal_level)
-76.0|-43.0

.exit
```



ros2 run py_scripts measure_wifi

ros2 run wifi_signal_visualizer wifi_signal_visualizer_node --ros-args -p generate_new_data:=False