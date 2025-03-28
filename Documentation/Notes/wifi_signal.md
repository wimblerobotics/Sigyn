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
.exit
```