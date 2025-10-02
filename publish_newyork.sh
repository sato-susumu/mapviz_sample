#!/bin/bash
# Publish GPS data for New York (Times Square)

ros2 topic pub /gps/fix sensor_msgs/msg/NavSatFix "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'gps_link'
  },
  status: {
    status: 0,
    service: 1
  },
  latitude: 40.758896,
  longitude: -73.985130,
  altitude: 10.0,
  position_covariance: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
  position_covariance_type: 2
}"
