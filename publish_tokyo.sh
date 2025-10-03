#!/bin/bash
# Publish GPS data for Tokyo Station

ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'gps_link'
  },
  status: {
    status: 0,
    service: 1
  },
  latitude: 35.681236,
  longitude: 139.767125,
  altitude: 5.0,
  position_covariance: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
  position_covariance_type: 2
}"
