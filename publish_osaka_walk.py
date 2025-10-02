#!/usr/bin/env python3
"""
Publish GPS data simulating walking from Osaka Station to Namba along Midosuji.
Walking speed: approximately 1.4 m/s (5 km/h)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
import math
import time

class OsakaWalkPublisher(Node):
    def __init__(self):
        super().__init__('osaka_walk_publisher')
        self.publisher = self.create_publisher(NavSatFix, '/gps/fix', 10)

        # Route waypoints: Osaka Station -> Umeda -> Yodoyabashi -> Shinsaibashi -> Namba
        # Following Midosuji street
        self.waypoints = [
            (34.702485, 135.495951),  # Osaka Station
            (34.701500, 135.496500),  # Umeda area
            (34.693500, 135.500500),  # Yodoyabashi
            (34.681500, 135.502000),  # Shinsaibashi
            (34.668500, 135.502500),  # Namba area
            (34.666700, 135.502100),  # Namba Station
        ]

        self.current_waypoint_idx = 0
        self.progress = 0.0  # Progress between current and next waypoint (0.0 to 1.0)

        # Walking speed: 1.4 m/s (5 km/h)
        # Update rate: 10 Hz
        self.update_rate = 10.0  # Hz
        self.speed_mps = 1.4  # meters per second

        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.update_rate, self.publish_position)

        self.get_logger().info('Started walking simulation from Osaka Station to Namba')
        self.get_logger().info(f'Walking speed: {self.speed_mps} m/s ({self.speed_mps * 3.6} km/h)')

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two GPS coordinates in meters."""
        R = 6371000  # Earth radius in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

        return R * c

    def interpolate_position(self, lat1, lon1, lat2, lon2, progress):
        """Linear interpolation between two GPS coordinates."""
        lat = lat1 + (lat2 - lat1) * progress
        lon = lon1 + (lon2 - lon1) * progress
        return lat, lon

    def publish_position(self):
        if self.current_waypoint_idx >= len(self.waypoints) - 1:
            self.get_logger().info('Reached Namba Station!')
            self.timer.cancel()
            return

        # Get current and next waypoint
        lat1, lon1 = self.waypoints[self.current_waypoint_idx]
        lat2, lon2 = self.waypoints[self.current_waypoint_idx + 1]

        # Calculate current position
        current_lat, current_lon = self.interpolate_position(lat1, lon1, lat2, lon2, self.progress)

        # Create and publish NavSatFix message
        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'

        msg.status.status = 0  # GPS fix
        msg.status.service = 1  # GPS service

        msg.latitude = current_lat
        msg.longitude = current_lon
        msg.altitude = 5.0

        # Position covariance (approximately 5m accuracy)
        msg.position_covariance = [25.0, 0.0, 0.0, 0.0, 25.0, 0.0, 0.0, 0.0, 25.0]
        msg.position_covariance_type = 2  # COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.publisher.publish(msg)

        # Update progress
        segment_distance = self.calculate_distance(lat1, lon1, lat2, lon2)
        distance_per_update = self.speed_mps / self.update_rate
        progress_increment = distance_per_update / segment_distance if segment_distance > 0 else 1.0

        self.progress += progress_increment

        # Move to next waypoint if needed
        if self.progress >= 1.0:
            self.current_waypoint_idx += 1
            self.progress = 0.0
            if self.current_waypoint_idx < len(self.waypoints) - 1:
                self.get_logger().info(f'Moving to waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints) - 1}')

def main(args=None):
    rclpy.init(args=args)
    node = OsakaWalkPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
