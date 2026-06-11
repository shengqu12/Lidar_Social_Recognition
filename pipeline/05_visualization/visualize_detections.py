import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA
import numpy as np
import os
import time

BINS_DIR = os.path.expanduser("~/Desktop/research/summer_research/dataset/hunt_library_bins")
PREDS_DIR = os.path.expanduser("~/Desktop/research/summer_research/O-LiPeDeT-Overhead-LiDAR-Person-Detection-and-Tracking/lidar-human-detection/outputs/preds_hunt_library/run_00/preds")

class DetectionVisualizer(Node):
    def __init__(self):
        super().__init__('detection_visualizer')
        self.pc_pub = self.create_publisher(PointCloud2, '/livox/lidar', 10)
        self.box_pub = self.create_publisher(MarkerArray, '/detected_boxes', 10)
        self.frame_files = sorted([f for f in os.listdir(BINS_DIR) if f.endswith('.bin')])
        self.idx = 0
        self.timer = self.create_timer(0.1, self.publish_frame)  # 10Hz
        self.get_logger().info(f'Loaded {len(self.frame_files)} frames')

    def publish_frame(self):
        if self.idx >= len(self.frame_files):
            self.idx = 0  # loop

        fname = self.frame_files[self.idx]
        bin_path = os.path.join(BINS_DIR, fname)
        txt_path = os.path.join(PREDS_DIR, fname + '.txt')

        now = self.get_clock().now().to_msg()

        # Publish point cloud
        pts = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
        pc_msg = PointCloud2()
        pc_msg.header = Header()
        pc_msg.header.stamp = now
        pc_msg.header.frame_id = 'livox_frame'
        pc_msg.height = 1
        pc_msg.width = len(pts)
        pc_msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        pc_msg.is_bigendian = False
        pc_msg.point_step = 16
        pc_msg.row_step = 16 * len(pts)
        pc_msg.data = pts.tobytes()
        pc_msg.is_dense = True
        self.pc_pub.publish(pc_msg)

        # Publish bounding boxes as markers
        marker_array = MarkerArray()
        # Clear previous markers
        clear = Marker()
        clear.header.stamp = now
        clear.header.frame_id = 'livox_frame'
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        if os.path.exists(txt_path):
            with open(txt_path) as f:
                lines = f.readlines()
            for i, line in enumerate(lines):
                parts = line.strip().split()
                if len(parts) < 8:
                    continue
                # box x y z dx dy dz heading confidence
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                dx, dy, dz = float(parts[4]), float(parts[5]), float(parts[6])
                conf = float(parts[8]) if len(parts) > 8 else 1.0

                m = Marker()
                m.header.stamp = now
                m.header.frame_id = 'livox_frame'
                m.ns = 'detections'
                m.id = i + 1
                m.type = Marker.CUBE
                m.action = Marker.ADD
                m.pose.position.x = x
                m.pose.position.y = y
                m.pose.position.z = z + dz/2  # center to bottom
                m.pose.orientation.w = 1.0
                m.scale.x = dx
                m.scale.y = dy
                m.scale.z = dz
                m.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.4)  # orange transparent
                m.lifetime.sec = 0
                m.lifetime.nanosec = 200000000  # 0.2s
                marker_array.markers.append(m)

        self.box_pub.publish(marker_array)

        if self.idx % 50 == 0:
            self.get_logger().info(f'Frame {self.idx}/{len(self.frame_files)}')
        self.idx += 1

def main():
    rclpy.init()
    node = DetectionVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
