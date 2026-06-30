import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class FakeObstacleCloud(Node):
    def __init__(self):
        super().__init__('fake_obstacle_cloud')
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('topic', '/ground_segmentation/obstacle_points')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('distance_x', 0.50)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('center_z', 0.35)
        self.declare_parameter('size_y', 0.35)
        self.declare_parameter('size_z', 0.55)
        self.declare_parameter('step', 0.025)

        topic = self.get_parameter('topic').value
        rate_hz = float(self.get_parameter('rate_hz').value)
        self.pub = self.create_publisher(PointCloud2, topic, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_cloud)
        self.get_logger().info(f'Publishing fake obstacle cloud on {topic}')

    def publish_cloud(self):
        frame_id = self.get_parameter('frame_id').value
        x = float(self.get_parameter('distance_x').value)
        center_y = float(self.get_parameter('center_y').value)
        center_z = float(self.get_parameter('center_z').value)
        size_y = float(self.get_parameter('size_y').value)
        size_z = float(self.get_parameter('size_z').value)
        step = float(self.get_parameter('step').value)

        points = []
        y_min = center_y - size_y / 2.0
        z_min = center_z - size_z / 2.0
        ny = max(1, int(math.floor(size_y / step)) + 1)
        nz = max(1, int(math.floor(size_z / step)) + 1)
        for iy in range(ny):
            y = y_min + iy * step
            for iz in range(nz):
                z = z_min + iz * step
                points.append((x, y, z))

        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = b''.join(struct.pack('<fff', *p) for p in points)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeObstacleCloud()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
