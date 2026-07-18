#!/usr/bin/env python3
import json
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class GraphVisualizer(Node):
    def __init__(self):
        super().__init__('graph_visualizer')

        default_graph = self._default_graph_path()

        self.declare_parameter('graph_file', default_graph)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('marker_topic', '/route_graph_markers')
        self.declare_parameter('publish_period', 1.0)
        self.declare_parameter('show_labels', True)

        self._graph_file = self.get_parameter('graph_file').value
        marker_topic = self.get_parameter('marker_topic').value
        publish_period = float(self.get_parameter('publish_period').value)

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._pub = self.create_publisher(MarkerArray, marker_topic, qos)

        self._markers = self._load_markers()
        self._timer = self.create_timer(publish_period, self._publish)

        self.get_logger().info(
            f'Publishing graph markers from {self._graph_file} on {marker_topic}'
        )

    def _load_markers(self):
        with open(self._graph_file, 'r', encoding='utf-8') as graph_file:
            graph = json.load(graph_file)

        frame_id = self.get_parameter('frame_id').value
        show_labels = bool(self.get_parameter('show_labels').value)

        points = {}
        edges = []
        for feature in graph.get('features', []):
            geometry = feature.get('geometry', {})
            properties = feature.get('properties', {})
            if geometry.get('type') == 'Point':
                node_id = int(properties['id'])
                coords = geometry['coordinates']
                points[node_id] = (float(coords[0]), float(coords[1]), 0.05)
            elif geometry.get('type') == 'MultiLineString':
                edges.append(properties)

        markers = MarkerArray()
        markers.markers.append(self._delete_all_marker(frame_id))

        edge_marker = self._edge_marker(frame_id)
        for edge in edges:
            start = points.get(int(edge['startid']))
            end = points.get(int(edge['endid']))
            if start is None or end is None:
                self.get_logger().warn(
                    f'Skipping edge {edge.get("id")} with missing node reference'
                )
                continue
            edge_marker.points.append(self._point(*start))
            edge_marker.points.append(self._point(*end))
        markers.markers.append(edge_marker)

        node_marker = self._node_marker(frame_id)
        for node_id, coords in sorted(points.items()):
            node_marker.points.append(self._point(*coords))
            if show_labels:
                markers.markers.append(self._label_marker(frame_id, node_id, coords))
        markers.markers.append(node_marker)

        self.get_logger().info(
            f'Loaded graph markers: nodes={len(points)}, edges={len(edges)}'
        )
        return markers

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        for marker in self._markers.markers:
            marker.header.stamp = stamp
        self._pub.publish(self._markers)

    def _delete_all_marker(self, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.action = Marker.DELETEALL
        return marker

    def _edge_marker(self, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = 'route_graph_edges'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color = ColorRGBA(r=0.1, g=0.45, b=1.0, a=0.9)
        return marker

    def _node_marker(self, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = 'route_graph_nodes'
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.95)
        return marker

    def _label_marker(self, frame_id, node_id, coords):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = 'route_graph_labels'
        marker.id = int(node_id)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = self._point(coords[0], coords[1], coords[2] + 0.35)
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.35
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        marker.text = str(node_id)
        return marker

    def _point(self, x, y, z):
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        return point

    def _default_graph_path(self):
        nav_hub_share = get_package_share_directory('nav_hub')
        return os.path.join(nav_hub_share, 'graphs', 'aceleradoras.json')


def main():
    rclpy.init()
    node = GraphVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
