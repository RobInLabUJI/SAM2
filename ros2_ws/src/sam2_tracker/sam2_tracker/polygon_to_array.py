#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonInstanceStamped, Point32
from example_interfaces.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension

class PolygonToArrayNode(Node):
    def __init__(self):
        super().__init__('polygon_to_array_converter')
        
        # Create subscriber for Polygon messages
        self.subscription = self.create_subscription(
            PolygonInstanceStamped,
            'polygon',  # Change this to your input topic name
            self.polygon_callback,
            10)
        
        # Create publisher for Int16MultiArray
        self.publisher = self.create_publisher(
            Int16MultiArray,
            'polygon_array',  # Change this to your output topic name
            10)

    def polygon_callback(self, msg):
        array_msg = Int16MultiArray()
        array_msg.layout = MultiArrayLayout()
        dim = MultiArrayDimension()
        dim.label = str(msg.polygon.id)
        dim.size = len(msg.polygon.polygon.points)
        array_msg.layout.dim = [dim]
        for point in msg.polygon.polygon.points:
            array_msg.data.append(int(round(point.x)))
            array_msg.data.append(int(round(point.y)))
        self.publisher.publish(array_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PolygonToArrayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
