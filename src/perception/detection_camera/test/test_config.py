#!/usr/bin/env python3
"""Test script to verify detection_camera configuration"""

import rclpy
from rclpy.node import Node

class ConfigTest(Node):
    def __init__(self):
        super().__init__('config_test')
        
        # Declare same parameters as main node
        self.declare_parameter('depth_node', '/zed/zed_node')
        self.declare_parameter('camera_node', '/zed/zed_node')
        self.declare_parameter('publishing_topic', '/cone_positions')
        self.declare_parameter('model_file', '/default/path/model.pt')
        self.declare_parameter('classes_file', '/default/path/classes.txt')
        self.declare_parameter('include_depth', True)
        self.declare_parameter('visualize', True)
        self.declare_parameter('detection_confidence', 0.5)
        
        # Get and print all parameters
        print("\n" + "="*60)
        print("Configuration Test Results:")
        print("="*60)
        print(f"depth_node: {self.get_parameter('depth_node').value}")
        print(f"camera_node: {self.get_parameter('camera_node').value}")
        print(f"publishing_topic: {self.get_parameter('publishing_topic').value}")
        print(f"model_file: {self.get_parameter('model_file').value}")
        print(f"classes_file: {self.get_parameter('classes_file').value}")
        print(f"include_depth: {self.get_parameter('include_depth').value}")
        print(f"visualize: {self.get_parameter('visualize').value}")
        print(f"detection_confidence: {self.get_parameter('detection_confidence').value}")
        print("="*60 + "\n")
        
        import os
        model_path = self.get_parameter('model_file').value
        classes_path = self.get_parameter('classes_file').value
        
        print(f"Model file exists: {os.path.exists(model_path)}")
        print(f"Classes file exists: {os.path.exists(classes_path)}")
        print("="*60 + "\n")

def main():
    rclpy.init()
    node = ConfigTest()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
