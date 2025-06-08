#!/usr/bin/python3

import glob 
import re
import os
import numpy as np
import json
from scipy.spatial.transform import Rotation as R
from generate_pointcloud import compute_point_cloud
from generate_pointcloud import save_point_cloud, visualize_point_cloud
import ast

import scansegmentapi.compact as CompactApi
from scansegmentapi.tcp_handler import TCPHandler
from scansegmentapi.compact_stream_extractor import CompactStreamExtractor
from scansegmentapi.udp_handler import UDPHandler

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

import time
import open3d as o3d

SEGMENTS_PER_FRAME = 12  # Number of segments per frame

PORT= 2115
IP= "192.168.1.2"
TRANSPORT_PROTOCOL = "UDP"  # or "TCP"
LIDAR_FREQUENCY = 50  # Scan frequency in Hz

frame_scanning_data={}
frame_robot_position_data={}

class ScannerNode(Node):
    def __init__(self):
        super().__init__("scanner_node")
        self.received = {"done": False}
        self.robot_position = None
        self.subscription = self.create_subscription(
            TFMessage, "/tf", self.tf_callback, 10
        )

    def tf_callback(self, msg):
        for transform in msg.transforms:
            
            if transform.child_frame_id == "tool0_controller":
                self.robot_position = {
                    'x': transform.transform.translation.x,
                    'y': transform.transform.translation.y,
                    'z': transform.transform.translation.z,
                    'rotation': {
                        'x': transform.transform.rotation.x,
                        'y': transform.transform.rotation.y,
                        'z': transform.transform.rotation.z,
                        'w': transform.transform.rotation.w
                    }
                }
                self.received["done"] = True
                self.get_logger().info("Captured robot position")

    def listen_to_robot_position(self, index):
        self.received["done"] = False
        self.robot_position = None
        while rclpy.ok() and not self.received["done"]:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.robot_position:
            frame_robot_position_data[index] = self.robot_position
        return self.received["done"]
    
def listen_to_lidar(index):
    """
    Listens to the LiDAR data stream and saves the segments to a file.
    """
    handler = None
    extractor = None
    try:
        transportLayer = UDPHandler(IP, PORT, 65535)
    # Receive LiDAR segments
        receiver = CompactApi.Receiver(transportLayer)
        segments, frameNumbers, segmentCounters = receiver.receive_segments(24)
        
        # Filter one full frame of segments
        unique_frame_numbers = np.unique(frameNumbers)
        if len(unique_frame_numbers) > 3 or len(unique_frame_numbers) < 2:
            raise ValueError(f"Unexpected number of unique frame numbers: {len(unique_frame_numbers)}.")
        frame_number = max({fn: frameNumbers.count(fn) for fn in unique_frame_numbers}, key=lambda k: frameNumbers.count(k))
        segments = [segment for segment in segments if segment["Modules"][0]["FrameNumber"] == frame_number]
        
        frame_scanning_data[index] = segments

        print(f"Received {len(segments)} segments for index {index}.")
    except Exception as e:
        print(f"Error in LiDAR scan for index {index}: {e}")
    finally:
        if receiver:
            try:
                receiver.close_connection()
            except Exception:
                pass
        if extractor:
            extractor.stop()
        if handler:
            try:
                handler.close()  # Close the UDP/TCP handler
            except AttributeError:
                print("Warning: UDPHandler does not have a close() method.")
                # Optionally, access the socket directly if exposed
                if hasattr(handler, 'socket'):
                    handler.socket.close()

def start_scanning(scanning_duration=10000, step_size=100):
    rclpy.init()
    node = ScannerNode()
    total_steps = scanning_duration // step_size
    for index in range(1, total_steps + 1):
        start_time = time.time()
        print(f"Starting scan for index {index}...")
        listen_to_lidar(index)
        if node.listen_to_robot_position(index):
            print(f"Completed scan for index {index}.")
        else:
            print(f"Failed to capture robot position for index {index}.")
        rclpy.spin_once(node, timeout_sec=0)  # No extra wait here

        elapsed = (time.time() - start_time) * 1000  # ms
        remaining = step_size - elapsed
        if remaining > 0:
            time.sleep(remaining / 1000.0)
        else:
            print(f"Warning: Step {index} took longer ({elapsed:.1f} ms) than step_size ({step_size} ms)")

    node.destroy_node()
    rclpy.shutdown()
    # Process points as before
    points = []
    for index in range(1, total_steps + 1):
        if index not in frame_scanning_data or index not in frame_robot_position_data:
            print(f"Skipping index {index} due to missing data.")
            continue
        segments = frame_scanning_data[index]
        robot_position = frame_robot_position_data[index]
        point_cloud = compute_point_cloud_per_frame(segments, robot_position)
        points.append(point_cloud)
    return np.vstack(points) if points else np.empty((0, 3))

def compute_point_cloud_per_frame(segments, robot_position):
    """
    Computes the point cloud from the segments and robot position.
    """
    points = []
    
    next_segment_idx=0
    for segment in segments:
        
        # Extract data from the first module (single-layer assumption)
        module = segment["Modules"][0]
        start_angle = module["ThetaStart"][0]  # Starting angle of the segment
        distances = np.array(module["SegmentData"][0]["Distance"][0])  # List of distances for beams
        num_beams = len(distances)
        




        
        if next_segment_idx >= len(segments)-1:
            next_segment_idx = 0  # Wrap around to the first segment if at the end
        else:
            next_segment_idx += 1







        # Compute angles for each beam in the segment
        if len(segments) > 1:
            next_segment = segments[next_segment_idx]  # Assuming segments are ordered
            next_module = next_segment["Modules"][0]
            next_start_angle = next_module["ThetaStart"][0]  # Starting angle of the next segment
            
            # Handle wrap-around case
            if next_start_angle < start_angle:
                next_start_angle += 2 * np.pi 
            angles = start_angle + (next_start_angle - start_angle) * np.arange(num_beams) / num_beams
        else:
            angles = np.linspace(start_angle, start_angle + 2 * np.pi, num_beams, endpoint=False)
        
        # Compute 2D coordinates in the scan plane (xy-plane)
        x = distances * np.cos(angles)
        y = distances * np.sin(angles)
        
        
        # Filter out invalid points (e.g., distance = 0 or NaN)
        valid_mask = (distances > 0) & (~np.isnan(distances))
        x = x[valid_mask]
        y = y[valid_mask]
        
        # Compute z-coordinate based on robot position
        z_coords = np.full(len(x), robot_position['z']*1000)
        
        # Combine into 3D points for this segment
        segment_points = np.column_stack((x + robot_position['x']*1000, y + robot_position['y']*1000, z_coords))
        points.append(segment_points)
    
    # Combine all points into a single array
    if points:
        points = np.vstack(points)
    else:
        points = np.empty((0, 3))
    
    return points

def visualize_point_cloud(points):
    """
    Visualize the point cloud using Open3D.
    
    Args:
        points: NumPy array of shape (N, 3) containing [x, y, z] coordinates.
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.visualization.draw_geometries([pcd], window_name="3D Point Cloud")

def main():   
    """
    Main function to start the scanning process and generate the point cloud.
    """

    # Start scanning and collecting data
    points = start_scanning(scanning_duration=10000, step_size=100)  # 10 seconds scanning with 1 second steps
    
    if points.size == 0:
        print("No points were collected. Exiting.")
        return
    
    # Save the point cloud to a file
    # save_point_cloud(points, filename="pointcloud.pcd")
    
    # Visualize the point cloud
    visualize_point_cloud(points)
    

if __name__ == "__main__":
    main()
