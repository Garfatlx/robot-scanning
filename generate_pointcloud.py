import numpy as np
import open3d as o3d
import scansegmentapi.compact as CompactApi
from scansegmentapi.tcp_handler import TCPHandler
from scansegmentapi.compact_stream_extractor import CompactStreamExtractor
from scansegmentapi.udp_handler import UDPHandler

# LiDAR and motion parameters
PORT = 2115
IP = "192.168.1.101"
TRANSPORT_PROTOCOL = "UDP"
ANGULAR_RESOLUTION = np.deg2rad(0.14)  # Angular resolution between beams (0.25 degrees)
SCAN_FREQUENCY = 20  # Scan frequency in Hz
LIDAR_SPEED = 10  # LiDAR speed in meters per second
SEGMENTS_PER_FRAME = 12  # Number of segments per frame for multiScan136

def compute_point_cloud(segments, frame_numbers, robot_motion=None):
    """
    Convert LiDAR segments to a 3D point cloud, grouping segments into frames.
    
    Args:
        segments: List of segment dictionaries from the LiDAR.
        frame_numbers: List of frame numbers corresponding to each segment.
        angular_resolution: Angle increment between beams (radians).
        lidar_speed: Speed of LiDAR motion (m/s).
    
    Returns:
        points: NumPy array of shape (N, 3) containing [x, y, z] coordinates.
    """
    points = []
    dt = 1.0 / SCAN_FREQUENCY  # Time interval between frames
    
    # Group segments by frame number
    unique_frames = np.unique(frame_numbers)
    for frame_num in unique_frames:
        # Get all segments for the current frame
        frame_indices = np.where(np.array(frame_numbers) == frame_num)[0]
        if len(frame_indices) != SEGMENTS_PER_FRAME:
            print(f"Warning: Frame {frame_num} has {len(frame_indices)} segments, expected {SEGMENTS_PER_FRAME}")
            continue  # Skip incomplete frames
        
        frame_points = []
        for idx in frame_indices:
            segment = segments[idx]
            # Extract data from the first module (single-layer assumption)
            module = segment["Modules"][0]
            start_angle = module["ThetaStart"][0]  # Starting angle of the segment
            distances = module["SegmentData"][0]["Distance"][0]  # List of distances for beams
            num_beams = len(distances)
            
            # Compute angles for each beam in the segment
            if idx == len(frame_indices):
                next_idx = 0
            else:
                next_idx = idx + 1

            next_segment = segments[next_idx]
            next_module = next_segment["Modules"][0]
            next_start_angle = next_module["ThetaStart"][0]  # Starting angle of the next segment
            if next_start_angle < start_angle:
                # Handle wrap-around case
                next_start_angle += 2 * np.pi 
            angles = start_angle + (next_start_angle - start_angle) *  np.arange(num_beams) / num_beams
            
            # Compute 2D coordinates in the scan plane (xy-plane)
            x = distances * np.cos(angles)
            y = distances * np.sin(angles)
            
            # Filter out invalid points (e.g., distance = 0 or NaN)
            valid_mask = (distances > 0) & (~np.isnan(distances))
            x = x[valid_mask]
            y = y[valid_mask]
            
            # Compute z-coordinate based on frame number (not segment index)
            if robot_motion is not None: 
                x = x + robot_motion[frame_num]["x"]
                y = y + robot_motion[frame_num]["y"]
                z = robot_motion[frame_num]["z"]
            else: # virtual mottion
                z = LIDAR_SPEED * frame_num * dt
            
            z_coords = np.full(len(x), z)
            
            # Combine into 3D points for this segment
            segment_points = np.column_stack((x, y, z_coords))
            frame_points.append(segment_points)
        
        # Combine points from all segments in the frame
        if frame_points:
            frame_points = np.vstack(frame_points)
            points.append(frame_points)
    
    # Combine all points into a single array
    if points:
        points = np.vstack(points)
    else:
        points = np.empty((0, 3))
    
    return points

def save_point_cloud(points, filename="pointcloud.pcd"):
    """
    Save points as a point cloud using Open3D.
    
    Args:
        points: NumPy array of shape (N, 3) containing [x, y, z] coordinates.
        filename: Output file name (e.g., .pcd or .ply).
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(filename, pcd)
    print(f"Point cloud saved to {filename}")

def visualize_point_cloud(points):
    """
    Visualize the point cloud using Open3D.
    
    Args:
        points: NumPy array of shape (N, 3) containing [x, y, z] coordinates.
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.visualization.draw_geometries([pcd], window_name="3D Point Cloud")

if __name__ == "__main__":
    # Initialize transport layer
    if TRANSPORT_PROTOCOL == "UDP":
        transportLayer = UDPHandler(IP, PORT, 65535)
    else:
        streamExtractor = CompactStreamExtractor()
        transportLayer = TCPHandler(streamExtractor, IP, PORT, 1024)

    # Receive LiDAR segments
    receiver = CompactApi.Receiver(transportLayer)
    segments, frameNumbers, segmentCounters = receiver.receive_segments(50)
    # print(segments)
    # print("=====================================Frame Numbers=====================================")
    # print(frameNumbers)
    # print("=====================================Segment Counters=====================================")
    # print(segmentCounters)
    # receiver.close_connection()

    # Optionally filter frames (e.g., every 5th frame)
    idx = np.where(np.array(frameNumbers) % 5 == 0)[0]
    filtered_segments = np.array(segments)[idx]
    filtered_frame_numbers = np.array(frameNumbers)[idx]
    filtered_segments = segments  # Use all segments
    filtered_frame_numbers = frameNumbers  # Use all frame numbers

    # Convert segments to point cloud
    points = compute_point_cloud(
        filtered_segments,
        filtered_frame_numbers
    )

    # Save the point cloud
    save_point_cloud(points, "object_pointcloud.pcd")

    # Visualize the point cloud (optional)
    visualize_point_cloud(points)

    # Print summary
    print(f"Generated point cloud with {len(points)} points")