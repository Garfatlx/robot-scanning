import numpy as np
import open3d as o3d
import scansegmentapi.compact as CompactApi
from scansegmentapi.tcp_handler import TCPHandler
from scansegmentapi.compact_stream_extractor import CompactStreamExtractor
from scansegmentapi.udp_handler import UDPHandler

# LiDAR and motion parameters
PORT = 2115
IP = "192.168.0.101"
TRANSPORT_PROTOCOL = "UDP"
ANGULAR_RESOLUTION = np.deg2rad(0.25)  # Angular resolution between beams (e.g., 0.25 degrees)
SCAN_FREQUENCY = 50.0  # Scan frequency in Hz (e.g., 50 Hz)
LIDAR_SPEED = 0  # LiDAR speed in meters per second (e.g., 0.1 m/s)

def compute_point_cloud(segments, angular_resolution, lidar_speed=None):
    """
    Convert LiDAR segments to a 3D point cloud.
    
    Args:
        segments: List of segment dictionaries from the LiDAR.
        angular_resolution: Angle increment between beams (radians).
        lidar_speed: Optional. Speed of LiDAR motion (m/s).
    
    Returns:
        points: NumPy array of shape (N, 3) containing [x, y, z] coordinates.
    """
    points = []
    base_time = None
    cumulative_distance = 0.0  # Used if lidar_speed is given

    for i, segment in enumerate(segments):
        module = segment["Modules"][0]
        start_angle = module["ThetaStart"][0]
        distances = module["SegmentData"][0]["Distance"][0]
        num_beams = len(distances)
        
        # Compute angle per beam
        angles = start_angle + np.arange(num_beams) * angular_resolution
        
        # Compute XY coordinates
        x = distances * np.cos(angles)
        y = distances * np.sin(angles)
        
        # Extract timestamp (in seconds)
        timestamp = segment.get("Timestamp", None)
        if timestamp is None:
            print("Warning: No timestamp found; assuming fixed spacing.")
            z = i if lidar_speed is None else lidar_speed * (i / SCAN_FREQUENCY)
        else:
            if base_time is None:
                base_time = timestamp
            time_elapsed = timestamp - base_time
            
            if lidar_speed is not None:
                z = lidar_speed * time_elapsed
            else:
                # Use time directly as relative z position (e.g., mm -> meters)
                z = time_elapsed

        z_coords = np.full(num_beams, z)
        segment_points = np.column_stack((x, y, z_coords))

        valid_mask = (distances > 0) & (~np.isnan(distances))
        segment_points = segment_points[valid_mask]
        points.append(segment_points)

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
    segments, frameNumbers, segmentCounters = receiver.receive_segments(800)
    receiver.close_connection()

    # Filter segments where frameNumber % 5 == 0 (first 5 segments)
    # idx = np.where(np.array(frameNumbers) % 5 == 0)
    # segmentsFrameNumberMod5 = np.array(segments)[idx][:5]
    segmentsFrameNumberMod5 = segments  # Use all segments

    # Convert segments to point cloud
    points = compute_point_cloud(
    segmentsFrameNumberMod5,
    angular_resolution=ANGULAR_RESOLUTION,
    lidar_speed=LIDAR_SPEED  # or None
)

    # Save the point cloud
    save_point_cloud(points, "object_pointcloud.pcd")

    # Visualize the point cloud (optional)
    visualize_point_cloud(points)

    # Print summary
    print(f"Generated point cloud with {len(points)} points")