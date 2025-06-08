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
ANGULAR_RESOLUTION = np.deg2rad(0.25)  # Angular resolution between beams (0.25 degrees)
SCAN_FREQUENCY = 20  # Scan frequency in Hz
LIDAR_SPEED = 10  # LiDAR speed in meters per second (in z-direction)
SEGMENTS_PER_FRAME = 12  # Number of segments per frame for multiScan136
FRAMES_TO_AVERAGE = 5  # Number of frames to average for z-coordinate noise reduction

def compute_point_cloud(segments, frame_numbers, angular_resolution, scan_frequency, lidar_speed, frames_to_average):
    """
    Convert LiDAR segments to a 3D point cloud, averaging z-coordinates across frames to reduce noise.
    Use x, y coordinates from the last frame in each set, account for LiDAR motion in z-direction.
    
    Args:
        segments: List of segment dictionaries from the LiDAR.
        frame_numbers: List of frame numbers corresponding to each segment.
        angular_resolution: Angle increment between beams (radians).
        scan_frequency: Frequency of scans (Hz).
        lidar_speed: Speed of LiDAR motion in z-direction (m/s).
        frames_to_average: Number of frames to average for z-coordinate noise reduction.
    
    Returns:
        points: NumPy array of shape (N, 3) containing [x, y, z] coordinates.
    """
    points = []
    dt = 1.0 / scan_frequency  # Time interval between frames
    
    # Group segments by frame number
    unique_frames = np.unique(frame_numbers)
    frame_groups = []
    for frame_num in unique_frames:
        frame_indices = np.where(np.array(frame_numbers) == frame_num)[0]
        if len(frame_indices) == SEGMENTS_PER_FRAME:
            frame_groups.append((frame_num, frame_indices))
        else:
            print(f"Warning: Frame {frame_num} has {len(frame_indices)} segments, expected {SEGMENTS_PER_FRAME}")
    
    # Process sets of frames_to_average frames
    for i in range(0, len(frame_groups) - frames_to_average + 1, frames_to_average):
        frame_set = frame_groups[i:i + frames_to_average]
        if len(frame_set) < frames_to_average:
            continue  # Skip incomplete sets
        
        # Initialize lists to store points and angles for the last frame
        final_frame_points = []
        final_frame_angles = []
        all_z_coords = []
        final_frame_num = frame_set[-1][0]  # Last frame's number for reference
        
        # Process all frames in the set to collect z-coordinates
        for frame_num, frame_indices in frame_set:
            frame_points = []
            frame_angles = []
            # Calculate z-offset due to LiDAR motion relative to the last frame
            time_diff = (final_frame_num - frame_num) * dt
            z_offset = lidar_speed * time_diff  # Positive offset (earlier frames are lower in z)
            
            for idx in frame_indices:
                segment = segments[idx]
                module = segment["Modules"][0]
                start_angle = module["ThetaStart"][0]
                distances = module["SegmentData"][0]["Distance"][0]
                num_beams = len(distances)
                
                # Compute angles for each beam in the segment
                angles = start_angle + np.arange(num_beams) * angular_resolution
                
                # Compute x, y, z coordinates
                x = distances * np.cos(angles)
                y = distances * np.sin(angles)
                z = np.full(len(distances), lidar_speed * frame_num * dt) - z_offset
                
                # Filter valid points
                valid_mask = (distances > 0) & (~np.isnan(distances))
                frame_points.append(np.column_stack((x, y, z))[valid_mask])
                frame_angles.append(angles[valid_mask])
            
            # Combine points and angles for this frame
            if frame_points:
                frame_points = np.vstack(frame_points)
                frame_angles = np.concatenate(frame_angles)
                
                if frame_num == final_frame_num:
                    final_frame_points = frame_points  # Save x, y, z from last frame
                    final_frame_angles = frame_angles  # Save angles from last frame
                else:
                    all_z_coords.append((frame_points, frame_angles))  # Save for z-averaging
        
        # Average z-coordinates for points with matching x, y (angles)
        if final_frame_points.size > 0 and all_z_coords:
            avg_points = []
            # Use x, y from the last frame
            final_x = final_frame_points[:, 0]
            final_y = final_frame_points[:, 1]
            
            # Initialize arrays for z-averaging
            avg_z = np.zeros(len(final_x))
            count = np.zeros(len(final_x), dtype=int)
            
            # Match points by angles and average z-coordinates
            for frame_points, frame_angles in all_z_coords:
                for i, ref_angle in enumerate(final_frame_angles):
                    # Find closest angle in this frame
                    idx = np.argmin(np.abs(frame_angles - ref_angle))
                    if np.abs(frame_angles[idx] - ref_angle) < angular_resolution / 2:
                        avg_z[i] += frame_points[idx, 2]
                        count[i] += 1
            
            # Compute average z-coordinates
            valid = count > 0
            avg_z[valid] /= count[valid]
            
            # Use the last frame's z-position for the averaged point cloud
            final_z = lidar_speed * final_frame_num * dt
            avg_z[valid] = final_z  # Set z to the last frame's position
            
            # Combine into 3D points
            frame_points = np.column_stack((final_x[valid], final_y[valid], avg_z[valid]))
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
    segments, frameNumbers, segmentCounters = receiver.receive_segments(800)
    receiver.close_connection()

    # Use all segments
    filtered_segments = segments
    filtered_frame_numbers = frameNumbers

    # Convert segments to point cloud with z-coordinate averaging
    points = compute_point_cloud(
        filtered_segments,
        filtered_frame_numbers,
        angular_resolution=ANGULAR_RESOLUTION,
        scan_frequency=SCAN_FREQUENCY,
        lidar_speed=LIDAR_SPEED,
        frames_to_average=FRAMES_TO_AVERAGE
    )

    # Save the point cloud
    save_point_cloud(points, "averaged_z_pointcloud.pcd")

    # Visualize the point cloud
    visualize_point_cloud(points)

    # Print summary
    print(f"Generated point cloud with {len(points)} points")