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

PROCESS_LIDAR_DATA = False
PROCESS_POSITION_DATA = False
GENERATE_POINT_CLOUD = True

SEGMENTS_PER_FRAME = 12  # Number of segments per frame

def process_lidar_data():
    for orig_file_name in glob.glob(os.path.join("test_results_original", "test[0-9]*_original.txt")):
        test_number = re.search(r"test(\d+)\_original.txt", orig_file_name)  # Extract test number
        sd_file_name = os.path.join(os.getcwd(), "segmentdata", f"test_segmentdata{test_number.group(1)}.txt")
        sc_file_name = os.path.join(os.getcwd(), "segmentcounter", f"test_segmentcounter{test_number.group(1)}.txt")
        fn_file_name = os.path.join(os.getcwd(), "framenumber", f"test_fn{test_number.group(1)}.txt")

        orig_file = open(orig_file_name, "r")
        sd_file = open(sd_file_name, "w")
        sc_file = open(sc_file_name, "w")
        fn_file = open(fn_file_name, "w")

        try:
            lines = orig_file.readlines()
            sc_idx = fn_idx = None
            indices_to_remove = []

            # Search from the back for the special headers
            for i in reversed(range(len(lines))):
                if i in indices_to_remove:
                    continue
                if re.search(r"={10,}Segment Counters={10,}", lines[i]):
                    sc_idx = i
                    if sc_idx + 1 < len(lines):
                        sc_file.write(lines[sc_idx + 1].strip() + "\n")
                        indices_to_remove.extend([sc_idx, sc_idx + 1])
                    continue
                if re.search(r"={10,}Frame Numbers={10,}", lines[i]):
                    fn_idx = i
                    if fn_idx + 1 < len(lines):
                        fn_file.write(lines[fn_idx + 1].strip() + "\n")
                        # indices_to_remove = sorted(set(indices_to_remove))
                        indices_to_remove.extend([fn_idx, fn_idx + 1])
                    continue
                if lines[i].startswith("Received segment"):
                    indices_to_remove.append(i)

            # Write the remaining lines to sd_file
            for idx, line in enumerate(lines):
                if idx not in indices_to_remove:
                    sd_file.write(line)

        finally:
            orig_file.close()
            sd_file.close()
            sc_file.close()
            fn_file.close()

def proces_position_data():
    translation_pattern = re.compile(r"Translation: x=([-\d\.e]+), y=([-\d\.e]+), z=([-\d\.e]+)")
    coords = {}
    for i in [6,7,8,9]:
        coords[i] = {'x': None, 'y': None, 'z': None}
        coords_file_name = os.path.join(os.getcwd(), "positiondata", f"coords{i}.txt")
        coords_file = open(coords_file_name, "r")
        lines = coords_file.readlines()
        for line in lines:
            match = translation_pattern.search(line)
            if match:
                x, y, z = match.groups()
                coords[i]['x'] = round(float(x),2)
                coords[i]['y'] = round(float(y),2)
                coords[i]['z'] = round(float(z),2)
        coords_file.close()

    for i in [1,2,3,4,5,10]:
        coords[i] = {'x': None, 'y': None, 'z': None}
        coords_file_name = os.path.join(os.getcwd(), "positiondata", f"coords{i}.txt")
        coords_file = open(coords_file_name, "r")
        lines = coords_file.readlines()
        x, y, z = compute_end_effector_position_from_tf(lines)
        coords[i]['x'] = round(float(x),2)
        coords[i]['y'] = round(float(y),2)
        coords[i]['z'] = round(float(z),2)
        coords_file.close()
    sorted_coords = {str(key): coords[key] for key in sorted(coords.keys())}
    with open(os.path.join(os.getcwd(), "positiondata", "coords_summary.json"), "w") as json_file:
        json.dump(sorted_coords, json_file, indent=2)

def compute_end_effector_position_from_tf(lines):
    """
    Parses tf message lines and computes the end effector position by chaining spatial transforms
    in the order implied by the frame connections (a -> b, b -> c, ...).

    Returns: (x, y, z) position of the end effector in the base frame.
    """

    # To compute the overall translation (end-effector position) from a chain of joint translations and rotations,
    # you need to apply each translation and rotation in sequence, accumulating the transformation.
    # This is typically done using homogeneous transformation matrices (4x4), where each joint's translation and rotation
    # is converted to a matrix, and then all matrices are multiplied in order.

    # Parse all transforms and build a mapping: (parent, child) -> {'translation': ..., 'rotation': ...}
    edge_to_transform = {}
    child_to_parent = {}
    parent_to_child = {}
    frame_re = re.compile(r"Frame:\s*(\S+)\s*->\s*(\S+)")
    translation_re = re.compile(r"Translation: x=([-\d\.e]+), y=([-\d\.e]+), z=([-\d\.e]+)")
    rotation_re = re.compile(r"Rotation: x=([-\d\.e]+), y=([-\d\.e]+), z=([-\d\.e]+), w=([-\d\.e]+)")

    for i in range(len(lines)):
        if not i+2 < len(lines):
            continue
        frame_match = frame_re.search(lines[i])

        if frame_match is None:
            continue

        parent, child = frame_match.groups()
        t_match = translation_re.search(lines[i+1])
        r_match = rotation_re.search(lines[i+2])
        if not t_match or not r_match:
            continue
        
        translation = tuple(float(x) for x in t_match.groups())
        rotation = tuple(float(x) for x in r_match.groups())
        edge_to_transform[(parent, child)] = {'translation': translation, 'rotation': rotation}
        child_to_parent[child] = parent
        parent_to_child[parent] = child

    # Find the chain order: start from the root (a parent that is not a child)
    all_parents = set(parent for parent, _ in edge_to_transform)
    all_children = set(child for _, child in edge_to_transform)
    roots = list(all_parents - all_children)
    if not roots:
        raise ValueError("No root frame found")
    root = roots[0]

    # Build the ordered chain of (parent, child) edges
    chain = []
    current = root
    while current in parent_to_child:
        next_child = parent_to_child[current]
        chain.append((current, next_child))
        current = next_child

    # Compose the transforms in order
    T = np.eye(4)
    for edge in chain:
        tf = edge_to_transform[edge]
        t = np.eye(4)
        t[:3, 3] = tf['translation']
        rot = R.from_quat(tf['rotation'])
        t[:3, :3] = rot.as_matrix()
        T = np.dot(T, t)
    position = T[:3, 3]
    return tuple(np.round(position, 4))

def sanitize_segmentdata_block(block, last_block):
    # "{'Modules':" has been deleted by the regex
    block = "{'Modules':" + block

    # Remove leading and trailing whitespace
    block = block.strip()
    
    # Replace single quotes with double quotes for JSON compatibility
    block = block.replace("'", '"')
    
    # Replace array( with [ to make it JSON compatible
    block = re.sub(r'array\(', '[', block)
    block = re.sub(r'\)', ']', block)
    
    # Replace Python boolean literals with JSON booleans
    block = block.replace('True', 'true').replace('False', 'false')
    
    # Remove trailing dot from numbers before a comma (e.g., 1., -> 1,)
    block = re.sub(r'(\d+)\.\]', r'\1]', block)
    block = re.sub(r'(\d+)\.,', r'\1,', block)

    # Remove trailing ] for last block
    if last_block and block.endswith(']'):
        block = block[:-1]
    
    # Remove trailing comma if present at the end of the string
    if block.endswith(','):
        block = block[:-1]
    
    return block
        

def load_point_cloud_and_robot_position_data():
    # Read segmentdata files
    segmentdata_dir = os.path.join(os.getcwd(), "segmentdata")
    segmentdata_files = sorted(
        glob.glob(os.path.join(segmentdata_dir, "test_segmentdata*.txt")),
        key=lambda x: int(re.search(r"test_segmentdata(\d+)\.txt", x).group(1))
    )
    segments_list = []
    for filename in segmentdata_files:
        with open(filename, "r") as f:
            content = f.read().replace('\n', '').replace('\r', '').replace(' ', '')
            blocks = re.split(r"\{'Modules'\s*:", content)
            blocks = blocks[1:]  # First block only containts "["
            sanitized_blocks = []
            last_block = False
            for block_idx, block in enumerate(blocks):
                if block_idx==len(blocks)-1:
                    last_block = True
                block = sanitize_segmentdata_block(block, last_block)
                try:
                    block_dict = json.loads(block)
                except Exception as e:
                    print(f"Error parsing first module as JSON: {e}")
                sanitized_blocks.append(block_dict)
        
        segments_list.append(sanitized_blocks)
        
    # print(segments_list[2][3]["Modules"][0]["SegmentCounter"])

    # Read framenumber files
    framenumber_dir = os.path.join(os.getcwd(), "framenumber")
    framenumber_files = sorted(
        glob.glob(os.path.join(framenumber_dir, "test_fn*.txt")),
        key=lambda x: int(re.search(r"test_fn(\d+)\.txt", x).group(1))
    )
    filtered_frame_numbers = []
    for filename in framenumber_files:
        with open(filename, "r") as f:
            content = f.read().strip()
            if content:
                lines = ast.literal_eval(content)
                filtered_frame_numbers.append(lines)
    
    # Read robot position data
    robot_coords_filename = os.path.join(os.getcwd(), "positiondata", "coords_summary.json")
    with open(robot_coords_filename, "r") as f:
        robot_coords = json.load(f)
    
    return segments_list, filtered_frame_numbers, robot_coords
    
def generate_point_cloud(segments_list, frame_numbers_list, robot_coords):
    robot_position_numbers = [int(i) for i in robot_coords.keys()]
    points = []

    for robot_pos in robot_position_numbers:
        segments = segments_list[robot_pos - 1]  # robot_pos is 1-indexed
        frame_numbers = frame_numbers_list[robot_pos - 1]  # robot_pos is 1-indexed

        unique_frames = np.unique(frame_numbers)
        for frame_num in unique_frames:
            # Get all segments for the current frame
            frame_indices = np.where(np.array(frame_numbers) == frame_num)[0]
            if len(frame_indices) != SEGMENTS_PER_FRAME:
                # print(f"Warning: Frame {frame_num} has {len(frame_indices)} segments, expected {SEGMENTS_PER_FRAME}")
                continue  # Skip incomplete frames")
            
            frame_points = []
            for idx in frame_indices:
                segment = segments[idx]
                # Extract data from the first module (single-layer assumption)
                module = segment["Modules"][0]
                start_angle = module["ThetaStart"][0][0]  # Starting angle of the segment
                distances = np.array(module["SegmentData"][0]["Distance"][0][0])  # List of distances for beams
                num_beams = len(distances)
                
                # Compute angles for each beam in the segment
                if idx == len(frame_indices):
                    next_idx = 0
                else:
                    next_idx = idx + 1
                if next_idx >= len(segments):
                    continue
                next_segment = segments[next_idx]
                next_module = next_segment["Modules"][0]
                next_start_angle = next_module["ThetaStart"][0][0]  # Starting angle of the next segment
                
                # Handle wrap-around case
                if next_start_angle < start_angle:
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
                x = x + robot_coords[str(robot_pos)]["x"]
                y = y + robot_coords[str(robot_pos)]["y"]
                z = robot_coords[str(robot_pos)]["z"] * 500

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


def main():   
    if PROCESS_LIDAR_DATA:
        process_lidar_data()
    
    if PROCESS_POSITION_DATA:
        proces_position_data()
    
    if GENERATE_POINT_CLOUD:
        segments_list, filtered_frame_numbers, robot_coords = load_point_cloud_and_robot_position_data()
        points = generate_point_cloud(segments_list, filtered_frame_numbers, robot_coords)
        # Save the point cloud
        save_point_cloud(points, "object_pointcloud.pcd")

        # Visualize the point cloud (optional)
        visualize_point_cloud(points)
    

if __name__ == "__main__":
    main()
