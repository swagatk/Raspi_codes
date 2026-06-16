import cv2
import sys
import math
import numpy as np
from config import APRILTAG_FAMILY

try:
    from pupil_apriltags import Detector as PoseDetector
except ImportError:
    print("Error: pupil_apriltags not found! Please install it.")
    sys.exit()

TAG_SIZE = 0.10  # 10 cm tag

# Initialize the AprilTag detector globally so it doesn't rebuild on every frame
apriltag_detector = PoseDetector(families=APRILTAG_FAMILY, quad_decimate=2.0)

def capture_home_tag_function(frame, camera_params):
    """
    Takes a video frame, runs AprilTag detection, and returns an array of detected tags.
    """
    if frame is None:
        return []
    
    # Check if the frame needs to be converted to grayscale
    if len(frame.shape) == 3:
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    else:
        frame_gray = frame
        
    tags = apriltag_detector.detect(
        frame_gray, 
        estimate_tag_pose=True, 
        camera_params=camera_params, 
        tag_size=TAG_SIZE
    )
    
    return tags

# --- POSE ESTIMATION LOGIC ---
SCALE = 2.0     # Scale factor for Picamera distance calibration

def get_wall_axes(tag_id):
    if 0 <= tag_id <= 5:
        return (1.0, 0.0), (0.0, 1.0)
    if 6 <= tag_id <= 11:
        return (0.0, 1.0), (-1.0, 0.0)
    if 12 <= tag_id <= 17:
        return (-1.0, 0.0), (0.0, -1.0)
    return (0.0, -1.0), (1.0, 0.0)

def heading_from_vector(vec_x, vec_y):
    return math.degrees(math.atan2(vec_y, vec_x)) % 360.0

def normalize_rotation(angle_degrees):
    return (angle_degrees + 180.0) % 360.0 - 180.0

def estimate_robot_pose(frame, camera_params, arena_map):
    tags = capture_home_tag_function(frame, camera_params)
    if not tags:
        return None

    xmax = max((max(info["x"] for info in arena_map.values()), 2.0)) if arena_map else 2.0
    ymax = max((max(info["y"] for info in arena_map.values()), 2.0)) if arena_map else 2.0

    pose_estimates = []
    
    for tag in tags:
        if tag.tag_id not in arena_map:
            continue

        if tag.pose_t[2][0] <= 0:
            continue

        tag_info = arena_map[tag.tag_id]
        tangent_vec, inward_normal_vec = get_wall_axes(tag.tag_id)
        
        cam_pos_in_tag = -np.dot(tag.pose_R.T, tag.pose_t)
        local_tangent = -cam_pos_in_tag[0][0] / SCALE
        local_inward = -cam_pos_in_tag[2][0] / SCALE

        rx = tag_info["x"] + local_tangent * tangent_vec[0] + local_inward * inward_normal_vec[0]
        ry = tag_info["y"] + local_tangent * tangent_vec[1] + local_inward * inward_normal_vec[1]
        
        weight = 1.0 / max(tag.pose_t[2][0] / SCALE, 0.05)
        pose_estimates.append((tag.tag_id, rx, ry, local_tangent, local_inward, weight))

    if not pose_estimates:
        return None

    best_tag = max(pose_estimates, key=lambda x: x[5])
    _, robot_x, robot_y, _, _, _ = best_tag

    robot_x = max(0.0, min(robot_x, xmax))
    robot_y = max(0.0, min(robot_y, ymax))

    heading_vec_x = 0.0
    heading_vec_y = 0.0

    for tag in tags:
        if tag.tag_id not in arena_map or tag.pose_t[2][0] <= 0:
            continue
        
        tag_info = arena_map[tag.tag_id]
        weight = 1.0 / max(tag.pose_t[2][0] / SCALE, 0.05)
        
        vec_x = tag_info["x"] - robot_x
        vec_y = tag_info["y"] - robot_y
        
        angle_to_tag_world = math.degrees(math.atan2(vec_y, vec_x))
        angle_in_cam = math.degrees(math.atan2(tag.pose_t[0][0], tag.pose_t[2][0]))
        
        true_heading = angle_to_tag_world + angle_in_cam
        
        heading_vec_x += math.cos(math.radians(true_heading)) * weight
        heading_vec_y += math.sin(math.radians(true_heading)) * weight

    if abs(heading_vec_x) > 1e-6 or abs(heading_vec_y) > 1e-6:
        robot_heading = heading_from_vector(heading_vec_x, heading_vec_y)
    else:
        robot_heading = 0.0
        
    return robot_x, robot_y, robot_heading
