import math
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg

def heading_from_vector(vec_x, vec_y):
    return math.degrees(math.atan2(vec_y, vec_x)) % 360.0

def normalize_rotation(angle_degrees):
    return (angle_degrees + 180.0) % 360.0 - 180.0

def get_home_target(arena_map, home_tags):
    home_points = [arena_map[tag_id] for tag_id in home_tags if tag_id in arena_map]
    if not home_points:
        return 1.0, 1.0 # fallback approximation
    mid_x = sum(point["x"] for point in home_points) / len(home_points)
    mid_y = sum(point["y"] for point in home_points) / len(home_points)
    return mid_x, mid_y

def generate_localization_image(robot_x, robot_y, robot_heading, arena_map, home_tags, save_path):
    fig = Figure(figsize=(8, 6), dpi=100)
    canvas = FigureCanvasAgg(fig)
    ax = fig.add_axes([0.1, 0.1, 0.6, 0.8])
    
    ax.set_xlim(-0.2, 2.2)
    ax.set_ylim(-0.2, 2.2)
    ax.set_aspect('equal')
    ax.set_title("Robot Arena")
    ax.grid(True)
    
    for tid, tinfo in arena_map.items():
        tx = tinfo["x"]
        ty = tinfo["y"]
        is_home = (tid in home_tags)
        c = 'blue' if is_home else 'black'
        
        # Determine labels for standard legend
        label = 'Home Tag' if (is_home and tid == home_tags[0]) else ('Arena Tag' if (not is_home and tid == 0) else "")
        
        if label:
            ax.plot(tx, ty, marker='s', color=c, markersize=8, linestyle='None', label=label)
        else:
            ax.plot(tx, ty, marker='s', color=c, markersize=8, linestyle='None')
            
        ax.text(tx, ty, f' {tid}', color=c, fontsize=9, verticalalignment='bottom')
        
    home_x, home_y = get_home_target(arena_map, home_tags)
    
    # Target Home Marker
    ax.plot(home_x, home_y, marker='x', color='dodgerblue', markersize=10, linestyle='None', label='Home Target')
    
    # Robot Position
    ax.plot(robot_x, robot_y, marker='o', color='red', markersize=12, linestyle='None', label='Robot')
    
    # Robot Orientation Heading
    rad = math.radians(robot_heading)
    dx_head = 0.2 * math.cos(rad)
    dy_head = 0.2 * math.sin(rad)
    ax.arrow(robot_x, robot_y, dx_head, dy_head, head_width=0.04, head_length=0.06, fc='red', ec='red')
    
    # Target path vector
    dx_t = home_x - robot_x
    dy_t = home_y - robot_y
    t_angle = heading_from_vector(dx_t, dy_t)
    turn_angle = normalize_rotation(t_angle - robot_heading)
    
    ax.arrow(robot_x, robot_y, dx_t, dy_t, head_width=0.03, head_length=0.05, fc='green', ec='green', 
             linestyle='--', length_includes_head=True, alpha=0.7, label='Target Path')
    
    dist = math.hypot(dx_t, dy_t)
    info_text = (f"=== TELEMETRY ===\n\n"
                 f"Pos X: {robot_x:.2f} m\n"
                 f"Pos Y: {robot_y:.2f} m\n"
                 f"Heading: {robot_heading:.1f}*\n\n"
                 f"--- ROUTE ---\n"
                 f"Dist: {dist:.2f} m\n"
                 f"Angle: {t_angle:.1f}*\n"
                 f"Turn: {turn_angle:+.1f}*")
                 
    fig.text(0.75, 0.5, info_text, transform=fig.transFigure, fontsize=12, ha='left', va='center', 
             family='monospace', bbox=dict(facecolor='white', alpha=0.9, edgecolor='black', boxstyle='round,pad=1'))
             
    ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1), borderaxespad=0.)
    
    canvas.draw()
    buf = canvas.buffer_rgba()
    map_img = np.asarray(buf)
    map_bgr = cv2.cvtColor(map_img, cv2.COLOR_RGBA2BGR)
    
    cv2.imwrite(save_path, map_bgr)
