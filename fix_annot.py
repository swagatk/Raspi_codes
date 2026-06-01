import re

with open('/home/pi/Raspi_codes/unibots2026/robot_2wd_12V/integration/motion_plan.py', 'r') as f:
    text = f.read()

old_fn = """def save_annotated_frame(vision, filename):
    if vision.frame is not None:
        annotated = vision.frame.copy()
        if vision.ball_detected:
            cv2.putText(annotated, f"Dist: {vision.ball_distance:.1f}cm", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        try:
            full_path = os.path.join(LOG_DIR, filename)
            cv2.imwrite(full_path, annotated)
        except Exception as e:
            logging.error(f"Failed to save image {full_path}: {e}")"""

new_fn = """def save_annotated_frame(vision, filename):
    if hasattr(vision, 'active_frame') and vision.active_frame is not None:
        annotated = vision.active_frame.copy()
    elif vision.frame is not None:
        annotated = vision.frame.copy()
    else:
        return
        
    if vision.ball_detected and hasattr(vision, 'ball_box') and vision.ball_box:
        x, y, w, h = vision.ball_box
        x1 = int(x - w / 2)
        y1 = int(y - h / 2)
        x2 = int(x + w / 2)
        y2 = int(y + h / 2)
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(annotated, f"Dist: {vision.ball_distance:.1f}cm | Area: {w*h:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    elif vision.ball_detected:
        cv2.putText(annotated, f"Dist: {vision.ball_distance:.1f}cm", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
    try:
        full_path = os.path.join(LOG_DIR, filename)
        cv2.imwrite(full_path, annotated)
    except Exception as e:
        logging.error(f"Failed to save image {full_path}: {e}")"""

text = text.replace(old_fn, new_fn)

with open('/home/pi/Raspi_codes/unibots2026/robot_2wd_12V/integration/motion_plan.py', 'w') as f:
    f.write(text)

