import re

with open('/home/pi/Raspi_codes/unibots2026/robot_2wd_12V/integration/motion_plan.py', 'r') as f:
    lines = f.readlines()

out_lines = []
for i, line in enumerate(lines):
    if line.strip() == 'if "vision" in globals() and vision:':
        continue
    if line.strip() == 'vision.update_active_camera(True)' or line.strip() == 'vision.update_active_camera(False)':
        continue
    
    out_lines.append(line)
    
    match_up = re.search(r'^(\s*)robot\.arm_up\(\)', line)
    if match_up:
        indent = match_up.group(1)
        out_lines.append(f"{indent}if 'vision' in globals() and vision:\n")
        out_lines.append(f"{indent}    vision.update_active_camera(True)\n")
        
    match_down = re.search(r'^(\s*)robot\.arm_down\(\)', line)
    if match_down:
        indent = match_down.group(1)
        out_lines.append(f"{indent}if 'vision' in globals() and vision:\n")
        out_lines.append(f"{indent}    vision.update_active_camera(False)\n")

with open('/home/pi/Raspi_codes/unibots2026/robot_2wd_12V/integration/motion_plan.py', 'w') as f:
    f.writelines(out_lines)

