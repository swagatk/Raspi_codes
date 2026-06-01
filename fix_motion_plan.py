import re

with open('/home/pi/Raspi_codes/unibots2026/robot_2wd_12V/integration/motion_plan.py', 'r') as f:
    lines = f.readlines()

new_lines = []
skip_next = False
for line in lines:
    if "robot.stop()" in line and "logging.error" not in line and "sys.exit" not in line and "finally:" not in line and "except" not in line and "if robot is not None" not in line:
        pass
    
    # We want to remove robot.stop() and vision.stop() if they are right before sys.exit(1)
    if "robot.stop()" in line:
        if '        robot.stop()\n' == line:
            continue
    if "vision.stop()" in line:
        if '        vision.stop()\n' == line:
            continue
            
    new_lines.append(line)

with open('/home/pi/Raspi_codes/unibots2026/robot_2wd_12V/integration/motion_plan.py', 'w') as f:
    f.writelines(new_lines)
