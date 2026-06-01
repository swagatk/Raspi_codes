with open('/home/pi/Raspi_codes/unibots2026/robot_2wd_12V/integration/vision_module.py', 'r') as f:
    lines = f.readlines()

new_lines = []
for line in lines:
    if line.strip() == 'self.ball_box = None':
        continue
    new_lines.append(line)
    if line.strip() == 'self.ball_detected = False':
        new_lines.append('        self.ball_box = None\n')

with open('/home/pi/Raspi_codes/unibots2026/robot_2wd_12V/integration/vision_module.py', 'w') as f:
    f.writelines(new_lines)
