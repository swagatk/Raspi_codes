import subprocess
import time

p = subprocess.Popen(["/home/pi/.virtualenvs/yolov8env/bin/python3", "/home/pi/Raspi_codes/unibots2026/robot_2wd_12V/integration/motion_plan.py"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
time.sleep(10)
p.terminate()
p.wait()
stdout, stderr = p.communicate()
print("STDOUT:", stdout.decode()[-500:])
print("STDERR:", stderr.decode()[-500:])
