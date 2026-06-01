import re

with open('/home/pi/Raspi_codes/unibots2026/robot_2wd_12V/integration/vision_module.py', 'r') as f:
    text = f.read()

old_update = """    def update_active_camera(self, arm_is_up):
        if arm_is_up:
            self.active_camera_params = self.usb_camera_params
            self.active_frame = self.frame
            self.tags = getattr(self, 'usb_tags', [])
        else:
            self.active_camera_params = getattr(self, 'picam_camera_params', self.usb_camera_params)
            self.active_frame = self.picam_frame
            self.tags = getattr(self, 'picam_tags', [])"""

new_update = """    def update_active_camera(self, arm_is_up):
        self.arm_is_up = arm_is_up
        
    @property
    def active_camera_params(self):
        return self.usb_camera_params if getattr(self, 'arm_is_up', True) else getattr(self, 'picam_camera_params', self.usb_camera_params)
        
    @property
    def active_frame(self):
        return self.frame if getattr(self, 'arm_is_up', True) else self.picam_frame
        
    @property
    def tags(self):
        return getattr(self, 'usb_tags', []) if getattr(self, 'arm_is_up', True) else getattr(self, 'picam_tags', [])"""

text = text.replace(old_update, new_update)

# We should also remove the direct initialization of tags and active_frame to avoid conflicts
text = text.replace("self.active_frame = None", "")
text = text.replace("self.tags = []", "    self.arm_is_up = True")

with open('/home/pi/Raspi_codes/unibots2026/robot_2wd_12V/integration/vision_module.py', 'w') as f:
    f.write(text)

