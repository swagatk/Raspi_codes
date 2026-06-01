import re

with open('/home/pi/Raspi_codes/unibots2026/robot_2wd_12V/integration/motion_plan.py', 'r') as f:
    text = f.read()

old_status = """    # Simple check for camera start
    time.sleep(2)
    if vision.active_frame is None and vision.frame is None:
        logging.error("Camera status: FAILED to capture frames. Exiting.")
        sys.exit(1)
    else:
        logging.info("Camera status: OK")"""

new_status = """    # Check individual camera statuses
    time.sleep(2)
    usb_ok = getattr(vision, 'frame', None) is not None
    picam_ok = getattr(vision, 'picam_frame', None) is not None
    
    if usb_ok:
        logging.info("USB Camera status: OK")
    else:
        logging.error("USB Camera status: FAILED to capture frames.")
        
    if picam_ok:
        logging.info("PiCamera status: OK")
    else:
        logging.error("PiCamera status: FAILED to capture frames.")
        
    if not usb_ok and not picam_ok:
        logging.error("Both cameras FAILED to initialize. Exiting.")
        sys.exit(1)
    elif not usb_ok or not picam_ok:
        logging.warning("Proceeding with partial camera capabilities.")"""

text = text.replace(old_status, new_status)

with open('/home/pi/Raspi_codes/unibots2026/robot_2wd_12V/integration/motion_plan.py', 'w') as f:
    f.write(text)

