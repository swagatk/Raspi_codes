
Please execute the following steps in a sequence


## Step 1: Initialization phase: 
- Check both camera status (USB and Picamera). Automatically detect USB device. 
- Check motor status
- Check ultrasonic sensor status
- Check Line-sensor status
- Move the ARM to UP pose.
- Exit if any error is encountered in starting any of the above sensors/actuators.
- If there is no error in initialization, Wait for physical button press to start. see the file `../test/button_test.py`. Run a parallel thread to check button press.
- Once button is pressed, start timer to keep track of 3 minute (180 second) interval of autonomous operation.
- Once button is pressed, second press should exit the program - stopping the motors, going to down pose etc.


## Step 2: Confirm Home Location ~ 10 second
- Use USB camera to detect one of the home tags. If USB fails, use Picamera. 
- Detect any of the middle two tags visible in the frame and confirm that
    these are the same home tag mentioned in the `config.py` file. Show error, tags detected do not match those given in the config file and exit by showing a suitable error.
- If home tags are not visible, go back a little and adjust view until tags are visible.
- If tags are not detected within this time, print an error message and exit.
- If the first tag is detected, save the image along with tag detection annotation as `./log/home_tag_1.jpg`. If the second tag is detected, save the image as `./log/home_tag_2.jpg`. Annotate these images with their respective tag ID. 
- print a message showing the completion of this step.
- If time is saved in this part, please add the remaining time to the interval of next step. 

## Step 3: Ready to Explore ~ 10 seconds
- Robot turns by 180 degrees to face opposite to the home tag wall. Create a user-defined variable in `config.py` to be adjusted by the user.
- print message showing the completion of step 3.
- If the time is saved in this part, add the remaining time to the next step. 

## Step 4: Search and Approach balls ~ (110 seconds)
- Use USB camera for ball detection.
- Search for orange ball. 
    - Rotate slowly until ball is detected in its camera view. 
- Detect the ball:   
    - The nearest ball with largest area should be selected as a target.  
    - Apply aspect-ratio to avoid spurious detection. 
    - Use camera calibration parameters to compute ball distance.
    - Save the target as `./log/ball_target_{datetime}.jpg` showing the target distance, pixel area and target bounding box in cyan color and other ball detections in blue color.
- Approach the target ball: 
    - Rotate the robot slowly so that the target ball comes to the center of the camera view. 
    - Once the target is in the center of camera view, move the robot slowly towards the target.
    - Adjust robot orientation (very slowly) to ensure that the target ball remains in the center of camera view while the robot is approaching.
    - Once `ball_pick_distance` is reached, the robot should stop moving.  
- Grab the ball: Once `ball_pick_distance` is reached, perform the ball capture manoeuvre.
    - __ball capture manoeuvrer__: 
        - put the arm down and open the gripper.
        - Move slowly forward for (`SLOW_FORWARD_PICK_MOTION=2`) seconds while opening/closing gripper `MAX_OPEN_CLOSE_TIMES=3` times.
        - Whenever the arm is in Down Pose for ball picking, the robot should use the Picamera to detect the apriltags to know its distance from the wall (use the user-defined parameter: `safe_wall_distance`). 
        - When arm is down and the robot is moving forward use the central ultrasonic sensor ('C') and picamera calibration to avoid collision with other robots or wall using the same `SAFE_WALL_DISTANCE` value. DONOT use the L or R sensor values when arm is down. Robot should move back if this distance is reached.
        - Close the gripper one last time before lifting the arm to the UP position.
- When ARM is UP pose, avoid obstacle while moving by using all the three ultrasonic sensor (L, R, C) readings. 
- repeat the steps: search --> approach --> pick ball until the time for this phase is completed.
- Print relevant messages such as "searching for ball", "approaching the ball", "Grabbing the ball" etc
- Print a messaage "Completion of Step 4"

# Step 5: Returning home ~ 50 seconds
- Use USB camera first to estimate robot pose by detecting apriltag on the walls. If detection fails, use Picamera. If both fails, go back for small time (1 second) and try again. Repeat until a home tag is detected successfully.

- Localize: Estimate robot pose using the `map.py` file. Save the robot localization map as we did in `../go_to_home.py` file as `./log/localize_{datetime}.jpg`.  Also save the image with detected tags as `./log/tag_detection_{datetime}.png`.
- Approach the home tag:
    - Rotate incrementally by small angles until robot direction aligns with home_tag direction.   
    - Move linearly towards the target by short distances if there are no obstacles ahead. If there are obstacles, avoid obstacle and recalculate the pose.
- Repeat above steps: localize --> approach until home wall is reached.  If linear distance to wall less than `FINE_TUNING_HOME_DIST_CM=50`, move cautiously adjusting robot angle and forward motion, so that the detected home tag remains in the center of the camera view. 
- Stop the robot when it reaches close to home wall (using the ultrasonic sensor stop_distance threshold.) Use the `HOME_WALL_STOP_DISTANCE` to stop.
- Please check if the home wall distance is reached by checking the home tag distance. Move the arm to Drop pose only the robot is within the `HOME_WALL_STOP_DISTANCE`. If the robot is beyond this distance, keep the arm in UP pose with grippers closed. 
- After dropping the balls, the robot should wait for the button press to exit the program. 
- If the time expires before executing 'drop ball' sequence, it should wait in its current pose for the buttton press to exit the prgrame.
## Aditional points to be taken into account
- 
- The bottom USB camera is primarily used for detecting balls and tags whenever the arm is in UP pose.
- If the arm is in down pose, please use the Picamera to detect tags and compute distance from the wall. 
- If the arm is in UP pose, please use the USB camera to detect balls and april tags - to be used for navigation, searching purposes. 
- All user-defined variables should be provided in the single file `config.py`.
- Whenever the program exits or gets interrupted by the user, the robot arm should go back to DOWN pose.
- Program could be stopped at any time using Keyboard interrupt 'Ctrl+C'


