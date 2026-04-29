
Keep checking for a button press in a thread. Once the button is pressed, 
the following activities should take place. If the button is pressed again, the robot should stop moving. 


## Step 1: Initialization phase: 
- Check camera status
- Check motor status
- Check ultrasonic sensor status
- Check button status
- Move the ARM to UP pose.
- Wait for button press to start


## Step 2: Confirm Home Location ~ 20 second
- Detect the middle two tags visible in the frame and confirm that
    these are the same home tag mentioned in the `config.py` file.
- If home tags are not visible, go back a little and adjust view until tags are visible.
- If tags are not detected within this time, print an error message and exit.
- If tags are successfully detected and confirmed, save the image as './log/home_tag.jpg'
- print a message showing the completion of this step.


## Step 3: Exploration ~ 2 minutes
- Search for orange ball
- Approach the ball
- Grab the ball
- Avoid obstacle while moving
- repeat the above 3 steps until the time is completed.
- save images every time target is detected with distance and target annotations as`./log/ball_detect_{time}.jpg`. 
- Print relevant messages such as "searching for target", "approaching the target", "Grabbing the ball" etc
- Use `go_to_ball.py` for information.

# Step 4: Returning home ~ 40 seconds
- Estimate robot pose using the `map.py` file.
- Save the robot localization map as we did in `go_to_home.py` file as `./log/localization.jpg`
- Approach the home tag. Look into `go_to_home.py` file.
- avoid obstacle while moving.
- Drop the ball.
- Go to arm down pose before exiting. 


