# Codes for 4WD Pi Robot

## Hardware
- [Robot Chassis Kit NP](https://thepihut.com/products/robot-chassis-kit-np)
- [4-Wheel motor driver](https://thepihut.com/products/motorshield-for-raspberry-pi)
- [Ultrasonic sensors](https://thepihut.com/products/ultrasonic-distance-sensor-hcsr04)

## Software
- Motor Control
- Reading ultrasonic sensors
- Simple Obstacle avoidance code

## Issues
- The motor hat uses almost 12 GPIO pins to drive 4 motors (3 pin / motor). 
So, if you are using 3 ultrasonic sensors, you will most likely reuse the same GPIO pins. 
There is no issue as long as they are used separately. Problem arises when you try to access them simultaneously, 
say, in obstacle avoidance algorithm. I partially solve this problem by releasing the pins immediately after use. 
I also tried parallel processing and it did not help much either. It boils down to making sure that each pin is accessed only by a single process at a given time.


