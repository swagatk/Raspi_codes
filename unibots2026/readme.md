# Building a 2WD Robot: The "Master-Slave" Architecture

## Introduction
This guide details how to build a robust, two-wheeled autonomous robot. Unlike simple beginner robots, this project uses a Hybrid Architecture:
* The Brain (Raspberry Pi 5): Runs the high-level logic, decision making, and Python scripts.
* The Muscle (Arduino Uno/Nano): Handles real-time motor control and precise sensor timing.
This separation ensures your robot reacts instantly to obstacles without the "jitter" caused by the Raspberry Pi operating system.

## Bill of Materials (BOM)
### Electronics
* Raspberry Pi 4 (Running Bookworm OS)
* Arduino Uno or Nano (Connected via USB cable)
* L298N Motor Driver Module ([ThePiHut](https://thepihut.com/products/l298n-motor-stepper-driver))
* 12V DC Motors (100 RPM) x2 ([ThePiHut](https://thepihut.com/products/dc-motor-12v-100rpm-inc-gearbox-wheel-silver-coupler-and-bracket-metal-gear))
* Ultrasonic Sensors (HC-SR04) x3 ([ThePiHut](https://thepihut.com/products/ultrasonic-distance-sensor-hcsr04))
* Push Button (Momentary switch)

### Power
* 11.1V LiPo Battery (For Motors) [[RSonline](https://uk.rs-online.com/web/p/rechargeable-battery-packs/1449414)]
* Power Bank or USB-C Supply (For Raspberry Pi) [[ThePiHut](https://thepihut.com/products/ansmann-10-000mah-type-c-18w-pd-power-bank)]
* Jumper Wires (Male-Male, Male-Female, Female-Female)

### Chassis
* Robot Chassis/Body  (180mm wide octagonal body) [[GitHub](https://github.com/swagatk/freecad_designs/tree/main/mobile_robot_base/mobile_base_180mm)]
* 65mm Wheels x2 ([ThePiHut](https://thepihut.com/products/wheel-pair-in-white-65mm-x-25mm))
* Supporting Caster Wheel (For balance) ([ThePiHut](https://thepihut.com/products/supporting-swivel-caster-wheel-1-3-diameter))

## Hardware Assembly & Wiring
### Step A: The Power Train (Motors & Driver)
**Safety Warning**: Never connect the 11.1V battery directly to the Raspberry Pi or Arduino. 

Connection for Motor Driver (L298N driver):

1. Motors: Connect Left Motor to OUT1/OUT2. Connect Right Motor to OUT3/OUT4.
2. Battery: Connect 11.1V Red (+) to 12V Input. Connect Black (-) to GND.
3. Arduino Power: Connect the Arduino via USB cable to the Raspberry Pi.
4. Common Ground (CRITICAL): Connect a wire from L298N GND to Arduino GND.

### Step B: Arduino Connections
We use the Arduino to handle the 5V logic of the sensors and driver.

#### Motor Driver Control:
* Remove the black plastic jumper caps from ENA and ENB on the L298N.

* ENA  Arduino Pin 10 (PWM Speed)
* IN1  Arduino Pin 8
* IN2  Arduino Pin 9
* IN3  Arduino Pin 12
* IN4  Arduino Pin 13
* ENB  Arduino Pin 11 (PWM Speed)

### Ultrasonic Sensors:
* Left Sensor: Trig  Pin 2, Echo  Pin 3
* Center Sensor: Trig  Pin 4, Echo  Pin 5
* Right Sensor: Trig  Pin 6, Echo  Pin 7
* Power: Connect all Sensor VCC pins to Arduino 5V and GND to GND.

### Step C: Raspberry Pi Connections
The Pi only needs two connections: one data link and one control button.
1. Data: Connect Arduino USB cable to any Pi USB port.
2. Start/Stop Button:
    * One leg  GPIO 25 (Physical Pin 22)
    * Other leg  GND (Physical Pin 20)

## Software Setup (Raspberry Pi)
On the Raspberry Pi 5 (Bookworm), we need to install the necessary libraries and grant permission to use the USB port.

1. Install Libraries:
Open a terminal and run:
```
sudo apt update
sudo apt install python3-serial python3-gpiozero python3-rpi-lgpio arduino
```

2. Grant USB Permissions:
Allow your user to access the Serial Port without sudo:

```
sudo usermod -a -G dialout $USER
```

Reboot your Pi after running this command (sudo reboot).

All the codes for this robot is available at this GITHUB link: https://github.com/swagatk/Raspi_codes/tree/master/unibots2026/robot_2wd_12V

## The Arduino Firmware (C++)
This code turns the Arduino into a "dumb" slave that listens for commands and reports sensor data.
1. Open Arduino IDE on the Pi.
2. Select Tools > Board > Arduino Uno.
3. Select Tools > Port > /dev/ttyACM0 (or similar).
4. Paste and Upload this code: `arduino_motor_test/arduino_motor_test.ino`

## Motor Test
It is important to test that the motors are properly connected. To check this, run the following command on Raspberry pi terminal:
```
python3 motor_test.py
```
if the motors not moving as expected, you need to change the following part of the code inside `arduino_motor_test/arduino_motor_test.ino` file:

```
// Interpret Command
    if (command == 'F') setMotors(motorSpeed, -motorSpeed);        // Forward
    else if (command == 'B') setMotors(-motorSpeed, motorSpeed); // Backward
    else if (command == 'L') setMotors(motorSpeed, motorSpeed);  // Left Spin
    else if (command == 'R') setMotors(-motorSpeed, -motorSpeed);  // Right Spin
    else if (command == 'S') stopMotors();                        // Stop
    // Speed control (1 = Slow, 2 = Medium, 3 = Fast)
    else if (command == '1') motorSpeed = 100;
    else if (command == '2') motorSpeed = 180;
    else if (command == '3') motorSpeed = 255;
  
```
Mostly, you will have change the negative sign to ensure that the motor is moving as required. Don't forget to upload the modify the code to arduino before running the python script. 

## Obstacle Avoidance (Python)
This script runs on the Raspberry Pi. It uses Threading to read sensors in the background while the main loop handles decision making and the button press. Use the source code file: `avoid_obstacle_with_button.py` to run your robot as described below. 

## How to Run Your Robot
1. Power Up: Turn on the 11.1V battery switch and power on the Raspberry Pi.
2. Upload Firmware: (Only needed once) Ensure Arduino has the code from Section 4.
3. Start Script: Run the following command on a terminal: 
```
python3 avoid_obstacle_with_button.py 
```
4. Action: Press the physical button connected to GPIO 25.
5. Stop: Press the button again to pause.

## Troubleshooting
* Robot is slow: Ensure ser.write(b'3') is being sent in the Python script to enable max speed.
* Robot spins in circles: Swap the motor wires (OUT1 & OUT2) or (OUT3 & OUT4) on the L298N to reverse direction.
* "Serial Error": Check if your port is /dev/ttyACM0 or /dev/ttyUSB0 using ls /dev/tty*.

