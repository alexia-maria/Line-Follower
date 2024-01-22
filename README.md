# Line-Follower
This project is part of the Introduction to Robotics course, undertaken during the third-year studies at the Faculty of Mathematics and Computer Science, University of Bucharest. It was fulfilled in a team of 3, La Furia Roja, alongside [@MariusAlexandru358](https://github.com/MariusAlexandru358) and [@Mirela89](https://github.com/Mirela89)


A line follower robot is a device with the primary function of autonomously tracking a predefined path marked on the ground. 

It has a reflectance sensor underneath, acting as its' eyes, that constantly watches for a specific color or contrast, usually a dark line on a lighter surface. As the robot moves along, the sensor communicates with its brain, letting it know where the line is. If the sensor sees the line on the left, the robot adjusts to the left; if on the right, it adjusts to the right. This communication helps the robot make quick decisions to keep itself right on the path, using its wheels to make precise adjustments. 

# How it works
The line follower incorporates an automatic sensor calibration feature for precise navigation. The calibration process involves the robot moving left and right along its axis, responding to the sensor readings.

The motors' speed is dynamically controlled using a PID (Proportional-Integral-Derivative) control mechanism. After a lot of testing on the final track, optimal values for the PID constants were determined, with KP set at 8, KI at 0.0001, and KD at 2.

# Components

| Component  | Link |
| ------------- | ------------- |
| Arduino Uno  | https://docs.arduino.cc/hardware/uno-rev3|
| Half Size Breadboard  | https://www.kiwi-electronics.com/en/400pt-half-size-breadboard-white-283|
| LIPo battery | https://www.techtarget.com/whatis/definition/lithium-polymer-battery-LiPo|
| QTR-8A reflectance sensor  | https://www.pololu.com/product/960|
| L293D Motor Driver | https://www.instructables.com/How-to-use-the-L293D-Motor-Driver-Arduino-Tutorial/  |
| DC Motors(2) + Wheels(2) | https://www.tutorialspoint.com/arduino/arduino_dc_motor.htm |
| Other components | zip ties, jumper wires (M/M & F/M), ball caster, chassis |

# Setup
The chassis was designed and built by our team, using a material similar to carboard which can be found in any art shop. The board was cut as to fit all the components on it. We also built a small "pocket" for the battery, in order to insert/remove it easily.

<p align="center">
  <img src="Line-Follower-picture.jpeg" alt="Line-Follower-picture" width="500">
</p>

# Functionality
For the grading we had 3 tries, meaning the robot had to complete 3 consecutive laps on the final track. In order to get the maximum grade, the track had to be completed in less than 20 seconds. In the video linked below is displayed the first lap, where the robot completed the track in aprox. 17 seconds. On the second lap, the robot completed the track faster, scoring a time of aprox. 16 seconds.

[Watch the video](https://www.youtube.com/shorts/_sgrDiwM9yc) to see this functionality in action.


# [Code] (https://github.com/alexia-maria/Line-Follower/blob/main/Line-Follower.ino)
