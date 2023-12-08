[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-24ddc0f5d75046c5622901739e7c5dd533143b0c8e959d652212380cedb1ea36.svg)](https://classroom.github.com/a/b-DAApyw)
# Project 2: Duckiebot Wheel Calibration Using Closed-Loop Controller

**Video Link:** [Replace with your video link]

## Objective:
Drive the Duckiebot in a straight line and a square pattern on the lab floor while integrating LED feedback. The Duckiebot should have a closed-loop controller that subscribes to the wheel ticks topics. Ideally, the difference in ticks between the two wheels should be zero when going in a straight line. When turning in place, one wheel will increment ticks positively and the other negatively. The goal is to have the robot move as stably as possible.

## Instructions:

### Notes:
You may choose to implement a PD (Proportional-Derivative) or a PID (Proportional-Integral-Derivative) controller. 

### 1. Wheel Calibration:
Before starting the project, calibrate the wheels for accurate movement. Adjust resolution parameters in the `wheel_encoder` node.

### 2. Build the Project on the Duckie:
- Execute these commands to build and launch the project code:
    ```
    catkin_make
    source devel/setup.bash
    roslaunch project2 project2.launch
    ```
- Ensure the Duckie changes colors every second.
- Verify that the color change is logged on the screen.

### 3. Project 2 Code:
Develop the code for the Duckiebot to perform these tasks:

#### a. LED Indication:
Before moving, the Duckiebot should use its LEDs to signal its intent to move.

#### b. Straight Line Movement (2 m):
The Duckiebot should move in a straight line for 2 meters, using LEDs to signal its intent.

#### c. Square Movement:
Navigate the Duckiebot in a square pattern on the lab floor. At each stop:
- Maximize the intensity of the rear LEDs.
- Change LED color to red, simulating brake lights.

#### d. Post-Movement LED Pattern:
After completing the square, display a cool pattern with the LEDs.

### Challenges Faced:
The main challenge we faced during this project was understanding the PID controller. We complicated the assignment by focusing too much\
on the ticks, when the solution was analyzing how the PID controller changed the robot's reaction. When the robot took a 90 degree turn,\
we wanted to reset the ticks and start fresh, which took a lot thought process and calculating. After taking a step back, we realized that\ 
the robot has a "true north" because of the PID controller. The robot will do anything to stay on the straight line it started on. To solve\ 
this problem, we needed to change the robot's "true north", which we defined as the setpoint, every time it turned the setpoint increases\ 
and is add/subtracted to the ticks. We also faced minor challenges like the friction and slippage from the environment, the robot wheels falling\ 
off and causing spikes in the tick values, the connection to the robot constently crashing, and inconsistent results. Overall, it was a fun learning\
experience.

### Submission:

1. Push your code to the repository.
2. Update the "Video Link:" section in this markdown file with a video showcasing your robot's performance.
