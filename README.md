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
Document any specific challenges you encountered during the project here. This can include issues with coding, hardware, calibration, or any unexpected behaviors observed in the Duckiebot. This section is crucial for reflecting on the learning process and troubleshooting.

### Submission:

1. Push your code to the repository.
2. Update the "Video Link:" section in this markdown file with a video showcasing your robot's performance.
