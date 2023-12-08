#!/usr/bin/env python3
#Created by Azariah Laulusa and Ky Nguyen

import rospy
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA
import random
import time
from math import pi
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped

#Global variables for left and right ticks
left_ticks = 0
right_ticks = 0
setpoint = 0 #Setpoint used for when the bot turns

#Callback functions for tick values
def left_encoder_callback(msg):
    global left_ticks
    left_ticks = msg.data * -1

def right_encoder_callback(msg):
    global right_ticks
    right_ticks = msg.data * -1


#Square Movement
def go_in_square():
    global setpoint
    pub = rospy.Publisher("wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size =1)
    cmd = WheelsCmdStamped()
    count = 0
    rospy.sleep(2)
    lights_move()
    while not rospy.is_shutdown():
        #Bobby moving forward
        lights_move()
        #Calculating timeout to move 1 meter
        #d = 1m
        #Vr = Vl = 0.5 m/s
        #t = d/v = 1m/0.5(m/s) = 2s
        #timeout should be 2 seconds long to move 1 meter
        #time increased due to the environment like friction and slippage
        timeout = time.time() + 3.5
        move_straight(timeout)
        #stopping Bobby
        lights_stop()
        cmd.vel_left = 0
        cmd.vel_right = 0
        pub.publish(cmd)
        rospy.sleep(3)
        #turning 90 degree
        lights_move()
        #rospy.sleep(2)
        cmd.vel_left = -0.5
        cmd.vel_right = 0.5
        pub.publish(cmd)
        #Calculations
        # Vl = -0.5
        # Vr = 0.5
        # l = 0.1m
        # W = (-0.5-0.5)/0.1 = 1/(1/10) = 10
        # Degree = pi/2
        # t = (pi/2)/10 = pi/20 = 0.157
        # We had to increase our speed a bit because of the environment like friction and slippage
        rospy.sleep(0.295) #0.395 (original speed)
        #Increment setpoint by 46 after every turn 
        setpoint += 46
        #stopping Bobby again
        lights_stop()
        cmd.vel_right = 0
        cmd.vel_left = 0
        pub.publish(cmd)
        rospy.sleep(3)
        count +=1
        if count == 4: #Performing loop 4 times for a square shape
            break

#Straight Line Movement
def move_straight(timeout):

    global setpoint

    #Calculating timeout that is used to move straight 2 meters
        #d = 2m
        #Vr = Vl = 0.5 m/s
        #t = d/v = 2m/0.5(m/s) = 4s
        #timeout should be 4 seconds long to move 2 meters
        #Increased the timeout due to PID adjustments and the environment (slippage and friction)

    #Subscribe to wheel encoder
    rospy.Subscriber('left_wheel_encoder_node/tick',WheelEncoderStamped,left_encoder_callback)
    rospy.Subscriber('right_wheel_encoder_node/tick',WheelEncoderStamped,right_encoder_callback)

    pub = rospy.Publisher("wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)
    cmd = WheelsCmdStamped()

    #Define PID parameters and tick values
    Kp = 0.1
    Ki = 0.001
    Kd = 0.001
    integral = 0
    base_velocity = 0.5
    previous_error = 0
    error_rate = 0
    output = 0

    #Conversion variable to put a limit on output
    conversion = 44.3

    #Signal intent with LEDs
    lights_move()

    #Function rate
    rate = rospy.Rate(10)

    while True:

        #Calculate control output
        rospy.loginfo("old_ticks")
        print(right_ticks)
        print(left_ticks)

        #Calculations and updates
        error = ((left_ticks - setpoint) - (right_ticks + setpoint))
        sampling_rate = 0.1
        error_rate = error - previous_error
        integral += error
        output = Kp*error+Ki*integral*sampling_rate+((Kd*error_rate)/sampling_rate) #PID controller
        print(output)

        #Divide output by conversion to get smaller changes
        output = output / conversion
        rospy.loginfo("after conversion")
        print(output)
        
        #Adjust only the left velocity using the output from the PID controller
        cmd.vel_right = base_velocity * -1
        cmd.vel_left = (base_velocity - output) * -1
        pub.publish(cmd)
        rospy.loginfo("right velocity")
        print(cmd.vel_right)
        rospy.loginfo("left velocity")
        print(cmd.vel_left)

        #Update previous_error
        previous_error = error
        if(time.time() > timeout):
            break

        #Sleep
        rate.sleep()


    #Prepare to stop
    lights_stop()
    cmd.vel_left = 0
    cmd.vel_right = 0
    rospy.sleep(2)
    pub.publish(cmd)


#LED Movement Indication
def lights_move():

    pub = rospy.Publisher('led_driver_node/led_pattern', LEDPattern, queue_size=10)
    rate = rospy.Rate(1)
    led = LEDPattern()

    #Display log information for moving LEDs
    rospy.loginfo("Changing to moving LED colors")

    #Color variable for all LEDs
    color = ColorRGBA()
    color.r = 0
    color.g = 1
    color.b = 0
    color.a = 0.5

    #Display moving LEDs
    led.rgb_vals = [color, color, color, color, color]
    pub.publish(led)

    #Sleep
    rospy.sleep(2)


#LED Brake Lights
def lights_stop():

    pub = rospy.Publisher('led_driver_node/led_pattern', LEDPattern, queue_size=10)
    rate = rospy.Rate(1)
    led = LEDPattern()

    #Display log information for stopping LEDs
    rospy.loginfo("Changing to stopping LED colors")

    #Variables for front LEDs and back LEDs
    front_color = ColorRGBA()
    back_color = ColorRGBA()

    #Assign color values for front LEDs
    front_color.r = 0
    front_color.g = 1
    front_color.b = 0
    front_color.a = 0.5

    #Assign color values for back LEDs
    back_color.r = 1
    back_color.g = 0
    back_color.b = 0
    back_color.a = 1

    #Display stopping LEDs
    led.rgb_vals = [front_color, back_color, back_color, front_color, front_color]
    pub.publish(led)

#Post-Movement LED Pattern
def christmas_lights():
    
    pub = rospy.Publisher('led_driver_node/led_pattern', LEDPattern, queue_size=10)
    rate = rospy.Rate(1)

    #Variables to keep track of previous LED colors and color change count
    previous_pattern = None
    count = 0

    #Loop for a continuous display of Christmas colors
    while not rospy.is_shutdown():
        led = LEDPattern()
        rospy.loginfo("Running Christmas lights pattern")

        #Define Christmas colors: red, green, and gold
        colors = [ 
            ColorRGBA(r=1, g=0, b=0, a=1),#red
            ColorRGBA(r=0, g=1, b=0, a=1),#green
            ColorRGBA(r=1, g=1, b=1, a=1),#white
            ColorRGBA(r=1, g=0.84, b=0, a=1),#gold
            ]

        #Create a pattern resembling blinking Christmas lights
        pattern = [random.choice(colors) for _ in range(5)]

        #Getting previous LEDs color
        if previous_pattern:
            pattern = [previous_pattern[(i-1)%len(pattern)] for i in range(len(pattern))]
        led.rgb_vals = pattern
        pub.publish(led)
        rate.sleep()
        previous_pattern = pattern

        #Keep track of the amount of color changes
        count += 1
        if(count == 4):#Once the colors reach a full loop then reset pattern
            previous_pattern = None
            count = 0


#Main
if __name__ == '__main__':
    rospy.init_node('project2_PI_controller')
    try:
        rospy.sleep(3)
        lights_move()
        rospy.sleep(2)
        timeout = time.time() + 5#moving 2 meters
        move_straight(timeout)
        rospy.sleep(2)
        go_in_square()
        rospy.sleep(2)
        lights_stop()
        rospy.sleep(2)
        christmas_lights()
    except rospy.ROSInterruptException:
        pass
