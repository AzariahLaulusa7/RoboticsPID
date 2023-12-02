#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA
import random

def project2():
    '''
    Sample Code. Replace with your Project 2 code
    '''
    pub = rospy.Publisher('led_driver_node/led_pattern', LEDPattern, queue_size=10)
    rospy.init_node('try_stuff')

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        led = LEDPattern()
        rospy.loginfo("Changed the LED color")
        color = ColorRGBA()
        color.r = random.uniform(0, 1)
        color.g = random.uniform(0, 1)
        color.b = random.uniform(0, 1)
        color.a = 1
        led.rgb_vals = [color, color, color, color, color]
        pub.publish(led)
        rate.sleep()
    '''
    End of sample code
    '''
if __name__ == '__main__':
    try:
        project2()
    except rospy.ROSInterruptException:
        pass
