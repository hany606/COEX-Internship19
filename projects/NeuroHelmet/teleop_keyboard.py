#!/usr/bin/env python
from __future__ import print_function

import rospy

from std_msgs.msg import String
import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to key_value
CTRL-C to quit
"""
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('key_value', String, queue_size = 10)
    rospy.init_node('teleop_keyboard')

    try:
        print(msg)
        while(1):
            key = getKey()
            print(key)
            if (key == '\x03'):
                break
            pub.publish(key)

    except Exception as e:
        print(e)

    finally:
        pub.publish("null")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)