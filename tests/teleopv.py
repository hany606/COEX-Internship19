# This code was taken and being modified from http://wiki.ros.org/teleop_twist_keyboard
import rospy

from std_msgs.msg import String
import sys, select, termios, tty

msg = """
Reading from the keyboard and Publishing to /keyboard_value topic!

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

    pub = rospy.Publisher('keyboard_value', String, queue_size = 10)
    rospy.init_node('keyboard_listner')

    try:
        print(msg)
        while(1):
            key = getKey()
            print(key)
            
            # \x03 is the encoding of CTRL-C
            if (key == '\x03'):
                break

            pub.publish(key)

    except Exception as e:
        print(e)

    finally:
        pub.publish("null")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
