# This code was taken and being modified from http://wiki.ros.org/teleop_twist_keyboard
import rospy

from std_msgs.msg import String
import sys, select, termios, tty
import pygame

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
    pygame.init()
    pygame.display.init()
    screen = pygame.display.set_mode((1,1))

    try:
        print(msg)
        while(1):
            # key = getKey()
            event = pygame.event.wait()
            key = ""
            print("type: ",event.type)

            if event.type == pygame.QUIT:
                break
            if event.type == pygame.KEYDOWN:
                key += event.unicode
                print("HELLO")
                if event.key == pygame.K_ESCAPE or event.unicode == 'q':
                    break
            print(key)
            
            # \x03 is the encoding of CTRL-C
            if (key == '\x03'):
                break

            pub.publish(key)

    except Exception as e:
        print(e)

    finally:
        pub.publish("null")
        pygame.quit()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


cat << EOF | sudo tee /etc/wpa_supplicant/wpa_supplicant.conf
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=GB

network={
    ssid="CLEVER-3808-6145_Hany"
    psk="cleverwifi_Hany"
    mode=2
    proto=RSN
    key_mgmt=WPA-PSK
    pairwise=CCMP
    group=CCMP
    auth_alg=OPEN
}

