#!/usr/bin/env python
import rospy
import sys, select, termios, tty

from ackermann_msgs.msg import AckermannDrive

keyBindings = {'w': [1, 0], 'd': [1, -1], 'a': [1, 1], 's': [-1, 0]}


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


speed = 0.5
turn = 0.25


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('/ackermann_cmd', AckermannDrive)
    rospy.init_node('keyop')

    x = 0
    th = 0
    status = 0

    try:
        while (1):
            key = getKey()
            print ('Key Pressed: ', key)

            if key in keyBindings.keys():
                x = keyBindings[key][0]
                th = keyBindings[key][1]
            else:
                x = 0
                th = 0
                if (key == '\x03'):
                    break
            msg = AckermannDrive()

            msg.speed = x * speed
            msg.acceleration = 1
            msg.jerk = 1
            msg.steering_angle = th * turn
            msg.steering_angle_velocity = 1
            print (msg)
            pub.publish(msg)

    except:
        print 'error'

    finally:
        msg = AckermannDrive()

        msg.speed = 0
        msg.acceleration = 1
        msg.jerk = 1
        msg.steering_angle = 0
        msg.steering_angle_velocity = 1
        pub.publish(msg)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)