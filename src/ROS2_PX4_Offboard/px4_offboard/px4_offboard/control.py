#!/usr/bin/env python3
import geometry_msgs.msg
import std_msgs.msg
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import sys

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


CONTROL_MESSAGES = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. 
Using the arrow keys and WASD you have Mode 2 RC controls.
W: Up
S: Down
A: Yaw Left
D: Yaw Right
Up Arrow: Pitch Forward
Down Arrow: Pitch Backward
Left Arrow: Roll Left
Right Arrow: Roll Right

Press SPACE to arm/disarm the drone
"""

move_bindings = {
    'w': (0, 0, 1, 0),
    's': (0, 0,-1, 0),
    'a': (0, 0, 0,-1),
    'd': (0, 0, 0, 1),
    '\x1b[A' : ( 0, 1, 0, 0), # Up Arrow
    '\x1b[B' : ( 0,-1, 0, 0), # Down Arrow
    '\x1b[C' : (-1, 0, 0, 0), # Right Arrow
    '\x1b[D' : ( 1, 0, 0, 0), # Left Arrow
}
# criar tecla pra zerar

# speed_binding = {
#     'q': (1.1, 1.1),
#     'z': (.9, .9),
#     'w': (1.1, 1),
#     'x': (.9, 1),
#     'e': (1, 1.1),
#     'c': (1, .9),
# }

keys = ['w', 's', 'a', 'd', '\x1b[A', '\x1b[B]', '\x1b[C]', '\x1b[D]']


def get_key(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)

        if key == '\x1b': # if the first character is \x1b, we might be dealing with an arrow key
            additional_chars = sys.stdin.read(2) # read the next two characters
            key += additional_chars # append these characters to the key

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    return key


def save_terminal_settings():
    if sys.platform == 'win32':
        return None
    
    return termios.tcgetattr(sys.stdin)


def restore_terminal_settings(old_settings):
    if sys.platform == 'win32':
        return
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    settings = save_terminal_settings()

    rclpy.init(args=args)

    namespace = sys.argv[1] if len(sys.argv) > 1 else 'px4_1'
    teleop = rclpy.create_node(f'{namespace}_teleop_twist_keyboard')

    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )

    velocity_publisher = teleop.create_publisher(geometry_msgs.msg.Twist, f'/{namespace}/offboard_velocity_cmd', qos_profile) # offboard velocity message
    arm_publisher = teleop.create_publisher(std_msgs.msg.Bool, f'/{namespace}/arm_message', qos_profile) # arm/disarm message

    arm_toggle = False

    speed = 0.5
    turn = .2
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0
    x_val = 0.0
    y_val = 0.0
    z_val = 0.0
    yaw_val = 0.0

    try:
        print(f'UAV Namespace: {namespace}')
        print(CONTROL_MESSAGES)

        while True:
            key = get_key(settings)

            if key in move_bindings.keys():
                x = move_bindings[key][0]
                y = move_bindings[key][1]
                z = move_bindings[key][2]
                th = move_bindings[key][3]
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0

                if key == '\x03':
                    break

            if key == ' ':  # ASCII value for space
                arm_toggle = not arm_toggle
                arm_message = std_msgs.msg.Bool()
                arm_message.data = arm_toggle

                arm_publisher.publish(arm_message)
                print(f"Arm toggle is now: {arm_toggle}")

            twist = geometry_msgs.msg.Twist()
            
            x_val += (x * speed)
            y_val += (y * speed)
            z_val += (z * speed)
            yaw_val += (th * turn)
            twist.linear.x = x_val
            twist.linear.y = y_val
            twist.linear.z = z_val
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = yaw_val

            velocity_publisher.publish(twist)
            print("X:",twist.linear.x, "   Y:",twist.linear.y, "   Z:",twist.linear.z, "   Yaw:",twist.angular.z)

    except Exception as e:
        print(e)

    finally:
        twist = geometry_msgs.msg.Twist()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        velocity_publisher.publish(twist)

        restore_terminal_settings(settings)


if __name__ == '__main__':
    main()