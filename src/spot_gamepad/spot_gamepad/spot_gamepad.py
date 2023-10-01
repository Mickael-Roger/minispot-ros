import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from time import time

import evdev


INPUT_DEVICE="/dev/input/event0"


laydown_button='BTN_A'
wakeup_button='BTN_Y'

class SpotGamepad(Node):

    def __init__(self):

        # Initialize Gamepad
        self.gamepad=evdev.InputDevice(INPUT_DEVICE)

        super().__init__('spot_gamepad')
        self.publisher_ = self.create_publisher(String, 'gamepad', 10)

        self.msg = String()
        


    def read_event(self):
        for event in self.gamepad.read_loop():
            if event.type == evdev.ecodes.EV_ABS:
                absevent = evdev.categorize(event)

                # If motion button
                if evdev.ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_HAT0Y':
                    code = absevent.event.value

                    if code == -1:
                        self.send_msg('{"type": "motion", "action":"forward","time":"' + str(time()) + '"}')
                    elif code == 1:
                        self.send_msg('{"type": "motion", "action":"backward","time":"' + str(time()) + '"}')
                    else:
                        self.send_msg('{"type": "motion", "action":"stop","time":"' + str(time()) + '"}')

                if evdev.ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_HAT0X':
                    code = absevent.event.value

                    if code == -1:
                        self.send_msg('{"type": "motion", "action":"left","time":"' + str(time()) + '"}')
                    elif code == 1:
                        self.send_msg('{"type": "motion", "action":"right","time":"' + str(time()) + '"}')
                    else:
                        self.send_msg('{"type": "motion", "action":"stop","time":"' + str(time()) + '"}')


                # If body position joystick
                if evdev.ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_X':
                    if absevent.event.value < 128:
                        self.send_msg('{"type": "motion", "action":"bodyleft","time":"' + str(time()) + '"}')
                    elif absevent.event.value > 128:
                        self.send_msg('{"type": "motion", "action":"bodyright","time":"' + str(time()) + '"}')
                    else:
                        self.send_msg('{"type": "motion", "action":"stop","time":"' + str(time()) + '"}')

                if evdev.ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_Y':
                    if absevent.event.value < 128:
                        self.send_msg('{"type": "motion", "action":"bodyfront","time":"' + str(time()) + '"}')
                    elif absevent.event.value > 128:
                        self.send_msg('{"type": "motion", "action":"bodyback","time":"' + str(time()) + '"}')
                    else:
                        self.send_msg('{"type": "motion", "action":"stop","time":"' + str(time()) + '"}')



                # If camera position joystick
                if evdev.ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_Z':
                    print('lateral camera: ' + str(absevent.event.value))

                if evdev.ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_RZ':
                    print('front camera: ' + str(absevent.event.value))


            # If action button
            if event.type == evdev.ecodes.EV_KEY:
                keyevent = evdev.categorize(event)

                # Laydown
                if laydown_button in evdev.ecodes.bytype[keyevent.event.type][keyevent.event.code]:
                    if keyevent.event.value == 0:
                        self.send_msg('{"type": "motion", "action":"laydown","time":"' + str(time()) + '"}')

                # Wake up
                if wakeup_button in evdev.ecodes.bytype[keyevent.event.type][keyevent.event.code]:
                    if keyevent.event.value == 0:
                        self.send_msg('{"type": "motion", "action":"wakeup","time":"' + str(time()) + '"}')




    def send_msg(self, message):

        self.msg.data = message

        self.publisher_.publish(self.msg)




def main(args=None):

    rclpy.init(args=args)

    gamepad=SpotGamepad()
    gamepad.read_event()

    gamepad.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
