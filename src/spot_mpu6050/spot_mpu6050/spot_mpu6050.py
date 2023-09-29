import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from mpu6050 import mpu6050
from math import atan2

from time import sleep, time

MAX_MSG=10
LOOP_TIME_S=0.01


class SpotGyro(Node):

    def __init__(self):

        try:
            self.gyro=mpu6050(0x68)
        except:
            raise Exception("Could not connect to Gyroscope MPU6050")

        self.angleFront=0.
        self.angleSide=0.

        super().__init__('mpu6050')
        self.publisher_ = self.create_publisher(String, 'gyro', 10)

        timer_period = LOOP_TIME_S

        self.timer = self.create_timer(timer_period, self.get_position)


    def get_position(self):

        try:
            accelval = self.gyro.get_accel_data()
            gyroval = self.gyro.get_gyro_data()

            self.angleFront=0.80*(self.angleFront+float(gyroval['y'])*0.01/131) + 0.20*atan2(accelval['x'],accelval['z'])*180/3.14159
            self.angleSide=0.80*(self.angleSide+float(gyroval['x'])*0.01/131) + 0.20*atan2(accelval['y'],accelval['z'])*180/3.14159

            
            msg = String()
            msg.data = '{"front":"' + str(self.angleFront) + '","side":"' + str(self.angleSide) + '","time":"' + str(time()) + '"}'

            self.publisher_.publish(msg)

        except Exception:
            pass



def main(args=None):
    rclpy.init(args=args)

    gyroPublisher = SpotGyro()

    rclpy.spin(gyroPublisher)

    gyroPublisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

