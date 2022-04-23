#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import can

from sensor_msgs.msg import Range
from rclpy.qos import qos_profile_sensor_data

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import math


SENSORS = [
    {'id': 0x00008D10, 'range': 0.2, 'name': 'front_left'},
    {'id': 0x00008D11, 'range': 0.2, 'name': 'front_right'},
    {'id': 0x00008D12, 'range': 0.2, 'name': 'back_left'},
    {'id': 0x00008D13, 'range': 0.2, 'name': 'back_right'},
]

INFRARED_CAN_ID = 0x00008D10
INFRARED_CAN_MASK = 0x1FFFFFF0


class InfraredDriver(Node):
    def __init__(self, can_bus):
        super().__init__('infrared_driver')
        self.bus = can_bus

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.__publisher = self.create_publisher(
            Range, 'infrared_sensors', qos_profile_sensor_data)

    def loop(self):
        while rclpy.ok():
            message = self.bus.recv(0.1)
            if not message:
                continue
            for sensor in SENSORS:
                if message.arbitration_id == sensor['id']:
                    range_msg = Range()
                    range_msg.header.frame_id = sensor['name']
                    range_msg.header.stamp = self.get_clock().now().to_msg()
                    range_msg.radiation_type = Range.INFRARED
                    range_msg.field_of_view = 0.523
                    range_msg.min_range = sensor['range']
                    range_msg.max_range = sensor['range']
                    
                    # If sensor is not detecting anything, we don't care about transforms
                    if message.data[0] == 0:
                        range_msg.range = math.inf
                        self.get_logger().info(f'Infrared sensor {sensor["name"]} RELEASED')
                        self.__publisher.publish(range_msg)
                        break

                    try:
                        now = rclpy.time.Time()
                        trans = self.tf_buffer.lookup_transform(
                            'map',
                            sensor['name'],
                            now)

                        trans_x = trans.transform.translation.x
                        trans_y = trans.transform.translation.y
                        x = trans.transform.rotation.x
                        y = trans.transform.rotation.y
                        z = trans.transform.rotation.z
                        w = trans.transform.rotation.w
                        trans_yaw = math.atan2(
                            2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

                        point_x = sensor['range'] * \
                            math.cos(trans_yaw) + trans_x
                        point_y = sensor['range'] * \
                            math.sin(trans_yaw) + trans_y

                        SHRINK = 0.1
                        if all([
                            (point_x >= -1.5 + SHRINK),
                            (point_x <= 1.5 - SHRINK),
                            (point_y >= -1.0 + SHRINK),
                            (point_y <= 1.0 - SHRINK)
                        ]):
                            # point is valid
                            self.get_logger().info(f'Infrared sensor {sensor["name"]} ENGAGED')
                            range_msg.range = -math.inf
                            self.__publisher.publish(range_msg)
                            break
                    except:
                        # print('tf2 lookup failed!')
                        break
            rclpy.spin_once()

                        
def main(args=None):
    rclpy.init(args=args)

    bus = can.ThreadSafeBus(bustype='socketcan',
                            channel='can0', bitrate=500000)

    # Set filters for receiving data
    bus.set_filters(filters=[{
        'can_id': INFRARED_CAN_ID,
        'can_mask': INFRARED_CAN_MASK,
        'extended': True
    }])
    driver = InfraredDriver(bus)
    driver.loop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()