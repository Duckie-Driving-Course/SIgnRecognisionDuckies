#!/usr/bin/env python3

import os
import rospy
from duckietown_msgs.msg import WheelsCmdStamped
from duckietown.dtros import DTROS, NodeType

CAR = os.environ['VEHICLE_NAME']
cmd = f'/{CAR}/car_cmd_switch_node/cmd'
car_cmd = f'/{CAR}/joy_mapper_node/car_cmd'
wheels_cmd = f'/{CAR}/wheels_driver_node/wheels_cmd'
wheels_cmd_executed = f'/{CAR}/wheels_driver_node/wheels_cmd_executed'
id_store_path = "/code/catkin_ws/src/SIgnRecognisionDuckies/packages/assets/sign_ids.txt"

class WheelsDriver(DTROS):
    def __init__(self, node_name):
        self.wheel = None
        self.twist = None
        self.timestamp = None
        self.tag_detected = False
        self.COMMAND_DICTIONARY = {
            "20": self.stop_command,
            "96": self.slow_down_command,
            "39": self.yield_command
        }

        super(WheelsDriver, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        self.wheels_cmd_pub = rospy.Publisher(
            wheels_cmd,
            WheelsCmdStamped,
            queue_size=1
        )

        self.wheels_cmd_sub = rospy.Subscriber(
            wheels_cmd,
            WheelsCmdStamped,
            self.wheels_cmd_cb,
            queue_size=1
        )

        self.wheels_cmd_executed_pub = rospy.Publisher(
            wheels_cmd_executed,
            WheelsCmdStamped,
            queue_size=1
        )

        self.wheels_cmd_executed_sub = rospy.Subscriber(
            wheels_cmd_executed,
            WheelsCmdStamped,
            self.wheels_cmd_executed_cb,
            queue_size=1
        )

    def stop_command(self):
        self.wheel.vel_left = 0
        self.wheel.vel_right = 0
        return 500

    def slow_down_command(self):
        self.wheel.vel_left = 0.05
        self.wheel.vel_right = 0.05
        return 2

    def yield_command(self):
        self.wheel.vel_left = 0
        self.wheel.vel_right = 0
        return 2

    def wheels_cmd_cb(self, wheels):
        # rospy.loginfo(f'Command: {type(wheels)}')
        pass

    def wheels_cmd_executed_cb(self, wheels):
        # rospy.loginfo(f'Command: {type(wheels)}')
        pass

    def run(self):
        self.wheel = WheelsCmdStamped()
        self.wheel.vel_right = 0.2
        self.wheel.vel_left = 0.2
        self.timestamp = rospy.Time.now().secs

        # self.twist = Twist2DStamped()
        # #self.twist.v = 40
        # self.twist.omega = 10

        detected_id = None
        resume = 10
        while not rospy.is_shutdown():
            elapsed_time = rospy.Time.now().secs - self.timestamp
            self.wheels_cmd_pub.publish(self.wheel)
            self.wheels_cmd_executed_pub.publish(self.wheel)
            f = open(id_store_path, "r")
            tag_id = f.read()
            f.close()
            print(tag_id)

            if detected_id != tag_id and tag_id != '':
                resume = self.COMMAND_DICTIONARY[tag_id]()
                self.timestamp = rospy.Time.now().secs
                detected_id = tag_id

            if elapsed_time > resume and detected_id:
                if tag_id == '' or tag_id == '39':
                    self.wheel.vel_right = 0.2
                    self.wheel.vel_left = 0.2
                    detected_id = None
                open(id_store_path, "w").close()
            print(f"{self.wheel.vel_right}, {self.wheel.vel_left}")

    def on_shutdown(self):

        rospy.loginfo('Shutting down...')
        self.wheel = WheelsCmdStamped()
        self.wheel.vel_right = 0
        self.wheel.vel_left = 0
        self.wheels_cmd_pub.publish(self.wheel)


if __name__ == "__main__":
    runner = WheelsDriver("wheels_driver_node")
    while not rospy.is_shutdown():
        try:
            runner.run()
            rospy.spin()
        except:
            rospy.on_shutdown(runner.on_shutdown)
