#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from darm_msgs.msg import UiStatus
import gripper_msgs.msg
from sensor_msgs.msg import JointState

class JointStateBridge(Node):
    def __init__(self):
        super().__init__('joint_state_bridge')

        self.js_pub = self.create_publisher(JointState, '/svaya/joint_states', 10)

        self.create_subscription(UiStatus, '/svaya/ui/status', self.js_callback, 10)
        self.create_subscription(gripper_msgs.msg.TesolloStatus, "/svaya/ui/gripper/status",self.gripper_status_callback,10)

        self.joint_names = ['J1_left', 'J2_left', 'J3_left', 'J4_left', 'J5_left', 'J6_left', 'L_link7_to_flange',
                            'J1_right', 'J2_right', 'J3_right', 'J4_right', 'J5_right', 'J6_right', 'R_link7_to_flange', 'neck_joint', 'head_joint',
                            'L_F1M1', 'L_F1M2', 'L_F1M3', 'L_F1M4', 'L_F2M1', 'L_F2M2', 'L_F2M3', 'L_F2M4', 'L_F3M1', 'L_F3M2', 'L_F3M3', 'L_F3M4', 
                            'R_F1M1', 'R_F1M2', 'R_F1M3', 'R_F1M4', 'R_F2M1', 'R_F2M2', 'R_F2M3', 'R_F2M4', 'R_F3M1', 'R_F3M2', 'R_F3M3', 'R_F3M4']

    def js_callback(self, msg: UiStatus):
        joint_positions = np.concatenate((msg.left_arm.position, msg.right_arm.position, msg.head.position))

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = joint_positions.tolist()

        self.js_pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




from rclpy.node import Node
from sensor_msgs.msg import JointState
from darm_msgs.msg import UiStatus
import numpy as np
import rclpy

class JointStateMerger(Node):
    def __init__(self):
        super().__init__('joint_state_merger')

        # Publisher to /svaya/joint_states
        self.joint_pub = self.create_publisher(JointState, '/svaya/joint_states', 10)

        # Subscribers
        self.create_subscription(UiStatus, '/svaya/ui/status', self.arm_callback, 10)
        #self.create_subscription(GripperStatus, '/svaya/ui/gripper/status', self.gripper_callback, 10)
        self.create_subscription(gripper_msgs.msg.TesolloStatus, "/svaya/ui/gripper/status",self.gripper_callback,10)

        # Dict to store joint states
        self.joint_positions = {}
        #self.joint_names_arm = [f'arm_joint_{i}' for i in range(1, 8)]  # replace with actual names
        #self.joint_names_gripper = [f'gripper_joint_{i}' for i in range(1, 5)]  # replace with actual names
        self.joint_names_arm = ['J1_left', 'J2_left', 'J3_left', 'J4_left', 'J5_left', 'J6_left', 'L_link7_to_flange',
                            'J1_right', 'J2_right', 'J3_right', 'J4_right', 'J5_right', 'J6_right', 'R_link7_to_flange', 'neck_joint', 'head_joint']
        self.joint_names_gripper = ['L_F1M1', 'L_F1M2', 'L_F1M3', 'L_F1M4', 'L_F2M1', 'L_F2M2', 'L_F2M3', 'L_F2M4', 'L_F3M1', 'L_F3M2', 'L_F3M3', 'L_F3M4', 
                            'R_F1M1', 'R_F1M2', 'R_F1M3', 'R_F1M4', 'R_F2M1', 'R_F2M2', 'R_F2M3', 'R_F2M4', 'R_F3M1', 'R_F3M2', 'R_F3M3', 'R_F3M4']


    def arm_callback(self, msg: UiStatus):
        # Extract left + right arm positions
        positions = np.concatenate((msg.left_arm.position, msg.right_arm.position))
        for name, pos in zip(self.joint_names_arm, positions):
            self.joint_positions[name] = pos
        self.publish_joint_state()

    def gripper_callback(self, msg: GripperStatus):
        # Extract finger positions (example for 4 fingers)
        finger_positions = msg.tesollo_status[0].finger_status[0].position  # adapt indexing
        for name, pos in zip(self.joint_names_gripper, finger_positions):
            self.joint_positions[name] = pos
        self.publish_joint_state()

    def publish_joint_state(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.joint_positions.keys())
        js.position = [self.joint_positions[n] for n in js.name]
        self.joint_pub.publish(js)

def main():
    rclpy.init()
    node = JointStateMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
