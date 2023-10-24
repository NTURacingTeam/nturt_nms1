# modified from https://github.com/f1tenth/f1tenth_gym_ros/blob/main/f1tenth_gym_ros/gym_bridge.py

import gym
import numpy as np
from transforms3d import euler
import os

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan


class F1tenthGymRosBridge(Node):

    def __init__(self):
        super().__init__("f1tenth_gym_ros_bridge")

        self.declare_parameter("drive_topic", "drive")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("scan_topic", "scan")

        self.declare_parameter("tf_ns", "f1tenth")
        self.declare_parameter("map_frame", "map")

        self.declare_parameter(
            "map",
            os.path.join(get_package_share_directory("nturt_nms1_sim"),
                         "maps", "levine.png"))
        self.declare_parameter("using_teleop", True)

        self.declare_parameter("sx", 0.0)
        self.declare_parameter("sy", 0.0)
        self.declare_parameter("stheta", 0.0)
        self.declare_parameter("scan_fov", 4.7)
        self.declare_parameter("scan_beams", 1080)

        drive_topic = self.get_parameter("drive_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        scan_topic = self.get_parameter("scan_topic").value

        self.tf_ns = self.get_parameter("tf_ns").value
        if self.tf_ns != "":
            self.tf_ns = self.tf_ns + "/"
        self.map_frame = self.get_parameter("map_frame").value

        map = self.get_parameter("map").value
        using_teleop = self.get_parameter("using_teleop").value

        sx = self.get_parameter("sx").value
        sy = self.get_parameter("sy").value
        stheta = self.get_parameter("stheta").value
        scan_fov = self.get_parameter("scan_fov").value
        scan_beams = self.get_parameter("scan_beams").value

        self.pose = [sx, sy, stheta]
        self.speed = [0.0, 0.0, 0.0]
        self.requested_speed = 0.0
        self.requested_steer = 0.0

        self.angle_min = -scan_fov / 2.0
        self.angle_max = scan_fov / 2.0
        self.angle_inc = scan_fov / (scan_beams - 1)

        # env backend
        self.env = gym.make("f110_gym:f110-v0",
                            map=os.path.splitext(map)[0],
                            map_ext=os.path.splitext(map)[1],
                            num_agents=1)
        self.obs, _, self.done, _ = self.env.reset(np.array([[sx, sy,
                                                              stheta]]))
        self.scan = list(self.obs["scans"][0])

        # tramsform broadcaster
        self.br = TransformBroadcaster(self)

        # publishers
        self.joint_states_pub = self.create_publisher(JointState,
                                                      "joint_states", 10)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.scan_pub = self.create_publisher(LaserScan, scan_topic, 10)

        # subscribers
        self.drive_sub = self.create_subscription(AckermannDriveStamped,
                                                  drive_topic, self.onDrive,
                                                  10)
        self.reset_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                  "/initialpose", self.onReset,
                                                  10)
        if using_teleop:
            self.teleop_sub = self.create_subscription(Twist, "/cmd_vel",
                                                       self.onTeleop, 10)

        # timer
        self.update_sim_timer = self.create_timer(
            0.01, self.update_sim_timer_callback)
        self.update_state_timer = self.create_timer(
            0.01, self.update_state_timer_callback)

    def onDrive(self, msg):
        self.requested_speed = msg.drive.speed
        self.requested_steer = msg.drive.steering_angle

    def onReset(self, msg):
        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y

        rqx = msg.pose.pose.orientation.x
        rqy = msg.pose.pose.orientation.y
        rqz = msg.pose.pose.orientation.z
        rqw = msg.pose.pose.orientation.w
        _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes="sxyz")

        self.obs, _, self.done, _ = self.env.reset(np.array([[rx, ry,
                                                              rtheta]]))

    def onTeleop(self, msg):
        self.requested_speed = msg.linear.x
        self.requested_steer = msg.angular.z

    def update_sim_timer_callback(self):
        self.obs, _, self.done, _ = self.env.step(
            np.array([[self.requested_steer, self.requested_speed]]))

        self.scan = list(self.obs["scans"][0])
        self.pose[0] = self.obs["poses_x"][0]
        self.pose[1] = self.obs["poses_y"][0]
        self.pose[2] = self.obs["poses_theta"][0]
        self.speed[0] = self.obs["linear_vels_x"][0]
        self.speed[1] = self.obs["linear_vels_y"][0]
        self.speed[2] = self.obs["ang_vels_z"][0]

    def update_state_timer_callback(self):
        t = self.get_clock().now().to_msg()

        # publish scan
        scan = LaserScan()
        scan.header.frame_id = self.tf_ns + "laser"
        scan.header.stamp = t

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = 0.0
        scan.range_max = 30.0
        scan.ranges = self.scan
        self.scan_pub.publish(scan)

        self._publish_joint_states(t)
        self._publish_odom(t)
        self._publish_transforms(t)

    def _publish_joint_states(self, t):
        js = JointState()
        js.header.stamp = t

        js.name = [
            self.tf_ns + "front_left_wheel_steering_joint",
            self.tf_ns + "front_right_wheel_steering_joint",
            self.tf_ns + "front_left_wheel_drive_joint",
            self.tf_ns + "front_right_wheel_drive_joint",
            self.tf_ns + "rear_left_wheel_drive_joint",
            self.tf_ns + "rear_right_wheel_drive_joint",
        ]

        js.position = 6 * [0.0]
        js.position[0] = self.requested_steer
        js.position[1] = self.requested_steer

        self.joint_states_pub.publish(js)

    def _publish_odom(self, t):
        odom = Odometry()
        odom.header.frame_id = self.tf_ns + self.map_frame
        odom.header.stamp = t

        odom.child_frame_id = self.tf_ns + "base_link"

        odom.pose.pose.position.x = self.pose[0]
        odom.pose.pose.position.y = self.pose[1]

        quat = euler.euler2quat(0., 0., self.pose[2], axes="sxyz")
        odom.pose.pose.orientation.w = quat[0]
        odom.pose.pose.orientation.x = quat[1]
        odom.pose.pose.orientation.y = quat[2]
        odom.pose.pose.orientation.z = quat[3]

        odom.twist.twist.linear.x = self.speed[0]
        odom.twist.twist.linear.y = self.speed[1]

        odom.twist.twist.angular.z = self.speed[2]

        self.odom_pub.publish(odom)

    def _publish_transforms(self, t):
        tf = TransformStamped()
        tf.header.frame_id = self.map_frame
        tf.header.stamp = t

        tf.child_frame_id = self.tf_ns + "base_link"

        tf.transform.translation.x = self.pose[0]
        tf.transform.translation.y = self.pose[1]
        tf.transform.translation.z = 0.0

        quat = euler.euler2quat(0.0, 0.0, self.pose[2], axes="sxyz")
        tf.transform.rotation.w = quat[0]
        tf.transform.rotation.x = quat[1]
        tf.transform.rotation.y = quat[2]
        tf.transform.rotation.z = quat[3]

        self.br.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    gym_bridge = F1tenthGymRosBridge()
    rclpy.spin(gym_bridge)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
