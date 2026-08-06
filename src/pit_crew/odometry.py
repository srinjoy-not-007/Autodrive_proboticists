"""Wheel + IMU odometry for the RoboRacer digital twin.

Speed from rear encoders (robust filtered derivative), heading from the IMU
orientation quaternion (permitted sensor topic). Publishes nav_msgs/Odometry
on /pit_crew/odom and the odom -> base_link TF that slam_toolbox / the
particle filter consume. Works live and during rosbag replay.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
import tf2_ros

from pit_crew import vehicle


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class WheelImuOdometry(Node):

    def __init__(self):
        super().__init__('wheel_imu_odometry')
        self.publish_tf = self.declare_parameter('publish_tf', True).value

        self.speed_est = vehicle.SpeedEstimator(alpha=0.5)
        self.x = 0.0
        self.y = 0.0
        self.yaw = None          # seeded by first IMU message
        self.yaw_offset = None   # first IMU yaw maps to odom yaw 0
        self.last_t = None

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        ns = '/autodrive/roboracer_1'
        self.create_subscription(Imu, f'{ns}/imu', self.on_imu, qos)
        self.create_subscription(JointState, f'{ns}/left_encoder',
                                 lambda m: self.on_enc('L', m), qos)
        self.create_subscription(JointState, f'{ns}/right_encoder',
                                 lambda m: self.on_enc('R', m), qos)
        self.pub = self.create_publisher(Odometry, '/pit_crew/odom', 10)
        self.tf = tf2_ros.TransformBroadcaster(self)

    def on_enc(self, wheel, msg: JointState):
        if msg.position:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.speed_est.update(wheel, t, msg.position[0])

    def on_imu(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        imu_yaw = yaw_from_quat(msg.orientation)
        if self.yaw_offset is None:
            self.yaw_offset = imu_yaw
        self.yaw = self._norm(imu_yaw - self.yaw_offset)

        if self.last_t is not None:
            dt = t - self.last_t
            if 0.0 < dt < 0.5:
                v = self.speed_est.speed
                self.x += v * math.cos(self.yaw) * dt
                self.y += v * math.sin(self.yaw) * dt
        self.last_t = t
        self.publish(msg)

    def publish(self, imu_msg: Imu):
        stamp = imu_msg.header.stamp
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = self.speed_est.speed
        odom.twist.twist.angular.z = imu_msg.angular_velocity.z
        self.pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = 'odom'
            tf.child_frame_id = 'base_link'
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = odom.pose.pose.orientation.z
            tf.transform.rotation.w = odom.pose.pose.orientation.w
            self.tf.sendTransform(tf)

    @staticmethod
    def _norm(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


def main():
    rclpy.init()
    node = WheelImuOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
