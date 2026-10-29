#!/usr/bin/env python3
"""Live DDS smoke test. Usage: ros_smoke.py /path/to/vshape_node."""
import math
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, PointCloud2
from std_srvs.srv import Trigger


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def marker_scan():
    scan = LaserScan()
    scan.header.frame_id = 'laser'
    scan.angle_min, scan.angle_increment = -0.6, 0.002
    scan.range_min, scan.range_max = 0.05, 10.0
    scan.angle_max = scan.angle_min + 600 * scan.angle_increment
    scan.ranges = [float('inf')] * 601
    for i in range(len(scan.ranges)):
        a = scan.angle_min + i * scan.angle_increment
        ray = (math.cos(a), math.sin(a))
        for sign in (-1, 1):
            arm = (-0.5 * math.cos(math.radians(35)),
                   sign * 0.5 * math.sin(math.radians(35)))
            den = cross(ray, arm)
            if abs(den) > 1e-10:
                distance = cross((2.0, 0.0), arm) / den
                along = cross((2.0, 0.0), ray) / den
                if distance > 0 and 0 <= along <= 1:
                    scan.ranges[i] = min(scan.ranges[i], distance)
    return scan


def main():
    executable = Path(sys.argv[1])
    assert executable.is_file(), 'ROS adapter executable has not been built'
    process = subprocess.Popen([
        str(executable), '--ros-args', '-p', 'use_sim_time:=true',
        '-p', 'output_rate:=30.0', '-p', 'lidar_frame:=laser',
        '-p', 'base_frame:=base_link', '-p', 'scan_topic:=/vshape_smoke/scan',
        '-p', 'iekf_odom_topic:=/vshape_smoke/odom',
        '-p', 'max_scan_points:=1000', '-p', 'marker_height:=0.263',
    ])
    rclpy.init()
    node = rclpy.create_node('vshape_smoke_driver')
    diagnostics, poses, clouds = [], [], []
    subscriptions = [
        node.create_subscription(DiagnosticArray, '/vshape/diagnostics', diagnostics.append, 20),
        node.create_subscription(PoseWithCovarianceStamped, '/vshape/pose', poses.append, 20),
        node.create_subscription(PointCloud2, '/vshape/deskewed_points', clouds.append, qos_profile_sensor_data),
    ]
    scan_pub = node.create_publisher(LaserScan, '/vshape_smoke/scan', qos_profile_sensor_data)
    odom_pub = node.create_publisher(Odometry, '/vshape_smoke/odom', qos_profile_sensor_data)
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    reset = node.create_client(Trigger, '/vshape/reset')
    scan = marker_scan()
    odom = Odometry()
    odom.child_frame_id, odom.header.frame_id = 'base_link', 'odom'

    def pump(stamp, duration=0.15, send_scan=False, send_odom=False, future_scan=False):
        msg = Clock()
        msg.clock.sec = int(stamp)
        msg.clock.nanosec = round((stamp - int(stamp)) * 1e9)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            clock_pub.publish(msg)
            if send_odom:
                odom.header.stamp = msg.clock
                odom_pub.publish(odom)
            if send_scan:
                scan.header.stamp = type(msg.clock)(
                    sec=msg.clock.sec + int(future_scan), nanosec=msg.clock.nanosec)
                scan_pub.publish(scan)
            rclpy.spin_once(node, timeout_sec=0.02)
        assert process.poll() is None, 'ROS adapter exited unexpectedly'

    def latest():
        assert diagnostics, 'no diagnostic output received'
        return {item.key: item.value for item in diagnostics[-1].status[0].values}

    try:
        assert reset.wait_for_service(timeout_sec=10), 'reset service unavailable'
        pump(10.0, duration=0.8)
        assert latest()['mode'] == 'UNINITIALIZED', latest()
        for i in range(12):
            pump(10.0 + i * 0.05, send_scan=True, send_odom=True)
        assert poses and clouds, 'V pose or deskewed cloud missing'
        pose = poses[-1]
        assert pose.header.frame_id == 'base_link'
        assert abs(pose.pose.pose.position.x - (2.0 - .5 * math.cos(math.radians(35)))) < 0.05, 'pose must be at V opening-face midpoint'
        assert abs(pose.pose.pose.position.y) < 0.05
        assert abs(pose.pose.pose.position.z - .263) < 1e-12
        assert abs(pose.pose.pose.orientation.w) > .999, 'heading must point outward'
        assert clouds[-1].header.frame_id == 'base_link' and clouds[-1].width > 20
        assert not node.get_publishers_info_by_topic('/vshape/robot_in_marker')
        assert not node.get_publishers_info_by_topic('/vshape/marker_in_base')
        assert latest()['deskew_status'] == 'instantaneous'
        # Exercise the swept acquisition route using the same static scene and zero twist.
        scan.time_increment = .02 / (len(scan.ranges)-1)
        pump(10.6, duration=.2, send_scan=True, send_odom=True)
        assert latest()['deskew_status'] == 'applied', latest()
        assert float(latest()['scan_duration']) > .019
        # A late odometry sample BEFORE the first ray changes its held motion.
        # The cloud and filter must both be recomputed, at the same scan stamp.
        pump(10.7, duration=.15, send_scan=True)
        cloud_before = clouds[-1]
        cloud_count = len(clouds)
        odom.header.stamp.sec = 10; odom.header.stamp.nanosec = 650000000
        odom.twist.twist.linear.x = .5
        end = time.monotonic() + .3
        while time.monotonic() < end:
            odom_pub.publish(odom)
            rclpy.spin_once(node, timeout_sec=.02)
        assert len(clouds) > cloud_count, 'late pre-scan odometry did not refresh deskew cloud'
        assert clouds[-1].header.stamp == cloud_before.header.stamp
        import struct
        previous_x = struct.unpack_from('<f', cloud_before.data, 0)[0]
        corrected_x = struct.unpack_from('<f', clouds[-1].data, 0)[0]
        assert abs(corrected_x - previous_x) > .001, 'late odometry had no deskew effect'
        odom.twist.twist.linear.x = 0.
        scan.time_increment = 0.
        assert all(math.isfinite(x) for x in pose.pose.covariance)
        assert pose.pose.covariance[14] >= 1e3, 'unobserved z covariance must not be zero'
        assert latest()['mode'] == 'TRACKING', latest()
        for key in ('processing_ms', 'candidate_score', 'support_fraction', 'normal_condition'):
            assert math.isfinite(float(latest()[key])), '{} unavailable'.format(key)
        assert 0 < float(latest()['support_fraction']) <= 1
        assert float(latest()['normal_condition']) >= 1
        count = len(poses)
        pump(11.3, duration=0.35, send_odom=True)
        assert len(poses) > count, 'missing scans stopped predicted output: {}'.format(latest())
        assert latest()['mode'] == 'PROPAGATION_ONLY', latest()
        assert float(latest()['last_measurement_age']) > 0.5, latest()
        pump(11.8, duration=0.35)
        assert latest()['mode'] == 'ODOMETRY_STALE', latest()
        assert float(latest()['odom_age']) > 0.2, latest()
        assert float(latest()['last_measurement_age']) > 0.5, latest()
        scan.header.frame_id = 'wrong_laser'
        pump(11.85, send_scan=True)
        assert 'frame' in latest()['reason'], latest()
        scan.header.frame_id = 'laser'
        valid_ranges = list(scan.ranges)
        scan.ranges = [1.0] * 1001
        pump(11.86, send_scan=True)
        assert 'max_scan_points' in latest()['reason'], latest()
        scan.ranges = valid_ranges
        scan.angle_increment = float('nan')
        pump(11.87, send_scan=True)
        assert 'metadata' in latest()['reason'], latest()
        scan.angle_increment = 0.002
        odom.child_frame_id = 'wrong_base'
        pump(11.88, send_odom=True)
        assert 'twist frame' in latest()['reason'], latest()
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = float('nan')
        pump(11.89, send_odom=True)
        assert 'nonfinite' in latest()['reason'], latest()
        odom.twist.twist.linear.x = 0.0
        pump(11.895, send_scan=True, future_scan=True)
        assert 'future' in latest()['reason'], latest()
        future = reset.call_async(Trigger.Request())
        deadline = time.monotonic() + 3
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert future.done() and future.result().success, 'reset service failed'
        pump(11.9)
        assert latest()['mode'] == 'UNINITIALIZED', latest()
        for i in range(5):
            pump(12.0 + i * 0.05, send_scan=True, send_odom=True)
        assert latest()['mode'] == 'TRACKING', latest()
        pump(2.0, duration=0.35)
        assert latest()['mode'] == 'UNINITIALIZED', 'clock rewind did not reset state'
        assert 'clock' in latest()['reason'], latest()
        for i in range(5):
            pump(2.1 + i * 0.05, send_scan=True, send_odom=True)
        assert latest()['mode'] == 'TRACKING', 'did not reacquire after clock rewind'
        print('PASS: live scan/odometry, single outward marker pose, fixed height, deskew cloud, covariance, missing scans, stale odometry, '
              'frame/size/metadata/NaN rejection, reset, clock rewind and reacquisition')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try:
            code = process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
            raise AssertionError('ROS adapter did not shut down cleanly')
        assert code == 0, 'ROS adapter shutdown returned {}'.format(code)
    assert subscriptions


if __name__ == '__main__':
    main()
