#!/usr/bin/env python3
"""Verify installed launch configuration via live ROS parameter/diagnostic services."""
import signal
import subprocess
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.srv import GetParameters


def main():
    process = subprocess.Popen([
        'ros2', 'launch', 'ros2_bot_estimator_lidar_vshape', 'vshape.launch.py',
        'use_sim_time:=true'])
    rclpy.init()
    node = rclpy.create_node('vshape_launch_smoke')
    diagnostics = []
    subscription = node.create_subscription(
        DiagnosticArray, '/vshape/diagnostics', diagnostics.append, 10)
    client = node.create_client(GetParameters, '/vshape_estimator/get_parameters')
    try:
        assert client.wait_for_service(timeout_sec=10), 'launched node parameter service unavailable'
        request = GetParameters.Request()
        request.names = ['use_sim_time', 'scan_topic', 'base_frame', 'lidar_frame', 'lidar_in_base.x',
                         'iekf_odom_topic', 'iekf_process_noise_lin', 'iekf_process_noise_ang',
                         'iekf_max_iterations', 'iekf_huber_threshold', 'iekf_state_history_duration']
        future = client.call_async(request)
        deadline = time.monotonic() + 5
        while (not future.done() or not diagnostics) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert future.done() and diagnostics, 'launched node did not report health'
        values = future.result().values
        assert values[0].bool_value
        assert values[1].string_value == '/bot_sensor/lidar_front/laser_scan'
        assert values[2].string_value == 'base_footprint'
        assert values[3].string_value == 'lidar_front'
        assert abs(values[4].double_value - 0.362) < 1e-12
        assert values[5].string_value == '/bot_sensor/odometer/odometry'
        assert abs(values[6].double_value - .0004) < 1e-12
        assert abs(values[7].double_value - .0001) < 1e-12
        assert values[8].integer_value == 10
        assert abs(values[9].double_value - 2.5) < 1e-12
        assert abs(values[10].double_value - 1.) < 1e-12
        assert diagnostics[-1].status[0].values[0].value == 'UNINITIALIZED'
        print('PASS: installed ros2 launch, YAML mounting/topic/frame parameters, '
              'use_sim_time, health output')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try:
            code = process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
            raise AssertionError('ros2 launch did not shut down cleanly')
        assert code == 0, 'ros2 launch returned {}'.format(code)
    assert subscription


if __name__ == '__main__':
    main()
