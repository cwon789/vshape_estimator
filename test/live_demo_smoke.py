#!/usr/bin/env python3
"""Integration probe: real Qt/ROS disconnect/reconnect and signal shutdown.

Run with sourced Foxy, QT_QPA_PLATFORM=offscreen and a private ROS_DOMAIN_ID.
RViz rendering is checked by the separate visible launcher; here a child process
stands in for RViz so only GUI/ROS ownership and shutdown are exercised.
"""
import os
from pathlib import Path
import signal
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import show_live_demo as demo
from PyQt5.QtCore import QTimer

OriginalWindow = demo.StudioWindow
original_popen = subprocess.Popen
checks = []


def child(*args, **kwargs):
    if args and args[0] and args[0][0] == 'rviz2':
        return original_popen([sys.executable, '-c', 'import time; time.sleep(30)'], **kwargs)
    return original_popen(*args, **kwargs)


class ProbeWindow(OriginalWindow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        QTimer.singleShot(1800, self.disconnect)

    def disconnect(self):
        self._toggle_ros()
        QTimer.singleShot(500, self.reconnect)

    def reconnect(self):
        self._toggle_ros()
        QTimer.singleShot(1500, self.check_rebound)

    def check_rebound(self):
        checks.append(self.bridge is not None)
        if self.bridge is not None:
            topics = {publisher.topic_name for publisher in self.bridge.node.publishers}
            checks.append('/vshape/demo/scene' in topics)
            checks.append('/vshape/demo/pose' in topics)
        QTimer.singleShot(200, self.stop_edited_demo)

    def stop_edited_demo(self):
        self.scene.markers[0].x += .01
        os.kill(os.getpid(), signal.SIGTERM)


demo.StudioWindow = ProbeWindow
demo.subprocess.Popen = child
code = demo.main()
assert code == 0 and len(checks) == 3 and all(checks), checks
print('PASS: ROS disconnect/reconnect rebind and SIGTERM with edited scene')
