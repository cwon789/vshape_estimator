#!/usr/bin/env python3
"""Visible small-target demo; launch through run_live_demo.sh."""
import math
import signal
import subprocess
import sys
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
SIMULATOR = PACKAGE.parents[1] / 'simulator' / 'simple-sim'
sys.path.insert(0, str(SIMULATOR))
import numpy as np
import yaml
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QApplication, QToolBar
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from engine import Project, RosOptions, Scene, VMarker, Wall
from engine.ros.rviz import rviz_config
from app.runtime_options import RuntimeOptions
from app.studio_window import StudioWindow
from app.theme import STYLE


def main():
    folder = Path('/tmp/vshape-live-demo')
    folder.mkdir(parents=True, exist_ok=True)
    parameters = yaml.safe_load((PACKAGE/'config/vshape.yaml').read_text())['vshape_estimator']['ros__parameters']
    height, arm, opening = (parameters[key] for key in ('marker_height', 'arm_length', 'opening_angle_deg'))
    model = SIMULATOR/'robots/templates/srp-robot-model-bot-amr-gtr-nano/AMR700D-LT900N2-C'
    target = VMarker('MiR V150', 2., 0., math.pi, arm, opening, z_min=height-.05, z_max=height+.05)
    scene = Scene(markers=[target], walls=[Wall('back', -3,-3,4,-3), Wall('right',4,-3,4,3),
                                           Wall('top',4,3,-3,3), Wall('left',-3,3,-3,-3)])
    Project(scene=scene, robot_model=str(model), robot_pose=(0.,0.,0.),
            ros=RosOptions(sim_time=True, tf=True, cmd_vel=True).to_dict()).save(folder/'vshape_demo.json')
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE)
    window = StudioWindow('AMR700D-LT900N2-C', folder/'vshape_demo.json',
                          RuntimeOptions(engine='lite', sensors='board', ui_fps=20), external_port=None)
    window.setWindowTitle('SimpleSim - MiR V150 docking pose - LIVE')
    window.setGeometry(72,30,1030,970)
    window.show()

    def shutdown(*_):
        # Signal shutdown cannot wait for a modal save prompt from edited demo state.
        window.hide()
        window.close()
        app.quit()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    window._toggle_ros()
    if window.bridge is None:
        raise RuntimeError(window.status.text())
    node = window.bridge.node
    pub = node.create_publisher(MarkerArray, '/vshape/demo/scene', 10)
    pose_pub = node.create_publisher(PoseStamped, '/vshape/demo/pose', 10)
    latest = {'mode':'Waiting', 'deskew_status':'waiting', 'scan_duration':'0'}
    ellipse = None

    def pose(message):
        nonlocal ellipse
        out = PoseStamped()
        out.header, out.pose = message.header, message.pose.pose
        pose_pub.publish(out)
        c = np.array(message.pose.covariance).reshape(6,6)
        values, vectors = np.linalg.eigh(c[:2,:2])
        latest['sigma'] = 'XY sigma %.1f / %.1f mm\nyaw sigma %.2f deg' % (
            1000*math.sqrt(max(0,c[0,0])), 1000*math.sqrt(max(0,c[1,1])),
            math.degrees(math.sqrt(max(0,c[5,5]))))
        m = Marker()
        m.header, m.ns, m.id, m.type = message.header, 'XY 2-sigma', 20, Marker.LINE_STRIP
        m.pose.orientation.w = 1.
        m.pose.position = message.pose.pose.position
        m.pose.position.z += .006
        m.scale.x = .006
        m.color.g, m.color.b, m.color.a = 1., .5, .9
        for angle in np.linspace(0,2*math.pi,65):
            xy = vectors @ (2*np.sqrt(np.maximum(values,0))*np.array([math.cos(angle),math.sin(angle)]))
            point = Point()
            point.x, point.y = float(xy[0]), float(xy[1])
            m.points.append(point)
        ellipse = m

    def diagnostic(message):
        if message.status:
            latest.update({v.key:v.value for v in message.status[0].values})

    subscriptions = [node.create_subscription(PoseWithCovarianceStamped,'/vshape/pose',pose,10),
                     node.create_subscription(DiagnosticArray,'/vshape/diagnostics',diagnostic,10)]
    bar = QToolBar('Demo motion',window)
    window.addToolBar(Qt.BottomToolBarArea,bar)
    automatic = bar.addAction('AUTO DEMO: forward / reverse + turning')
    automatic.setCheckable(True)
    automatic.setChecked(True)
    automatic.toggled.connect(lambda on: window.drive(0.,0.) if not on else None)
    window.set_manual(True)
    start = window.sim.time

    def drive():
        if window.bridge is None:
            automatic.setChecked(False)
            return
        if automatic.isChecked() and window.running:
            wave = math.cos(2*math.pi*(window.sim.time-start)/24.)
            window.drive(.10*wave,.04*wave)

    def markers():
        nonlocal node, pub, pose_pub, subscriptions, ellipse
        if window.bridge is None:
            return
        if window.bridge.node is not node:
            node = window.bridge.node
            pub = node.create_publisher(MarkerArray, '/vshape/demo/scene', 10)
            pose_pub = node.create_publisher(PoseStamped, '/vshape/demo/pose', 10)
            subscriptions = [node.create_subscription(PoseWithCovarianceStamped, '/vshape/pose', pose, 10),
                             node.create_subscription(DiagnosticArray, '/vshape/diagnostics', diagnostic, 10)]
            ellipse = None
            latest.update(mode='Waiting', deskew_status='waiting', scan_duration='0')
        array = MarkerArray()
        for index,(first,second) in enumerate(target.segments()):
            m = Marker()
            m.header.frame_id = 'odom'
            m.ns, m.id, m.type = 'MiR physical target', index, Marker.CUBE
            m.pose.position.x, m.pose.position.y = (first[0]+second[0])/2, (first[1]+second[1])/2
            m.pose.position.z = height
            yaw = math.atan2(second[1]-first[1],second[0]-first[0])
            m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            m.scale.x, m.scale.y, m.scale.z = arm, .003, .10
            m.color.r, m.color.g, m.color.a = 1., .65, 1.
            array.markers.append(m)
        # Reference plane for controller offsets: join the two arm tips.
        face = Marker()
        face.header.frame_id = 'odom'
        face.ns, face.id, face.type = 'Opening face reference', 3, Marker.LINE_STRIP
        face.pose.orientation.w = 1.
        face.scale.x = .005
        face.color.r, face.color.g, face.color.b, face.color.a = .7, .7, .7, .65
        for _, endpoint in target.segments():
            point = Point()
            point.x, point.y, point.z = endpoint[0], endpoint[1], height
            face.points.append(point)
        array.markers.append(face)
        text = Marker()
        text.header.frame_id = 'odom'
        text.ns, text.id, text.type = 'Status', 2, Marker.TEXT_VIEW_FACING
        text.pose.orientation.w = 1.
        text.pose.position.x, text.pose.position.y, text.pose.position.z = 1., .85, .6
        text.scale.z = .065
        text.color.g, text.color.b, text.color.a = 1., .5, 1.
        text.text = 'MiR V150 '+latest['mode']+'\n'+latest.get('sigma','')+'\ndeskew '+latest['deskew_status']
        text.text += ' %.1f ms' % (float(latest['scan_duration'])*1000)
        array.markers.append(text)
        if ellipse is not None:
            array.markers.append(ellipse)
        pub.publish(array)

    motion_timer, marker_timer = QTimer(), QTimer()
    motion_timer.timeout.connect(drive)
    motion_timer.start(50)
    marker_timer.timeout.connect(markers)
    marker_timer.start(200)
    scans = [(name,topic) for name,(_,topic) in window.sensor_configs.items()]
    config = yaml.safe_load(rviz_config([],scans,window.bridge.options))
    displays = config['Visualization Manager']['Displays']
    for display in displays:
        if display['Class'] == 'rviz_default_plugins/LaserScan':
            display.update(Enabled=False,Value=False,Name='Raw '+display['Name']+' (debug)')
        if display['Class'] == 'rviz_default_plugins/TF':
            display['Show Names'], display['Show Axes'] = False, False
    topic = lambda name, reliability: {'Value':name, 'Reliability Policy':reliability,
                                       'History Policy':'Keep Last', 'Depth':10}
    displays.append({'Class':'rviz_default_plugins/MarkerArray','Name':'V target + XY covariance',
                     'Enabled':True,'Topic':topic('/vshape/demo/scene','Reliable'),'Value':True})
    displays.append({'Class':'rviz_default_plugins/PointCloud2','Name':'DESKEWED LiDAR (filter input)',
                     'Enabled':True,'Topic':topic('/vshape/deskewed_points','Best Effort'),
                     'Style':'Points','Size (Pixels)':3,'Size (m)':.012,'Alpha':1.,
                     'Color Transformer':'FlatColor','Color':'30; 220; 255','Position Transformer':'XYZ',
                     'Decay Time':.1,'Value':True})
    displays.append({'Class':'rviz_default_plugins/Pose','Name':'V FACE-CENTER DOCKING POSE',
                     'Enabled':True,'Topic':topic('/vshape/demo/pose','Reliable'),'Color':'40; 255; 120',
                     'Shape':'Arrow','Alpha':1.,'Shaft Length':.30,'Shaft Radius':.012,
                     'Head Length':.10,'Head Radius':.035,'Value':True})
    config['Visualization Manager']['Views']['Current'] = {
        'Class':'rviz_default_plugins/Orbit','Name':'Current View','Target Frame':'odom','Distance':5.,
        'Pitch':1.12,'Yaw':0.,'Focal Point':{'X':.9,'Y':0.,'Z':.1},'Near Clip Distance':.01,
        'Value':'Orbit (rviz_default_plugins)'}
    config['Window Geometry'].update({'X':1105,'Y':30,'Width':810,'Height':970,
                                      'Hide Left Dock':False,'Hide Right Dock':True})
    (folder/'vshape.rviz').write_text(yaml.safe_dump(config,sort_keys=False))
    with (folder/'rviz.log').open('w') as log:
        rviz = subprocess.Popen(['rviz2','-d',str(folder/'vshape.rviz'),'--ros-args','-p','use_sim_time:=true'],
                                stdout=log,stderr=subprocess.STDOUT)
        try:
            print('READY: small V; one outward pose; deskew cloud; automatic motion',flush=True)
            return app.exec_()
        finally:
            if rviz.poll() is None:
                rviz.terminate()
                try:
                    rviz.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    rviz.kill()
                    rviz.wait()
    assert subscriptions


if __name__ == '__main__':
    sys.exit(main())
