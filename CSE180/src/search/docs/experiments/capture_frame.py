"""Capture one RGB+depth pair (exact same stamp) plus camera info.

Usage: python3 capture_frame.py <outdir> <tag>
Saves <tag>_rgb.png, <tag>_depth.npy, <tag>_info.json
"""
import json
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)
from sensor_msgs.msg import Image, CameraInfo
import cv_bridge

OUT = sys.argv[1]
TAG = sys.argv[2]

QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST)


class Cap(Node):
    def __init__(self):
        super().__init__('frame_capture')
        self.bridge = cv_bridge.CvBridge()
        self.img = None
        self.dep = None
        self.info = None
        self.create_subscription(Image, '/rgbd_camera/image', self.cb_img, QOS)
        self.create_subscription(Image, '/rgbd_camera/depth_image', self.cb_dep, QOS)
        self.create_subscription(CameraInfo, '/rgbd_camera/camera_info', self.cb_info, 10)

    def cb_img(self, m):
        self.img = m

    def cb_dep(self, m):
        self.dep = m

    def cb_info(self, m):
        self.info = m


def stamp_of(m):
    return m.header.stamp.sec * 10 ** 9 + m.header.stamp.nanosec


def main():
    rclpy.init()
    node = Cap()
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

    # find a fresh image + the depth frame carrying the same stamp
    t0 = time.time()
    picked = False
    while time.time() - t0 < 30 and not picked:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.img is None:
            continue
        s = stamp_of(node.img)
        t1 = time.time()
        while time.time() - t1 < 1.5:
            if node.dep is not None and stamp_of(node.dep) == s:
                picked = True
            rclpy.spin_once(node, timeout_sec=0.05)
            if picked:
                break
        if not picked:
            node.img = None

    if not picked:
        print('FAILED to get a matched RGB/depth pair')
        node.destroy_node()
        rclpy.shutdown()
        return

    img, dep = node.img, node.dep
    os.makedirs(OUT, exist_ok=True)
    rgb = np.frombuffer(bytes(img.data), dtype=np.uint8).reshape(img.height, img.width, 3)
    depth = np.frombuffer(bytes(dep.data), dtype=np.float32).reshape(dep.height, dep.width)

    # save RGB as PNG (PIL, RGB order = the sensor's rgb8 convention)
    from PIL import Image as PILImage
    PILImage.fromarray(rgb, 'RGB').save(os.path.join(OUT, '%s_rgb.png' % TAG))
    np.save(os.path.join(OUT, '%s_depth.npy' % TAG), depth)
    if node.info is not None:
        with open(os.path.join(OUT, '%s_info.json' % TAG), 'w') as f:
            json.dump({'frame_id': node.info.header.frame_id, 'width': node.info.width,
                       'height': node.info.height, 'distortion_model': node.info.distortion_model,
                       'k': list(node.info.k), 'd': list(node.info.d),
                       'stamp': [node.info.header.stamp.sec, node.info.header.stamp.nanosec]}, f, indent=1)

    print('saved %s: stamp=%d.%09d enc=%s %dx%d frame=%s' % (
        TAG, img.header.stamp.sec, img.header.stamp.nanosec, img.encoding,
        img.width, img.height, img.header.frame_id))
    print('depth: enc=%s finite=%d inf=%d min=%.3f max=%.3f' % (
        dep.encoding, int(np.isfinite(depth).sum()), int(np.isinf(depth).sum()),
        float(np.nanmin(depth[np.isfinite(depth)])) if np.isfinite(depth).any() else -1,
        float(np.nanmax(depth[np.isfinite(depth)])) if np.isfinite(depth).any() else -1))

    node.destroy_node()
    rclpy.shutdown()


main()
