import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

QOS = QoSProfile(depth=5,
                 reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST)


class Inspect(Node):
    def __init__(self):
        super().__init__('cam_inspect')
        self.img = None
        self.dep = None
        self.info = None
        self.n_img = 0
        self.n_dep = 0
        self.n_info = 0
        self.create_subscription(Image, '/rgbd_camera/image', self.cb_img, QOS)
        self.create_subscription(Image, '/rgbd_camera/depth_image', self.cb_dep, QOS)
        self.create_subscription(CameraInfo, '/rgbd_camera/camera_info', self.cb_info, 10)

    def cb_img(self, m):
        self.img = m
        self.n_img += 1

    def cb_dep(self, m):
        self.dep = m
        self.n_dep += 1

    def cb_info(self, m):
        self.info = m
        self.n_info += 1


def stamp_str(s):
    return f'{s.sec}.{s.nanosec:09d}'


def main():
    rclpy.init()
    node = Inspect()
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    buf = Buffer()
    listener = TransformListener(buf, node)  # noqa: F841

    t0 = time.time()
    while (node.img is None or node.dep is None or node.info is None) and time.time() - t0 < 40:
        rclpy.spin_once(node, timeout_sec=0.2)

    if node.img is None or node.dep is None or node.info is None:
        print('INCOMPLETE: img=%s dep=%s info=%s' % (node.img is not None, node.dep is not None, node.info is not None))
        node.destroy_node()
        rclpy.shutdown()
        return

    img, dep, info = node.img, node.dep, node.info
    print('--- RGB ---')
    print('frame_id=%r enc=%r w=%d h=%d step=%d bigendian=%d stamp=%s' % (
        img.header.frame_id, img.encoding, img.width, img.height, img.step,
        img.is_bigendian, stamp_str(img.header.stamp)))
    arr = np.frombuffer(bytes(img.data), dtype=np.uint8)
    print('data len=%d expected=%d' % (arr.size, img.step * img.height))

    print('--- DEPTH ---')
    print('frame_id=%r enc=%r w=%d h=%d step=%d bigendian=%d stamp=%s' % (
        dep.header.frame_id, dep.encoding, dep.width, dep.height, dep.step,
        dep.is_bigendian, stamp_str(dep.header.stamp)))
    d = np.frombuffer(bytes(dep.data), dtype=np.float32).reshape(dep.height, dep.width)
    print('depth dtypesize=%d total=%d' % (d.itemsize, d.size))
    finite = np.isfinite(d)
    print('finite=%d nan=%d inf=%d zero=%d' % (finite.sum(), np.isnan(d).sum(),
                                               np.isinf(d).sum(), (d == 0).sum()))
    good = d[finite & (d > 0.0)]
    if good.size:
        print('valid range min=%.3f max=%.3f median=%.3f' % (good.min(), good.max(), np.median(good)))
    print('center pixel d=%.4f  corner d=%.4f' % (d[dep.height // 2, dep.width // 2], d[0, 0]))

    print('--- STAMPS ---')
    print('rgb==depth stamp: %s' % (img.header.stamp.sec == dep.header.stamp.sec and
                                    img.header.stamp.nanosec == dep.header.stamp.nanosec))
    print('info stamp=%s  rgb stamp=%s  equal=%s' % (
        stamp_str(info.header.stamp), stamp_str(img.header.stamp),
        info.header.stamp.sec == img.header.stamp.sec and
        info.header.stamp.nanosec == img.header.stamp.nanosec))

    print('--- CAMERA_INFO ---')
    print('frame_id=%r w=%d h=%d model=%r' % (info.header.frame_id, info.width, info.height,
                                              info.distortion_model))
    print('K=%s' % (list(info.k),))
    print('D=%s' % (list(info.d),))
    print('R=%s' % (list(info.r),))
    print('P=%s' % (list(info.p),))
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    print('fx=%.3f fy=%.3f cx=%.3f cy=%.3f' % (fx, fy, cx, cy))
    print('hfov_from_K=%.4f rad (%.1f deg)' % (2.0 * np.arctan(img.width / (2.0 * fx)),
                                               2.0 * np.arctan(img.width / (2.0 * fx)) * 180 / np.pi))

    print('--- RATES (last window) ---')
    c_img, c_dep, c_info = node.n_img, node.n_dep, node.n_info
    t0 = time.time()
    while time.time() - t0 < 6.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    dt = time.time() - t0
    print('rgb=%.2f Hz depth=%.2f Hz info=%.2f Hz (wall clock; sim may run slower)' % (
        (node.n_img - c_img) / dt, (node.n_dep - c_dep) / dt, (node.n_info - c_info) / dt))
    print('sim now=%s' % stamp_str(node.get_clock().now().to_msg()))

    print('--- TF at rgb stamp ---')
    for target in ['map', 'odom', 'base_link', 'base_footprint', 'oakd_rgb_camera_frame']:
        try:
            tr = buf.lookup_transform(target, img.header.frame_id, img.header.stamp,
                                      rclpy.duration.Duration(seconds=2.0))
            q = tr.transform.rotation
            p = tr.transform.translation
            print('%s <- %s: t=(%.4f, %.4f, %.4f) q=(%.4f, %.4f, %.4f, %.4f)' % (
                target, img.header.frame_id, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            print('%s <- %s: FAILED %s' % (target, img.header.frame_id, str(ex)[:160]))

    print('--- TF zero-time fallback ---')
    for target in ['map', 'odom']:
        try:
            tr = buf.lookup_transform(target, img.header.frame_id, rclpy.time.Time(),
                                      rclpy.duration.Duration(seconds=2.0))
            p = tr.transform.translation
            print('%s <- %s: t=(%.4f, %.4f, %.4f)' % (target, img.header.frame_id, p.x, p.y, p.z))
        except Exception as ex:  # noqa: BLE001
            print('%s <- %s: FAILED %s' % (target, img.header.frame_id, str(ex)[:160]))

    print('--- TF frames known to buffer ---')
    print(sorted(buf.all_frames_as_yaml().splitlines())[:40])

    node.destroy_node()
    rclpy.shutdown()


main()
