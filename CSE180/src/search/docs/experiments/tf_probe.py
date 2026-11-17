"""Probe TF timing: timestamped lookups at image stamps, and AMCL map->odom cadence."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)


class Probe(Node):
    def __init__(self):
        super().__init__('tf_probe')
        self.results = []          # (source_stamp, requested, ok, note)
        self.future_results = []
        self.map_odom = 0
        self.map_odom_stamps = []
        self.n_img = 0
        self.create_subscription(Image, '/rgbd_camera/image', self.cb_img, QOS)
        self.create_subscription(TFMessage, '/tf', self.cb_tf, 100)
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)

    def cb_tf(self, msg):
        for tr in msg.transforms:
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                self.map_odom += 1
                self.map_odom_stamps.append(tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9)
            if tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_footprint':
                pass

    def cb_img(self, msg):
        self.n_img += 1
        if self.n_img > 40 or self.n_img % 2 == 1:
            return
        src = Time.from_msg(msg.header.stamp)
        # exact-stamp lookup, immediately upon arrival
        try:
            tr = self.buf.lookup_transform('map', msg.header.frame_id, msg.header.stamp,
                                           Duration(seconds=0.05))
            p = tr.transform.translation
            self.results.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                                 'exact', True, '%.3f %.3f %.3f' % (p.x, p.y, p.z)))
        except Exception as ex:  # noqa: BLE001
            self.results.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                                 'exact', False, str(ex)[:110]))
        # slightly-future lookup: does tf2 clamp or throw?
        try:
            tr = self.buf.lookup_transform('map', msg.header.frame_id, src + Duration(seconds=0.2),
                                           Duration(seconds=0.05))
            p = tr.transform.translation
            self.future_results.append(('+0.2s', True, '%.3f %.3f %.3f' % (p.x, p.y, p.z)))
        except Exception as ex:  # noqa: BLE001
            self.future_results.append(('+0.2s', False, str(ex)[:110]))


def main():
    rclpy.init()
    node = Probe()
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    t0 = time.time()
    while time.time() - t0 < 12.0:
        rclpy.spin_once(node, timeout_sec=0.05)

    ok = [r for r in node.results if r[2]]
    bad = [r for r in node.results if not r[2]]
    print('exact-stamp map lookups: %d ok, %d failed (of %d)' % (len(ok), len(bad), len(node.results)))
    for r in bad[:4]:
        print('  FAIL src=%.3f %s' % (r[0], r[3]))
    for r in ok[:3]:
        print('  OK   src=%.3f t=%s' % (r[0], r[3]))
    fok = [r for r in node.future_results if r[1]]
    fbad = [r for r in node.future_results if not r[1]]
    print('future (+0.2s) lookups: %d ok, %d failed' % (len(fok), len(fbad)))
    for r in fbad[:2]:
        print('  FUTURE FAIL %s' % r[2])
    print('images seen=%d' % node.n_img)
    print('map->odom /tf messages=%d' % node.map_odom)
    st = node.map_odom_stamps
    if len(st) > 1:
        d = [b - a for a, b in zip(st, st[1:])]
        print('map->odom interval: min=%.3f max=%.3f mean=%.3f over %d samples' % (
            min(d), max(d), sum(d) / len(d), len(d)))
    print('amcl_pose publisher check skipped')
    node.destroy_node()
    rclpy.shutdown()


main()
