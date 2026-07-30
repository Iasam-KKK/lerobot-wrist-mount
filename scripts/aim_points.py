#!/usr/bin/env python3
"""Find where the wrist camera actually looks, per demo pose.

The bench objects were placed by guesswork and end up at the edge of frame, or
out of it. Rather than contort the arm to find the objects - which would mean
re-tuning a verified camera pose - this drives the arm to each pose in the demo
plan, reads the REAL tf for the optical frame, and intersects the optical axis
with the table plane. The resulting XY is where an object should sit to land
dead centre in that shot.

Run against a live sim:
    python3 scripts/aim_points.py
"""
import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

POSES = [
    ("look down at bench", [0.00, -0.55, 0.95, 0.60, 0.00]),
    ("sweep right",        [-0.45, -0.45, 0.90, 0.55, 0.00]),
    ("sweep left",         [0.35, -0.50, 0.95, 0.60, 0.00]),
    ("descend",            [0.10, -0.75, 1.15, 0.70, 0.00]),
    ("wrist roll",         [0.10, -0.75, 1.15, 0.70, 0.90]),
    # Mirrors ARM_PLAN in demo_motion.py - keep the two in step, or this tool
    # reports on poses the demo no longer visits. The closing pose repeats the
    # opening one so the clip loops.
    ("return to opening",  [0.00, -0.55, 0.95, 0.60, 0.00]),
]

TABLE_Z = 0.0125          # cube centre height

# Bench objects, as placed in worlds/pick_bench.sdf.
OBJECTS = [
    ("red",  (0.245, -0.010, 0.0125)),
    ("blue", (0.247, -0.073, 0.0125)),
    ("cyl",  (0.254,  0.103, 0.020)),
]

HFOV = 1.204277
HALF_H = HFOV / 2.0
HALF_V = math.atan(math.tan(HALF_H) * 480 / 640)


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return [
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ]


class Aim(Node):
    def __init__(self):
        super().__init__("aim_points")
        self.buf = Buffer()
        self.lis = TransformListener(self.buf, self)
        self.arm = ActionClient(self, FollowJointTrajectory,
                                "/arm_controller/follow_joint_trajectory")

    def goto(self, pos, secs=3.5):
        t = JointTrajectory()
        t.joint_names = ARM_JOINTS
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in pos]
        p.velocities = [0.0] * len(pos)
        p.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        t.points.append(p)
        g = FollowJointTrajectory.Goal()
        g.trajectory = t
        fut = self.arm.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        h = fut.result()
        if h is None or not h.accepted:
            return False
        res = h.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=20.0)
        return True

    def aim(self):
        for _ in range(40):                      # let tf settle
            rclpy.spin_once(self, timeout_sec=0.1)
        try:
            tf = self.buf.lookup_transform("world", "wrist_cam_optical_frame",
                                           rclpy.time.Time())
        except Exception as e:                   # noqa: BLE001
            self.get_logger().warn("tf lookup failed: %s" % e)
            return None
        t = tf.transform.translation
        R = quat_to_matrix(tf.transform.rotation)
        p = (t.x, t.y, t.z)
        d = (R[0][2], R[1][2], R[2][2])          # optical +Z = view direction
        if d[2] > -1e-3:                         # not pointing downward
            return (p, d, None)
        k = (TABLE_Z - p[2]) / d[2]
        hit = (p[0] + k*d[0], p[1] + k*d[1], TABLE_Z)
        return (p, d, hit)

    def run(self):
        if not self.arm.wait_for_server(timeout_sec=60.0):
            self.get_logger().error("arm_controller not available")
            return
        print("\n%-22s %-26s %-26s %s" % ("pose", "camera xyz (world)",
                                          "aim point on table", "range"))
        print("-" * 96)
        for name, pos in POSES:
            if not self.goto(pos):
                print("%-22s  REJECTED" % name)
                continue
            r = self.aim()
            if r is None:
                print("%-22s  tf unavailable" % name)
                continue
            p, d, hit = r
            if hit is None:
                print("%-22s (%6.3f,%6.3f,%6.3f)  camera is NOT looking downward "
                      "(dz=%+.3f)" % (name, p[0], p[1], p[2], d[2]))
                continue
            rng = math.dist(p, hit)
            print("%-22s (%6.3f,%6.3f,%6.3f)   x=%+.3f  y=%+.3f          %.3f m"
                  % (name, p[0], p[1], p[2], hit[0], hit[1], rng))

            # Where does each object actually land in the frame?
            tf = self.buf.lookup_transform("world", "wrist_cam_optical_frame",
                                           rclpy.time.Time())
            R = quat_to_matrix(tf.transform.rotation)
            for oname, o in OBJECTS:
                v = [o[i] - p[i] for i in range(3)]
                # world -> optical is R transposed
                c = [sum(R[r][k] * v[r] for r in range(3)) for k in range(3)]
                if c[2] <= 0:
                    print("      %-5s BEHIND CAMERA" % oname)
                    continue
                h = math.degrees(math.atan2(c[0], c[2]))
                w = math.degrees(math.atan2(c[1], c[2]))
                inframe = abs(h) < math.degrees(HALF_H) and abs(w) < math.degrees(HALF_V)
                print("      %-5s h=%+6.1f deg  v=%+6.1f deg   %s"
                      % (oname, h, w, "IN FRAME" if inframe else "out"))


def main():
    rclpy.init()
    n = Aim()
    try:
        n.run()
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
