#!/usr/bin/env python3
"""Scripted bench sweep - the 30-second video.

Drives the arm through poses that put the three bench objects through the wrist
camera's field of view, then closes and opens the gripper. The point is to show
the wrist feed tracking the scene while the arm moves, which is the thing a
static URDF cannot demonstrate.

    ros2 launch so101_wrist_cam_description sim.launch.py demo:=true

or standalone against an already-running sim:

    ros2 run so101_wrist_cam_description demo_motion.py
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# (seconds_from_start, [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll])
# Every value is inside the upstream limits:
#   pan +-1.91986  lift +-1.74533  elbow +-1.69  flex +-1.65806  roll -2.74385..2.84121
ARM_PLAN = [
    (4.0, [0.00, -0.55, 0.95, 0.60, 0.00]),   # look down at the bench
    (8.0, [-0.45, -0.45, 0.90, 0.55, 0.00]),  # sweep right, cylinder into frame
    (12.0, [0.35, -0.50, 0.95, 0.60, 0.00]),  # sweep left across both cubes
    (16.0, [0.10, -0.75, 1.15, 0.70, 0.00]),  # descend toward the red cube
    (20.0, [0.10, -0.75, 1.15, 0.70, 0.90]),  # roll the wrist, camera orbits
    (24.0, [0.00, -0.30, 0.70, 0.40, 0.00]),  # back up to a neutral view
]

GRIPPER_PLAN = [(2.5, 1.20), (5.0, 0.10), (7.5, 1.20)]   # open, close, open


def _traj(joints, plan):
    t = JointTrajectory()
    t.joint_names = joints
    for secs, pos in plan:
        p = JointTrajectoryPoint()
        p.positions = [float(x) for x in (pos if isinstance(pos, list) else [pos])]
        p.velocities = [0.0] * len(p.positions)
        p.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        t.points.append(p)
    return t


class BenchSweep(Node):
    def __init__(self):
        super().__init__("wrist_cam_bench_sweep")
        self.arm = ActionClient(self, FollowJointTrajectory,
                                "/arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self, FollowJointTrajectory,
                                 "/gripper_controller/follow_joint_trajectory")

    def run(self):
        self.get_logger().info("waiting for controllers...")
        if not self.arm.wait_for_server(timeout_sec=60.0):
            self.get_logger().error(
                "arm_controller action server never appeared. Is ros2_control "
                "installed and did the spawners succeed? "
                "Check: ros2 control list_controllers")
            return False
        self.grip.wait_for_server(timeout_sec=15.0)

        self.get_logger().info("sending gripper open/close/open")
        g = FollowJointTrajectory.Goal()
        g.trajectory = _traj(["gripper"], GRIPPER_PLAN)
        self.grip.send_goal_async(g)

        self.get_logger().info("sending arm bench sweep (~24 s)")
        a = FollowJointTrajectory.Goal()
        a.trajectory = _traj(ARM_JOINTS, ARM_PLAN)
        fut = self.arm.send_goal_async(a)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)

        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("arm trajectory REJECTED by the controller")
            return False

        self.get_logger().info("trajectory accepted; sweeping")
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=60.0)
        self.get_logger().info("bench sweep complete")
        return True


def main():
    rclpy.init()
    node = BenchSweep()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
