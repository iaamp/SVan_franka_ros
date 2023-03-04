#!/usr/bin/env python

import sys
import rospy
import time

from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult, GripperCommandActionGoal
from std_msgs.msg import String

ws_center_pose = [1.2992429602774254, 1.6139419262757837, 1.4127428199330816, -1.8329834672526308, 1.4350932842536965, 2.9705418931663345, -2.2793759904454354]
takeoff_pose = [2.1307805330962464, 1.7018341177621268, 1.5694990341801802, -0.9448265606144016, 1.080990578797153, 3.0075618901575294, -1.947074397492556]
outside_window_pose = [0.7169841309012028, 1.6256357435606916, 1.4877381699560648, -2.3776628111135167, 0.9164642189601866, 3.1047851827609696, -1.789196010248952]
in_window_pose = [-0.2538744584145655, 1.3272666761822178, 1.267664436340332, -2.398848422330722, 0.18187557684050662, 2.095510131095694, -1.212683315916769]
in_window_pose2 = [0.01562881993585232, 1.488419455875519, 1.2752702706676966, -2.592436686532539, 0.15240817970699735, 2.578389013523954, -1.1866229154732493]
inside_window_pose = [-0.38711653370189636, 1.3470894337477093, 1.3823093889851303, -1.9803360956258942, 0.1256682511471894, 1.5870134175382467, -1.016727114505525]
before_maintenance_pose = [-0.6016448412192495, 1.1006669021828872, 1.249589584384048, -2.2272721897133994, 0.8649787907467947, 2.3848401197459963, -1.589397165185875]
maintenance_pose = [-0.641380364711754, 1.2683572617080576, 1.3777927872134015, -2.1596437425446093, 1.5048078764087163, 2.7829607169363233, -2.1258594694738524]

joints = ["panda_joint1", "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]

" Retrieve the UAV on the platform into the hub"
class HubPandaJointControl:

    def __init__(self):
        rospy.init_node('hub_panda_joint_control')

        self.action = rospy.resolve_name('~follow_joint_trajectory')
        self.client = SimpleActionClient(self.action, FollowJointTrajectoryAction)
        rospy.loginfo("move_to_start: Waiting for '" + self.action + "' action to come up")
        self.client.wait_for_server()

        tmp = rospy.resolve_name('~takeoff_joint_pose')
        self.takeoff_poses = rospy.get_param(tmp, None)

        self.takeoff_poses = [
            before_maintenance_pose,
            inside_window_pose,
            in_window_pose,
            in_window_pose2,
            outside_window_pose,
            takeoff_pose
        ]
        self.retrieve_poses = [
            outside_window_pose,
            in_window_pose,
            inside_window_pose,
            before_maintenance_pose,
            maintenance_pose
        ]
        self.takeoff_to_landing_poses = [
            ws_center_pose
        ]

        # self.poses = [self.pose_0]

        # self.gripper_close_cmd = GripperCommandActionGoal()
        # self.gripper_close_cmd.goal.command.position = 0.02
        # self.gripper_close_cmd.goal.command.max_effort = 1.0

        # self.gripper_open_cmd = GripperCommandActionGoal()
        # self.gripper_open_cmd.goal.command.position = 0.033
        # self.gripper_open_cmd.goal.command.max_effort = 1.0
        # self.pub_gripper_cmd = rospy.Publisher("/franka_gripper/gripper_action/goal", GripperCommandActionGoal, queue_size=10)

        # self.main()
        self.trigger_sub = rospy.Subscriber("/hub_panda_joint_control/trigger", String, self.cb_trigger)
        self.joint_states_topic = rospy.resolve_name('~joint_states')

    def cb_trigger(self, msg):
        if msg.data == 'retrieve_uav':
            self.retrieve_uav()
        elif msg.data == 'send_uav':
            self.send_uav()
        elif msg.data == 'to_landing':
            self.to_landing()

    def retrieve_uav(self):
        print('retrieve uav')
        print("retrieve poses: ", self.retrieve_poses)
        self.run_joint_pose_trajectory(self.retrieve_poses)

    def send_uav(self):
        print('send uav')
        print("takeoff poses: ", self.takeoff_poses)
        self.run_joint_pose_trajectory(self.takeoff_poses)

    def to_landing(self):
        print('to landing')
        self.run_joint_pose_trajectory(self.takeoff_to_landing_poses)

    def run_joint_pose_trajectory(self, poses):
        for i, pose in enumerate(poses):
            joint_state = rospy.wait_for_message(self.joint_states_topic, JointState)
            pose = dict(zip(joint_state.name, pose))
            initial_pose = dict(zip(joint_state.name, joint_state.position))
            max_movement = max(abs(pose[joint] - initial_pose[joint]) for joint in pose)

            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration.from_sec(
                # Use either the time to move the furthest joint with 'max_dq' or 500ms,
                # whatever is greater
                max(max_movement / rospy.get_param('~max_dq', 0.5), 0.5)
            )
            goal = FollowJointTrajectoryGoal()

            goal.trajectory.joint_names, point.positions = [list(x) for x in zip(*pose.items())]
            point.velocities = [0] * len(pose)
            print("velocities: ", point.velocities)

            goal.trajectory.points.append(point)
            #goal.goal_time_tolerance = rospy.Duration.from_sec(0.5)
            goal.goal_time_tolerance = rospy.Duration.from_sec(0.2)

            self.client.send_goal_and_wait(goal)

            result = self.client.get_result()
            if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
                rospy.logerr('move_to_start: Movement was not successful: ' + {
                    FollowJointTrajectoryResult.INVALID_GOAL:
                    """
                    The joint pose you want to move to is invalid (e.g. unreachable, singularity...).
                    Is the 'joint_pose' reachable?
                    """,

                    FollowJointTrajectoryResult.INVALID_JOINTS:
                    """
                    The joint pose you specified is for different joints than the joint trajectory controller
                    is claiming. Does you 'joint_pose' include all 7 joints of the robot?
                    """,

                    FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
                    """
                    During the motion the robot deviated from the planned path too much. Is something blocking
                    the robot?
                    """,

                    FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:
                    """
                    After the motion the robot deviated from the desired goal pose too much. Probably the robot
                    didn't reach the joint_pose properly
                    """,
                }[result.error_code])

            else:
                rospy.loginfo('Center platform: Successfully moved into center pose')

    # def main(self):
    #     time.sleep(10)

    #     for i, pose in enumerate(self.poses):
    #         # After aligning gripper with platform handle: grip platform
    #         if i == 4:
    #             time.sleep(2)
    #             self.pub_gripper_cmd.publish(self.gripper_open_cmd)
    #             time.sleep(2)
    #         if pose is None:
    #             rospy.logerr('center platform: Could not find required parameter ')
    #             sys.exit(1)
    #         topic = rospy.resolve_name('~joint_states')
    #         rospy.loginfo("Center platform: Waiting for message on topic '" + topic + "'")
    #         joint_state = rospy.wait_for_message(topic, JointState)
    #         initial_pose = dict(zip(joint_state.name, joint_state.position))

    #         max_movement = max(abs(pose[joint] - initial_pose[joint]) for joint in pose)

    #         point = JointTrajectoryPoint()
    #         point.time_from_start = rospy.Duration.from_sec(
    #             # Use either the time to move the furthest joint with 'max_dq' or 500ms,
    #             # whatever is greater
    #             max(max_movement / rospy.get_param('~max_dq', 0.5), 0.5)
    #         )
    #         goal = FollowJointTrajectoryGoal()

    #         goal.trajectory.joint_names, point.positions = [list(x) for x in zip(*pose.items())]
    #         point.velocities = [0] * len(pose)

    #         goal.trajectory.points.append(point)
    #         goal.goal_time_tolerance = rospy.Duration.from_sec(0.5)

    #         rospy.loginfo('Sending trajectory Goal to move into next config')
    #         self.client.send_goal_and_wait(goal)

    #         result = self.client.get_result()
    #         if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
    #             rospy.logerr('move_to_start: Movement was not successful: ' + {
    #                 FollowJointTrajectoryResult.INVALID_GOAL:
    #                 """
    #                 The joint pose you want to move to is invalid (e.g. unreachable, singularity...).
    #                 Is the 'joint_pose' reachable?
    #                 """,

    #                 FollowJointTrajectoryResult.INVALID_JOINTS:
    #                 """
    #                 The joint pose you specified is for different joints than the joint trajectory controller
    #                 is claiming. Does you 'joint_pose' include all 7 joints of the robot?
    #                 """,

    #                 FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
    #                 """
    #                 During the motion the robot deviated from the planned path too much. Is something blocking
    #                 the robot?
    #                 """,

    #                 FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:
    #                 """
    #                 After the motion the robot deviated from the desired goal pose too much. Probably the robot
    #                 didn't reach the joint_pose properly
    #                 """,
    #             }[result.error_code])

    #         else:
    #             rospy.loginfo('Center platform: Successfully moved into center pose')


def main():
    joint_controller = HubPandaJointControl()
    rospy.spin()

if __name__ == '__main__':
    main()
