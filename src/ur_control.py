# Python 2/3 compatibility imports
from __future__ import print_function

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class UR_robot(object):
    """UR_robot moveit control class"""

    def __init__(self):
        super(UR_robot, self).__init__()

        ## First initialize `moveit_commander`
        moveit_commander.roscpp_initialize(sys.argv)
        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints). This interface can be used to plan and execute motions:
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        
        ## 设置机器人的最大速度和最大加速度
        self.move_group.set_max_acceleration_scaling_factor(0.3)
        self.move_group.set_max_velocity_scaling_factor(0.3)
        self.move_group.set_goal_joint_tolerance(0.001)
        self.move_group.set_goal_position_tolerance(0.1)
        self.move_group.set_goal_orientation_tolerance(0.1)
        # self.add_box_objects('table', (0, 0, -0.1), (1.5, 1.5, 0.01))
    
    def go_to_goal_pose(self, pose_goal):
        move_group = self.move_group
        ### Using pose goal method ###
        move_group.clear_pose_targets()

        move_group.set_pose_target(pose_goal)

        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()
        
    def go_to_cartesian_pose(self, pose_goal):
        ### Using catesian path method ###
        move_group = self.move_group
        
        waypoints = []
        waypoints.append(pose_goal)
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        move_group.execute(plan, wait=True)

        return fraction
    
    def get_current_pose(self):
        # 获取并打印当前的末端执行器位置和姿态
        current_pose = self.move_group.get_current_pose().pose
        rospy.loginfo("Current Pose of the End-Effector: Position - x: {0}, y: {1}, z: {2}; Orientation - x: {3}, y: {4}, z: {5}, w: {6}".format(
            current_pose.position.x,
            current_pose.position.y,
            current_pose.position.z,
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w
        ))
        
        return current_pose
    
    def get_current_joint_angle(self):
        # 获取并打印当前的机器人关节角
        joint_goal = self.move_group.get_current_joint_values()
        rospy.loginfo("Current joints angles of the Robot: Shoulder Pan : {0}, Shoulder Lift: {1}, Elbow: {2}; Wrist - 1: {3}, 2: {4}, 3: {5}, w".format(
            joint_goal[0],
            joint_goal[1],
            joint_goal[2],
            joint_goal[3],
            joint_goal[4],
            joint_goal[5]
        ))
        
        return joint_goal
    
    def go_to_goal_joint_angle(self, joints):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = joints[0]
        joint_goal[1] = joints[1]
        joint_goal[2] = joints[2]
        joint_goal[3] = joints[3]
        joint_goal[4] = joints[4]
        joint_goal[5] = joints[5]
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        
        
    def reset(self):
        move_group = self.move_group
        move_group.set_named_target('home')
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()  
    
    # def add_box_objects(self, box_name, pose, box_size):
    #     box_pose = geometry_msgs.msg.PoseStamped()
    #     box_pose.header.frame_id = "base_link"
    #     box_pose.pose.orientation.w = 1.0
    #     box_pose.pose.position.x = pose[0]
    #     box_pose.pose.position.y = pose[1]
    #     box_pose.pose.position.z = pose[2]
    #     self.scene.add_box(box_name, box_pose, size = box_size)

    def run(self):
        rospy.spin()

def main():
    ur_robot = UR_robot()
    ur_robot.run()

if __name__ == "__main__":
    main()
