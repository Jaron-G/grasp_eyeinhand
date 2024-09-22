#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
import tkinter as tk
import numpy as np
from real_gripper_control import GripperControl
from ur_control import UR_robot
import geometry_msgs.msg


def construct_pose(position, quaternion, frame_id = "base_link") -> geometry_msgs.msg.Pose:

    # pose_under_camera = function(x, y)
    robot_pose = geometry_msgs.msg.Pose()
    robot_pose.position.x = position[0]
    robot_pose.position.y = position[1]
    robot_pose.position.z = position[2]

    robot_pose.orientation.w = quaternion[0]
    robot_pose.orientation.x = quaternion[1]
    robot_pose.orientation.y = quaternion[2]
    robot_pose.orientation.z = quaternion[3]

    # print(transforms3d.euler.quat2euler((0, 7.07109031e-01, 7.07104531e-01, 0), 'rxyz'))
    return robot_pose


ur = UR_robot()
path = "/catkin_ws/src/grasp_eyeinhand/images/"
gripper = GripperControl()
class Application(tk.Frame):
    def __init__(self, master=None):
        super().__init__(master)
        self.master = master
        self.pack()
        self.create_widgets()
        self.matched_model = None
        self.final_pose = None
        self.pose_up = None
        self.pose_g = None
        self.rotation_matrix = None

    def create_widgets(self):
        self.frame1 = tk.Frame(self)
        self.frame2 = tk.Frame(self)
        self.reset_vrobot_button = tk.Button(self.frame1, text="Reset Robot", command=self.press_reset_robot_button, width=28)
        self.reset_vrobot_button.pack(pady=5)
        self.detect_pose_button = tk.Button(self.frame1, text="Obtain Pose", command=self.press_obtain_pose_button, width=28)
        self.detect_pose_button.pack(pady=5)
        self.calc_pose_button = tk.Button(self.frame1, text="Move Pose", command=self.press_move_pose_button, width=28)
        self.calc_pose_button.pack(pady=5)
        # self.collision_detect_button = tk.Button(self.frame1, text="Perform collision detect", command=self.press_collision_detect_button, width=28)
        # self.collision_detect_button.pack(pady=5)
        # self.move_robot_button = tk.Button(self.frame1, text="Start grasp", command=self.press_move_robot_button, width=28)
        # self.move_robot_button.pack(pady=5)
        self.quit = tk.Button(self.frame1, text="QUIT", fg="red", command=self.master.destroy, bd=3, width=28)
        self.quit.pack(pady=5)

        self.photo = tk.PhotoImage(file="/catkin_ws/src/grasp_eyeinhand/images/robot.png")
        self.photo_label = tk.Label(self.frame2,justify = tk.LEFT,image = self.photo)
        self.photo_label.pack(side="right")

        self.message_var = tk.StringVar(self,value='Please click the buttons in order !')  # 储存文字的类
        self.message_box = tk.Label(self.frame1, textvariable=self.message_var, bg='lightblue',width=28)
        self.message_box.pack(pady=5)

        self.debug_var = tk.IntVar()
        self.debug_checkbox = tk.Checkbutton(self, text="Debug Mode", variable=self.debug_var, command=self.debug_check)
        self.debug_checkbox.pack()

        self.frame1.pack(side="left")  # 左框架对齐
        self.frame2.pack(side="right")  # 右框架对齐

    # 在终端打印当前模式信息
    def debug_check(self):
        if  self.debug_var.get()==1 :
            print("Start debug mode !") 
        else:
            print("Start normal mode !")

    def press_obtain_pose_button(self):
        current_pose = ur.get_current_pose()
        current_position = np.array([(current_pose.position.x) ,current_pose.position.y, current_pose.position.z])
        #transforms3d四元数转矩阵函数的四元数格式为(w,x,y,z)
        current_rotation = np.array([current_pose.orientation.w,current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z])
        np.save(path+ "current_position.npy",current_position)
        np.save(path+ "current_rotation.npy",current_rotation)
        print("current_rotation.npy",current_rotation)
        print("current_position.npy",current_position)
        # gripper.control_gripper(255)# gripper.close()
        # rospy.sleep(2)
        # width = -255/85*28 +255
        # gripper.control_gripper(width)# gripper.open()
        self.message_var.set("obtain_pose ccompleted!")
        
    def press_move_pose_button(self):
        position = np.load(path+ "current_position.npy")
        rotation = np.load(path+ "current_rotation.npy")
        print("current_rotation.npy",rotation)
        print("current_position.npy",position)
        print("current_position.npy",position[0])
        robot_pose = construct_pose(position,rotation)
        ur.go_to_goal_pose(robot_pose)
        self.message_var.set("move_pose ccompleted!")    
        
    def press_reset_robot_button(self):
        rospy.loginfo("Reset Robot !")
        moveit_commander.roscpp_initialize(sys.argv)
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -1.544
        joint_goal[2] = 1.544 
        joint_goal[3] = -1.5707
        joint_goal[4] = -1.5707
        joint_goal[5] = -1.5707
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        rospy.loginfo("Reset robot successfully!")
        self.message_var.set("Reset Robot successfully !")

def main():
    root = tk.Tk()
    root.title(' grasping eye in hand')
    root.geometry('400x300')
    app = Application(master=root)
    app.mainloop()

if __name__ == '__main__':
    rospy.init_node('main_process')
    main()
    rospy.spin()
