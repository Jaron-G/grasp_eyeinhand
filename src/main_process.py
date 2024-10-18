#!/usr/bin/env python3
import os
import sys
import tkinter as tk
import tkinter.messagebox

import numpy as np

import rospy
import geometry_msgs.msg
import moveit_commander

from real_gripper_control import GripperControl
from ur_control import UR_robot

from detect_pose.srv import DetectPose
from calc_pose.srv import CalcPose
from grasp_object.srv import GraspObject


rospy.sleep(1)
ur = UR_robot()
path = "/catkin_ws/src/grasp_eyeinhand/config/"
gripper = GripperControl()
rospy.sleep(2)
DEBUG = False


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

def calc_pose_client():
    rospy.wait_for_service('calc_pose_service')
    try:
        calc_pose = rospy.ServiceProxy('calc_pose_service', CalcPose)
        resp1 = calc_pose()
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def detect_pose_client():
    rospy.wait_for_service('detect_pose_service')
    try:
        detect_pose = rospy.ServiceProxy('detect_pose_service', DetectPose)
        resp1 = detect_pose()
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def grasp_object_client():
    rospy.wait_for_service('grasp_object_service')
    try:
        grasp_object = rospy.ServiceProxy('grasp_object_service', GraspObject)
        resp1 = grasp_object()
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

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
        self.gripper_width = 0

    def create_widgets(self):
        self.frame1 = tk.Frame(self)
        self.frame2 = tk.Frame(self)
        self.frame3 = tk.Frame(self)
        
        #检测功能按钮区
        self.reset_vrobot_button = tk.Button(self.frame3, text="Reset Robot", command=self.press_reset_robot_button, width=28)
        self.reset_vrobot_button.pack(pady=5)
        self.detect_pose_button = tk.Button(self.frame3, text="detect_pose", command=self.press_detect_pose_button, width=28)
        self.detect_pose_button.pack(pady=5)
        self.calc_pose_button = tk.Button(self.frame3, text="calc_pose", command=self.press_calc_pose_button, width=28)
        self.calc_pose_button.pack(pady=5)
        self.grasp_object_button = tk.Button(self.frame3, text="grasp_object", command=self.press_grasp_object_button, width=28)
        self.grasp_object_button.pack(pady=5)
        
        #调试功能按钮区
        self.reset_robot_button = tk.Button(self.frame1, text="Reset Robot", command=self.press_reset_robot_button, width=28)
        self.reset_robot_button.pack(pady=5)
        self.obtain_pose_button = tk.Button(self.frame1, text="Obtain Pose", command=self.press_obtain_pose_button, width=28)
        self.obtain_pose_button.pack(pady=5)
        self.move_pose_button = tk.Button(self.frame1, text="Move Pose", command=self.press_move_pose_button, width=28)
        self.move_pose_button.pack(pady=5)
        self.obtain_joint_angle_button = tk.Button(self.frame1, text="Obtain Joint Angle", command=self.press_obtain_joint_angle_button, width=28)
        self.obtain_joint_angle_button.pack(pady=5)
        self.move_joint_angle_button = tk.Button(self.frame1, text="Move Joint Angle", command=self.press_move_joint_angle_button, width=28)
        self.move_joint_angle_button.pack(pady=5)
        self.action_button = tk.Button(self.frame1, text="Action", command=self.press_action_button, width=28)
        self.action_button.pack(pady=5)
        self.open_gripper_button = tk.Button(self.frame1, text="Open Gripper To Width", command=self.press_open_gripper_button, width=28)
        self.open_gripper_button.pack(pady=5)
        self.select_action_button = tk.Button(self.frame1, text="Select action to do", command=self.press_action_select_button, width=28)
        self.select_action_button.pack(pady=5)
        self.quit = tk.Button(self.frame1, text="QUIT", fg="red", command=self.master.destroy, bd=3, width=28)
        self.quit.pack(pady=5)

        # self.photo = tk.PhotoImage(file="/catkin_ws/src/grasp_eyeinhand/images/robot.png")
        # self.photo_label = tk.Label(self.frame2,justify = tk.LEFT,image = self.photo).grid(row = 0,column = 0)
        # self.photo_label.pack(side="right")

        self.message_var = tk.StringVar(self,value='Please click the buttons in order !')  # 储存文字的类
        self.message_box = tk.Label(self.frame1, textvariable=self.message_var, bg='lightblue',width=28)
        self.message_box.pack(pady=5)
        
        self.gripper_width_label = tk.Label(self.frame2,text = 'gripper width').grid(row = 0,column = 0)
        self.entry0 = tk.Entry(self.frame2)
        self.entry0.grid(row = 0,column = 1,padx = 2, pady = 5)

        # Shoulder Pan
        self.shoulder_pan_joint_label = tk.Label(
            self.frame2, text="Shoulder Pan Joint Angle :"
        ).grid(row=1, column=0)
        self.entry1 = tk.Entry(self.frame2)
        self.entry1.grid(row=1, column=1, padx=2, pady=5)

        # Shoulder Lift
        self.shoulder_lift_joint_label = tk.Label(
            self.frame2, text=" Shoulder Lift Joint Angle :"
        ).grid(row=2, column=0)
        self.entry2 = tk.Entry(self.frame2)
        self.entry2.grid(row=2, column=1, padx=2, pady=5)

        # Elbow
        self.elbow_joint_label = tk.Label(self.frame2, text="Elbow Joint Angle :").grid(
            row=3, column=0
        )
        self.entry3 = tk.Entry(self.frame2)
        self.entry3.grid(row=3, column=1, padx=2, pady=5)

        # Wrist 1
        self.wrist_1_joint_label = tk.Label(
            self.frame2, text="Wrist 1 Joint Angle :"
        ).grid(row=4, column=0)
        self.entry4 = tk.Entry(self.frame2)
        self.entry4.grid(row=4, column=1, padx=2, pady=5)

        # Wrist 2
        self.wrist_2_joint_label = tk.Label(
            self.frame2, text="Wrist 2 Joint Angle :"
        ).grid(row=5, column=0)
        self.entry5 = tk.Entry(self.frame2)
        self.entry5.grid(row=5, column=1, padx=2, pady=5)
        # Wrist 3
        self.wrist_3_joint_label = tk.Label(
            self.frame2, text="Wrist 3 Joint Angle :"
        ).grid(row=6, column=0)
        self.entry6 = tk.Entry(self.frame2)
        self.entry6.grid(row=6, column=1, padx=2, pady=5)
        
        # action path
        self.action_path_label = tk.Label(self.frame2, text="action path :").grid(
            row=0, column=3
        )
        self.entry_action_path = tk.Entry(self.frame2)
        self.entry_action_path.grid(row=0, column=4, padx=2, pady=5)

        # position.x
        self.position_x_label = tk.Label(self.frame2, text="position.x :").grid(
            row=1, column=3
        )
        self.entry_position_x = tk.Entry(self.frame2)
        self.entry_position_x.grid(row=1, column=4, padx=2, pady=5)

        # position.y
        self.position_y_label = tk.Label(self.frame2, text="position.y :").grid(
            row=2, column=3
        )
        self.entry_position_y = tk.Entry(self.frame2)
        self.entry_position_y.grid(row=2, column=4, padx=2, pady=5)

        # position.z
        self.position_z_label = tk.Label(self.frame2, text="position.z :").grid(
            row=3, column=3
        )
        self.entry_position_z = tk.Entry(self.frame2)
        self.entry_position_z.grid(row=3, column=4, padx=2, pady=5)

        # orientation.x
        self.orientation_x_label = tk.Label(self.frame2, text="orientation.x :").grid(
            row=4, column=3
        )
        self.entry_orientation_x = tk.Entry(self.frame2)
        self.entry_orientation_x.grid(row=4, column=4, padx=2, pady=5)

        # orientation.y
        self.orientation_y_label = tk.Label(self.frame2, text="orientation.y :").grid(
            row=5, column=3
        )
        self.entry_orientation_y = tk.Entry(self.frame2)
        self.entry_orientation_y.grid(row=5, column=4, padx=2, pady=5)

        # orientation.z
        self.orientation_z_label = tk.Label(self.frame2, text="orientation.z :").grid(
            row=6, column=3
        )
        self.entry_orientation_z = tk.Entry(self.frame2)
        self.entry_orientation_z.grid(row=6, column=4, padx=2, pady=5)

        # orientation.w
        self.orientation_w_label = tk.Label(self.frame2, text="orientation.w :").grid(
            row=7, column=3
        )
        self.entry_orientation_w = tk.Entry(self.frame2)
        self.entry_orientation_w.grid(row=7, column=4, padx=2, pady=5)
        
        self.debug_var = tk.IntVar()
        self.debug_checkbox = tk.Checkbutton(self, text="Debug Mode", variable=self.debug_var, command=self.debug_check)
        self.debug_checkbox.pack()

        self.frame1.pack(side="left")  # 左框架对齐
        self.frame2.pack(side="right")  # 右框架对齐
        self.frame3.pack(side="top") 
        # self.frame3.pack(side="bottom")


    # 在终端打印当前模式信息
    def debug_check(self):
        if  self.debug_var.get()==1 :
            print("Start joint mode !") 
        else:
            print("Start pose mode !")

    def press_calc_pose_button(self):
        rospy.loginfo("Start calc_pose !")
        response = calc_pose_client()
        rospy.loginfo("calc_pose completed!")
        self.message_var.set("calc_pose ccompleted!")
        
    def press_detect_pose_button(self):
        rospy.loginfo("Start detect_pose!")
        response = detect_pose_client()
        rospy.loginfo("detect_pose completed!")
        self.message_var.set("detect_pose completed!")
        
    def press_grasp_object_button(self):
        rospy.loginfo("Start grasp_object !")
        response = grasp_object_client()
        rospy.loginfo("grasp_object completed!")
        self.message_var.set("grasp_object completed!")
        
        
    def press_open_gripper_button(self):
        if  0 < int(self.entry0.get()) < 85 :
            self.gripper_width = int(self.entry0.get())
            width = -255/85*self.gripper_width + 255
            gripper.control_gripper(width)# gripper.open()
            self.message_var.set("open gripper completed!")  
            return True
        else:
            print('Please enter 0-85')
            self.entry0.delete(0)
            tkinter.messagebox.showwarning(
                title="Warning", message="Gripper width should be 0-85 !"
            )
            return False
        
    def press_action_select_button(self):
        action_path = "/catkin_ws/src/grasp_eyeinhand/action/"
        action = self.entry_action_path.get()
        print("You select Action " + self.entry0.get() + " to do!")
        self.message_var.set(self.entry0.get() + " to do!")

        if self.debug_var.get() == 1:
            print("Start action joint mode !")
            joint_goal = np.load(action_path + action + "/current_joint_angle.npy")
            ur.go_to_goal_joint_angle(joint_goal)
            rospy.sleep(3)
        else:
            print("Start action pose mode !")
            position = np.load(action_path + action + "/current_position.npy")
            rotation = np.load(action_path + action + "/current_rotation.npy")
            robot_pose = construct_pose(position, rotation)
            ur.go_to_goal_pose(robot_pose)
            rospy.sleep(5)
        self.message_var.set("action completed!")

    def press_obtain_pose_button(self):
        current_pose = ur.get_current_pose()
        current_position = np.array([(current_pose.position.x) ,current_pose.position.y, current_pose.position.z])
        #transforms3d四元数转矩阵函数的四元数格式为(w,x,y,z)
        current_rotation = np.array([current_pose.orientation.w,current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z])
        pose = np.array(
            [
                current_position[0],
                current_position[1],
                current_position[2],
                current_rotation[0],
                current_rotation[1],
                current_rotation[2],
                current_rotation[3],
            ]
        )
        pose_entry = [
            self.entry_position_x,
            self.entry_position_y,
            self.entry_position_z,
            self.entry_orientation_w,
            self.entry_orientation_x,
            self.entry_orientation_y,
            self.entry_orientation_z,
        ]

        for i in range(7):
            pose_entry[i].delete(0, 100)
            pose_entry[i].insert(0, pose[i])
        np.save(path+ "current_position.npy",current_position)
        np.save(path+ "current_rotation.npy",current_rotation)
        print("current_rotation.npy",current_rotation)
        print("current_position.npy",current_position)
        self.message_var.set("obtain_pose ccompleted!")
        
    def press_obtain_joint_angle_button(self):
        current_joint_angle = ur.get_current_joint_angle()
        joint_angle = np.array([current_joint_angle[0]
                                ,current_joint_angle[1]
                                ,current_joint_angle[2]
                                ,current_joint_angle[3]
                                ,current_joint_angle[4]
                                ,current_joint_angle[5]])
        joint_entry = [
            self.entry1,
            self.entry2,
            self.entry3,
            self.entry4,
            self.entry5,
            self.entry6,
        ]

        for i in range(6):
            joint_entry[i].delete(0, 100)
            joint_entry[i].insert(0, joint_angle[i]*180/np.pi)
        np.save(path+ "current_joint_angle.npy",joint_angle)
        print("current_joint_angle.npy",joint_angle)
        self.message_var.set("obtain joint angle completed!")
        
    def press_move_pose_button(self):
        position = np.load(path+ "current_position.npy")
        rotation = np.load(path+ "current_rotation.npy")
        print("current_rotation.npy",rotation)
        print("current_position.npy",position)
        
        pose = np.array(
            [
                position[0],
                position[1],
                position[2],
                rotation[0],
                rotation[1],
                rotation[2],
                rotation[3]
            ]
        )
        pose_entry = [
            self.entry_position_x,
            self.entry_position_y,
            self.entry_position_z,
            self.entry_orientation_w,
            self.entry_orientation_x,
            self.entry_orientation_y,
            self.entry_orientation_z,
        ]

        for i in range(7):
            pose_entry[i].delete(0, 100)
            pose_entry[i].insert(0, pose[i])
        
        robot_pose = construct_pose(position,rotation)
        ur.go_to_goal_pose(robot_pose)
        self.message_var.set("Move pose ccompleted!")    
        
    def press_move_joint_angle_button(self):
        joint_goal = np.load(path+ "current_joint_angle.npy")
        joint_entry = [
            self.entry1,
            self.entry2,
            self.entry3,
            self.entry4,
            self.entry5,
            self.entry6,
        ]
        for i in range(6):
            joint_entry[i].delete(0, 100)
            joint_entry[i].insert(0, joint_goal[i])
        ur.go_to_goal_joint_angle(joint_goal)
        self.message_var.set("Move joint goal completed!")  
        
    # def press_action_button(self):
    #     action_list = ["Aa01","Aa02","Aa04"]
    #     action_path = "/catkin_ws/src/grasp_eyeinhand/dongzuo/"
    #     for i in action_list:
    #         position = np.load(action_path+ i + "/current_position.npy")
    #         rotation = np.load(action_path+ i+  "/current_rotation.npy")
    #         robot_pose = construct_pose(position,rotation)
    #         ur.go_to_goal_pose(robot_pose)
    #         rospy.sleep(5)
    #     self.message_var.set("action completed!")    
    
    def press_action_button(self):
        action_path = "/catkin_ws/src/grasp_eyeinhand/action/"
        action_list = os.listdir(action_path)
        action_list.sort()
        
        for i in action_list:
            if i[-1] == 'M':
                if self.debug_var.get() == 1:
                    print("Start action joint mode !")
                    joint_goal = np.load(action_path+ i + "/current_joint_angle.npy")
                    ur.go_to_goal_joint_angle(joint_goal)
                    rospy.sleep(3)
                else:
                    print("Start action pose mode !")
                    position = np.load(action_path + i + "/current_position.npy")
                    rotation = np.load(action_path + i + "/current_rotation.npy")
                    robot_pose = construct_pose(position, rotation)
                    ur.go_to_goal_pose(robot_pose)
                    rospy.sleep(3)
                print("action" + i + "complete")
            elif i[-1] == 'K':
                gripper_width = 85
                width = -255/85*gripper_width +255
                gripper.control_gripper(width)# gripper.open()
                rospy.sleep(2)
            elif i[-1] == 'G':
                gripper_width = np.loadtxt(action_path+ i + "/gripper_width.txt")
                width = -255/85*gripper_width +255
                gripper.control_gripper(width)
                rospy.sleep(2)
        self.message_var.set("action completed!") 

    
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
        
        
    def press_open_gripper_button(self):
        if  0 < int(self.entry0.get()) < 85 :
            self.gripper_width = int(self.entry0.get())
            width = -255/85*self.gripper_width +255
            gripper.control_gripper(width)# gripper.open()
            self.message_var.set("open gripper completed!")  
            return True
        else:
            print('Please enter 0-85')
            self.entry0.delete(0)
            tkinter.messagebox.showwarning(
                title="Warning", message="Gripper width should be 0-85 !"
            )
            return False    

def main():
    root = tk.Tk()
    root.title(' grasping eye in hand')
    root.geometry('1200x480')
    app = Application(master=root)
    app.mainloop()

if __name__ == '__main__':
    main()

