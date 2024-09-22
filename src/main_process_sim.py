#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
import tkinter as tk
import numpy as np

from detect_pose.srv import DetectPose
from calc_pose.srv import CalcPose
from grasp_object.srv import GraspObject

DEBUG = False

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

    def create_widgets(self):
        self.frame1 = tk.Frame(self)
        self.frame2 = tk.Frame(self)
        self.reset_vrobot_button = tk.Button(self.frame1, text="Reset Robot", command=self.press_reset_robot_button, width=28)
        self.reset_vrobot_button.pack(pady=5)
        self.detect_pose_button = tk.Button(self.frame1, text="detect_pose", command=self.press_detect_pose_button, width=28)
        self.detect_pose_button.pack(pady=5)
        self.calc_pose_button = tk.Button(self.frame1, text="calc_pose", command=self.press_calc_pose_button, width=28)
        self.calc_pose_button.pack(pady=5)
        self.grasp_object_button = tk.Button(self.frame1, text="grasp_object", command=self.press_grasp_object_button, width=28)
        self.grasp_object_button.pack(pady=5)
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

    # def press_registration_button(self):
    #     rospy.loginfo("Start perform registration !")
    #     if self.debug_var.get() == 1:
    #         debug_mode = True
    #     else:
    #         debug_mode = False
    #     response = registration_client(debug_mode)
    #     self.final_pose, self.matched_model = response.final_pose, response.matched_model
    #     rospy.loginfo("Registration completed !")
    #     self.message_var.set("Registration completed !")

    # def press_pose_transformation_button(self):
    #     rospy.loginfo("Pose transformation !")
    #     response = pose_transformation_client(self.final_pose, self.matched_model)
    #     self.rotation_matrix, self.pose_g, self.pose_up = response.rotation_matrix, response.grasp_pose, response.up_pose
    #     rospy.loginfo("Pose transformation completed !")
    #     self.message_var.set("Pose transformation completed !")

    # def press_collision_detect_button(self):
    #     matched_matrix = np.array(self.final_pose).reshape(4,4)
    #     coincide_num_points = 40
    #     is_collided,pose_g,pose_up= contact_detect(self.matched_model,matched_matrix,coincide_num_points)
    #     offset = 0
    #     while is_collided:
    #         print("Collision occurred, please re-register！")
    #         self.message_var.set("Obtain point cloud completed!")
    #         offset = offset + 70
    #         is_collided,pose_g,pose_up = contact_detect(self.matched_model, matched_matrix, coincide_num_points, offset)
    #         print(is_collided)
    #     self.message_var.set(" Perform the fetching normally")
        
    #     # self.pose_g_with_offset = tuple(pose_g)
    #     # self.pose_up_with_offset = tuple(pose_up)
    #     self.pose_g = tuple(pose_g)
    #     self.pose_up = tuple(pose_up)

    #     # print("pose_g_with_offset:",self.pose_g_with_offset)
    #     # print("pose_g_up_with_offset:",self.pose_up_with_offset)
    #     print("pose_g:",self.pose_g)
    #     print("pose_g_up:",self.pose_up)

    # def press_move_robot_button(self):
    #     rospy.loginfo("Start grasp !")
    #     result = move_robot_to_pose_client(self.rotation_matrix, self.pose_g, self.pose_up)
    #     if result:
    #         rospy.loginfo("Robot moved successfully!")
    #     else:
    #         rospy.loginfo("Failed to move robot.")
    #     self.message_var.set("Robot moved successfully!")
    
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
