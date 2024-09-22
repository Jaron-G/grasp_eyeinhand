import sys
import tkinter as tk
import numpy as np
import tkinter.messagebox

path = "D:\\PycharmProjects\\collision_detect\\main_ui\\"


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
        self.reset_robot_button = tk.Button(
            self.frame1,
            text="Reset Robot",
            command=self.press_reset_robot_button,
            width=28,
        )
        self.reset_robot_button.pack(pady=5)
        self.obtain_pose_button = tk.Button(
            self.frame1,
            text="Obtain Pose",
            command=self.press_obtain_pose_button,
            width=28,
        )
        self.obtain_pose_button.pack(pady=5)
        self.move_pose_button = tk.Button(
            self.frame1, text="Move Pose", command=self.press_move_pose_button, width=28
        )
        self.move_pose_button.pack(pady=5)
        self.obtain_joint_angle_button = tk.Button(
            self.frame1,
            text="Obtain Joint Angle",
            command=self.press_obtain_joint_angle_button,
            width=28,
        )
        self.obtain_joint_angle_button.pack(pady=5)
        self.move_joint_angle_button = tk.Button(
            self.frame1,
            text="Move Joint Angle",
            command=self.press_move_joint_angle_button,
            width=28,
        )
        self.move_joint_angle_button.pack(pady=5)
        self.action_button = tk.Button(
            self.frame1, text="Action", command=self.press_action_button, width=28
        )
        self.action_button.pack(pady=5)
        self.open_gripper_button = tk.Button(
            self.frame1,
            text="Open Gripper To Width",
            command=self.press_open_gripper_button,
            width=28,
        )
        self.open_gripper_button.pack(pady=5)
        self.quit = tk.Button(
            self.frame1,
            text="QUIT",
            fg="red",
            command=self.master.destroy,
            bd=3,
            width=28,
        )
        self.quit.pack(pady=5)

        # self.photo = tk.PhotoImage(file=path + "robot.png")
        # self.photo_label = tk.Label(
        #     self.frame2, justify=tk.LEFT, image=self.photo
        # ).grid(row=0, column=0)
        # self.photo_label.pack(side="right")

        self.message_var = tk.StringVar(
            self, value="Please click the buttons in order !"
        )  # 储存文字的类
        self.message_box = tk.Label(
            self.frame1, textvariable=self.message_var, bg="lightblue", width=28
        )
        self.message_box.pack(pady=5)

        # 夹爪开合
        self.gripper_width_label = tk.Label(self.frame2, text="gripper width :").grid(
            row=0, column=0
        )
        self.entry0 = tk.Entry(self.frame2)
        self.entry0.grid(row=0, column=1, padx=2, pady=5)

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
        self.debug_checkbox = tk.Checkbutton(
            self, text="Debug Mode", variable=self.debug_var, command=self.debug_check
        )
        self.debug_checkbox.pack()

        self.frame1.pack(side="left")  # 左框架对齐
        self.frame2.pack(side="right")  # 右框架对齐
        # self.frame3.pack(side="bottom")

    # 在终端打印当前模式信息
    def debug_check(self):
        if self.debug_var.get() == 1:
            print("Start debug mode !")
        else:
            print("Start normal mode !")

    def press_open_gripper_button(self):
        if 0 <= int(self.entry0.get()) <= 85:
            self.gripper_width = int(self.entry0.get())
            width = -255 / 85 * self.gripper_width + 255
            print("Gripper open" + self.entry1.get() + " mm !")
            self.message_var.set("open gripper completed!")
            return True
        else:
            print("Please enter 0-85")
            self.entry0.delete(0)
            tkinter.messagebox.showwarning(
                title="Warning", message="Gripper width should be 0-85 ！"
            )
            return False

    def press_obtain_pose_button(self):
        current_position = np.array([1.99, 299.3, 3])
        # transforms3d四元数转矩阵函数的四元数格式为(w,x,y,z)
        current_rotation = np.array([1.1, 0.22, 3, 4])

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
            self.entry_orientation_x,
            self.entry_orientation_y,
            self.entry_orientation_z,
            self.entry_orientation_w,
        ]

        for i in range(6):
            pose_entry[i].delete(0, 100)
            pose_entry[i].insert(0, pose[i])

        np.save(path + "current_position.npy", current_position)
        np.save(path + "current_rotation.npy", current_rotation)
        print("current_rotation.npy", current_rotation)
        print("current_position.npy", current_position)
        self.message_var.set("obtain_pose ccompleted!")

    def press_obtain_joint_angle_button(self):
        current_joint_angle = np.array([9.9, 8.123, 0.3337, 6, 5, 4])
        joint_angle = np.array(
            [
                current_joint_angle[0],
                current_joint_angle[1],
                current_joint_angle[2],
                current_joint_angle[3],
                current_joint_angle[4],
                current_joint_angle[5],
            ]
        )
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
            joint_entry[i].insert(0, joint_angle[i])

        np.save(path + "current_joint_angle.npy", joint_angle)
        print("current_joint_angle.npy", joint_angle)
        self.message_var.set("obtain joint angle completed!")

    def press_move_pose_button(self):
        position = np.load(path + "current_position.npy")
        rotation = np.load(path + "current_rotation.npy")
        print("current_rotation.npy", rotation)
        print("current_position.npy", position)
        pose = np.array(
            [
                position[0],
                position[1],
                position[2],
                rotation[0],
                rotation[1],
                rotation[2],
                rotation[3],
                rotation[4],
            ]
        )
        pose_entry = [
            self.entry_position_x,
            self.entry_position_y,
            self.entry_position_z,
            self.entry_orientation_x,
            self.entry_orientation_y,
            self.entry_orientation_z,
            self.entry_orientation_w,
        ]

        for i in range(7):
            pose_entry[i].delete(0, 100)
            pose_entry[i].insert(0, pose[i])

        print("机器人移动到指定位置{},{}".format(position, rotation))
        self.message_var.set("Move pose completed!")

    def press_move_joint_angle_button(self):
        joint_goal = np.load(path + "current_joint_angle.npy")
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

        print("机器人移动到指定关节{}位置".format(joint_goal))
        self.message_var.set("Move joint goal completed!")

    def press_action_button(self):
        action_list = ["Aa01", "Aa02", "Aa04"]
        action_path = path + "Action\\"
        for i in action_list:
            position = np.load(action_path + i + "\\current_position.npy")
            rotation = np.load(action_path + i + "\\current_rotation.npy")
            print("机器人移动到指定位置{},{}".format(position, rotation))
        self.message_var.set("action ccompleted!")

    def press_reset_robot_button(self):
        print("Reset Robot !")
        joint_goal = np.array([0, 0, 0, 0, 0, 0])
        joint_goal[0] = 0
        joint_goal[1] = -1.544
        joint_goal[2] = 1.544
        joint_goal[3] = -1.5707
        joint_goal[4] = -1.5707
        joint_goal[5] = -1.5707
        print("Reset robot successfully!")
        self.message_var.set("Reset Robot successfully !")


def main():
    root = tk.Tk()
    root.title("test grasp")
    root.geometry("800x400")
    app = Application(master=root)
    app.mainloop()


if __name__ == "__main__":
    main()
