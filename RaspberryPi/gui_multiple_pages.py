import tkinter as tk
from PIL import Image, ImageTk
from enum import Enum
from tkinter import messagebox

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class App(tk.Tk):

    def __init__(self):
        super().__init__()
        self.title("Delta Robot GUI")
        self.attributes('-fullscreen', True)
        self.bind("<Escape>", lambda event: self.quit())

        self.container = tk.Frame(self)
        self.container.pack(fill="both", expand=True)

        self.frames = {}

        for F in (StartScreen, ManualScreen, AutomaticScreen):
            frame = F(parent=self.container, controller=self)
            self.frames[F] = frame
            frame.grid(row=0, column=0, sticky="nsew")

        self.show_frame(StartScreen)

    def show_frame(self, screen_class):
        frame = self.frames[screen_class]
        frame.tkraise()


#---Start screen---
class StartScreen(tk.Frame):

    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller

        self.configure_grid()

        # Logo
        logo_img = Image.open("twist_bilder/image.png")
        logo_img = logo_img.resize((300, 150))
        self.logo_photo = ImageTk.PhotoImage(logo_img)
        logo_label = tk.Label(self, image=self.logo_photo)
        logo_label.grid(row=0, column=0, columnspan=2, pady=30)

        # Mode selection
        tk.Label(self, text="Velg modus:", font=("Arial", 20)).grid(row=1, column=0, columnspan=2, pady=10)

        tk.Button(self, text="Manuell Modus", font=("Arial", 16),
        width=20, height=2,
        command=lambda: controller.show_frame(ManualScreen)).grid(row=2, column=0, pady=10, padx=10)

        tk.Button(self, text="Automatisk Modus", font=("Arial", 16),

        width=20, height=2,

        command=lambda: controller.show_frame(AutomaticScreen)).grid(row=2, column=1, pady=10, padx=10)

    def configure_grid(self):
        self.grid_columnconfigure((0, 1), weight=1)
        self.grid_rowconfigure(0, weight=1)

class ManualScreen(tk.Frame):

    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller

        center = tk.Frame(self)
        center.place(relx=0.5, rely=0.5, anchor="center")

        tk.Label(center, text="Manuell kontroll", font=("Arial", 20)).grid(row=0, column=1, pady=20)

        # D-pad
        btn_up = tk.Button(center, text="↑", font=("Arial", 24), width=5, command=lambda: self.move("Up"), bg="red", fg="white")
        btn_up.grid(row=1, column=1, pady=5)

        btn_left = tk.Button(center, text="←", font=("Arial", 24), width=5, command=lambda: self.move("Left"), bg="red", fg="white")
        btn_left.grid(row=2, column=0, padx=5)

        btn_down = tk.Button(center, text="↓", font=("Arial", 24), width=5, command=lambda: self.move("down"), bg="red", fg="white")
        btn_down.grid(row=2, column=1, pady=5)
        btn_right = tk.Button(center, text="→", font=("Arial", 24), width=5, command=lambda: self.move("right"), bg="red", fg="white")
        btn_right.grid(row=2, column=2, padx=5)

        # Back button
        tk.Button(center, text="Tilbake", font=("Arial", 14),
        command=lambda: controller.show_frame(StartScreen)).grid(row=3, column=1, pady=20)

    def move(self, direction):
        print("Robot moves ", direction)



class AutomaticScreen(tk.Frame):

    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller

        tk.Label(self, text="Velg din Twist:", font=("Arial", 18)).pack(pady=20)

        button_frame = tk.Frame(self)
        button_frame.pack()

        class Twist(Enum):
            COCOS = 1
            DAIM = 2
            CARAMEL = 3
            CRISP = 4
            FRANSK = 5
            GOLDEN = 6
            JAPP = 7
            NOTTI = 8

        self.Twist = Twist

        self.images = []

        image_files = [
            "twist_bilder/cocos.png",
            "twist_bilder/daim.png",
            "twist_bilder/caramel.png",
            "twist_bilder/crisp.png",
            "twist_bilder/fransk.png",
            "twist_bilder/golden.png",
            "twist_bilder/japp.png",
            "twist_bilder/notti.png"
            ]

        for i, img_file in enumerate(image_files):
            img = Image.open(img_file).resize((100, 100))
            photo = ImageTk.PhotoImage(img)
            self.images.append(photo)

            btn = tk.Button(button_frame, image=photo,
            command=lambda x=i+1: self.on_button_click(Twist(x)))

            btn.grid(row=i // 4, column=i % 4, padx=10, pady=10)

        tk.Button(self, text="Tilbake", font=("Arial", 14),
            command=lambda: controller.show_frame(StartScreen)).pack(pady=20)


    def on_button_click(self, twist):
        messagebox.showinfo("Selection", f"Du valgte {twist.name}")
        print("Twisten valgt er nummer", twist.value)
        twist_publisher.send_twist(twist.name)


class TwistPublisher(Node):

    def __init__(self):
        super().__init__('twist_publisher')
        self.publisher_ = self.create_publisher(String, 'twist_selection', 10)

    def send_twist(self, twist_name: str):
        msg = String()
        msg.data = twist_name
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sent twist: {msg.data}")

if __name__ == "__main__":

    #Start ROS 2
    rclpy.init()
    twist_publisher =TwistPublisher()

    ros_thread = threading.Thread(target=rclpy.spin, args=(twist_publisher,), daemon=True)
    ros_thread.start()

    app = App()
    app.mainloop()


