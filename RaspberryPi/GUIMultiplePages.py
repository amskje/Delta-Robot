import tkinter as tk
from PIL import Image, ImageTk
from enum import Enum
from tkinter import messagebox

 

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading



#gjøre bakgrunn til kanpp liks som bakrunn, legge dette til i tk.Button: bg='black', fg='red', borderwidth=0, highlightthickness=0, relief='flat'





# --- Global Variables ---

send_message = True

 
# --- Twist Enum ---

class Twist(Enum):
    Cocos = 1
    Daim = 2
    Karamell = 3
    Crisp = 4
    Fransk = 5
    Golden = 6
    Japp = 7
    Notti = 8
    Eclairs = 9
    Toffee = 10
    Lakris = 11
    Banan = 12
    
# --- ROS Node ---

class TwistPublisher(Node):

    def __init__(self):
        super().__init__('twist_publisher')
        self.publisher_ = self.create_publisher(String, 'PI_command', 10)

 

    def send_twist(self, twist_name: str):
        msg = String()
        msg.data = twist_name
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sent twist: {msg.data}")

 

# --- App Class ---

class App(tk.Tk):

    def __init__(self):
        super().__init__()
        self.title("Delta Robot GUI")
        self.attributes('-fullscreen', True)
        self.configure(bg='black')
        self.bind("<Escape>", lambda event: self.quit())

        container = tk.Frame(self, bg='black')
        container.pack(fill="both", expand=True)

        self.frames = {}

        for F in (StartScreen, ManualScreen, AutomaticScreen, TestScreen):
            frame = F(container, self)
            self.frames[F] = frame
            frame.place(relwidth=1, relheight=1)

        self.show_frame(StartScreen)

    def show_frame(self, screen_class):
        frame = self.frames[screen_class]
        frame.tkraise()

 

# --- Start Screen ---

class StartScreen(tk.Frame):

    def __init__(self, parent, controller):
        super().__init__(parent, bg="black")
        self.controller = controller

        logo_img = Image.open("pictures/logo.png").resize((300, 150))
        self.logo_photo = ImageTk.PhotoImage(logo_img)

        tk.Label(self, image=self.logo_photo, bg="black").pack(pady=30)

        tk.Label(self, text="Velg modus:", font=("Arial", 20), fg="white", bg="black").pack(pady=10)

        button_frame = tk.Frame(self, bg="black")
        button_frame.pack()

        tk.Button(button_frame, text="Manuell Modus", font=("Arial", 16), width=20, height=2, bg='black', fg='red', borderwidth=0, highlightthickness=0, relief='flat',
                  command=lambda: controller.show_frame(ManualScreen)).grid(row=0, column=0, padx=10, pady=10)

        tk.Button(button_frame, text="Automatisk Modus", font=("Arial", 16), width=20, height=2,
                  command=lambda: controller.show_frame(AutomaticScreen)).grid(row=0, column=1, padx=10, pady=10)

        tk.Button(button_frame, text="Test Modus", font=("Arial", 16), width=20, height=2,
                  command=lambda: controller.show_frame(TestScreen)).grid(row=0, column=2, padx=10, pady=10)

 

# --- Manual Screen ---

class ManualScreen(tk.Frame):

    def __init__(self, parent, controller):
        super().__init__(parent, bg="black")
        self.controller = controller

        center = tk.Frame(self, bg="black")
        center.place(relx=0.5, rely=0.5, anchor="center")

        tk.Label(center, text="Manuell kontroll", font=("Arial", 20), fg="white", bg="black").grid(row=0, column=1, pady=20)

        tk.Button(center, text="↑", font=("Arial", 24), width=5, bg="red", fg="white",
                  command=lambda: self.move("Up")).grid(row=1, column=1, pady=5)

        tk.Button(center, text="←", font=("Arial", 24), width=5, bg="red", fg="white",
                  command=lambda: self.move("Left")).grid(row=2, column=0, padx=5)

        tk.Button(center, text="↓", font=("Arial", 24), width=5, bg="red", fg="white",
                  command=lambda: self.move("Down")).grid(row=2, column=1, pady=5)

        tk.Button(center, text="→", font=("Arial", 24), width=5, bg="red", fg="white",
                  command=lambda: self.move("Right")).grid(row=2, column=2, padx=5)

        tk.Button(center, text="Tilbake", font=("Arial", 14),
                  command=lambda: controller.show_frame(StartScreen)).grid(row=3, column=1, pady=20)

    def move(self, direction):
        print("Robot moves", direction)

 

# --- Automatic Screen ---

class AutomaticScreen(tk.Frame):

    def __init__(self, parent, controller):
        super().__init__(parent, bg="black")
        self.controller = controller

        tk.Label(self, text="Velg din Twist:", font=("Arial", 18), fg="white", bg="black").pack(pady=20)

        button_frame = tk.Frame(self, bg="black")
        button_frame.pack(expand=True, fill="both")

        self.images = []

        image_files = [f"pictures/twist/{tw.name.lower()}.png" for tw in Twist]

        for i, twist in enumerate(Twist):
            try:
                image_path = f"pictures/twist/{twist.name.lower()}.png"
                print(f"Loading: {image_path}")
                img = Image.open(image_path).resize((100, 100))
                photo = ImageTk.PhotoImage(img)
                self.images.append(photo)  # Prevent garbage collection
                btn = tk.Button(button_frame, image=photo, command=lambda t=twist: self.on_button_click(t))
                btn.grid(row=i // 4, column=i % 4, padx=10, pady=10)
            except Exception as e:
                print(f"Failed to load {twist.name}: {e}")


        tk.Button(self, text="Tilbake", font=("Arial", 14),
                  command=lambda: controller.show_frame(StartScreen)).pack(pady=20)

 

    def on_button_click(self, twist):

        if send_message:
            twist_publisher.send_twist(twist.name)

        messagebox.showinfo("Valg", f"Du valgte {twist.name}")

 
# --- Test Screen ---

class TestScreen(tk.Frame):

    def __init__(self, parent, controller):
        super().__init__(parent, bg="black")
        self.controller = controller

        tk.Label(self, text="Velg en twist:", font=("Arial", 18), fg="white", bg="black").place(x=300, y=30)

        self.images = []

        image_files = [f"pictures/twist/{tw.name.lower()}.png" for tw in Twist]

        positions = [(50, 100), (200, 100), (350, 100), (500, 100),
                     (50, 250), (200, 250), (350, 250), (500, 250)]

        for i, twist in enumerate(Twist):
            img = Image.open(image_files[i]).resize((100, 100))
            photo = ImageTk.PhotoImage(img)
            self.images.append(photo)

            btn = tk.Button(self, image=photo, command=lambda t=twist: self.on_button_click(t), bg='black', activebackground='black', borderwidth=0, highlightthickness=0, relief='flat',)
            btn.place(x=positions[i][0], y=positions[i][1])

        logo_img = Image.open("pictures/placeholder.jpg").resize((160, 430))
        self.logo_photo = ImageTk.PhotoImage(logo_img)

        tk.Label(self, image=self.logo_photo, bg="black").place(x=640, y=45)

        tk.Button(self, text="Tilbake", font=("Arial", 14),
                  command=lambda: controller.show_frame(StartScreen)).place(x=350, y=365)

    def on_button_click(self, twist):

        if send_message:
            twist_publisher.send_twist(twist.name)

 

# --- Main ---

if __name__ == "__main__":

    if send_message:
        rclpy.init()
        twist_publisher = TwistPublisher()
        threading.Thread(target=rclpy.spin, args=(twist_publisher,), daemon=True).start()

    app = App()
    app.mainloop()