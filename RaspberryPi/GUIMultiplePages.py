import tkinter as tk
from PIL import Image, ImageTk
from enum import Enum
from tkinter import messagebox

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

# --- Global Style ---
button_style = {
    "bg": "#cc0000",                 # Red
    "fg": "white",
    "activebackground": "#990000",  # Darker red on press
    "activeforeground": "white",
    "borderwidth": 0,
    "highlightthickness": 0,
    "relief": "flat",
    "font": ("Helvetica", 16)
}

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

# --- App ---
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

        logo_img = Image.open("pictures/DRLogo.png")
        logo_img.thumbnail((400, 200), Image.Resampling.LANCZOS)
        self.logo_photo = ImageTk.PhotoImage(logo_img)
        tk.Label(self, image=self.logo_photo, bg="black").pack(pady=(60, 40))

        tk.Label(self, text="Velg modus:", font=("Helvetica", 20), fg="white", bg="black").pack(pady=10)

        button_frame = tk.Frame(self, bg="black")
        button_frame.pack(pady=(0, 40))

        tk.Button(button_frame, text="Manuell Modus",
            command=lambda: controller.show_frame(ManualScreen),
            **button_style).grid(row=0, column=0, padx=20, pady=10, sticky="ew")

        tk.Button(button_frame, text="Automatisk Modus",
            command=lambda: controller.show_frame(AutomaticScreen),
            **button_style).grid(row=0, column=1, padx=20, pady=10, sticky="ew")

        tk.Button(button_frame, text="Test Modus",
            command=lambda: controller.show_frame(TestScreen),
            **button_style).grid(row=0, column=2, padx=20, pady=10, sticky="ew")

# --- Manual Screen ---
class ManualScreen(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent, bg="black")
        self.controller = controller

        center = tk.Frame(self, bg="black")
        center.place(relx=0.5, rely=0.5, anchor="center")

        tk.Label(center, text="Manuell kontroll", font=("Helvetica", 20), fg="white", bg="black").grid(row=0, column=1, pady=20)

        tk.Button(center, text="↑", width=5,
                  command=lambda: self.move("Up"),
                  **button_style).grid(row=1, column=1, pady=5)

        tk.Button(center, text="←", width=5,
                  command=lambda: self.move("Left"),
                  **button_style).grid(row=2, column=0, padx=5)

        tk.Button(center, text="↓", width=5,
                  command=lambda: self.move("Down"),
                  **button_style).grid(row=2, column=1, pady=5)

        tk.Button(center, text="→", width=5,
                  command=lambda: self.move("Right"),
                  **button_style).grid(row=2, column=2, padx=5)

        tk.Button(center, text="Tilbake",
                  command=lambda: controller.show_frame(StartScreen),
                  **button_style).grid(row=3, column=1, pady=20)

    def move(self, direction):
        print("Robot moves", direction)

# --- Automatic Screen ---
class AutomaticScreen(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent, bg="black")
        self.controller = controller

        tk.Label(self, text="Velg din Twist:", font=("Arial", 18), fg="white", bg="black").pack(pady=(20, 10))

        # Container frame for the twist grid
        grid_container = tk.Frame(self, bg="black", height=500)
        grid_container.pack(padx=40, pady=(0, 20))
        grid_container.pack_propagate(False)



        # Create 4 columns and 3 rows that expand equally
        for col in range(4):
            grid_container.columnconfigure(col, weight=1)
        for row in range(3):
            grid_container.rowconfigure(row, weight=1)

        self.images = []

        for i, twist in enumerate(Twist):
            try:
                image_path = f"pictures/twist/{twist.name.lower()}.png"
                img = Image.open(image_path)

                # Larger image size
                if twist == Twist.Notti:
                    img.thumbnail((90, 90), Image.Resampling.LANCZOS)
                else:
                    img.thumbnail((130, 130), Image.Resampling.LANCZOS)

                photo = ImageTk.PhotoImage(img)
                self.images.append(photo)

                btn = tk.Button(
                    grid_container,
                    image=photo,
                    command=lambda t=twist: self.on_button_click(t),
                    bg='black',
                    borderwidth=0,
                    highlightthickness=0,
                    relief='flat',
                    activebackground='black'
                )
                btn.image = photo
                btn.grid(
                    row=i // 4,
                    column=i % 4,
                    padx=10,
                    pady=10,
                    sticky="nsew"
                )

            except Exception as e:
                print(f"Failed to load {twist.name}: {e}")

        # "Tilbake" button anchored at the bottom
        tk.Button(
            self,
            text="Tilbake",
            command=lambda: controller.show_frame(StartScreen),
            **button_style
        ).pack(pady=20)


    def on_button_click(self, twist):
        if send_message:
            twist_publisher.send_twist(twist.name)
        messagebox.showinfo("Valg", f"Du valgte {twist.name}")

# --- Test Screen ---
class TestScreen(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent, bg="black")
        self.controller = controller

        tk.Label(self, text="Velg en twist:", font=("Helvetica", 18), fg="white", bg="black").place(x=300, y=30)

        self.images = []
        image_files = [f"pictures/twist/{tw.name.lower()}.png" for tw in Twist]

        positions = [
            (50, 100), (200, 100), (350, 100), (500, 100),
            (50, 250), (200, 250), (350, 250), (500, 250),
            (50, 400), (200, 400), (350, 400), (500, 400)
        ]

        for i, twist in enumerate(Twist):
            img = Image.open(image_files[i]).resize((100, 100))
            photo = ImageTk.PhotoImage(img)
            self.images.append(photo)

            btn = tk.Button(self, image=photo,
                            command=lambda t=twist: self.on_button_click(t),
                            bg='black', activebackground='black',
                            borderwidth=0, highlightthickness=0, relief='flat')
            btn.place(x=positions[i][0], y=positions[i][1])
            btn.image = photo

        logo_img = Image.open("pictures/placeholder.jpg").resize((160, 430))
        self.logo_photo = ImageTk.PhotoImage(logo_img)
        tk.Label(self, image=self.logo_photo, bg="black").place(x=640, y=45)

        tk.Button(self, text="Tilbake",
                  command=lambda: controller.show_frame(StartScreen),
                  **button_style).place(x=350, y=365)

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
