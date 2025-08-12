
import tkinter as tk
from PIL import Image, ImageTk
from enum import Enum
from tkinter import messagebox

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys

import time

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


BG_color = "gray"

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
    Marsipan = 13

# --- ROS Node ---
class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher')
        self.publisher_ = self.create_publisher(String, 'PI_command', 10)
        self.subscription = self.create_subscription(
            String,
            'PI_feedback',
            self.feedback_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.message_handlers = {}  # Set externally from GUI

    def send_msg(self, message: str):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sent message: {msg.data}")
    


    def feedback_callback(self, msg: String):
        message = msg.data.strip()
        self.get_logger().info(f"Received message: {message}")
        handler = self.message_handlers.get(message)
        if handler:
            handler()
        else:
            self.get_logger().warn(f"No handler registered for message: {message}")

    def register_handler(self, message: str, handler_func):
        """ Register a handler for a specific message value. """
        self.message_handlers[message] = handler_func

    def wait_for_subscriber(self, timeout_sec=10):
        start = time.time()
        while time.time() - start < timeout_sec:
            if self.publisher_.get_subscription_count() > 0:
                self.get_logger().info("Subscriber discovered!")
                return True
            self.get_logger().info("Waiting for subscriber...")
            time.sleep(0.5)
        self.get_logger().warn("No subscriber discovered within timeout.")
        return False

# --- App ---
class App(tk.Tk):
    def __init__(self):
        super().__init__()  # must come first

        self.title("Delta Robot GUI")
        self.configure(bg=BG_color)

        # Now we can safely get screen size
        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        print(f"Detected screen size: {screen_width}x{screen_height}")

        # Use geometry instead of fullscreen
        self.geometry(f"{screen_width}x{screen_height}+0+0")
        self.overrideredirect(True)  # optional: remove title bar
        print(f"Detected screen size: {screen_width}x{screen_height}")

        self.bind("<Escape>", lambda event: self.quit())

        container = tk.Frame(self, bg=BG_color)
        container.pack(fill="both", expand=True)

        self.frames = {}

        for F in (LoadingScreen, StartScreen, ManualScreen, AutomaticScreen):
            frame = F(container, self)
            self.frames[F] = frame
            frame.place(relwidth=1, relheight=1)

        self.show_frame(LoadingScreen)

    def show_frame(self, screen_class):
        frame = self.frames[screen_class]
        frame.tkraise()

class LoadingScreen(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent, bg=BG_color)
        self.controller = controller
        self.dot_count = 0
        self.max_dots = 3

        # Load and display the logo
        logo_img = Image.open("pictures/DRLogo.png")
        logo_img.thumbnail((400, 200), Image.Resampling.LANCZOS)
        self.logo_photo = ImageTk.PhotoImage(logo_img)

        tk.Label(self, image=self.logo_photo, bg=BG_color).pack(pady=(100, 40))

        # Loading text label
        self.loading_label = tk.Label(self, text="Loading", font=("Helvetica", 20), fg="white", bg=BG_color)
        self.loading_label.pack()

        twist_publisher.send_msg("PI_READY")
        
        self.animate_loading()

    def animate_loading(self):
        # Cycle through dot count
        dots = '.' * (self.dot_count % (self.max_dots + 1))
        self.loading_label.config(text=f"Loading{dots}")
        self.dot_count += 1
        self.after(500, self.animate_loading)  # update every 500ms

# --- Start Screen ---
class StartScreen(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent, bg=BG_color)
        self.controller = controller

        """
        tk.Button(
            self,
            text="Exit to Desktop",
            command=self.exit_to_desktop,
            **button_style
        ).place(x=20, y=10, anchor="nw")
        """


        logo_img = Image.open("pictures/DRLogo.png")
        logo_img.thumbnail((400, 200), Image.Resampling.LANCZOS)
        self.logo_photo = ImageTk.PhotoImage(logo_img)
        tk.Label(self, image=self.logo_photo, bg=BG_color).pack(pady=(60, 40))

        tk.Label(self, text="Velg modus:", font=("Helvetica", 20), fg="white", bg=BG_color).pack(pady=10)

        button_frame = tk.Frame(self, bg=BG_color)
        button_frame.pack(pady=(0, 40))

        tk.Button(button_frame, text="Manuell Styring",
            command=lambda: controller.show_frame(ManualScreen),
            **button_style).grid(row=0, column=0, padx=20, pady=10, sticky="ew")

        tk.Button(button_frame, text="Automatisk Plukking",
            command=lambda: controller.show_frame(AutomaticScreen),
            **button_style).grid(row=0, column=1, padx=20, pady=10, sticky="ew")
        """"
        tk.Button(button_frame, text="Test Modus",
            command=lambda: controller.show_frame(TestScreen),
            **button_style).grid(row=0, column=2, padx=20, pady=10, sticky="ew")
        """

    def exit_to_desktop(self):
        print("Exiting GUI to desktop safely...")
        self.controller.destroy()  # Destroys the main Tk window (if desired)

class ManualScreen(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent, bg=BG_color)
        self.controller = controller

        tk.Label(self, text="Manuell", font=("Helvetica", 16, "bold"), fg="#cc0000", bg=BG_color).place(x=20, y=10)

        # Layout: Left = circle canvas, Right = buttons
        main_frame = tk.Frame(self, bg=BG_color)
        main_frame.place(relx=0.5, rely=0.5, anchor="center", relwidth=0.95, relheight=0.85)

        # Set canvas size slightly smaller than screen height
        canvas_size = int(min(self.winfo_screenheight(), self.winfo_screenwidth()) * 0.85)
        self.canvas_size = canvas_size
        self.radius = canvas_size // 2

        # Canvas with transparent background (BG_color)
        self.canvas = tk.Canvas(main_frame, bg=BG_color, highlightthickness=0,
                                width=canvas_size, height=canvas_size)
        self.canvas.pack(side="left", padx=20, pady=20)

        # Draw centered white circle (oval)
        self.canvas.create_oval(0, 0, canvas_size, canvas_size, fill="white", outline="black")

        # Bind click handler
        self.canvas.bind("<Button-1>", self.on_canvas_click)

        # Button column on the right
        button_frame = tk.Frame(main_frame, bg=BG_color)
        button_frame.pack(side="right", padx=20, pady=20, fill="y")

        tk.Button(button_frame, text="↑", width=6,
                  command=lambda: self.move("Up"), **button_style).pack(pady=10)

        tk.Button(button_frame, text="↓", width=6,
                  command=lambda: self.move("Down"), **button_style).pack(pady=10)

        tk.Button(button_frame, text="Pump On", width=10,
                  command=lambda: twist_publisher.send_msg("PUMP_ON"), **button_style).pack(pady=10)

        tk.Button(button_frame, text="Pump Off", width=10,
                  command=lambda: twist_publisher.send_msg("PUMP_OFF"), **button_style).pack(pady=10)

        tk.Button(button_frame, text="Tilbake", width=10,
                  command=lambda: controller.show_frame(StartScreen), **button_style).pack(pady=30)


    def on_canvas_click(self, event):
        cx = self.canvas_size / 2
        cy = self.canvas_size / 2
        rel_x = event.x - cx
        rel_y = event.y - cy

        distance_squared = rel_x**2 + rel_y**2

        # Only allow clicks inside the circle
        if distance_squared > self.radius**2:
            print("Click outside circle — ignored.")
            return

        # Normalize: divide by radius
        norm_x = rel_x / self.radius
        norm_y = rel_y / self.radius

        print(f"Normalized click: ({norm_x:.3f}, {norm_y:.3f})")
        twist_publisher.send_msg(f"CLICK {norm_x:.3f},{norm_y:.3f}")



    def move(self, direction):
        twist_publisher.send_msg(f"MOVE_{direction.upper()}")


# --- Automatic Screen ---
class AutomaticScreen(tk.Frame):

    def __init__(self, parent, controller):
        super().__init__(parent, bg=BG_color)
        self.controller = controller
        self.waiting_animation_running = False
        self.dot_count = 0
        # Popup dimensions
        self.popup_width = 400
        self.popup_height = 200
        self.send_message = True

        self.active_twist = None

        # Text in top left corner
        tk.Label(self, text="Auto", font=("Helvetica", 16, "bold"), fg="#cc0000", bg=BG_color).place(x=20, y=10)

        tk.Label(self, text="Velg din Twist:", font=("Arial", 18), fg="white", bg=BG_color).pack(pady=(20, 10))

        # Container frame for the twist grid
        grid_container = tk.Frame(self, bg=BG_color, height=500)
        grid_container.pack(padx=40, pady=(0, 20))
        grid_container.pack_propagate(False)



        # Create 4 columns and 3 rows that expand equally
        for col in range(5):
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
                    bg=BG_color,
                    activebackground=BG_color,
                    borderwidth=0,
                    highlightthickness=0,
                    relief='flat'
                )
                btn.image = photo
                btn.grid(
                    row=i // 4,
                    column=i % 4,
                    padx=10,
                    pady=4,
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

        # Register GUI callback with ROS node
        #twist_publisher.register_handler("PICKED_UP", self.twist_picked_up)

        # Transparent overlay for blocking buttons in the background
        self.blocker = tk.Frame(self, bg="", width=1, height=1)
        self.blocker.place_forget()
        self.blocker.bind("<Button-1>", lambda e: "break")  # absorb clicksself.blocker = tk.Frame(self, bg="", width=1, height=1)
        self.blocker.place_forget()
        self.blocker.bind("<Button-1>", lambda e: "break")  # absorb clicks

        # Create the loading overlay frame

        self.loading_overlay = tk.Frame(self, bg="#4c4c4c")
        self.loading_overlay.place_forget()  # Start hidden

        # Content inside the frame
        self.loading_label_title = tk.Label(self.loading_overlay, text="Henter ...",
                                            font=("Helvetica", 18), fg="white", bg="#4c4c4c")
        self.loading_label_title.pack(pady=(20, 10))

        self.loading_label_status = tk.Label(self.loading_overlay, text="Vennligst vent...",
                                            font=("Helvetica", 14), fg="white", bg="#4c4c4c")
        self.loading_label_status.pack()

        tk.Button(self.loading_overlay, text="Avbryt", command=self.abort_and_close_overlay,
                font=("Helvetica", 12), bg="#cc0000", fg="white", width=15,
                activebackground="#990000", activeforeground="white",
                borderwidth=0, highlightthickness=0, relief="flat").pack(pady=10)
        
        twist_publisher.register_handler("EMPTY", lambda: self.after(0, self.handle_twist_empty))
        twist_publisher.register_handler("DELIVERED", lambda: self.after(0, self.handle_twist_delivered))
        twist_publisher.register_handler("LOST", lambda: self.after(0, self.handle_twist_lost))


    def on_button_click(self, twist):
        if self.send_message:
            twist_publisher.send_msg(twist.name)
            self.active_twist = twist.name
            self.show_loading_overlay()


    def show_loading_overlay(self):
        self.loading_label_title.config(text="Vennligst vent...")
        self.loading_label_status.config(text=f"{self.active_twist} levert")
        self.dot_count = 0
        self.waiting_animation_running = True

        # Make the blocker cover the whole window
        self.blocker.place(relx=0, rely=0, relwidth=1, relheight=1)
        self.blocker.lift()

        # Lift the overlay above the blocker
        self.loading_overlay.lift()
        self.loading_overlay.place(relx=0.5, rely=0.5, anchor="center",
                                width=self.popup_width, height=self.popup_height)
        self.animate_dots_overlay()


    def twist_picked_up(self):
        # Called from ROS thread; use `after` to safely update GUI
        print("Twist picked up, closing overlay.")
        #LEGGE TIL HER

    def abort_and_close_overlay(self):
        if self.send_message:
            twist_publisher.send_msg("ABORT")
        self.waiting_animation_running = False
        self.loading_overlay.place_forget()
        self.blocker.place_forget()
        self.active_twist = None
    
    def close_overlay(self):
        self.waiting_animation_running = False
        self.loading_overlay.place_forget()
        self.blocker.place_forget()
        self.active_twist = None

    def animate_dots_overlay(self):
        if not self.waiting_animation_running:
            return
        dots = "." * (self.dot_count % 4)
        self.loading_label_status.config(text=f"Vennligst vent{dots}")
        self.dot_count += 1
        self.after(500, self.animate_dots_overlay)
    
    def handle_twist_empty(self):
        if self.active_twist:
            self.waiting_animation_running = False
            self.loading_label_status.config(text=f"Tomt for {self.active_twist}")
            self.after(2000, self.close_overlay)

    def handle_twist_delivered(self):
        if self.active_twist:
            self.waiting_animation_running = False
            self.loading_label_status.config(text=f"{self.active_twist} levert")
            self.after(2000, self.close_overlay)
    
    def handle_twist_lost(self):
        if self.active_twist:
            self.waiting_animation_running = False
            self.loading_label_status.config(text=f"{self.active_twist} mistet. Prøv igjen.")
            self.after(2000, self.close_overlay)

def reboot_app(app_to_close):
    print("🛑 REBOOT message received — exiting app.")
    app_to_close.quit()
    app_to_close.destroy()
    sys.exit(1)


# --- Main ---
if __name__ == "__main__":
    #time.sleep(5)  # wait for ROS networking to be ready

    rclpy.init()
    twist_publisher = TwistPublisher()
    threading.Thread(target=rclpy.spin, args=(twist_publisher,), daemon=True).start()
    #twist_publisher.wait_for_subscriber(timeout_sec=10)


    app = App()

    # Register handlers for ROS messages
    twist_publisher.register_handler("SETUP_FINISHED", lambda: app.after(0, lambda: app.show_frame(StartScreen)))

    twist_publisher.register_handler("REBOOT", lambda: app.after(0, lambda: reboot_app(app)))

    app.mainloop()