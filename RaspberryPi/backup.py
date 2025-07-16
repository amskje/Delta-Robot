class AutomaticScreen(tk.Frame):

    def __init__(self, parent, controller):
        super().__init__(parent, bg="black")
        self.controller = controller

        tk.Label(self, text="Velg din Twist:", font=("Arial", 18), fg="white", bg="black").pack(pady=20)

        container_frame = tk.Frame(self, bg="black")
        container_frame.pack(expand=True)

        button_frame = tk.Frame(container_frame, bg="black")
        button_frame.place(relx=0.5, rely=0, anchor="n")  # Horizontally centered at top of container


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
                btn.grid(row=i // 6, column=i % 6, padx=10, pady=10)
            except Exception as e:
                print(f"Failed to load {twist.name}: {e}")


        tk.Button(self, text="Tilbake", font=("Arial", 14),
                  command=lambda: controller.show_frame(StartScreen)).pack(pady=20)

 

    def on_button_click(self, twist):

        if send_message:
            twist_publisher.send_twist(twist.name)

        messagebox.showinfo("Valg", f"Du valgte {twist.name}")