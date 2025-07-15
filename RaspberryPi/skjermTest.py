import tkinter as tk

def on_button_press():
    print("Button was pressed")

# Create the main window
root = tk.Tk()
root.configure(bg="black")      # Set background to black
root.attributes('-fullscreen', True)  # Make fullscreen

# Create the button
button = tk.Button(
    root,
    text="Press Me",
    fg="white",          # Text color
    bg="red",            # Button background
    font=("Arial", 24),  # Font and size
    command=on_button_press
)

# Place the button in the center
button.pack(expand=True)

# Start the GUI event loop
root.mainloop()
