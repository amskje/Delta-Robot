import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
from enum import Enum

class Twist(Enum):
COCOS = 1
DAIM = 2
CARAMEL = 3
CRISP = 4
FRANSK = 5
GOLDEN = 6
JAPP = 7
NOTTI = 8

def on_button_click(twist: Twist):
messagebox.showinfo("Selection", f"Du valgte {twist.name}")

root = tk.Tk()
root.title("Touchscreen Button GUI")

root.attributes('-fullscreen', True)

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

images = []

for i, img_file in enumerate(image_files):
img = Image.open(img_file)
img = img.resize((100,100), Image.ANTIALIAS)
photo =ImageTk.PhotoImage(img)
images.append(photo)

btn = tk.Button(root, image =photo, command=lambda x=i+1: on_button_click(Twist(x)))
btn.grid(row=i//4, column=i%4, padx=10, pady=10)

root.mainloop()


