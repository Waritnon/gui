import webview
from tkinter import Tk, Button
import time

# Function to load the webpage using pywebview
def load_webpage():
    webview.create_window('Webview', 'http://192.168.162.201:3000/ros_control', fullscreen=False)
    webview.start()

# Function to close the webview window
def close_webview():
    if webview.windows:
        webview.windows[0].destroy()  # This closes the first webview window

# Initialize Tkinter window
root = Tk()
root.geometry("800x600")

# Create an Exit button
exit_button = Button(root, text="Exit", command=close_webview)
exit_button.pack(pady=20)

# Start the webview in the main thread
root.after(0, load_webpage)

# Run the Tkinter event loop
root.mainloop()
