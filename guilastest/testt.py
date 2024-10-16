import tkinter as tk
from tkinter import ttk

# สร้างหน้าต่างหลัก
root = tk.Tk()
root.title("Combobox Example")

# สร้าง Combobox พร้อมตัวเลือก
options = ["Option 1", "Option 2", "Option 3", "Option 4", "Option 5"]
combo = ttk.Combobox(root, values=options)
combo.set("Select an option")  # ข้อความเริ่มต้น
combo.pack(pady=10)

# สร้างปุ่มเพื่อแสดงค่าที่เลือก
def show_selection():
    selection = combo.get()
    print(f"Selected: {selection}")

button = tk.Button(root, text="Show Selection", command=show_selection)
button.pack(pady=10)

root.mainloop()
