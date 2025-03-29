from pathlib import Path
import sqlite3
from tkinter import Tk, Canvas, Button, PhotoImage, messagebox, ttk, font
import tkinter.font as tkFont
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import json
import threading

class RosGui(Node):
    def __init__(self):
        super().__init__('ros_gui')
        self.publisher_ = self.create_publisher(String, 'sound_command', 10)

class MyServer:
    def __init__(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind(('localhost', 7000))
        self.server.listen(1)
        print("Waiting for connection...")

        # Start listening in a separate thread
        self.thread = threading.Thread(target=self.accept_connection, daemon=True)
        self.thread.start()

    def accept_connection(self):
        self.connection, self.address = self.server.accept()
        self.connection.settimeout(10)
        print(f"Connected to {self.address}")
    
    def recived(self):
         # เช็คก่อนว่า connection มีค่าหรือยัง
        if self.connection is None:
            print("No client connected yet!")
            return None
        try:
            data = self.connection.recv(1024).decode()
            return data
        except socket.error as e:
            print(f"Socket error: {e}")
            return None
            
class HealthMonitorApp:
    # Init
    ###################################################################################################################################################
    def __init__(self,server):
        self.server = server
        rclpy.init(args=None)
        self.ros = RosGui()
        self.OUTPUT_PATH = Path(__file__).parent
        self.ASSETS_PATH = self.OUTPUT_PATH / Path('image') 
        self.window = Tk()
        self.window.geometry("1024x600")
        self.window.configure(bg="#525050")
        bigfont = tkFont.Font(family="Helvetica", size=20)
        self.window.option_add("*TCombobox*Listbox*Font", bigfont)
        style = ttk.Style()
        style.theme_use('default')
        style.configure("TCombobox", font=("Helvetica", 20),width=22)
        
        self.canvas = Canvas(
            self.window,
            bg="#525050",
            height=600,
            width=1024,
            bd=0,
            highlightthickness=0,
            relief="ridge"
        )
        self.canvas.place(x=0, y=0)
        # Database connection
        self.image_background = None
        self.emotion_image = None
        self.current_image_index = 0
        self.vitalpage_state = False
        self.table_name = None
        self.id = 'null'
        self.name = 'null'
        self.room_num = 0
        self.weight = 0
        self.height = 0
        self.date = 'null'
        self.time = 'null'
        self.pr = 0
        self.spo2 = 0
        self.temp = 0
        self.sys = 0
        self.dia = 0
        self.password = '123456'
        self.bindpass = []
        self.window.bind("<Escape>", self.exit_app)
        self.pagestate = False
        self.index = 0  # Start at first frame
        self.delay = 25  # Delay in milliseconds (10 FPS)
        self.battery_percentage = 100
        self.map_battery_percentage = 939 + 0.47*self.battery_percentage
        self.togglehidden_state = False
        self.load_images()
        self.create_widgets()
        self.canvas.tag_bind(self.emotion_image, "<Button-1>", self.on_image_click)
        self.canvas.tag_bind(self.battery, "<Button-1>", self.on_image_click)
        self.start()
        
        
    def on_image_click(self,event):
        self.open_menu()

    def command1(self):
        self.ros.publisher_.publish(String(data="hello"))

    def command2(self):
        self.ros.publisher_.publish(String(data="test2"))
    
    def command3(self):
        self.ros.publisher_.publish(String(data="test3"))
    
    def command4(self):
        self.ros.publisher_.publish(String(data="test4"))

    def command5(self):
        self.ros.publisher_.publish(String(data="test5"))
    
    def command6(self):
        self.ros.publisher_.publish(String(data="test6"))

    def command7(self):
        self.ros.publisher_.publish(String(data="entry"))
    
    def command8(self):
        self.ros.publisher_.publish(String(data="item"))
    
    def command9(self):
        self.ros.publisher_.publish(String(data="way"))
    
    def command10(self):
        self.ros.publisher_.publish(String(data="gone"))

    def relative_to_assets(self, path: str) -> Path:
        return self.ASSETS_PATH / Path(path)

    ###################################################################################################################################################
    def load_images(self):
        self.image_main = PhotoImage(file=self.relative_to_assets("main.png"))
        self.animation_folder = "new"
        # Load images into a list
        self.animation_images = []
        for i in range(44):  # From 0 to 43
            filename = f"Timeline 1_{i:04d}.png"  # Generates names like "Timeline 1_0000.png"
            img_path = os.path.join(self.animation_folder, filename)
            
            if os.path.exists(img_path):  # Check if file exists before loading
                self.animation_images.append(PhotoImage(file=img_path))
        self.image_select_room= PhotoImage(file=self.relative_to_assets("roomselect.png"))
        self.image_menu= PhotoImage(file=self.relative_to_assets("menu.png"))
        self.image_login= PhotoImage(file=self.relative_to_assets("login.png"))
        self.image_hidden_bg= PhotoImage(file=self.relative_to_assets("hidden_bg.png"))
        self.image_battery= PhotoImage(file=self.relative_to_assets("battery.png"))

    # Create all widgets
    ###################################################################################################################################################
        
    def create_widgets(self):
        self.background = self.canvas.create_image(514, 302, image=self.image_main)
        self.emotion_image = self.canvas.create_image(514, 302, image=self.animation_images[self.index])
        self.battery = self.canvas.create_image(965,30,image=self.image_battery)
        self.battery_guage = self.canvas.create_rectangle(939, 19, 986, 41, fill="forestgreen")
        self.id_text = self.canvas.create_text(350, 208, text=self.id, font=("Helvetica", 32), fill="black",state="hidden")
        self.name_text = self.canvas.create_text(350, 275, text=self.name, font=("Helvetica", 32), fill="black",state="hidden")
        self.room_num_text = self.canvas.create_text(350, 348, text=self.room_num, font=("Helvetica", 32), fill="black",state="hidden")
        self.weight_text = self.canvas.create_text(350, 422, text=self.weight, font=("Helvetica", 32), fill="black",state="hidden")
        self.height_text = self.canvas.create_text(350, 495, text=self.height, font=("Helvetica", 32), fill="black",state="hidden")
        self.date_text = self.canvas.create_text(350, 565, text=self.date, font=("Helvetica", 32), fill="black",state="hidden")
        self.time_text = self.canvas.create_text(850, 208, text=self.time, font=("Helvetica", 32), fill="black",state="hidden")
        self.pr_text = self.canvas.create_text(850, 275, text=self.pr, font=("Helvetica", 32), fill="black",state="hidden")
        self.spo2_text = self.canvas.create_text(850, 348, text=self.spo2, font=("Helvetica", 32), fill="black",state="hidden")
        self.temp_text = self.canvas.create_text(850, 422, text=self.dia, font=("Helvetica", 32), fill="black",state="hidden")
        self.sys_text = self.canvas.create_text(850, 495, text=self.sys, font=("Helvetica", 32), fill="black",state="hidden")
        self.dia_text = self.canvas.create_text(850, 565, text=self.dia, font=("Helvetica", 32), fill="black",state="hidden")
        self.pass_bg = self.canvas.create_rectangle(20, 220, 540, 380, fill="white", state="hidden")
        self.pass_dec = self.canvas.create_rectangle(569, 0, 574, 600, fill="white", state="hidden")
        self.hidden_bg = self.canvas.create_image(132, 330, image=self.image_hidden_bg, state="hidden")
        self.a = self.canvas.create_text(80, 300, text='X', font=("Helvetica", 80), fill="black",state="hidden")
        self.b = self.canvas.create_text(160, 300, text='X', font=("Helvetica", 80), fill="black",state="hidden")
        self.c = self.canvas.create_text(240, 300, text='X', font=("Helvetica", 80), fill="black",state="hidden")
        self.d = self.canvas.create_text(320, 300, text='X', font=("Helvetica", 80), fill="black",state="hidden")
        self.e = self.canvas.create_text(400, 300, text='X', font=("Helvetica", 80), fill="black",state="hidden")
        self.f = self.canvas.create_text(480, 300, text='X', font=("Helvetica", 80), fill="black",state="hidden")

    # First start function
    ###################################################################################################################################################
        
    def start(self):
        self.update_emotion()  
        self.window.overrideredirect(True)
        self.window.overrideredirect(False)
        self.window.attributes('-fullscreen', True)
        self.window.resizable(False, False) 
        self.window.mainloop()

    def update_emotion(self):
        self.battery_data_received = server.recived()
        self.battery_percentage = json.loads(self.battery_data_received)
        self.map_battery_percentage = 939 + 0.47*self.battery_percentage
        self.canvas.coords(self.battery_guage, 939, 19, self.map_battery_percentage, 41)
        if self.vitalpage_state == False:
            self.index = (self.index + 1) % len(self.animation_images)  # Loop back to first image
            self.window.after(1,self.canvas.itemconfig(self.emotion_image, image=self.animation_images[self.index]))
            self.window.after(self.delay, self.update_emotion)  # Schedule next frame
        elif self.vitalpage_state == True:
            self.index = 0

    def test1(self):
        self.cap_button = Button( 
            text = 'HELLO',
            borderwidth=2,
            highlightthickness=2,
            command=self.command1,
            fg = "white",
            bg = "purple",
            relief="raised"
        )
        self.cap_button.place(x=25.0, y=150, width=100, height=50)
    def test2(self):
        self.cap_button = Button(
            text = 'TEST2',
            borderwidth=2,
            highlightthickness=2,
            command=self.command2,
            fg = "white",
            bg = "purple",
            relief="raised"
        )
        self.cap_button.place(x=25.0, y=235, width=100, height=50)
    def test3(self):
        self.cap_button = Button(
            text = 'TEST3',
            borderwidth=2,
            highlightthickness=2,
            command=self.command3,
            fg = "white",
            bg = "purple",
            relief="raised"
        )
        self.cap_button.place(x=25.0, y=320, width=100, height=50)
    
    def test4(self):
        self.cap_button = Button( 
            text = 'TEST4',
            borderwidth=2,
            highlightthickness=2,
            command=self.command4,
            fg = "white",
            bg = "purple",
            relief="raised"
        )
        self.cap_button.place(x=25.0, y=405, width=100, height=50)
    def test5(self):
        self.cap_button = Button(
            text = 'TEST5',
            borderwidth=2,
            highlightthickness=2,
            command=self.command5,
            fg = "white",
            bg = "purple",
            relief="raised"
        )
        self.cap_button.place(x=25.0, y=490, width=100, height=50)
    def test6(self):
        self.cap_button = Button(
            text = 'TEST6',
            borderwidth=2,
            highlightthickness=2,
            command=self.command6,
            fg = "white",
            bg = "purple",
            relief="raised"
        )
        self.cap_button.place(x=135.0, y=150, width=100, height=50)
    def test7(self):
        self.cap_button = Button(
            text = 'ENTRY',
            borderwidth=2,
            highlightthickness=2,
            command=self.command7,
            fg = "white",
            bg = "purple",
            relief="raised"
        )
        self.cap_button.place(x=135.0, y=235, width=100, height=50)
    def test8(self):
        self.cap_button = Button(
            text = 'ITEM',
            borderwidth=2,
            highlightthickness=2,
            command=self.command8,
            fg = "white",
            bg = "purple",
            relief="raised"
        )
        self.cap_button.place(x=135.0, y=320, width=100, height=50)
    def test9(self):
        self.cap_button = Button(
            text = 'WAY',
            borderwidth=2,
            highlightthickness=2,
            command=self.command9,
            fg = "white",
            bg = "purple",
            relief="raised"
        )
        self.cap_button.place(x=135.0, y=405, width=100, height=50)
    def test10(self):
        self.cap_button = Button(
            text = 'GONE',
            borderwidth=2,
            highlightthickness=2,
            command=self.command10,
            fg = "white",
            bg = "purple",
            relief="raised"
        )
        self.cap_button.place(x=135.0, y=490, width=100, height=50)
    
    def create_hiddenbutton(self):
        self.hiddenbutton_image = PhotoImage(file=self.relative_to_assets("hidden.png"))
        self.hidden_button = Button(
            image=self.hiddenbutton_image,
            borderwidth=0,
            highlightthickness=0,
            command=self.toggle_hidden,
            relief="raised"
        )
        self.hidden_button.place(x=1.0, y=1, width=160, height=55)

    def create_historybutton(self):
        self.historybutton_image = PhotoImage(file=self.relative_to_assets("vitalbutton.png"))
        self.history_button = Button(  
            image=self.historybutton_image,
            borderwidth=3,
            highlightthickness=0,
            command=self.open_login,
            relief="raised"
        )
        self.history_button.place(x=270.0, y=150, width=500.0, height=150.0)

    def create_callbutton(self):
        self.callbutton_image = PhotoImage(file=self.relative_to_assets("emerbutton.png"))
        self.call_button = Button(  
            image=self.callbutton_image,
            borderwidth=3,
            highlightthickness=0,
            command=None,
            relief="raised"
        )
        self.call_button.place(x=270.0, y=350, width=500.0, height=150.0)

    def create_backbutton(self):
        self.backbutton_image = PhotoImage(file=self.relative_to_assets("back.png"))
        self.back_button = Button(
            image=self.backbutton_image,  
            borderwidth=1,
            highlightthickness=0,
            command=self.close_vital,
            relief="flat"
        )
        self.back_button.place(x=865.0, y=5)

    def toggle_hidden(self):
        if self.togglehidden_state == False:
            self.window.after(1, lambda:self.canvas.itemconfig(self.hidden_bg, state="normal"))
            self.test1()
            self.test2()
            self.test3()
            self.test4()
            self.test5()
            self.test6()
            self.test7()
            self.test8()
            self.test9()
            self.test10()
            self.togglehidden_state = True
        elif self.togglehidden_state == True:
            self.clear_info_frame() 
            self.open_menu()
            self.window.after(1, lambda:self.canvas.itemconfig(self.hidden_bg, state="hidden"))
            self.togglehidden_state = False

    def open_menu(self):
        self.clear_info_frame()  
        self.pagestate = False
        self.vitalpage_state = True
        self.window.after(1, lambda:self.canvas.itemconfig(self.emotion_image, state="hidden"))
        self.create_backbutton()
        self.create_historybutton()
        self.create_callbutton()
        self.create_hiddenbutton()
        self.current_image_index = 0
        self.window.after(1, lambda:self.canvas.itemconfig(self.background, image=self.image_menu))

    def update_pass(self,num):
        if(num == 'reset'):
            self.bindpass.clear()
            self.window.after(1,self.canvas.itemconfig(self.a,text = 'X'))
            self.window.after(1,self.canvas.itemconfig(self.b,text = 'X'))
            self.window.after(1,self.canvas.itemconfig(self.c,text = 'X'))
            self.window.after(1,self.canvas.itemconfig(self.d,text = 'X'))
            self.window.after(1,self.canvas.itemconfig(self.e,text = 'X'))
            self.window.after(1,self.canvas.itemconfig(self.f,text = 'X'))
        else:
            if(num != 'del'):self.bindpass.append(num)

            if len(self.bindpass) == 1:
                if(num == 'del'):
                    self.bindpass.pop()
                    self.window.after(1,lambda:self.canvas.itemconfig(self.a,text = 'X'))  
                else:
                    self.window.after(1,lambda:self.canvas.itemconfig(self.a,text = num))
            if len(self.bindpass) == 2:
                if(num == 'del'):
                    self.bindpass.pop()
                    self.window.after(1,lambda:self.canvas.itemconfig(self.b,text = 'X'))
                else:
                    self.window.after(1,lambda:self.canvas.itemconfig(self.b,text = num))
            if len(self.bindpass) == 3:
                if(num == 'del'):
                    self.bindpass.pop()
                    self.window.after(1,lambda:self.canvas.itemconfig(self.c,text = 'X'))
                else:
                    self.window.after(1,lambda:self.canvas.itemconfig(self.c,text = num))
            if len(self.bindpass) == 4:
                if(num == 'del'):
                    self.bindpass.pop()
                    self.window.after(1,lambda:self.canvas.itemconfig(self.d,text = 'X'))
                else:    
                    self.window.after(1,lambda:self.canvas.itemconfig(self.d,text = num))
            if len(self.bindpass) == 5:
                if(num == 'del'):
                    self.bindpass.pop()
                    self.window.after(1,lambda:self.canvas.itemconfig(self.e,text = 'X'))
                else:    
                    self.window.after(1,lambda:self.canvas.itemconfig(self.e,text = num))
            if len(self.bindpass) == 6:
                self.canvas.itemconfig(self.e,text = num)
                check = ''.join(map(str, self.bindpass))
                if(self.password == check):
                    self.window.after(1,lambda:self.canvas.itemconfig(self.pass_bg,fill='green3'))
                    self.window.after(1,lambda:self.canvas.itemconfig(self.a,text = 'X'))
                    self.window.after(1,lambda:self.canvas.itemconfig(self.b,text = 'X'))
                    self.window.after(1,lambda:self.canvas.itemconfig(self.c,text = 'X'))
                    self.window.after(1,lambda:self.canvas.itemconfig(self.d,text = 'X'))
                    self.window.after(1,lambda:self.canvas.itemconfig(self.e,text = 'X'))
                    self.window.after(1,lambda:self.canvas.itemconfig(self.f,text = 'X'))
                    self.window.after(100,lambda: self.open_vital())
                    self.bindpass.clear()
                else:
                    self.window.after(1,lambda:self.canvas.itemconfig(self.pass_bg,fill='red'))
                    self.window.after(1,lambda:self.canvas.itemconfig(self.a,text = 'X'))
                    self.window.after(1,lambda:self.canvas.itemconfig(self.b,text = 'X'))
                    self.window.after(1,lambda:self.canvas.itemconfig(self.c,text = 'X'))
                    self.window.after(1,lambda:self.canvas.itemconfig(self.d,text = 'X'))
                    self.window.after(1,lambda:self.canvas.itemconfig(self.e,text = 'X'))
                    self.window.after(1,lambda:self.canvas.itemconfig(self.f,text = 'X'))
                    self.bindpass.clear()
                    self.window.after(500, lambda: self.canvas.itemconfig(self.pass_bg, fill='white'))
        #print(self.bindpass)

    def open_login(self):
        self.clear_info_frame()
        self.togglehidden_state = False
        self.window.after(1,lambda:self.canvas.itemconfig(self.hidden_bg,state="hidden"))
        self.window.after(1,lambda:self.canvas.itemconfig(self.a,text = 'X'))
        self.window.after(1,lambda:self.canvas.itemconfig(self.b,text = 'X'))
        self.window.after(1,lambda:self.canvas.itemconfig(self.c,text = 'X'))
        self.window.after(1,lambda:self.canvas.itemconfig(self.d,text = 'X'))
        self.window.after(1,lambda:self.canvas.itemconfig(self.e,text = 'X'))
        self.window.after(1,lambda:self.canvas.itemconfig(self.f,text = 'X'))
        self.bindpass.clear()
        self.window.after(1,lambda:self.canvas.itemconfig(self.pass_bg,state="normal",fill = 'white'))
        self.window.after(1,lambda:self.canvas.itemconfig(self.pass_dec,state="normal"))
        self.window.after(1, lambda:self.canvas.itemconfig(self.background, image=self.image_login))
        self.button_1 = Button(  borderwidth=2,highlightthickness=0,font= font.Font(family="Helvetica", size=44, weight="bold"),
                                 text = "1",bg = 'white',command=lambda:self.update_pass(1),relief="ridge",)
        self.button_2 = Button(  borderwidth=2,highlightthickness=0,font= font.Font(family="Helvetica", size=44, weight="bold"),
                                 text = "2",bg = 'white',command=lambda:self.update_pass(2),relief="ridge")
        self.button_3 = Button(  borderwidth=2,highlightthickness=0,font= font.Font(family="Helvetica", size=44, weight="bold"),
                                 text = "3",bg = 'white',command=lambda:self.update_pass(3),relief="ridge")
        self.button_4 = Button(  borderwidth=2,highlightthickness=0,font= font.Font(family="Helvetica", size=44, weight="bold"),
                                 text = "4",bg = 'white',command=lambda:self.update_pass(4),relief="ridge")
        self.button_5 = Button(  borderwidth=2,highlightthickness=0,font= font.Font(family="Helvetica", size=44, weight="bold"),
                                 text = "5",bg = 'white',command=lambda:self.update_pass(5),relief="ridge")
        self.button_6 = Button(  borderwidth=2,highlightthickness=0,font= font.Font(family="Helvetica", size=44, weight="bold"),
                                 text = "6",bg = 'white',command=lambda:self.update_pass(6),relief="ridge")
        self.button_7 = Button(  borderwidth=2,highlightthickness=0,font= font.Font(family="Helvetica", size=44, weight="bold"),
                                 text = "7",bg = 'white',command=lambda:self.update_pass(7),relief="ridge")
        self.button_8 = Button(  borderwidth=2,highlightthickness=0,font= font.Font(family="Helvetica", size=44, weight="bold"),
                                 text = "8",bg = 'white',command=lambda:self.update_pass(8),relief="ridge")
        self.button_9 = Button(  borderwidth=2,highlightthickness=0,font= font.Font(family="Helvetica", size=44, weight="bold"),
                                 text = "9",bg = 'white',command=lambda:self.update_pass(9),relief="ridge")
        self.button_0 = Button(  borderwidth=2,highlightthickness=0,font= font.Font(family="Helvetica", size=44, weight="bold"),
                                 text = "0",bg = 'white',command=lambda:self.update_pass(0),relief="ridge")
        self.del_button = Button(  borderwidth=2,highlightthickness=0,font= font.Font(family="Helvetica", size=28, weight="bold"),
                                 text = "DEL",bg = 'grey',command=lambda:self.update_pass('del'),relief="ridge") 
        self.reset_button = Button(  borderwidth=2,highlightthickness=0,font= font.Font(family="Helvetica", size=28, weight="bold"),
                                 text = "RESET",bg = 'yellow',command=lambda:self.update_pass('reset'),relief="ridge") 
        
        self.window.after(1,lambda:self.button_1.place(x=574, y=0, width=150.0, height=150.0))
        self.window.after(1,lambda:self.button_2.place(x=724,y=0, width=150.0, height=150.0))
        self.window.after(1,lambda:self.button_3.place(x=874, y=0, width=150.0, height=150.0))
        self.window.after(1,lambda:self.button_4.place(x=574, y=150, width=150.0, height=150.0))
        self.window.after(1,lambda:self.button_5.place(x=724, y=150, width=150.0, height=150.0))
        self.window.after(1,lambda:self.button_6.place(x=874, y=150, width=150.0, height=150.0))
        self.window.after(1,lambda:self.button_7.place(x=574, y=300, width=150.0, height=150.0))
        self.window.after(1,lambda:self.button_8.place(x=724, y=300, width=150.0, height=150.0))
        self.window.after(1,lambda:self.button_9.place(x=874, y=300, width=150.0, height=150.0))
        self.window.after(1,lambda:self.reset_button.place(x=574, y=450, width=150.0, height=150.0))
        self.window.after(1,lambda:self.button_0.place(x=724, y=450, width=150.0, height=150.0))
        self.window.after(1,lambda:self.del_button.place(x=874, y=450, width=150.0, height=150.0))
        self.window.after(1,lambda:self.canvas.itemconfig(self.a,state="normal"))
        self.window.after(1,lambda:self.canvas.itemconfig(self.b,state="normal"))
        self.window.after(1,lambda:self.canvas.itemconfig(self.c,state="normal"))
        self.window.after(1,lambda:self.canvas.itemconfig(self.d,state="normal"))
        self.window.after(1,lambda:self.canvas.itemconfig(self.e,state="normal"))
        self.window.after(1,lambda:self.canvas.itemconfig(self.f,state="normal"))
        self.window.after(1,lambda:self.create_backbutton())
        self.window.after(1,lambda:self.back_button.place(x=5, y=505))

    def open_vital(self):
        """Open the vital page and display table, date, and time comboboxes."""
        self.window.after(1,lambda:self.canvas.itemconfig(self.pass_bg,state="hidden"))
        self.window.after(1,lambda:self.canvas.itemconfig(self.pass_dec,state="hidden"))
        self.clear_info_frame()
        self.create_backbutton()
        self.pagestate = True
        self.vitalpage_state = True
        self.pr = 0
        self.spo2 = 0
        self.sys = 0
        self.dia = 0
        self.window.after(1,lambda:self.canvas.itemconfig(self.a,state="hidden"))
        self.window.after(1,lambda:self.canvas.itemconfig(self.b,state="hidden"))
        self.window.after(1,lambda:self.canvas.itemconfig(self.c,state="hidden"))
        self.window.after(1,lambda:self.canvas.itemconfig(self.d,state="hidden"))
        self.window.after(1,lambda:self.canvas.itemconfig(self.e,state="hidden"))
        self.window.after(1,lambda:self.canvas.itemconfig(self.f,state="hidden"))
        self.window.after(1, lambda:self.canvas.itemconfig(self.background, image=self.image_select_room))
        self.id = 'null'
        self.name = 'null'
        self.room_num = 0
        self.weight = 0
        self.height = 0
        self.date = 'null'
        self.time = 'null'
        self.pr = 0
        self.spo2 = 0
        self.temp = 0
        self.sys = 0
        self.dia = 0
        self.window.after(1, lambda:self.canvas.itemconfig(self.id_text, text=self.id, state="normal"))
        self.window.after(1, lambda:self.canvas.itemconfig(self.name_text, text=self.name, state="normal"))
        self.window.after(1, lambda:self.canvas.itemconfig(self.room_num_text, text=self.room_num, state="normal"))
        self.window.after(1, lambda:self.canvas.itemconfig(self.weight_text, text=self.weight,state="normal"))
        self.window.after(1, lambda:self.canvas.itemconfig(self.height_text, text=self.height, state="normal"))
        self.window.after(1, lambda:self.canvas.itemconfig(self.date_text, text=self.date, state="normal"))
        self.window.after(1, lambda:self.canvas.itemconfig(self.time_text, text=self.time,state="normal"))
        self.window.after(1, lambda:self.canvas.itemconfig( self.pr_text, text=self.pr, state="normal"))
        self.window.after(1, lambda:self.canvas.itemconfig(self.spo2_text, text=self.spo2,state="normal"))
        self.window.after(1, lambda:self.canvas.itemconfig(self.temp_text, text=self.dia, state="normal"))
        self.window.after(1, lambda:self.canvas.itemconfig(self.sys_text, text=self.sys, state="normal"))
        self.window.after(1, lambda:self.canvas.itemconfig(self.dia_text, text=self.dia, state="normal"))
        self.patient_combobox = ttk.Combobox(
            self.window, 
            font=("Helvetica", 20), 
            state="readonly",
            width=30
        )
        self.patient_combobox.place(x=340, y=133)
        self.window.after(1,self.patient_combobox.set("SELECT PATIENT"))
        self.patient_combobox.bind("<<ComboboxSelected>>", self.on_patient_select)
        try:
            conn = sqlite3.connect('patient_database.db')
            cursor = conn.cursor()
            cursor.execute("SELECT * FROM patients;")
            tables = cursor.fetchall()
            
            if tables:
                # สร้างรายการสำหรับ Combobox (แสดง ID และชื่อ)
                patient_list = [f"{row[0]} - {row[1]}" for row in tables]
                self.patient_combobox['values'] = patient_list
                self.patient_combobox['state'] = 'readonly'
                
                # เก็บข้อมูลทั้งหมดสำหรับการแสดงผล
                self.patient_data = {f"{row[0]} - {row[1]}": row for row in tables}
            else:
                self.patient_combobox['values'] = []
                self.patient_combobox.set("ไม่มีข้อมูลผู้ป่วย")
                self.patient_combobox['state'] = 'disabled'
                
            conn.close()
        except sqlite3.Error as e:
            print(f"Error loading patient data: {e}")
            self.patient_combobox.set("เกิดข้อผิดพลาดในการโหลดข้อมูล")

    def on_patient_select(self, event):
        """เมื่อเลือกผู้ป่วยจาก Combobox"""
        selected_patient = self.patient_combobox.get()
        
        if selected_patient in self.patient_data:
            patient = self.patient_data[selected_patient]
            self.id = patient[0]
            self.name = patient[1]
            self.room_num = patient[2]
            self.weight = patient[3]
            self.height = patient[4]
            self.date = patient[5]
            self.time = patient[6]
            self.pr = patient[7]
            self.spo2 = patient[8]
            self.temp = patient[9]
            self.sys = patient[10]
            self.dia = patient[11]
            self.window.after(1, lambda:self.canvas.itemconfig(self.id_text, text=self.id, state="normal"))
            self.window.after(1, lambda:self.canvas.itemconfig(self.name_text, text=self.name, state="normal"))
            self.window.after(1, lambda:self.canvas.itemconfig(self.room_num_text, text=self.room_num, state="normal"))
            self.window.after(1, lambda:self.canvas.itemconfig(self.weight_text, text=self.weight,state="normal"))
            self.window.after(1, lambda:self.canvas.itemconfig(self.height_text, text=self.height, state="normal"))
            self.window.after(1, lambda:self.canvas.itemconfig(self.date_text, text=self.date, state="normal"))
            self.window.after(1, lambda:self.canvas.itemconfig(self.time_text, text=self.time,state="normal"))
            self.window.after(1, lambda:self.canvas.itemconfig( self.pr_text, text=self.pr, state="normal"))
            self.window.after(1, lambda:self.canvas.itemconfig(self.spo2_text, text=self.spo2,state="normal"))
            self.window.after(1, lambda:self.canvas.itemconfig(self.temp_text, text=self.dia, state="normal"))
            self.window.after(1, lambda:self.canvas.itemconfig(self.sys_text, text=self.sys, state="normal"))
            self.window.after(1, lambda:self.canvas.itemconfig(self.dia_text, text=self.dia, state="normal"))

    def clear_info_frame(self):
        """Clear the info frame to remove old buttons."""
        for widget in self.window.place_slaves():
            if isinstance(widget, Button):
                widget.destroy()

    def close_vital(self):
        if self.vitalpage_state == True:
            self.patient_combobox.destroy()
            self.window.after(1, lambda:self.canvas.itemconfig(self.id_text, text=self.id, state="hidden"))
            self.window.after(1, lambda:self.canvas.itemconfig(self.name_text, text=self.name, state="hidden"))
            self.window.after(1, lambda:self.canvas.itemconfig(self.room_num_text, text=self.room_num, state="hidden"))
            self.window.after(1, lambda:self.canvas.itemconfig(self.weight_text, text=self.weight,state="hidden"))
            self.window.after(1, lambda:self.canvas.itemconfig(self.height_text, text=self.height, state="hidden"))
            self.window.after(1, lambda:self.canvas.itemconfig(self.date_text, text=self.date, state="hidden"))
            self.window.after(1, lambda:self.canvas.itemconfig(self.time_text, text=self.time,state="hidden"))
            self.window.after(1, lambda:self.canvas.itemconfig( self.pr_text, text=self.pr, state="hidden"))
            self.window.after(1, lambda:self.canvas.itemconfig(self.spo2_text, text=self.spo2,state="hidden"))
            self.window.after(1, lambda:self.canvas.itemconfig(self.temp_text, text=self.dia, state="hidden"))
            self.window.after(1, lambda:self.canvas.itemconfig(self.sys_text, text=self.sys, state="hidden"))
            self.window.after(1, lambda:self.canvas.itemconfig(self.dia_text, text=self.dia, state="hidden"))
        self.vitalpage_state = False
        self.togglehidden_state = False
        self.clear_info_frame()
        self.window.after(1, lambda:self.canvas.itemconfig(self.hidden_bg, state="hidden"))
        self.window.after(1,lambda:self.canvas.itemconfig(self.pass_bg,state="hidden"))
        self.window.after(1,lambda:self.canvas.itemconfig(self.pass_dec,state="hidden"))
        self.window.after(1,lambda:self.canvas.itemconfig(self.a,state="hidden"))
        self.window.after(1,lambda:self.canvas.itemconfig(self.b,state="hidden"))
        self.window.after(1,lambda:self.canvas.itemconfig(self.c,state="hidden"))
        self.window.after(1,lambda:self.canvas.itemconfig(self.d,state="hidden"))
        self.window.after(1,lambda:self.canvas.itemconfig(self.e,state="hidden"))
        self.window.after(1,lambda:self.canvas.itemconfig(self.f,state="hidden"))
        self.window.after(1, self.back_button.destroy())
        self.window.after(1, self.canvas.itemconfig(self.background, image=self.image_main))
        self.window.after(1, self.canvas.itemconfig(self.emotion_image, state="normal"))
        self.update_emotion()         

    # Exit function
    ###################################################################################################################################################

    def exit_app(self, event):
        self.window.destroy()

    ###################################################################################################################################################
if __name__ == "__main__":
    server = MyServer()
    app = HealthMonitorApp(server)
