from pathlib import Path
import sqlite3
from tkinter import Tk, Canvas, Button, PhotoImage, messagebox, ttk, font
import tkinter.font as tkFont
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RosGui(Node):
    def __init__(self):
        super().__init__('ros_gui')
        self.publisher_ = self.create_publisher(String, 'sound_command', 10)
        
class HealthMonitorApp:
    # Init
    ###################################################################################################################################################
    
    def __init__(self):
        rclpy.init(args=None)
        self.ros = RosGui()
        self.audio_playing = False
        self.OUTPUT_PATH = Path(__file__).parent
        self.ASSETS_PATH = self.OUTPUT_PATH / Path('image') 
        self.window = Tk()
        self.window.geometry("1024x600")
        self.window.configure(bg="#525050")


        # self.cap = cv2.VideoCapture(0)
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)


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
        self.conn = None
        self.cursor = None
        #self.db_name = 'your_database.db'  # Change to your SQLite database name 
        self.db_name = "/home/pannoi_db.db"
        self.connect_db()
        self.image_background = None
        self.emotion_image = None
        self.current_image_index = 0
        self.vitalpage_state = False
        self.table_name = None
        self.pr = 0
        self.spo2 = 0
        self.sys = 0
        self.dia = 0
        self.password = '123456'
        self.bindpass = []
        self.window.bind("<a>", self.command1)
        self.window.bind("<s>", self.command2)
        self.window.bind("<d>", self.command3)
        self.window.bind("<Escape>", self.exit_app)
        self.pagestate = False
        self.index = 0 
        self.delay = 20 
        self.load_images()
        self.create_widgets()
        self.canvas.tag_bind(self.emotion_image, "<Button-1>", self.on_image_click)
        self.start()
        
    def on_image_click(self,event):
        self.open_menu()    

    def connect_db(self):
        """Connect to the SQLite database."""
        try:
            self.conn = sqlite3.connect(self.db_name)
            self.cursor = self.conn.cursor()
        except sqlite3.Error as e:
            print(f"Error connecting to database: {e}")
            raise

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
        self.animation_images = []
        for i in range(44):  
            filename = f"Timeline 1_{i:04d}.png"  
            img_path = os.path.join(self.animation_folder, filename)
            
            if os.path.exists(img_path):  
                self.animation_images.append(PhotoImage(file=img_path))
        self.image_select_room= PhotoImage(file=self.relative_to_assets("roomselect.png"))
        self.image_menu= PhotoImage(file=self.relative_to_assets("menu.png"))
        self.image_login= PhotoImage(file=self.relative_to_assets("login.png"))

    # Create all widgets
    ###################################################################################################################################################
        
    def create_widgets(self):
        self.background = self.canvas.create_image(514, 302, image=self.image_main)
        self.emotion_image = self.canvas.create_image(514, 302, image=self.animation_images[self.index])
        self.pr_text = self.canvas.create_text(845, 310, text=self.pr, font=("Helvetica", 32), fill="black",state="hidden")
        self.spo2_text = self.canvas.create_text(845, 391, text=self.dia, font=("Helvetica", 32), fill="black",state="hidden")
        self.sys_text = self.canvas.create_text(845, 462, text=self.sys, font=("Helvetica", 32), fill="black",state="hidden")
        self.dia_text = self.canvas.create_text(845, 538, text=self.dia, font=("Helvetica", 32), fill="black",state="hidden")
        self.pass_bg = self.canvas.create_rectangle(20, 220, 540, 380, fill="white", state="hidden")
        self.pass_dec = self.canvas.create_rectangle(569, 0, 574, 600, fill="white", state="hidden")
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
        self.cap_button.place(x=800.0, y=150, width=100, height=50)
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
        self.cap_button.place(x=800.0, y=230, width=100, height=50)
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
        self.cap_button.place(x=800.0, y=310, width=100, height=50)
    
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
        self.cap_button.place(x=800.0, y=390, width=100, height=50)
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
        self.cap_button.place(x=800.0, y=470, width=100, height=50)
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
        self.cap_button.place(x=910.0, y=150, width=100, height=50)
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
        self.cap_button.place(x=910.0, y=230, width=100, height=50)
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
        self.cap_button.place(x=910.0, y=310, width=100, height=50)
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
        self.cap_button.place(x=910.0, y=390, width=100, height=50)
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
        self.cap_button.place(x=910.0, y=470, width=100, height=50)
    
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

    def open_menu(self):
        self.clear_info_frame()  
        self.pagestate = False
        self.vitalpage_state = True
        self.window.after(1, lambda:self.canvas.itemconfig(self.emotion_image, state="hidden"))
        self.create_backbutton()
        self.create_historybutton()
        self.create_callbutton()
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
        self.window.after(1, lambda:self.canvas.itemconfig(self.pr_text,text = self.pr, state="normal"))
        self.window.after(1, lambda:self.canvas.itemconfig(self.spo2_text,text = self.spo2, state="normal"))
        self.window.after(1, lambda:self.canvas.itemconfig(self.sys_text,text = self.sys,  state="normal"))
        self.window.after(1, lambda:self.canvas.itemconfig(self.dia_text,text = self.dia,  state="normal"))
        # Show table name combobox
        self.show_table_buttons()
        self.show_date_buttons(initial=True)
        self.show_time_buttons(initial=True)

    def clear_info_frame(self):
        """Clear the info frame to remove old buttons."""
        for widget in self.window.place_slaves():
            if isinstance(widget, Button):
                widget.destroy()

    def close_vital(self):
        self.vitalpage_state = False
        if(self.pagestate == True):
            self.time_combobox.destroy()
            self.table_combobox.destroy()
            self.date_combobox.destroy()
        self.clear_info_frame()
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
        self.window.after(1, self.canvas.itemconfig(self.pr_text, state="hidden"))
        self.window.after(1, self.canvas.itemconfig(self.spo2_text, state="hidden"))
        self.window.after(1, self.canvas.itemconfig(self.sys_text, state="hidden"))
        self.window.after(1, self.canvas.itemconfig(self.dia_text, state="hidden"))

        self.update_emotion()         

    def show_table_buttons(self):
        """Fetch and display the table names from the database using a read-only Combobox.""" 
        try:
            # Fetch all table names from the database
            self.cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
            tables = self.cursor.fetchall()

            if tables:
                table_list = [table[0] for table in tables]  # Extract table names into a list

                # Create a read-only Combobox for selecting tables
                self.table_combobox = ttk.Combobox(self.window, values=table_list, font=("Helvetica", 26), state="readonly",width= 14)
                self.table_combobox.place(x=166, y=158)  # Adjust placement as needed
                self.table_combobox.set("Select a room")  # Set the default prompt text

                # Create an action for the combobox selection
                def on_table_select(event):
                    selected_table = self.table_combobox.get()
                    self.table_name = selected_table
                    self.pr = 0
                    self.spo2 = 0
                    self.sys = 0
                    self.dia = 0
                    self.canvas.itemconfig(self.pr_text, text=self.pr)
                    self.canvas.itemconfig(self.spo2_text, text=self.spo2)
                    self.canvas.itemconfig(self.sys_text, text=self.sys)
                    self.canvas.itemconfig(self.dia_text, text=self.dia)
                    # Enable and update date and time comboboxes with relevant options
                    self.show_date_buttons(initial=False)
                    self.show_time_buttons(initial=False)
                    self.time_combobox['state'] = 'disabled'

                # Bind the combobox selection event to the function
                self.table_combobox.bind("<<ComboboxSelected>>", on_table_select)
        except sqlite3.Error as e:
            print(f"Error fetching tables: {e}")
            #messagebox.showerror("Database Error", f"Error fetching tables: {e}")


    def show_date_buttons(self, initial=False):
        """Fetch and display the latest dates from the selected table using a read-only Combobox."""
        if initial:
            # Create an empty, disabled combobox for dates initially
            self.date_combobox = ttk.Combobox(self.window, font=("Helvetica", 26), state="disabled",width= 14)
            self.window.after(1,self.date_combobox.place(x=166, y=293))  # Adjust placement as needed
            self.window.after(1,self.date_combobox.set("Select a date"))
            return

        self.create_backbutton()

        try:
            # Fetch the last 20 dates, ordered by date in descending order
            self.cursor.execute(f"""
                SELECT DISTINCT date 
                FROM {self.table_name} 
                ORDER BY date DESC 
                LIMIT 20;
            """)
            dates = self.cursor.fetchall()

            if dates:
                date_list = [date[0] for date in dates]  # Extract date values into a list

                # Update and enable the date combobox with the fetched dates
                self.date_combobox['values'] = date_list
                self.date_combobox['state'] = 'readonly'  # Enable it now
                self.date_combobox.set("Select a date")

                # Create an action for the combobox selection
                def on_date_select(event):
                    self.selected_date = self.date_combobox.get()  # Store the selected date
                    self.show_time_buttons(initial=False)  # Show time options based on the selected date

                # Bind the combobox selection event to the function
                self.date_combobox.bind("<<ComboboxSelected>>", on_date_select)
            else:
                self.date_combobox['state'] = 'disabled'  # Disable if no dates are available
                self.date_combobox.set("No dates available")
        except sqlite3.Error as e:
            print(f"Error fetching dates: {e}")
            #messagebox.showerror("Database Error", f"Error fetching dates: {e}")

    def show_time_buttons(self, initial=False):
        """Fetch and display times from the selected date using a read-only Combobox."""
        if initial:
            # Create an empty, disabled combobox for times initially
            self.time_combobox = ttk.Combobox(self.window, font=("Helvetica", 26), state="disabled",width= 15)
            self.window.after(1,self.time_combobox.place(x=690, y=158))  # Adjust placement as needed
            self.window.after(1,self.time_combobox.set("Select a time"))
            return

        # Clear previous buttons if any
        self.create_backbutton()

        try:
            if hasattr(self, 'table_name') and hasattr(self, 'selected_date'):
                # Fetch times based on the selected table and date
                self.cursor.execute(f"""
                    SELECT DISTINCT time 
                    FROM {self.table_name} 
                    WHERE date = ? 
                    ORDER BY time DESC 
                    LIMIT 20;
                """, (self.selected_date,))  # Use the stored selected_date
                times = self.cursor.fetchall()

                if times:
                    time_list = [time[0] for time in times]  # Extract time values into a list

                    # Update and enable the time combobox with the fetched times
                    self.time_combobox['values'] = time_list
                    self.time_combobox['state'] = 'readonly'  # Enable it now
                    self.time_combobox.set("Select a time")

                    # Create an action for the combobox selection
                    def on_time_select(event):
                        selected_time = self.time_combobox.get()
                        self.show_values(selected_time, self.selected_date, self.table_name)

                    # Bind the combobox selection event to the function
                    self.time_combobox.bind("<<ComboboxSelected>>", on_time_select)
                else:
                    self.time_combobox['state'] = 'disabled'  # Disable if no times are available
                    self.time_combobox.set("No times available")
        except sqlite3.Error as e:
            print(f"Error fetching times: {e}")
            #messagebox.showerror("Database Error", f"Error fetching times: {e}")


    def show_values(self, time, date, table_name):
        """Fetch and print PR, SpO2, Temp, Sys, Dia values for the selected time and date."""
        try:
            self.cursor.execute(f"""
                SELECT PR, SpO2, Temp, Sys, Dia 
                FROM {table_name} 
                WHERE date = ? AND time = ?
            """, (date, time))  # Use the selected table name
            results = self.cursor.fetchall()
            if results:
                result_text = f"Values for {time} on {date}:\n"
                for row in results:
                    result_text += f"PR: {row[0]}, SpO2: {row[1]}, Temp: {row[2]}, Sys: {row[3]}, Dia: {row[4]}\n"
                self.pr, self.spo2, self.temp, self.sys, self.dia = row
                self.canvas.itemconfig(self.pr_text, text=self.pr)
                self.canvas.itemconfig(self.spo2_text, text=self.spo2)
                self.canvas.itemconfig(self.sys_text, text=self.sys)
                self.canvas.itemconfig(self.dia_text, text=self.dia)
                #print(result_text)
        except sqlite3.Error as e:
            print(f"Error fetching values for time {time} on date {date}: {e}")
            #messagebox.showerror("Database Error", f"Error fetching values: {e}")


    # Exit function
    ###################################################################################################################################################

    def exit_app(self, event):
        self.conn.close()  # Close database connection
        self.window.destroy()

    ###################################################################################################################################################
if __name__ == "__main__":
    app = HealthMonitorApp()
