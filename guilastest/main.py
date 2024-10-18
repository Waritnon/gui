from pathlib import Path
import sqlite3
import threading
import pygame
from tkinter import Tk, Canvas, Button, PhotoImage, messagebox, ttk
import tkinter.font as tkFont
import cv2
from PIL import ImageTk, Image , ImageDraw
import webview
from tkinter import Tk


class HealthMonitorApp:
    # Init
    ###################################################################################################################################################
    
    def __init__(self):
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
        # Database connection
        self.conn = None
        self.cursor = None
        #self.db_name = 'your_database.db'  # Change to your SQLite database name 
        self.db_name = "/home/pannoi_db.db"
        self.connect_db()

        self.image_background = None
        self.image_face1 = None
        self.image_face2 = None
        self.image_background = None
        self.emotion_image = None
        self.presentstate = False
        self.audio = None
        self.current_image_index = 0
        self.nextsound = False
        self.vitalpage_state = False
        self.table_name = None
        self.pr = 0
        self.spo2 = 0
        self.temp = 0
        self.sys = 0
        self.dia = 0
        self.window.bind("<a>", self.command1)
        self.window.bind("<s>", self.command2)
        self.window.bind("<d>", self.command3)
        self.window.bind("<Escape>", self.exit_app)

        self.load_images()
        self.create_widgets()
        self.start()
        
        

    def connect_db(self):
        """Connect to the SQLite database."""
        try:
            self.conn = sqlite3.connect(self.db_name)
            self.cursor = self.conn.cursor()
        except sqlite3.Error as e:
            print(f"Error connecting to database: {e}")
            raise

    def command1(self, event):
        self.window.after(1, self.canvas.itemconfig(self.present1, state="hidden"))
        self.window.after(1, self.canvas.itemconfig(self.present2, state="hidden"))
        self.audio = "entry.wav"
        threading.Thread(target=self.play).start()

    def command2(self, event):
        self.window.after(1, self.canvas.itemconfig(self.present1, state="hidden"))
        self.window.after(1, self.canvas.itemconfig(self.present2, state="hidden"))
        self.audio = "gone.wav"
        threading.Thread(target=self.play).start()
    
    def command3(self, event):
        self.window.after(1, self.canvas.itemconfig(self.present1, state="normal"))
        self.window.after(1, self.canvas.itemconfig(self.present2, state="normal"))
        self.audio = "item.wav"
        threading.Thread(target=self.play).start()

    def relative_to_assets(self, path: str) -> Path:
        return self.ASSETS_PATH / Path(path)

    # Open threaded to play sound
    ###################################################################################################################################################
    def play(self):
        self.play_audio(self.audio)
        
    # Play audio file
    ###################################################################################################################################################      
    def play_audio(self, file_path):
        pygame.init()
        pygame.mixer.init()
        try:
            pygame.mixer.music.stop()        
            pygame.mixer.music.load(file_path)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(100)
        except pygame.error as e:
            print(f"Error playing audio: {e}")
        finally:
            pygame.mixer.quit()

    ###################################################################################################################################################
    def load_images(self):
        self.image_main = PhotoImage(file=self.relative_to_assets("main.png"))
        self.image_face1 = PhotoImage(file=self.relative_to_assets("original.png"))
        self.image_face2 = PhotoImage(file=self.relative_to_assets("blink.png"))
        self.image_present = PhotoImage(file=self.relative_to_assets("present.png"))
        self.image_select_room= PhotoImage(file=self.relative_to_assets("roomselect.png"))
        self.image_select_date= PhotoImage(file=self.relative_to_assets("dateselect.png"))

    # Create all widgets
    ###################################################################################################################################################
        
    def create_widgets(self):
        self.background = self.canvas.create_image(514, 302, image=self.image_main)
        self.emotion_image = self.canvas.create_image(514, 302, image=self.image_face1)
        self.present1 = self.canvas.create_image(914, 480, image=self.image_present)
        self.present2 = self.canvas.create_image(110, 480, image=self.image_present)
        self.pr_text = self.canvas.create_text(845, 310, text=self.pr, font=("Helvetica", 28), fill="black",state="hidden")
        self.spo2_text = self.canvas.create_text(845, 367, text=self.dia, font=("Helvetica", 28), fill="black",state="hidden")
        self.temp_text = self.canvas.create_text(845, 424, text=self.temp, font=("Helvetica", 28), fill="black",state="hidden")
        self.sys_text = self.canvas.create_text(845, 481, text=self.sys, font=("Helvetica", 28), fill="black",state="hidden")
        self.dia_text = self.canvas.create_text(845, 538, text=self.dia, font=("Helvetica", 28), fill="black",state="hidden")

        self.canvas.itemconfig(self.present1, state="hidden")
        self.canvas.itemconfig(self.present2, state="hidden")

    # First start function
    ###################################################################################################################################################
        
    def start(self):
        self.create_capbutton()
        self.update_emotion()  
        self.window.overrideredirect(True)
        self.window.overrideredirect(False)
        self.window.attributes('-fullscreen', True)
        self.window.resizable(False, False) 
        self.window.mainloop()

    # Update emotion loop
    ###################################################################################################################################################
    def load_webpage(self):
        webview.create_window('Webview', 'http://192.168.162.201:3000/ros_control', fullscreen=True)
        webview.start()

    def update_emotion(self):
        if self.vitalpage_state == False:
            #ret, self.videocap = self.cap.read()
            #resized_rgb = cv2.cvtColor(self.videocap, cv2.COLOR_BGR2RGB)
           # self.videocap_pil = Image.fromarray(resized_rgb)
            #self.photo = ImageTk.PhotoImage(self.videocap_pil)
           #self.canvas.itemconfig(self.background,image=self.photo)
            #self.window.after(1,self.canvas.itemconfig(self.emotion_image,state="hidden"))
            if self.current_image_index <= 40:
                self.window.after(1,self.canvas.itemconfig(self.emotion_image, image=self.image_face1))
                self.current_image_index += 1
            else:
                self.window.after(1,self.canvas.itemconfig(self.emotion_image, image=self.image_face2))
                self.current_image_index = 0
            if(self.presentstate == True):
                if(self.current_image_index%10 == 0):
                    self.window.after(1,self.canvas.itemconfig(self.present1,state="hidden"))
                    self.window.after(1,self.canvas.itemconfig(self.present2,state="hidden"))
                else: 
                    self.window.after(1,self.canvas.itemconfig(self.present1,state="normal"))
                    self.window.after(1,self.canvas.itemconfig(self.present2,state="normal"))
            self.window.after(50, self.update_emotion)


    def create_capbutton(self):
        self.capbutton_image = PhotoImage(file=self.relative_to_assets("cap.png"))
        self.cap_button = Button(
            image=self.capbutton_image,  
            borderwidth=0,
            highlightthickness=0,
            command=self.open_vital,
            relief="flat"
        )
        self.cap_button.place(x=640.0, y=15, width=380.0, height=220.0)

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

    def open_vital(self):
        """Open the vital page and display table, date, and time comboboxes."""
        self.vitalpage_state = True
        self.window.after(1, self.canvas.itemconfig(self.emotion_image, state="hidden"))
        self.create_backbutton()
        self.pr = 0
        self.spo2 = 0
        self.temp = 0
        self.sys = 0
        self.dia = 0
        self.window.after(1, self.canvas.itemconfig(self.pr_text,text = self.pr, state="normal"))
        self.window.after(1, self.canvas.itemconfig(self.spo2_text,text = self.spo2, state="normal"))
        self.window.after(1, self.canvas.itemconfig(self.temp_text,text = self.temp,  state="normal"))
        self.window.after(1, self.canvas.itemconfig(self.sys_text,text = self.sys,  state="normal"))
        self.window.after(1, self.canvas.itemconfig(self.dia_text,text = self.dia,  state="normal"))
        # Show table name combobox
        self.clear_info_frame()  
        self.show_table_buttons()
        # Initially show date and time comboboxes but keep them disabled
        self.show_date_buttons(initial=True)
        self.show_time_buttons(initial=True)

    def show_table_buttons(self):
        """Fetch and display the table names from the database using a read-only Combobox."""
        self.window.after(1, self.canvas.itemconfig(self.background, image=self.image_select_room))
        self.clear_info_frame()  
        self.create_backbutton()
        
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
                    self.temp = 0
                    self.sys = 0
                    self.dia = 0
                    self.canvas.itemconfig(self.pr_text, text=self.pr)
                    self.canvas.itemconfig(self.spo2_text, text=self.spo2)
                    self.canvas.itemconfig(self.temp_text, text=self.temp)
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
            self.date_combobox.place(x=166, y=293)  # Adjust placement as needed
            self.date_combobox.set("Select a date")
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
            self.time_combobox.place(x=690, y=158)  # Adjust placement as needed
            self.time_combobox.set("Select a time")
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
                self.canvas.itemconfig(self.temp_text, text=self.temp)
                self.canvas.itemconfig(self.sys_text, text=self.sys)
                self.canvas.itemconfig(self.dia_text, text=self.dia)
                print(result_text)
        except sqlite3.Error as e:
            print(f"Error fetching values for time {time} on date {date}: {e}")
            #messagebox.showerror("Database Error", f"Error fetching values: {e}")


    def clear_info_frame(self):
        """Clear the info frame to remove old buttons."""
        for widget in self.window.place_slaves():
            if isinstance(widget, Button):
                widget.destroy()

    def close_vital(self):
        self.vitalpage_state = False
        self.time_combobox.destroy()
        self.table_combobox.destroy()
        self.date_combobox.destroy()
        self.clear_info_frame()
        self.window.after(1, self.back_button.destroy())
        self.window.after(1, self.canvas.itemconfig(self.background, image=self.image_main))
        self.window.after(1, self.canvas.itemconfig(self.emotion_image, state="normal"))
        self.window.after(1, self.create_capbutton)
        self.window.after(1, self.canvas.itemconfig(self.pr_text, state="hidden"))
        self.window.after(1, self.canvas.itemconfig(self.spo2_text, state="hidden"))
        self.window.after(1, self.canvas.itemconfig(self.temp_text, state="hidden"))
        self.window.after(1, self.canvas.itemconfig(self.sys_text, state="hidden"))
        self.window.after(1, self.canvas.itemconfig(self.dia_text, state="hidden"))
        self.update_emotion()         

    # Exit function
    ###################################################################################################################################################

    def exit_app(self, event):
        self.conn.close()  # Close database connection
        self.window.destroy()

    ###################################################################################################################################################
if __name__ == "__main__":
    app = HealthMonitorApp()
