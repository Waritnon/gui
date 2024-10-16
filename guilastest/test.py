from pathlib import Path
from tkinter import Tk, Canvas, Button, PhotoImage
import threading
import pygame

class HealthMonitorApp:
#init
###################################################################################################################################################
    
    def __init__(self):
        self.audio_playing = False
        self.OUTPUT_PATH = Path(__file__).parent
        self.ASSETS_PATH = self.OUTPUT_PATH / Path('image') 
        self.window = Tk()
        self.window.geometry("1024x600")
        self.window.configure(bg="#525050")

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
        self.window.bind("<a>", self.command1)
        self.window.bind("<s>", self.command2)
        self.window.bind("<d>", self.command3)
        self.window.bind("<Escape>", self.exit_app)
        self.load_images()
        self.create_widgets()
        self.start()

    def command1(self, event):
        self.window.after(1,self.canvas.itemconfig(self.present1,state="hidden"))
        self.window.after(1,self.canvas.itemconfig(self.present2,state="hidden"))
        self.presentstate = False
        self.audio = "entry.wav"
        threading.Thread(target=self.play).start()
    
    def command2(self, event):
        self.window.after(1,self.canvas.itemconfig(self.present1,state="hidden"))
        self.window.after(1,self.canvas.itemconfig(self.present2,state="hidden"))
        self.presentstate = False
        self.audio = "gone.wav"
        threading.Thread(target=self.play).start()
    
    def command3(self, event):
        self.window.after(1,self.canvas.itemconfig(self.present1,state="normal"))
        self.window.after(1,self.canvas.itemconfig(self.present2,state="normal"))
        self.presentstate = True
        self.audio = "item.wav"
        threading.Thread(target=self.play).start()


    def relative_to_assets(self, path: str) -> Path:
        return self.ASSETS_PATH / Path(path)
    
#open threaded to play sound
###################################################################################################################################################
    def play(self):
        self.nextsound = True
        self.play_audio(self.audio)
       
        
#play audio file
###################################################################################################################################################      
    def play_audio(self, file_path):
        pygame.init()
        pygame.mixer.init()
        self.nextsound = False
        try:
            pygame.mixer.music.stop()        
            pygame.mixer.music.load(file_path)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(100)
                if self.nextsound == True:
                    pygame.mixer.music.stop() 
                    pygame.mixer.quit()
                    return 
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
        self.image_vitalpage = PhotoImage(file=self.relative_to_assets("vital.png"))


#create all widgets
###################################################################################################################################################
        
    def create_widgets(self):
        self.background = self.canvas.create_image(514, 302, image=self.image_main)
        self.emotion_image = self.canvas.create_image(514, 302, image=self.image_face1)
        self.present1 = self.canvas.create_image(914, 480, image=self.image_present)
        self.present2 = self.canvas.create_image(110, 480, image=self.image_present)
        self.canvas.itemconfig(self.present1,state="hidden")
        self.canvas.itemconfig(self.present2,state="hidden")
#first start function
###################################################################################################################################################
        
    def start(self):
        self.create_capbutton()
        self.update_emotion()  
        self.window.overrideredirect(True)
        self.window.overrideredirect(False)
        self.window.attributes('-fullscreen', True)
        self.window.resizable(False, False) 
        self.window.mainloop()

#update emotion loop
###################################################################################################################################################

    def update_emotion(self):
        if self.vitalpage_state == False:
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
        self.capbutton_image = PhotoImage(file = self.relative_to_assets("cap.png"))
        self.cap_button = Button(
            image=self.capbutton_image,  
            borderwidth=0,
            highlightthickness=0,
            command=self.open_vital,
            relief="flat"
        )
        self.cap_button.place(x=640.0, y=15, width=380.0, height=220.0)

    def create_backbutton(self):
        self.backbutton_image = PhotoImage(file = self.relative_to_assets("back.png"))
        self.back_button = Button(
            image=self.backbutton_image,  
            borderwidth=0,
            highlightthickness=0,
            command=self.close_vital,
            relief="flat"
        )
        self.back_button.place(x=750.0, y=510)
    
    def open_vital(self):
        self.vitalpage_state = True
        self.window.after(1,self.cap_button.destroy())
        self.window.after(1,self.canvas.itemconfig(self.background, image=self.image_vitalpage))
        self.window.after(1,self.canvas.itemconfig(self.emotion_image,state="hidden"))
        self.create_backbutton()
    
    def close_vital(self):
        self.vitalpage_state = False
        self.window.after(1,self.back_button.destroy())
        self.window.after(1,self.canvas.itemconfig(self.background, image=self.image_main))
        self.window.after(1,self.canvas.itemconfig(self.emotion_image,state="normal"))
        self.window.after(1,self.create_capbutton)
        self.update_emotion()         

#exit function
###################################################################################################################################################

    def exit_app(self, event):
        self.window.destroy()

###################################################################################################################################################
if __name__ == "__main__":
    app = HealthMonitorApp()
