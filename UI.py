from tkinter import * 
import threading 
import queue
import numpy as np
import pyEIT
import matplotlib.pyplot as plt
import matplotlib.backends.backend_tkagg as tkagg
import sys

class App(Frame):
    def __init__(self, master):
        super().__init__(master)

        master.protocol("WM_DELETE_WINDOW", self.on_close)
        # self.configure(background="lightgrey")

        self.r = DoubleVar()
        self.perm = DoubleVar()
        self.n_el = IntVar()
        self.r.set(0.1)
        self.perm.set(0.6)
        self.n_el.set(16)
        
        self.initWidgets()

    def initWidgets(self):
        self.fwdSimFrame = Frame(self, bd=2,relief='solid', bg='white')
        self.fwdSimFrame.grid(row=0,column=0,padx=5,pady=5)

        self.settingsFrame = Frame(self, bd=2,relief='solid', bg='white')
        self.settingsFrame.grid(row=1,column=0,sticky='w',padx=5,pady=5)

        self.figMesh,self.axesMesh = plt.subplots(figsize=(4.5,3))
        self.canvasMesh = tkagg.FigureCanvasTkAgg(self.figMesh, master = self.fwdSimFrame)  
        self.canvasMesh.get_tk_widget().grid(row=0,column=0)
        self.drawMesh('')

        self.figFwd,self.axesFwd = plt.subplots(figsize=(4.5,3))
        self.canvasFwd = tkagg.FigureCanvasTkAgg(self.figFwd, master = self.fwdSimFrame) 
        self.canvasFwd.get_tk_widget().grid(row=1,column=0)
        self.drawFWD('')

        self.rLabel = Label(self.settingsFrame, text='Radius',bg='white')
        self.rLabel.grid(row=0,column=0,padx=10,pady=2,sticky='w')
        self.rEntry = Entry(self.settingsFrame, textvariable=self.r)
        self.rEntry.grid(row=0,column=1,padx=10,pady=2)
        self.rEntry.bind('<Key-Return>', self.drawMesh)
        self.rEntry.bind('<Key-Return>', self.drawFWD, '+')

        self.permLabel = Label(self.settingsFrame,text='Permitivity',bg='white')
        self.permLabel.grid(row=1,column=0,padx=10,pady=2,sticky='w')
        self.permEntry = Entry(self.settingsFrame, textvariable=self.perm)
        self.permEntry.grid(row=1,column=1,padx=10,pady=2)
        self.permEntry.bind('<Key-Return>', self.drawMesh)
        self.permEntry.bind('<Key-Return>', self.drawFWD, '+')

        self.n_elLabel = Label(self.settingsFrame,text='Electrodes',bg='white')
        self.n_elLabel.grid(row=2,column=0,padx=10,pady=2,sticky='w')
        self.n_elLabel = Entry(self.settingsFrame, textvariable=self.n_el)
        self.n_elLabel.grid(row=2,column=1,padx=10,pady=2)
        self.n_elLabel.bind('<Key-Return>', self.drawMesh)
        self.n_elLabel.bind('<Key-Return>', self.drawFWD, '+')
  
    def drawMesh(self,_):
        self.axesMesh.cla()
        self.mesh = pyEIT.init_fem(self.r.get(),self.perm.get(),self.n_el.get())

        pts,tri,el_pos=pyEIT.display_fem(self.mesh)
        self.axesMesh.triplot(pts[:, 0], pts[:, 1], tri, linewidth=1)
        self.axesMesh.plot(pts[el_pos, 0], pts[el_pos, 1], "ro")
        self.axesMesh.set_title('Mesh in Use')
        self.canvasMesh.draw()

    def drawFWD(self,_):
        self.axesFwd.cla()
        self.protocol = pyEIT.init_forward(self.mesh)

        v0 = pyEIT.display_forward(self.mesh,self.protocol)
        self.axesFwd.plot(np.arange(0,len(v0), 1), v0)
        self.axesFwd.set_title('Forward Model Solution')
        self.axesFwd.grid(True)
        self.axesFwd.set_xlim(-1)
        # self.axesFwd.set_ylim(0)
        self.canvasFwd.draw()

    def on_close(self):
        self.quit()  # Stop tkinter loop
        self.destroy()  # Destroy tkinter widgets
        sys.exit()  # Exit Python program


root = Tk()
root.geometry("1500x1000")  # Set the window size
app = App(root)
app.place(x=0, y=0, width=1500, height=1000)  # Position and size the frame
root.mainloop()