from tkinter import * 
import threading 
import queue
import numpy as np
# import pyEIT
import EIT
import USB
import LOG
import matplotlib.pyplot as plt
import matplotlib.backends.backend_tkagg as tkagg
import sys
import time

PERIOD = 1/30

class App(Frame):
    def __init__(self, master, eitHandle):
        super().__init__(master)

        master.protocol("WM_DELETE_WINDOW", self.on_close)
        self.logger = LOG.logger()
        # self.configure(background="lightgrey")
        self.eitHandle = eitHandle

        self.r = DoubleVar()
        self.perm = DoubleVar()
        self.n_el = IntVar()
        self.h0 = DoubleVar()
        self.p = DoubleVar()
        self.lam = DoubleVar()

        self.r.set(self.eitHandle.forward.r)
        self.perm.set(self.eitHandle.forward.perm)
        self.n_el.set(self.eitHandle.forward.n_el)
        self.h0.set(self.eitHandle.forward.h0)
        self.p.set(self.eitHandle.inverse.p)
        self.lam.set(self.eitHandle.inverse.lam)
        
        self.initWidgets()

        self.after(int(0.5*PERIOD*1000),self.updateEIT)

    def initFrames(self):
        self.fwdSimFrame = Frame(self, bd=2,relief='solid', bg='white',width=475, height = 925)
        self.fwdSimFrame.grid(row=0,column=0,padx=5,pady=5,sticky='nsew')
        self.fwdSimFrame.grid_propagate(False)

        self.settingsFrame = Frame(self, bd=2,relief='solid', bg='white',width=475,height=100)
        self.settingsFrame.grid(row=1,column=0,sticky='nsew',padx=5,pady=5)
        self.settingsFrame.grid_propagate(False)

        self.simFrame = Frame(self, bd=2, relief='solid', bg='white',width=900,height=925)
        self.simFrame.grid(row=0,column=1,padx=5,pady=5, sticky='nsew')
        self.simFrame.grid_propagate(False)

        self.controlFrame = Frame(self, bd=2, relief='solid', bg='white',width=900,height=100)
        self.controlFrame.grid(row=1,column=1,sticky='nsew',padx=5,pady=5)
        self.controlFrame.grid_propagate(False)

    def initFwdSim(self):
        self.figMesh,self.axesMesh = plt.subplots(figsize=(4.5,3))
        self.canvasMesh = tkagg.FigureCanvasTkAgg(self.figMesh, master = self.fwdSimFrame)  
        self.canvasMesh.get_tk_widget().grid(row=0,column=0,sticky='nsew')
        self.figMesh,self.axesMesh = self.eitHandle.drawMesh(self.figMesh,self.axesMesh)
        self.canvasMesh.draw()

        self.figFwd,self.axesFwd = plt.subplots(figsize=(4.5,3))
        self.canvasFwd = tkagg.FigureCanvasTkAgg(self.figFwd, master = self.fwdSimFrame) 
        self.canvasFwd.get_tk_widget().grid(row=1,column=0,sticky='nsew')
        self.figFwd,self.axesFwd = self.eitHandle.drawForward(self.figFwd,self.axesFwd)
        self.canvasFwd.draw()

        self.figMeas,self.axesMeas = plt.subplots(figsize=(4.5,3))
        self.canvasMeas = tkagg.FigureCanvasTkAgg(self.figMeas, master = self.fwdSimFrame) 
        self.canvasMeas.get_tk_widget().grid(row=2,column=0,sticky='nsew')
        self.canvasMeas.draw()

    def initSettings(self):
        self.rLabel = Label(self.settingsFrame, text='Radius',bg='white')
        self.rLabel.grid(row=0,column=0,padx=10,pady=5,sticky='w')
        self.rEntry = Entry(self.settingsFrame, textvariable=self.r)
        self.rEntry.grid(row=0,column=1,padx=10,pady=5)
        self.rEntry.bind('<Key-Return>', self.updateForward)

        self.permLabel = Label(self.settingsFrame,text='Permitivity',bg='white')
        self.permLabel.grid(row=1,column=0,padx=10,pady=5,sticky='w')
        self.permEntry = Entry(self.settingsFrame, textvariable=self.perm)
        self.permEntry.grid(row=1,column=1,padx=10,pady=5)
        self.permEntry.bind('<Key-Return>', self.updateForward)

        self.n_elLabel = Label(self.settingsFrame,text='Electrodes',bg='white')
        self.n_elLabel.grid(row=2,column=0,padx=10,pady=5,sticky='w')
        self.n_elEntry = Entry(self.settingsFrame, textvariable=self.n_el)
        self.n_elEntry.grid(row=2,column=1,padx=10,pady=5)
        self.n_elEntry.bind('<Key-Return>', self.updateForward)

        self.h0Label = Label(self.settingsFrame,text='H0',bg='white')
        self.h0Label.grid(row=0,column=2,padx=10,pady=5,sticky='w')
        self.h0Entry = Entry(self.settingsFrame, textvariable=self.h0)
        self.h0Entry.grid(row=0,column=3,padx=10,pady=5)
        self.h0Entry.bind('<Key-Return>', self.updateForward)

        self.pLabel = Label(self.settingsFrame,text='P',bg='white')
        self.pLabel.grid(row=1,column=2,padx=10,pady=5,sticky='w')
        self.pEntry = Entry(self.settingsFrame, textvariable=self.p)
        self.pEntry.grid(row=1,column=3,padx=10,pady=5)
        self.pEntry.bind('<Key-Return>', self.updateInverse)

        self.lamLabel = Label(self.settingsFrame,text='Lambda',bg='white')
        self.lamLabel.grid(row=2,column=2,padx=10,pady=5,sticky='w')
        self.lamEntry = Entry(self.settingsFrame, textvariable=self.lam)
        self.lamEntry.grid(row=2,column=3,padx=10,pady=5)
        self.lamEntry.bind('<Key-Return>', self.updateInverse)

    def initEITSim(self):
        self.simLabel = Label(self.simFrame, text='EIT Reconstruction',bg='white',font=('Times', 16,'underline','bold'), justify='center')
        self.simLabel.grid(row=0,column=0,sticky='nsew')

        self.figEIT, self.axesEIT = plt.subplots(figsize=(8.5,8.5))
        self.canvasEIT = tkagg.FigureCanvasTkAgg(self.figEIT, master=self.simFrame)
        self.canvasEIT.get_tk_widget().grid(row=1, column=0,sticky='nsew')
        self.canvasEIT.draw()

    def initControl(self):
        self.refButton = Button(self.controlFrame, text='Set Reference', font=('Times', 12), 
                                width=15,height=2, command=self.eitHandle.setRef)
        self.refButton.grid(row=0,column=0,sticky='nsew',padx=10,pady=20)

        self.stopButton = Button(self.controlFrame, text='Stop', font=('Times', 12), 
                                width=15,height=2, command=self.eitHandle.stop)
        self.stopButton.grid(row=0, column=1,sticky='w',padx=10,pady=10)

        self.recordButton = Button(self.controlFrame, text='Start Log', font=('Times', 12), 
                                width=15,height=2, command=self.logger.startLog)
        self.recordButton.grid(row=0, column=2,sticky='w',padx=10,pady=10)


    def initWidgets(self):
        self.initFrames()
        self.initFwdSim()
        self.initSettings()
        self.initEITSim()
        self.initControl()

    def updateForward(self,_):
        self.eitHandle.updateForward(self.r.get(),self.n_el.get(),self.perm.get(),self.h0.get())
        self.figMesh,self.axesMesh = self.eitHandle.drawMesh(self.figMesh,self.axesMesh)
        self.canvasMesh.draw()
        self.figFwd,self.axesFwd = self.eitHandle.drawForward(self.figFwd,self.axesFwd)
        self.canvasFwd.draw()

    def updateInverse(self,_):
        self.eitHandle.updateInverse(self.p.get(),self.lam.get(),self.perm.get())

    def updateEIT(self):
        self.figEIT, self.axesEIT = self.eitHandle.displayEIT(self.figEIT, self.axesEIT,self.logger)
        self.canvasEIT.draw()

        self.figMeas,self.axesMeas = self.eitHandle.displayMeas(self.figMeas,self.axesMeas)
        self.canvasMeas.draw()
        self.after(int(PERIOD*1000),self.updateEIT)

    def on_close(self):
        self.quit()  # Stop tkinter loop
        self.destroy()  # Destroy tkinter widgets
        print(self.logger.getData())        
        sys.exit()  # Exit Python program

def main():
    eitHandle = EIT.EIT(r=0.1,n_el=16,perm=0.6,h0=0.01,p=0.5,lam=0.01)
    # reader = USB.dataReader(9600,'COM12',1e5,1e3,PERIOD,eitHandle.callback)
    
    time.sleep(5)

    root = Tk()
    root.geometry("1400x1100")  # Set the window size
    app = App(root, eitHandle)

    app.place(x=0, y=0, width=1400, height=1100)  # Position and size the frame
    root.mainloop()

main()