import numpy as np
import serial as serial
import threading 
import queue
import time
import sys
import re
import scipy.signal as sig
import struct

class USB():
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.data = np.zeros(256)   
        
        
    def readUSB(self):
        raw_data = self.ser.read(512)
        line = list(struct.unpack(f'<{256}H', raw_data))
        self.ser.flush()
        mags = self.parseLine(line)
        return mags  

    def readBuffer(self, size=128):
        raw_data = self.ser.read(size*2)
        line = list(struct.unpack(f'<{size}H', raw_data))
        self.getAmp(line) 

    def getAmp(self,line):
        idx = line[0]
        print(idx)
        data = line[1:]
        amp = np.max(data) - np.min(data)
        self.data[idx] = amp
        # print(self.data)

    

    def parseLine(self, line):
        if line is None:
            return
        data = []
        for i in range(len(line)):
            C1 = i // 16 + 1
            V1 = i % 16 + 1

            C2 = C1 + 1 if C1 < 16 else 1
            V2 = V1 + 1 if V1 < 16 else 1

            if C1 != V1 and C1 != V2 and C2 != V1:
                data.append(line[i])
        return np.array(data)
                
    def connect(self):
        self.ser = serial.Serial(self.port, self.baud)


# class DataProcessor:

        # data = re.match(r'A(\d+)V(\d+)C(\d+)\s+(\d+)', line)
        # if data is None:
        #     return
        # A = int(data.group(1))
        # V = int(data.group(2))
        # C = int(data.group(3))
        # adc_value = int(data.group(4))
        # if adc_value != 0:
        #     if (A<5):
        #         firstElec = 4*(A-1) + V + 1
        #         secondElec = firstElec + 1 if firstElec < 16 else 1
        #         firstStim = C+1
        #         secondStim = firstStim + 1 if firstStim < 16 else 1

        #         ID = f'V{firstElec}:{secondElec}|C{firstStim}:{secondStim}'
        #         with self.lock:
        #             if ID not in self.dataDict:
        #                 self.dataDict[ID] = [adc_value]
        #             else:
        #                 self.dataDict[ID].append(adc_value)

# class dataAnalyser:
#     # def __init__(self):
#     #     self.fs = fs
#     #     self.f_excitation = f_excitation

#     def key2elec(self,key):
#         v_part, c_part = key.split('|')
#         V = v_part.split(':')
#         C = c_part.split(':')
#         V[0] = V[0].replace('V', '')
#         C[0] = C[0].replace('C', '')
#         return (V,C)

#     def custom_sort_key(self,item):
#         key = item[0]  
#         v_part, c_part = key.split('|')
#         V = v_part.split(':')
#         C = c_part.split(':')
#         V[0] = V[0].replace('V', '')
#         C[0] = C[0].replace('C', '')
#         return (int(C[0]), int(V[0]))
    
#     def average_magnitude(self,data_dict):
#         result = []

#         for key,values in sorted(data_dict.items(), key=self.custom_sort_key):
#             C, V = self.key2elec(key)
#             if C[0] != V[0] and C[0] != V[1] and C[1] != V[0]:

#                 value = np.mean(values)*3.3/(2**12*50)
#                 result.append(value)
#         return result
    
#     def printDict(self, data_dict):
#         i = 1
#         for key, values in sorted(data_dict.items(), key=self.custom_sort_key): 
#             C, V = self.key2elec(key)
#             if C[0] != V[0] and C[0] != V[1] and C[1] != V[0]:

#                 print(f'{i}: {key}, {np.mean(values)}')
#                 i+=1
    
    # def calculate_magnitude(self, data_dict):
    #     result = []
    #     for key, values in sorted(data_dict.items(), key=self.custom_sort_key):
    #         C, V = self.key2elec(key)
    #         if C[0] != V[0] or C[0] != V[1] or C[1] != V[0]:
    #             # data = sig.medfilt(values,7)
    #             amplitude = self.find_amplitude(values)
    #             if amplitude > 100:
    #                 print(values)
    #             result.append(amplitude)
    #     return np.array(result)

    # def find_amplitude(self, data):
    #     data_np = np.array(data) - np.mean(data)
    #     return (np.max(data_np) - np.min(data_np))/2
    #     # fft_result = np.fft.fft(data_np)
    #     # magnitude = np.abs(fft_result)
    #     # n = len(data_np)
    #     # frequencies = np.fft.fftfreq(n, d=1 / self.fs)
    #     # index = np.where(np.isclose(frequencies, self.f_excitation))[0]
    #     # return magnitude[index] 


class dataReader():
    def __init__(self,port,baud):
        self.usb = USB(port,baud)

        self.usb.connect()

        self.dataQueue = queue.Queue()

        self.reader_thread = threading.Thread(target=self.reading, daemon=True)
        self.reader_thread.start()

    def reading(self):
        while True:
            self.usb.readBuffer()
            
            # line = self.usb.readUSB()
            # print(line)
            self.dataQueue.put(self.usb.parseLine(self.usb.data)*3.3/(2**12*50))




import matplotlib.pyplot as plt
import matplotlib.animation as Anim 

# data_queue.put([1,2,3,4,5,6])


# usb = USB('COM14',9600)
# usb.connect()

reader = dataReader('COM14',9600)
# while True:
#     pass

fig,axes = plt.subplots()

def show(mag):
    axes.cla()
    # axes.set_ylim(0.01)
    axes.grid(True)
    axes.plot(mag)
    plt.pause(0.01)

while True:
    data = reader.dataQueue.get()
    print(len(data))
    show(data)
    time.sleep(1)

# while True:
#     pass

# 




    

# fig, axes = plt.subplots()

# ani = Anim.FuncAnimation(fig=fig, func=show, fargs=(data_queue,fig,axes,),frames=40, interval=40)
# plt.show()


                    


        
 

       
