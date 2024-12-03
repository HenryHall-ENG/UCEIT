import numpy as np
import serial as serial
import threading 
import queue
import time
import sys
import re
import scipy.signal as sig

class USB():
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        
    def readUSB(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode().strip()
            self.ser.flushInput()
            return line

    def connect(self):
        self.ser = serial.Serial(self.port, self.baud)

class DataProcessor:
    def __init__(self):
        self.dataDict = {}
        self.lock = threading.Lock()

    def parseLine(self, line):
        if line is None:
            return
        data = re.match(r'A(\d+)V(\d+)C(\d+)\s+(\d+)', line)
        if data is None:
            return
        A = int(data.group(1))
        V = int(data.group(2))
        C = int(data.group(3))
        adc_value = int(data.group(4))
        if (A<5):
            firstElec = 4*(A-1) + V
            secondElec = firstElec + 1 if firstElec < 16 else 1
            firstStim = C+1
            secondStim = firstStim + 1 if firstStim < 16 else 1

            ID = f'V{firstElec}:{secondElec}|C{firstStim}:{secondStim}'
            with self.lock:
                if ID not in self.dataDict:
                    self.dataDict[ID] = [adc_value]
                else:
                    self.dataDict[ID].append(adc_value)

class dataAnalyser:
    def __init__(self, fs, f_excitation):
        self.fs = fs
        self.f_excitation = f_excitation

    def key2elec(self,key):
        v_part, c_part = key.split('|')
        V = v_part.split(':')
        C = c_part.split(':')
        V[0] = V[0].replace('V', '')
        C[0] = C[0].replace('C', '')
        return (V,C)

    def custom_sort_key(self,item):
        key = item[0]  
        v_part, c_part = key.split('|')
        V = v_part.split(':')
        C = c_part.split(':')
        V[0] = V[0].replace('V', '')
        C[0] = C[0].replace('C', '')
        return (int(V[0]), int(C[0]))
    
    def calculate_magnitude(self, data_dict):
        result = []
        for key, values in sorted(data_dict.items(), key=self.custom_sort_key):
            C, V = self.key2elec(key)
            if C[0] != V[0] or C[0] != V[1] or C[1] != V[0]:
                data = sig.medfilt(values,7)
                amplitude = self.find_amplitude(data)
                if amplitude > 1000:
                    print(data)
                result.append(amplitude)
        return np.array(result)

    def find_amplitude(self, data):
        data_np = np.array(data) - np.mean(data)
        return (np.max(data_np) - np.min(data_np))/2
        # fft_result = np.fft.fft(data_np)
        # magnitude = np.abs(fft_result)
        # n = len(data_np)
        # frequencies = np.fft.fftfreq(n, d=1 / self.fs)
        # index = np.where(np.isclose(frequencies, self.f_excitation))[0]
        # return magnitude[index] 


class dataReader():
    def __init__(self,port,baud,fs,f_excitation,period,callback):
        self.usb = USB(port,baud)
        self.processor = DataProcessor()
        self.analyzer = dataAnalyser(fs, f_excitation)

        self.usb.connect()

        self.callback = callback
        self.lastTime = time.time()
        self.period = period
        self.reader_thread = threading.Thread(target=self.reading, daemon=True)
        self.reader_thread.start()
    
    def reading(self):
        while True:
            line = self.usb.readUSB()
            if line:
                self.processor.parseLine(line)

                currentTime = time.time()
                if currentTime - self.lastTime >= self.period:
                    self.lastTime = currentTime
                    results = self.analyzer.calculate_magnitude(self.processor.dataDict)
                    self.processor.dataDict={}
                    self.callback(results)

import matplotlib.pyplot as plt
import queue


data_queue = queue.Queue()


def show(mag):
    plt.close()
    axes = plt.axes()
    axes.grid(True)
    axes.plot(mag)
    plt.show()
    

    
def dataQ(mag):
    data_queue.put(mag)

reader = dataReader('COM14',9600, 1e5,1e3,10,dataQ)


while True:
    mag = data_queue.get()
    if mag is None:  
        break
    show(mag)

                    


        
 

       
