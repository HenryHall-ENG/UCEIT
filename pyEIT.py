import numpy as np
import matplotlib.pyplot as plt

import pyeit
import pyeit.mesh as mesh
import pyeit.eit.protocol as protocol
from pyeit.eit.fem import EITForward
from pyeit.eit.greit import GREIT

import serial
import matplotlib.animation as Anim 
import re
import time

fs = 1e6
f_excitation = 1e3
PORT = 'COM6'
BAUD = 9600

def init_fem(r,h0,n_el):
    def _fd(pts):
        """shape function"""
        return pyeit.mesh.shape.circle(pts, pc=[0, 0], r=r)
    n_el = n_el  
    mesh_obj = mesh.create(n_el, fd=_fd, h0=h0)
    
    return mesh_obj
    
def display_fem(mesh_obj):
    pts = mesh_obj.node
    tri = mesh_obj.element
    el_pos = mesh_obj.el_pos

    fig, ax = plt.subplots(figsize=(6, 4))
    ax.triplot(pts[:, 0], pts[:, 1], tri, linewidth=1)
    ax.plot(pts[el_pos, 0], pts[el_pos, 1], "ro")
    plt.show()

def init_forward(mesh_obj, stimulation=1, measurement=1):
    protocol_obj = protocol.create(mesh_obj.n_el, dist_exc=stimulation, step_meas=measurement, parser_meas="std")

    fwd = EITForward(mesh_obj, protocol_obj)
    v0 = fwd.solve_eit()

    plt.plot(np.arange(0,len(v0), 1), v0)
    plt.show()
    return protocol_obj

def init_inverse(mesh_obj, protocol_obj):
    eit = GREIT(mesh_obj, protocol_obj)
    eit.setup(p=0.50, lamb=0.01, perm=1, jac_normalized=True)
    return eit

def inverse(eit,data1,data2):
    ds = eit.solve(data2, data1, normalize=True)
    x, y, ds = eit.mask_value(ds, mask_value=np.NAN)
    return ds

def displayEIT(ds, fig, axes):
    axes.cla()
    im = axes.imshow(np.real(ds), interpolation="none", cmap=plt.cm.viridis)
    axes.axis("equal")
    fig.colorbar(im, ax=axes.ravel().tolist())

def initEIT(r,h0,n_el):
    mesh_obj=init_fem(r,h0.n_el)
    protocol_obj=init_forward(mesh_obj)
    eit=init_inverse(mesh_obj,protocol_obj)
    return eit

def read_serial(ser):
    line = ser.readline().decode().strip()
    ser.flushInput()
    data = re.match(r'A(\d+)V(\d+)C(\d+)\s+(\d+)', line)
    return data


def proccess_data(raw_data, data_dict):
    A = int(raw_data.group(1))
    V = int(raw_data.group(2))
    C = int(raw_data.group(3))
    adc_value = int(raw_data.group(4))

    firstElec = 4*(A-1) + V
    secondElec = firstElec + 1 if firstElec < 16 else 1
    firstStim = C+1
    secondStim = firstStim + 1 if firstStim < 16 else 1

    ID = f'V{firstElec}:{secondElec}|C{firstStim}:{secondStim}'
    if ID not in data_dict:
        data_dict[ID] = [adc_value]
    else:
        data_dict[ID].append(adc_value)
    return data_dict

def key2elec(key):
    v_part, c_part = key.split('|')
    V = v_part.split(':')
    C = c_part.split(':')
    V[0] = V[0].replace('V', '')
    C[0] = C[0].replace('C', '')
    return int(V),int(C)

def min_length(data_dict):
    lengths = [len(value) for value in data_dict.values()]
    return min(lengths)

def custom_sort_key(item):
    key = item[0]  
    C,V = key2elec(key)
    return (V[0], C[0])
    
def find_amp(data):
    data_np = np.array(data)
    fft_result = np.fft.fft(data_np)
    magnitude = np.abs(fft_result)
    n = len(data_np)
    frequencies = np.fft.fftfreq(n, d=1/fs)
    index = np.where(np.isclose(frequencies, f_excitation))[0]

    return magnitude[index]


def calc_magnitude(data_dict):
    magnitude = []
    for key,values in sorted(data_dict.items(),key=custom_sort_key):
        C,V = key2elec(key)
        if C[0] != V[0] and C[1] != V[1]:
            magnitude.append(find_amp(values))
    return magnitude


def main():
    ser = serial.Serial(PORT, BAUD)
    EIT = initEIT(0.1,0.01,16)

    data_dictionary = {}
    magNew = []
    magOld = []

    ds = 0
    fig, axes = plt.subplots(1)

    ani = Anim.FuncAnimation(fig=fig, func=update, fargs=(DS,fig,axes,),frames=40, interval=40)

    while True: 
        time.sleep(0.01)
        raw_data = read_serial(ser)
        data_dictionary=proccess_data(raw_data, data_dictionary)
        if len(data_dictionary) > 256:
            if min_length(data_dictionary) > 100:
                magOld = magNew
                magNew = calc_magnitude(data_dictionary)
                data_dictionary = {}
                if len(magOld) > 0:
                    DS = inverse(EIT,magOld,magNew)


def update(frame, ds, fig, axes):
    displayEIT(ds, fig, axes)

