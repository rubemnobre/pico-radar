import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.colors import LinearSegmentedColormap
import serial
import serial.tools.list_ports
import numpy as np
import re
import threading
from multiprocessing import Queue, Event
import signal
import sys

N_PEAKS = 5
THRESHOLD = 0.0002
DECAY = 0.95
MAX_POINTS = 50
TIME_MODE = False
INTERVAL = 0.1

plt.style.use('dark_background')

def pico_run(port, q, end):
    while True:
        if end.is_set():
            break
        try:
            begin = port.read()
            if begin is None:
                continue

            if begin == b"b":
                data = port.read_until(b"e")
                text = data.decode("ascii")
                
                m = re.search(r"m" + r"(.+);"*N_PEAKS, text)
                f = re.search(r"f" + r"(.+);"*N_PEAKS, text)
                a = re.search(r"a(.+);", text)
                if (m is not None) and (f is not None) and (a is not None): 
                    data = {'text' : text, 'freqs' : [float(f[i]) for i in range(1,N_PEAKS+1)], 'mags' : [float(m[i]) for i in range(1,N_PEAKS+1)], 'ang' : float(a[1])}
                    if not q.full():
                        q.put_nowait(data)
        except:
            end.set()
            break



if __name__ == "__main__":
    ports = serial.tools.list_ports.comports()
    pico = None
    for p in ports:
        if p.vid == 11914 and p.pid == 10:
            pico = serial.Serial(p.name, 115200)
    if pico is None:
        raise Exception("Pico not found in serial ports")
    pico.timeout = 0.01

    q = Queue(20)
    end = Event()

    def kill(signum, frame):
        end.set()
        pico_thread.join()

    signal.signal(signal.SIGTERM, kill)

    pico_thread = threading.Thread(target=pico_run, args=(pico, q, end))
    pico_thread.start()

    angles = []
    mags = []
    freqs = []
    intensity = []
    times = []
    time = 0
    

    def animate(__):
        global angles, mags, freqs, intensity, time, times

        datas = []
        while not q.empty():
            datas.append(q.get_nowait())
        
        for data in datas:
            for i in range(N_PEAKS):
                if data['mags'][i] > THRESHOLD:
                    angles.append(data['ang'])
                    freqs.append(data['freqs'][i])
                    mags.append(10*np.sqrt(data['mags'][i]/THRESHOLD))
                    intensity.append(1)
                    times.append(time)
        time += INTERVAL
        
        if len(angles) > MAX_POINTS:
            angles = angles[-MAX_POINTS:]
            freqs = freqs[-MAX_POINTS:]
            mags = mags[-MAX_POINTS:]
            intensity = intensity[-MAX_POINTS:]
            times = times[-MAX_POINTS:]

        plt.cla()
        plt.ylim((0, 200))
        colors = [[0,1,0,0],[0,1,0,0.5],[0,1,0,1]]
        cmap = LinearSegmentedColormap.from_list("", colors)

        if TIME_MODE == False:
            plt.xlim((0, 90))
            intensity = [i*DECAY for i in intensity[-MAX_POINTS:]]
            plt.scatter(angles, freqs, s=mags, c=intensity, cmap=cmap, vmin=0, vmax=1)
        else:
            if len(times) > 0:
                plt.xlim((max(0,time-MAX_POINTS*INTERVAL), time))
            plt.scatter(times, freqs, s=mags, c=intensity, cmap=cmap, vmin=0, vmax=1)

        plt.grid()

    ani = FuncAnimation(plt.gcf(), animate, interval=INTERVAL*1000)
    plt.show()
    kill(0,0)

    