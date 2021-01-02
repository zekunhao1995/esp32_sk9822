import serial
import numpy as np
import time

ser = serial.Serial('COM5', 921600, timeout=0)
print(ser.is_open)
#line = ser.readline()   # read a '\n' terminated line
#print(line)
"""
for i in range(65535):
    
    intensity = 0 if i % 2 == 0 else 65535
    ser.write('{:05d}\n'.format(intensity).encode("ascii"))
    time.sleep()
"""

t = 0
while True:
    t = (t + 1)%24576
    i = np.sin((t/512) * 2 * np.pi)
    i = np.clip((i+1)/2, 0, 1)
    #i = i ** 2
    intensity = int(i*65535)
    #print(intensity)
    rgain = 256
    ggain = 256
    bgain = 256
    ser.write('{:05d},{:03d},{:03d},{:03d}\n'.format(intensity, rgain, ggain, bgain).encode("ascii"))
    time.sleep(0.001)
ser.close()