import serial
import struct
from math import *
import errno

M_PI =  3.1415926535897932385


  
ser = serial.Serial(
        port='COM5',
        baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
      )


for i in range(100):
    s = ser.read()
    print(str(i)+"  "+str(s))    

   

ser.close