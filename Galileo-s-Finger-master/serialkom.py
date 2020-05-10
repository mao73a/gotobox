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
        timeout=2
      )

def encodeAlt(pAlt):
    return round((pAlt+90.0)*0xffff/180.0)
    
def encodeAz(pAz):
    return round(pAz*0xffff/360.0)
    

print ("start")
s = ser.read()
ser.write(b'goto')
ser.write(encodeAz(0.14).to_bytes(2,'big'))
ser.write(encodeAlt(+32.16).to_bytes(2,'big'))
ser.write(b'.')
print("napisane")
for i in range(1):
    s = ser.readline()
    print(s)
    if s[0:5]==b'AZAL:':
       sep=s.find(b',')
       print(int(s[5:sep].decode('cp1250')))
       print(int(s[sep+1:].decode('cp1250')))


   

ser.close