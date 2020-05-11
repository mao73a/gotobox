#!/usr/bin/env python
# -*- coding: utf-8 -*-
import struct
import time
import socket, select
from math import *
import serial
import errno
import ephem
import sys
import datetime

M_PI =  3.1415926535897932385

home = ephem.Observer()
home.date = datetime.datetime.utcnow()
home.lat, home.lon ='50.014111','19.873399'
home.elevation = 219
home.epoch = '2020'

#na dzien (Spika)
ephemra = '13:26:17.26'
ephemdec = '-11:16:07.9'
#star = ephem.Equatorial(ephemra, ephemdec, epoch=home.epoch) #https://rhodesmill.org/pyephem/
body = ephem.FixedBody()


try:
    ser = serial.Serial(
            port='COM5',
            baudrate = 9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=2
          )
except serial.SerialException as e:
    err = e.args[0]
    print (e)
    sys.exit(1)
      
      
      
      
def angleToDecimal(pAngle):
    return float(repr(pAngle))/M_PI*180.0

def angleToStellarium(pAngle):
    #print(pAngle)
    return float(repr(pAngle))/M_PI*0x80000000


def stellariumToArdiuno(pEphemra, pEphemdec):
    body._ra =  pEphemra*(M_PI/0x80000000) 
    body._dec = pEphemdec*(M_PI/0x80000000)    
    body._epoch = home.epoch

    body.compute(home)
    print("ra2= " + str(body._ra))
    print("dec2= " + str(body._dec))    
    #print("azimuth= " + str(body.az))
    #print("altitude= " + str(body.alt))
    #print(angleToDecimal(body.az))
    #print(angleToDecimal(body.alt))
    return angleToDecimal(body.az), angleToDecimal(body.alt)


def arduinoToStellarium(pAzimuth, pAltitude):
    s = str(home.radec_of(float(ephem.degrees(pAzimuth)), float(ephem.degrees(pAltitude))))
    home.date = datetime.datetime.utcnow()
    pos = home.radec_of(float(ephem.degrees(str(pAzimuth))), float(ephem.degrees(str(pAltitude))))
    print('arduinoToStellarium Az/pAltitude: '+str(pAzimuth)+' / '+str(pAltitude))
    print('arduinoToStellarium Ra/Dec: '+str(pos[0])+' / '+str(pos[1]))
    #print(float(pos[0])*180/M_PI)
    #print(float(pos[1])*180/M_PI)
    return angleToStellarium(pos[0]), angleToStellarium(pos[1])
    

def arduionoEncodeAlt(pAlt):
    return round((pAlt+90.0)*0xffff/180.0)
    
def arduionoEncodeAz(pAz):
    return round(pAz*0xffff/360.0)

def serialSendAltAzToArduino(pAz, pAlt):
    #send position to telescope
    print('serialSendAltAzToArduino: '+str(pAz)+', '+str(pAlt))
    x=int(arduionoEncodeAz(pAz))
    y=int(arduionoEncodeAlt(pAlt))
    print('encoded: '+str(x)+',   '+str(y))
    ser.write(b'goto')
    ser.write(arduionoEncodeAz(pAz).to_bytes(2,'big'))
    ser.write(arduionoEncodeAlt(pAlt).to_bytes(2,'big'))
    ser.write(b'.')
    print("Coordinates sent to telescope:"+str(pAz)+", "+str(pAlt))

    
      
# List of socket objects that are currently open
open_sockets = []

# AF_INET means IPv4.
# SOCK_STREAM means a TCP connection.
# SOCK_DGRAM would mean an UDP "connection".
listening_socket = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
listening_socket.setblocking(False)
listening_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# The parameter is (host, port).
# The host, when empty or when 0.0.0.0, means to accept connections for
# all IP addresses of current machine. Otherwise, the socket will bind
# itself only to one IP.
# The port must greater than 1023 if you plan running this script as a
# normal user. Ports below 1024 require root privileges.
listening_socket.bind( ("", 10001) )

# The parameter defines how many new connections can wait in queue.
# Note that this is NOT the number of open connections (which has no limit).
# Read listen(2) man page for more information.
listening_socket.listen(5)

current_position = []

    
def printit(ra_int, dec_int):
    h = ra_int
    d = floor(0.5 + dec_int*(360*3600*1000/4294967296.0));
    dec_sign = ''
    if d >= 0:
        if d > 90*3600*1000:
            d =  180*3600*1000 - d;
            h += 0x80000000;
        dec_sign = '+';
    else:
        if d < -90*3600*1000:
            d = -180*3600*1000 - d;
            h += 0x80000000;
        d = -d;
        dec_sign = '-';
    
    
    h = floor(0.5+h*(24*3600*10000/4294967296.0));
    ra_ms = h % 10000; h /= 10000;
    ra_s = h % 60; h /= 60;
    ra_m = h % 60; h /= 60;
    
    h %= 24;
    dec_ms = d % 1000; d /= 1000;
    dec_s = d % 60; d /= 60;
    dec_m = d % 60; d /= 60;

    print ("ra =", h,"h", ra_m,"m",ra_s,".",ra_ms)
    print ("dec =",dec_sign, d,"d", dec_m,"m",dec_s,".",dec_ms)

    ephemra = str(trunc(h)) +':'+ str(trunc(ra_m)) +':'+ str(trunc(ra_s)) +'.'+ str(trunc(ra_ms))
    ephemdec = str(dec_sign) + str(trunc(d)) +':'+ str(trunc(dec_m)) +':'+ str(trunc(dec_s)) +'.'+ str(trunc(dec_ms))
    
    
    #print("ra =", h,"h", ra_m,"m",ra_s,".",ra_ms, sep=" ")
    #print("dec =",dec_sign, d,"d", dec_m,"m",dec_s,".",dec_ms)
    #print("ephemra =" + ephemra)
    #print("ephemdec =" +ephemdec)
    


tele_Az=0
tele_Al=0
while True:
    # Waits for I/O being available for reading from any socket object.
    print("Waiting for Stellarium to connect...\n")    
    rlist, wlist, xlist = select.select( [listening_socket] + open_sockets, [], [] )
    print("Stellarium connected.\n")
    for i in rlist:
        if i is listening_socket:
            print("Accepting connection.\n")
            new_socket, addr = listening_socket.accept()
            open_sockets.append(new_socket)
        else:
            while True:        

                tele = ser.readline() #wait(timeout)
                if (tele!=b''):
                    print(tele)
                    if tele[0:5]==b'AZAL:':
                       #data received from Arduiono; send them to Stellarium 
                       sep=tele.find(b',')
                       tele_Az=int(tele[5:sep].decode('cp1250'))
                       tele_Al=int(tele[sep+1:].decode('cp1250'))
                       (int_tele_Az, int_tele_Al)=arduinoToStellarium(tele_Az/10.0, tele_Al/10.0)
                       reply = struct.pack("3iIii", 24, 0, int(time.time()), int(int_tele_Az), int(int_tele_Al), 0)
                       i.send(reply)
                       i.send(reply)
                       #i.send(reply)
                       print('Data sent to Stellarium')
                       
                #print("Waitng for telescope...");                       
                try:
                    data = i.recv(1024) #nowait(timeout=0)
                except socket.error as e:
                    err = e.args[0]
                    if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
                        #print ("No data available")
                        continue
                    else:
                        # a "real" error occurred
                        print (e)
                        sys.exit(1)
                        
                else:
                    if data == "":
                        print("     if 3.1\n")
                        open_sockets.remove(i)
                        print ("Connection closed.")
                    else:

                           
                        print("Receiving GoTo command...n")
                        #print (repr(data))
                        if data==b'':
                            print ("Stellarium has ended connection?\n")
                        else:    
                            data = struct.unpack("3iIi", data)                            
                            print ("otrzymane wsp: ")
                            print ("%x, %o" % (data[3], data[3]))
                            ra = data[3]*(M_PI/0x80000000)
                            dec = data[4]*(M_PI/0x80000000)
                            cdec = cos(dec)

                            desired_pos = []
                            desired_pos.append(cos(ra)*cdec)
                            desired_pos.append(sin(ra)*cdec)
                            desired_pos.append(sin(dec))
                            ra_int=data[3]
                            de_int=data[4]                                                
                            print ("desired pos ra, dec: ")                        
                            print ("RA=",ra_int, "DE=", de_int)                        
                            #printit(data[3], data[4])
                            (x, y)= stellariumToArdiuno(data[3], data[4]) 
                            serialSendAltAzToArduino(x, y)


ser.close