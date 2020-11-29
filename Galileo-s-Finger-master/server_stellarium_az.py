#!/usr/bin/env python
# -*- coding: utf-8 -*-
import struct
import time
import signal
import socket, select
from math import *
import serial #pip install pyserial
import errno
import ephem  #requires MS Visual C++ 14 (6GB Install!)
import sys
import datetime
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-s", "--serial", default='COM1', help="Serial port name (COM1, COM2, COM3, etc). You need to install CH340 driver for Arduino to see USB port as Serial port. Default: COM1")
parser.add_argument("-lat", "--latitude",  help="Observer  latitude in decimal format, ex 51.5266. Positive=North, Negative=South")
parser.add_argument("-lon", "--longitude", help="Observer longitude in decimal format, ex -0.0798. Positive=East, Negative=West")
parser.add_argument("-ele", "--elevation", help="Observer elevation above sea level in meters, ex 123. Default: 0")
parser.add_argument("-epo", "--epoch", default='2020', help="Epoch year, default: 2020")
parser.add_argument("-v", "--verbose",  help="Verbose mode")
args = parser.parse_args()

home = ephem.Observer()
home.date = datetime.datetime.utcnow()
home.epoch =str(datetime.datetime.now().year) # '2020'
home.lat, home.lon ='51.476836','0' #Greenwich
home.elevation = 0


if  args.longitude:
   home.lon =  args.longitude
if  args.latitude:
   home.lat =  args.latitude
if args.elevation:
   home.elevation =  int(args.elevation)
if args.epoch:
   home.epoch =  args.epoch

print("Serial port="+ args.serial)   
print("Longitude="+ str(home.lon))
print("Latitude="+  str(home.lat))
print("Elevation="+ str(round(home.elevation))+"m")
print("Epoch="+ str(home.epoch)+"\n")


M_PI =  3.1415926535897932385

#na dzien (Spika)
ephemra = '13:26:17.26'
ephemdec = '-11:16:07.9'
#star = ephem.Equatorial(ephemra, ephemdec, epoch=home.epoch) #https://rhodesmill.org/pyephem/
body = ephem.FixedBody()

def signal_handler(signal, frame):
  sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

print("Connecting to serial port "+args.serial)     
try:
    ser = serial.Serial(
            port=args.serial,
            baudrate = 115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            dsrdtr=False,
            timeout=2
          )
except serial.SerialException as e:
    err = e.args[0]
    print (e)
    sys.exit(1)
      
time.sleep(5)      
print("Serial port connected.\n")      
      
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
    print("  ->RA= " + str(body._ra)+" DEC= " + str(body._dec))    
    #print("azimuth= " + str(body.az))
    #print("altitude= " + str(body.alt))
    #print(angleToDecimal(body.az))
    #print(angleToDecimal(body.alt))
    return angleToDecimal(body.az), angleToDecimal(body.alt)


def arduinoToStellarium(pAzimuth, pAltitude):
    s = str(home.radec_of(float(ephem.degrees(pAzimuth)), float(ephem.degrees(pAltitude))))
    home.date = datetime.datetime.utcnow()
    pos = home.radec_of(float(ephem.degrees(str(pAzimuth))), float(ephem.degrees(str(pAltitude))))
    if args.verbose:
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
    if args.verbose:
        print('serialSendAltAzToArduino: '+str(pAz)+', '+str(pAlt))

    x=int(arduionoEncodeAz(pAz))
    y=int(arduionoEncodeAlt(pAlt))
    if args.verbose:    
        print('encoded: '+str(x)+',   '+str(y))
    ser.write(b'goto')
    ser.write(arduionoEncodeAz(pAz).to_bytes(2,'big'))
    ser.write(arduionoEncodeAlt(pAlt).to_bytes(2,'big'))
    ser.write(b'.')
    if args.verbose:    
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

tele_Az=0
tele_Al=0
while True:
    try:
        # Waits for I/O being available for reading from any socket object.
        print("Waiting for Stellarium to connect to localhost:10001...\n")    
        rlist, wlist, xlist = select.select( [listening_socket] , [], [],10 )
        rlist+= open_sockets
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
                        if args.verbose:
                            print(tele)
                        if tele[0:5]==b'AZAL:':
                           #data received from Arduiono; send them to Stellarium 
                           sep=tele.find(b',')
                           tele_Az=float(tele[5:sep].decode('cp1250'))
                           tele_Al=float(tele[sep+1:].decode('cp1250'))
                           (int_tele_Az, int_tele_Al)=arduinoToStellarium(tele_Az, tele_Al)
                           reply = struct.pack("3iIii", 24, 0, int(time.time()), int(int_tele_Az), int(int_tele_Al), 0)
                           i.send(reply)
                           i.send(reply)
                           #i.send(reply)
                           if args.verbose:
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

                            print("Receiving GOTO command from Stellarium...")
                            #print (repr(data))
                            if data==b'':
                                print ("Stellarium has ended connection?")
                            else:    
                                data = struct.unpack("3iIi", data)                            
                                print ("Received coordinates (row numbers): ")
                                print ("  %x, %o" % (data[3], data[3]))
                                ra = data[3]*(M_PI/0x80000000)
                                dec = data[4]*(M_PI/0x80000000)
                                cdec = cos(dec)

                                desired_pos = []
                                desired_pos.append(cos(ra)*cdec)
                                desired_pos.append(sin(ra)*cdec)
                                desired_pos.append(sin(dec))
                                ra_int=data[3]
                                de_int=data[4]                                                
                                print ("  RA=",ra_int, "DE=", de_int)                        
                                (x, y)= stellariumToArdiuno(data[3], data[4]) 
                                print ("  ALT=",x, "LON=", y)     
                                serialSendAltAzToArduino(x, y)
                                print ("  Coorinates sent to telecope.") 
    except SystemExit:
        print("Program terminated.")
        sys.exit(1)
    except:
        print("Network connection error:", sys.exc_info()[0])
        for i in rlist:
            open_sockets.remove(i)
        open_sockets=[]
        print("Trying to reconnect...")


ser.close