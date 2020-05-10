import serial

for x in range(6):
  print(x)

ser = serial.Serial(
        port='COM5',
        baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
      )
      
ser.write("helo".encode())
for i in range(20):
    print(str(i)+"  ")
    s = ser.readline()
    print(s)          
    
    
ser.close
print ("Koniec")    