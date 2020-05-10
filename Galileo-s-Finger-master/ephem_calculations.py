import ephem
import datetime
from math import radians

M_PI =  3.1415926535897932385

home = ephem.Observer()
home.date = datetime.datetime.utcnow()
home.lat, home.lon ='50.014169','19.872986'
home.elevation = 219
home.epoch = '2020'

#na dzien (Spika)
ephemra = '13:26:17.26'
ephemdec = '-11:16:07.9'

#J2000
#ephemra = '13:25:11.53'
#ephemdec = '-11:09:41.5'

def angleToDecimal(pAngle):
    return float(repr(pAngle))/M_PI*180.0

def angleToStellarium(pAngle):
    return float(repr(pAngle))*M_PI/(0x80000000)
    
star = ephem.Equatorial(ephemra, ephemdec, epoch=home.epoch) #https://rhodesmill.org/pyephem/
body = ephem.FixedBody()
body._ra = star.ra
body._dec = star.dec
body._epoch = star.epoch    

body.compute(home)
print("ra2= " + str(body._ra))
print("dec2= " + str(body._dec))    
print("azimuth= " + str(body.az))
print("altitude= " + str(body.alt))
print(angleToDecimal(body.az))
print(angleToDecimal(body.alt))



#s = str(home.radec_of(repr(body.az), repr(body.alt)))
s = str(home.radec_of(float(ephem.degrees(body.az)), float(ephem.degrees(body.alt))))
print(home.radec_of(float(ephem.degrees(body.az)), float(ephem.degrees(body.alt))))
home.date = datetime.datetime.utcnow()
pos = home.radec_of(float(ephem.degrees(body.az)), float(ephem.degrees(body.alt)))
print(float(pos[0])*180/M_PI)
print(float(pos[1])*180/M_PI)


