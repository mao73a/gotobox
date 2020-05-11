import ephem
import datetime
from math import radians

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

def angleToDecimal(pAngle):
    return float(repr(pAngle))/M_PI*180.0

def angleToStellarium(pAngle):
    print(pAngle)
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
    #s = str(home.radec_of(float(ephem.degrees(pAzimuth)), float(ephem.degrees(pAltitude))))
    home.date = datetime.datetime.utcnow()
    #pos = home.radec_of(float(ephem.degrees(body.az)), float(ephem.degrees(body.alt)))
    pos = home.radec_of(float(ephem.degrees(str(pAzimuth))), float(ephem.degrees(str(pAltitude))))
    print(str(pos[0])+' - '+str(pos[1]))
    print(float(pos[0])*180/M_PI)
    print(float(pos[1])*180/M_PI)
    return angleToStellarium(pos[0]), angleToStellarium(pos[1])

(x, y)= stellariumToArdiuno(2404846656, -134442907) #Spika
print("Az="+str(x)+"  Alt="+str(y))

(ra, dec) = arduinoToStellarium(x, y)
print("ra="+str(ra)+"  dec="+str(dec))