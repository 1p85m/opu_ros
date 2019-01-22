import math
import time

class coord_calc(object):

    eqrau = [0.0,
        2440.0,            # Mercury
        6051.8,            # Venus
        0.0,
        3389.9,            # Mars
        69134.1,           # Jupiter
        57239.9,           # Saturn
        25264.3,           # Uranus
        24553.1,           # Neptune
        1151.0,            # Pluto
        1737.53,           # Moon
        696000.0          ]# Sun
    tai_utc = 36.0 # tai_utc=TAI-UTC  2015 July from ftp://maia.usno.navy.mil/ser7/tai-utc.dat


    def __init__(self):
        #self.jpl = SPK.open('./de430.bsp')
        # from https://pypi.python.org/pypi/jplephem
        #self.geomech = geomech.geomech_monitor_client('172.20.0.12',8101)
        pass


    def apply_kisa_test(self,az1,el2,hosei):

        kisa = self.read_kisa_file(hosei,12)

        #calculate the values of correction
        cos_az=cos(az1);
        sin_az=sin(az1);
        cos_el=cos(el1);
        sin_el=sin(el1);

        a1 = kisa[0]
        a2 = kisa[1]
        a3 = kisa[2]
        b1 = kisa[3]
        b2 = kisa[4]
        b3 = kisa[5]
        c1 = kisa[6]
        c2 = kisa[7]
        d1 = kisa[8]
        d2 = kisa[9]
        e1 = kisa[10]
        e2 = kisa[11]
        g1 = kisa[12]

        #basic correction for AzEl mount
        d_az =  a1*sin_el + a2 + a3*cos_el + b1*sin_az*sin_el - b2*cos_az*sin_el
        d_el =  b1*cos_az + b2*sin_az + b3 + g1*el1*180./math.pi

        #For radio observations
        d_az = d_az + c1*sin(az1-el1) + c2*cos(az1-el1) + d1 + e1*cos_el - e2*sin_el
        d_el = d_el + c1*cos(az1-el1) - c2*sin(az1-el1) + d2 + e1*sin_el + e2*cos_el

        #convert to encoder offset on the horizon
        d_az =  d_az / cos_el

        #apply the correction values ->  radians
        delta[0] = az1 +(d_az/60.0)*math.pi/180.
        delta[1] = el1 +(d_el/60.0)*math.pi/180.

        return delta


    def read_kisa_file(self, hosei, num):
        f = open('/home/necst/ros/src/necst/lib/' + hosei)

        line = f.readline()
        kisa = [0]*num
        n = 0

        while line:
            line = line.rstrip()
            kisa[n] = float(line)
            line = f.readline()
            n = n+1
        f.close
        return kisa
