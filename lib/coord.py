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
"""
    def apply_kisa(self, az, el, hosei):
        """ from [coordinate.cpp]
        #kisa parameter
        (daz[0], de[1], kai_az[2], omega_az[3], eps[4], kai2_az[5], omega2_az[6], kai_el[7], omega_el[8], kai2_el[9], omega2_el[10], g[11], gg[12], ggg[13], gggg[14],
        del[15], de_radio[16], del_radio[17], cor_v[18], cor_p[19], g_radio[20], gg_radio[21], ggg_radio[22], gggg_radio[23])
        """
        kisa = self.read_kisa_file(hosei,24)
        geo_kisa = self.read_kisa_file("hosei_opt_geomech.txt",10)
        """
        #geo_kisa parameter
        (kai[0], omega[1], gxc[2], gyc[3], scx[4], scy[5], tx1[6], tx2[7], ty1[8], ty2[9])
        """
        DEG2RAD = math.pi/180
        RAD2DEG = 180/math.pi
        ARCSEC2RAD = math.pi/(180*60*60.)
        kisa[3] = kisa[3]*DEG2RAD
        kisa[6] = kisa[6]*DEG2RAD
        kisa[8] = kisa[8]*DEG2RAD
        kisa[10] = kisa[10]*DEG2RAD
        kisa[19] = kisa[19]*DEG2RAD
        el_d = el*RAD2DEG
        delta = [0,0]

        # reference from src/coord/correct.h
        # line 242, 248
        dx = kisa[2]*math.sin(kisa[3]-az)*math.sin(el)+kisa[4]*math.sin(el)+kisa[0]*math.cos(el)+kisa[1]+kisa[5]*math.cos(2*(kisa[3]-az))*math.sin(el)\
            +kisa[16]+kisa[18]*math.cos(el+kisa[19])
        delta[0] = -dx # arcsec

        dy = -kisa[7]*math.cos(kisa[8]-az)-kisa[9]*math.sin(2*(kisa[10]-az))+kisa[15]+kisa[11]*el_d+kisa[12]*el_d*el_d+kisa[13]*el_d*el_d*el_d+kisa[14]*el_d*el_d*el_d*el_d\
            +kisa[17]-kisa[18]*math.sin(el+kisa[19])+kisa[20]*el_d+kisa[21]*el_d*el_d+kisa[22]*el_d*el_d*el_d+kisa[23]*el_d*el_d*el_d*el_d
        delta[1] = -dy # arcsec
        if(math.fabs(math.cos(el))>0.001):
            delta[0]=delta[0]/math.cos(el)

        geo_x = geo_kisa[0]*(-math.sin((geo_kisa[1]-az)*(math.pi/180.))+math.cos((geo_kisa[1]-az)*(math.pi/180.)))+geo_kisa[2]
        geo_y = geo_kisa[0]*(-math.sin((geo_kisa[1]-az)*(math.pi/180.))-math.cos((geo_kisa[1]-az)*(math.pi/180.)))+geo_kisa[3]
        ret = self.geomech.read_geomech_col() # ret[0] = geomech_x, ret[1] = geomech_y

        gx = ret[0]-geo_x
        gy = ret[1]-geo_y
        ggx = -((gx+gy)/math.sqrt(2))*math.sin(el*(math.pi/180.)) # arcsec
        ggy = -(gx-gy)/math.sqrt(2) # arcsec

        delta[0] = delta[0]-ggx
        delta[1] = delta[1]-ggy
        return delta


    def apply_kisa_test(self, az, el, hosei):
        """ from [coordinate.cpp]
        #kisa parameter
        (daz[0], de[1], kai_az[2], omega_az[3], eps[4], kai2_az[5], omega2_az[6], kai_el[7], omega_el[8], kai2_el[9], omega2_el[10], g[11], gg[12], ggg[13], gggg[14],
        del[15], de_radio[16], del_radio[17], cor_v[18], cor_p[19], g_radio[20], gg_radio[21], ggg_radio[22], gggg_radio[23])
        """
        kisa = self.read_kisa_file(hosei,24)

        DEG2RAD = math.pi/180
        RAD2DEG = 180/math.pi
        ARCSEC2RAD = math.pi/(180*60*60.)
        kisa[3] = kisa[3]*DEG2RAD
        kisa[6] = kisa[6]*DEG2RAD
        kisa[8] = kisa[8]*DEG2RAD
        kisa[10] = kisa[10]*DEG2RAD
        kisa[19] = kisa[19]*DEG2RAD
        el_d = el*RAD2DEG
        delta = [0,0]

        # reference from src/coord/correct.h
        # line 242, 248
        dx = kisa[2]*math.sin(kisa[3]-az)*math.sin(el)+kisa[4]*math.sin(el)+kisa[0]*math.cos(el)+kisa[1]+kisa[5]*math.cos(2*(kisa[3]-az))*math.sin(el)\
            +kisa[16]+kisa[18]*math.cos(el+kisa[19])
        delta[0] = -dx # arcsec

        dy = -kisa[7]*math.cos(kisa[8]-az)-kisa[9]*math.sin(2*(kisa[10]-az))+kisa[15]+kisa[11]*el_d+kisa[12]*el_d*el_d+kisa[13]*el_d*el_d*el_d+kisa[14]*el_d*el_d*el_d*el_d\
            +kisa[17]-kisa[18]*math.sin(el+kisa[19])+kisa[20]*el_d+kisa[21]*el_d*el_d+kisa[22]*el_d*el_d*el_d+kisa[23]*el_d*el_d*el_d*el_d
        delta[1] = -dy # arcsec
        if(math.fabs(math.cos(el))>0.001):
            delta[0]=delta[0]/math.cos(el)

        return delta
"""

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
