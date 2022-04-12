#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: Arnaud Villeneuve
"""

from math import sqrt

class SpotCG:
    def __init__(self, _Spot):
        self.Spot   = _Spot

    def CG_calculation(self, thetalf, thetarf, thetarr, thetalr):
        cgposlf = (self.Spot.FK_Weight(thetalf, 1))
        cgposrf = (self.Spot.FK_Weight(thetarf, -1))
        cgposrr = (self.Spot.FK_Weight(thetarr, -1))
        cgposlr = (self.Spot.FK_Weight(thetalr, 1))

        Weightsum = self.Spot.Weight_Body+4 * (self.Spot.Weight_Shoulder+self.Spot.Weight_Leg+self.Spot.Weight_Foreleg)

        xcglf = (cgposlf[0]+self.Spot.xlf)*self.Spot.Weight_Shoulder+(cgposlf[1] + self.Spot.xlf)*self.Spot.Weight_Leg+(cgposlf[2]+self.Spot.xlf)*self.Spot.Weight_Foreleg
        xcgrf = (cgposrf[0]+self.Spot.xrf)*self.Spot.Weight_Shoulder+(cgposrf[1] + self.Spot.xrf)*self.Spot.Weight_Leg+(cgposrf[2]+self.Spot.xrf)*self.Spot.Weight_Foreleg
        xcgrr = (cgposrr[0]+self.Spot.xrr)*self.Spot.Weight_Shoulder+(cgposrr[1] + self.Spot.xrr)*self.Spot.Weight_Leg+(cgposrr[2]+self.Spot.xrr)*self.Spot.Weight_Foreleg
        xcglr = (cgposlr[0]+self.Spot.xlr)*self.Spot.Weight_Shoulder+(cgposlr[1] + self.Spot.xlr)*self.Spot.Weight_Leg+(cgposlr[2]+self.Spot.xlr)*self.Spot.Weight_Foreleg
        xcg = (xcglf+xcgrf+xcgrr+xcglr+self.Spot.xCG_Body*self.Spot.Weight_Body)/Weightsum

        ycglf = (cgposlf[3]+self.Spot.ylf)*self.Spot.Weight_Shoulder+(cgposlf[4] + self.Spot.ylf)*self.Spot.Weight_Leg+(cgposlf[5]+self.Spot.ylf)*self.Spot.Weight_Foreleg
        ycgrf = (cgposrf[3]+self.Spot.yrf)*self.Spot.Weight_Shoulder+(cgposrf[4] + self.Spot.yrf)*self.Spot.Weight_Leg+(cgposrf[5]+self.Spot.yrf)*self.Spot.Weight_Foreleg
        ycgrr = (cgposrr[3]+self.Spot.yrr)*self.Spot.Weight_Shoulder+(cgposrr[4] + self.Spot.yrr)*self.Spot.Weight_Leg+(cgposrr[5]+self.Spot.yrr)*self.Spot.Weight_Foreleg
        ycglr = (cgposlr[3]+self.Spot.ylr)*self.Spot.Weight_Shoulder+(cgposlr[4] + self.Spot.ylr)*self.Spot.Weight_Leg+(cgposlr[5]+self.Spot.ylr)*self.Spot.Weight_Foreleg
        ycg = (ycglf+ycgrf+ycgrr+ycglr+self.Spot.yCG_Body*self.Spot.Weight_Body)/Weightsum

        zcglf = (cgposlf[6]+self.Spot.zlf)*self.Spot.Weight_Shoulder+(cgposlf[7] + self.Spot.zlf)*self.Spot.Weight_Leg+(cgposlf[8]+self.Spot.zlf)*self.Spot.Weight_Foreleg
        zcgrf = (cgposrf[6]+self.Spot.zrf)*self.Spot.Weight_Shoulder+(cgposrf[7] + self.Spot.zrf)*self.Spot.Weight_Leg+(cgposrf[8]+self.Spot.zrf)*self.Spot.Weight_Foreleg
        zcgrr = (cgposrr[6]+self.Spot.zrr)*self.Spot.Weight_Shoulder+(cgposrr[7] + self.Spot.zrr)*self.Spot.Weight_Leg+(cgposrr[8]+self.Spot.zrr)*self.Spot.Weight_Foreleg
        zcglr = (cgposlr[6]+self.Spot.zlr)*self.Spot.Weight_Shoulder+(cgposlr[7] + self.Spot.zlr)*self.Spot.Weight_Leg+(cgposlr[8]+self.Spot.zlr)*self.Spot.Weight_Foreleg
        zcg = (zcglf+zcgrf+zcgrr+zcglr+self.Spot.zCG_Body*self.Spot.Weight_Body)/Weightsum

        return (xcg, ycg, zcg)

    def CG_distance(self, x_legs, y_legs, z_legs, xcg, ycg, stance):

        # line equation c * x + s * y - p  = 0
        # with c = a/m et s = b/m

        a1 = (y_legs[0]-y_legs[2])
        b1 = -(x_legs[0]-x_legs[2])
        m1 = sqrt(a1**2 + b1**2)
        c1 = a1/m1
        s1 = b1/m1

        a2 = (y_legs[1]-y_legs[3])
        b2 = -(x_legs[1]-x_legs[3])
        m2 = sqrt(a2**2 + b2**2)
        c2 = a2/m2
        s2 = b2/m2

        p1 = c1*x_legs[0] + s1*y_legs[0]
        p2 = c2*x_legs[1] + s2*y_legs[1]

        """ Dstance calculation """
        d1 = c1*xcg + s1*ycg - p1
        d2 = c2*xcg + s2*ycg - p2

        """ intersection calculation """
        # perpendicalar line equation -s * x + c * y - q = 0

        q1 = -s1*xcg + c1*ycg
        q2 = -s2*xcg + c2*ycg

        xint1 = c1*p1 - s1*q1
        yint1 = c1*q1 + s1*p1

        xint2 = c2*p2 - s2*q2
        yint2 = c2*q2 + s2*p2

        """ Check if inside sustentation triangle """
        d = 0
        xint = xcg
        yint = ycg
        if (stance[0] == False) | (stance[2] == False):
            d = d2
            xint = xint2
            yint = yint2

        if (stance[1] == False) | (stance[3] == False):
            d = d1
            xint = xint1
            yint = yint1

        balance = True

        if (stance[0] == False) & (d < 0):
            balance = False

        if (stance[1] == False) & (d > 0):
            balance = False

        if (stance[2] == False) & (d > 0):
            balance = False

        if (stance[3] == False) & (d < 0):
            balance = False

        if (balance == False):
            d = -abs(d)
        else:
            d = abs(d)

        return (d, xint, yint, balance)
