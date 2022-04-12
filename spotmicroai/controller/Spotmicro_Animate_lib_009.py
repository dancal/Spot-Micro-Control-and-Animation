#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: Arnaud Villeneuve
"""

from time import sleep, time
from math import pi, sin, cos, atan, atan2, sqrt
import numpy as np
import pygame

class SpotAnim:
    """Display colors """
    BLACK = (0,   0,   0)
    WHITE = (255, 255, 255)
    BLUE = (24,  84, 231)
    GREEN = (51, 204, 51)
    RED = (255,   0,   0)
    GREY = (225, 225, 225)
    DARK_GREY = (100, 100, 100)
    DARK_RED = (176, 0, 0)
    VIOLET = (207, 175, 255)
    DARKVIOLET = (103, 87, 127)
    CYAN = (0, 255, 255)
    DARK_CYAN = (0, 127, 127)
    MED_GREY = (170, 170, 170)

    """Axis coordinates for animation"""
    xX = [0, 100]
    yX = [0, 0]
    zX = [0, 0]

    xY = [0, 0]
    yY = [0, 100]
    zY = [0, 0]

    xZ = [0, 0]
    yZ = [0, 0]
    zZ = [0, 100]

    """Floor coordinates for animation"""
    xFloor = [500, 500, -500, -500]
    yFloor = [500, -500, -500, 500]
    zFloor = [0, 0, 0, 0]

    conv = 0

    def __init__(self, _Spot):
        self.Spot   = _Spot

        pygame.init()
        # seems necessary to have access to keyboard
        pygame.display.set_caption("SPOTMICRO")
        self.screen      = pygame.display.set_mode((600, 600))

    def display_rotate(self, x_spot, y_spot, z_spot, theta_spot, thetax, thetaz, xl, yl, zl):
        """ rotation related to movement"""
        line = []
        Ma = np.zeros(9)
        Mb = np.zeros(9)
        M1 = np.zeros(9)
        Ma = self.Spot.xyz_rotation_matrix(
            theta_spot[3], theta_spot[4], theta_spot[2]+theta_spot[5], False)
        Mb = self.Spot.xyz_rotation_matrix(theta_spot[0], theta_spot[1], 0, False)
        M1 = self.Spot.xyz_rotation_matrix(thetax, 0, thetaz, True)

        for i in range(0, len(xl)):
            # absolute coordinates of Spot lines in x,y,z frame
            out0 = self.Spot.new_coordinates(Ma, xl[i], yl[i], zl[i], x_spot, y_spot, z_spot)
            out = self.Spot.new_coordinates(Mb, out0[0], out0[1], out0[2], 0, 0, 0)

            # Coordinaites for dispaly on screen
            disp = self.Spot.new_coordinates(M1, out[0], out[1], out[2], 0, 0, 0)
            yd = disp[1]
            xd = disp[0]/2**(yd*SpotAnim.conv/2000)
            zd = disp[2]/2**(yd*SpotAnim.conv/2000)

            line.append([int(300+xd), int(300-zd)])
        return line

    def animate(self, pos, t, thetax, thetaz, angle, center_x, center_y, thetalf, thetarf, thetarr, thetalr, walking_speed, walking_direction, steering, stance):

        theta_spot = pos[12]
        x_spot = pos[13]
        y_spot = pos[14]
        z_spot = pos[15]
        CGabs = [x_spot[7], y_spot[7], z_spot[7]]
        dCG = [x_spot[8], y_spot[8], z_spot[8]]
        ZMP = [x_spot[9], y_spot[9], z_spot[9]]
        M = self.Spot.xyz_rotation_matrix(theta_spot[0], theta_spot[1], theta_spot[2], False)
        ZMPabs = self.Spot.new_coordinates(M, ZMP[0], ZMP[1], 0, x_spot[1], y_spot[1], z_spot[1])

        self.screen.fill(SpotAnim.WHITE)
        #pos = self.Spot.walking (track,x_offset,steering_radius,steering_angle,cw,h_amp,v_amp,b_height,ra_longi,ra_lat,0.2,t)

        "Floor Display"""
        line = SpotAnim.display_rotate(self, -x_spot[0], -y_spot[0], -z_spot[0], [theta_spot[0], theta_spot[1], 0, 0, 0, 0], thetax, thetaz, SpotAnim.xFloor, SpotAnim.yFloor, SpotAnim.zFloor)
        pygame.draw.polygon(self.screen, SpotAnim.GREY, line, 0)

        """Floor Grid Display """
        for i in range(0, 11):
            line = SpotAnim.display_rotate(self, -x_spot[0], -y_spot[0], -z_spot[0], [theta_spot[0], theta_spot[1], 0, 0, 0, 0], thetax, thetaz, [-500+i*100, -500+i*100], [-500, 500], [0, 0])
            pygame.draw.lines(self.screen, SpotAnim.DARK_GREY, False, line, 1)
            line = SpotAnim.display_rotate(self, -x_spot[0], -y_spot[0], -z_spot[0], [theta_spot[0], theta_spot[1], 0, 0, 0, 0], thetax, thetaz, [-500, 500], [-500+i*100, -500+i*100], [0, 0])
            pygame.draw.lines(self.screen, SpotAnim.DARK_GREY, False, line, 1)

        """ X,Y,Z frame display"""
        line = SpotAnim.display_rotate(self, -x_spot[0], -y_spot[0], -z_spot[0], [0, 0, 0, 0, 0, 0], thetax, thetaz, SpotAnim.xX, SpotAnim.yX, SpotAnim.zX)
        pygame.draw.lines(self.screen, SpotAnim.RED, False, line, 2)

        line = SpotAnim.display_rotate(self, -x_spot[0], -y_spot[0], -z_spot[0], [0, 0, 0, 0, 0, 0], thetax, thetaz, SpotAnim.xY, SpotAnim.yY, SpotAnim.zY)
        pygame.draw.lines(self.screen, SpotAnim.GREEN, False, line, 2)

        line = SpotAnim.display_rotate(self, -x_spot[0], -y_spot[0], -z_spot[0], [0, 0, 0, 0, 0, 0], thetax, thetaz, SpotAnim.xZ, SpotAnim.yZ, SpotAnim.zZ)
        pygame.draw.lines(self.screen, SpotAnim.BLUE, False, line, 2)

        """ Radius display """
        center_display = True
        if (steering < 2000):
            lineR = SpotAnim.display_rotate(self, -x_spot[0], -y_spot[0], -z_spot[0], [0, 0, 0, 0, 0, 0], thetax, thetaz, [center_x, x_spot[0]], [center_y, y_spot[0]], [0, 0])
        else:
            center_x1 = x_spot[0] + (center_x-x_spot[0])/steering*2000
            center_y1 = y_spot[0] + (center_y-y_spot[0])/steering*2000
            lineR = SpotAnim.display_rotate(self, -x_spot[0], -y_spot[0], -z_spot[0], [0, 0, 0, 0, 0, 0], thetax, thetaz, [center_x1, x_spot[0]], [center_y1, y_spot[0]], [0, 0])
            center_display = False

        """ Direction display """
        xd = x_spot[0] + walking_speed * cos(theta_spot[2] + walking_direction-pi/2)
        yd = y_spot[0] + walking_speed * sin(theta_spot[2] + walking_direction-pi/2)
        lineD = SpotAnim.display_rotate(self, -x_spot[0], -y_spot[0], -z_spot[0], [0, 0, 0, 0, 0, 0], thetax, thetaz, [xd, x_spot[0]], [yd, y_spot[0]], [0, 0])

        """Legs lines"""

        leglf = self.Spot.FK(thetalf, 1)
        legrf = self.Spot.FK(thetarf, -1)
        legrr = self.Spot.FK(thetarr, -1)
        leglr = self.Spot.FK(thetalr, 1)

        """ Center of Gravity """
        # Calculation of CG absolute position
        lineCG = SpotAnim.display_rotate(self, -x_spot[0], -y_spot[0], -z_spot[0], [0, 0, 0, 0, 0, 0], thetax, thetaz, [CGabs[0], CGabs[0]], [CGabs[1], CGabs[1]], [0, CGabs[2]])

        xleglf = [self.Spot.xlf, self.Spot.xlf+leglf[0], self.Spot.xlf +
                  leglf[1], self.Spot.xlf+leglf[2], self.Spot.xlf+pos[0]]
        yleglf = [self.Spot.ylf, self.Spot.ylf+leglf[3], self.Spot.ylf +
                  leglf[4], self.Spot.ylf+leglf[5], self.Spot.ylf+pos[1]]
        #zleglf =[b_height,b_height+leglf[6],b_height+leglf[7],b_height+leglf[8],b_height+pos[2]]
        zleglf = [0, leglf[6], leglf[7], leglf[8], pos[2]]
        linelf = SpotAnim.display_rotate(self, x_spot[1]-x_spot[0], y_spot[1]-y_spot[0], z_spot[1]-z_spot[0], theta_spot, thetax, thetaz, xleglf, yleglf, zleglf)

        xlegrf = [self.Spot.xrf, self.Spot.xrf+legrf[0], self.Spot.xrf +legrf[1], self.Spot.xrf+legrf[2], self.Spot.xrf+pos[3]]
        ylegrf = [self.Spot.yrf, self.Spot.yrf+legrf[3], self.Spot.yrf +legrf[4], self.Spot.yrf+legrf[5], self.Spot.yrf+pos[4]]
        #zlegrf =[b_height,b_height+legrf[6],b_height+legrf[7],b_height+legrf[8],b_height+pos[5]]
        zlegrf = [0, legrf[6], legrf[7], legrf[8], pos[5]]
        linerf = SpotAnim.display_rotate(self, x_spot[1]-x_spot[0], y_spot[1]-y_spot[0], z_spot[1]-z_spot[0], theta_spot, thetax, thetaz, xlegrf, ylegrf, zlegrf)

        xlegrr = [self.Spot.xrr, self.Spot.xrr+legrr[0], self.Spot.xrr + legrr[1], self.Spot.xrr+legrr[2], self.Spot.xrr+pos[6]]
        ylegrr = [self.Spot.yrr, self.Spot.yrr+legrr[3], self.Spot.yrr + legrr[4], self.Spot.yrr+legrr[5], self.Spot.yrr+pos[7]]
        #zlegrr =[b_height,b_height+legrr[6],b_height+legrr[7],b_height+legrr[8],b_height+pos[8]]
        zlegrr = [0, legrr[6], legrr[7], legrr[8], pos[8]]
        linerr = SpotAnim.display_rotate(self, x_spot[1]-x_spot[0], y_spot[1]-y_spot[0], z_spot[1]-z_spot[0], theta_spot, thetax, thetaz, xlegrr, ylegrr, zlegrr)

        xleglr = [self.Spot.xlr, self.Spot.xlr+leglr[0], self.Spot.xlr +leglr[1], self.Spot.xlr+leglr[2], self.Spot.xlr+pos[9]]
        yleglr = [self.Spot.ylr, self.Spot.ylr+leglr[3], self.Spot.ylr +leglr[4], self.Spot.ylr+leglr[5], self.Spot.ylr+pos[10]]
        #zleglr =[b_height,b_height+leglr[6],b_height+leglr[7],b_height+leglr[8],b_height+pos[11]]
        zleglr = [0, leglr[6], leglr[7], leglr[8], pos[11]]
        linelr = SpotAnim.display_rotate(self, x_spot[1]-x_spot[0], y_spot[1]-y_spot[0], z_spot[1]-z_spot[0], theta_spot, thetax, thetaz, xleglr, yleglr, zleglr)
        """ Body frame lines """
        lineb = SpotAnim.display_rotate(self, x_spot[1]-x_spot[0], y_spot[1]-y_spot[0], z_spot[1]-z_spot[0], theta_spot, thetax, thetaz, [
                                        self.Spot.xlf, self.Spot.xrf, self.Spot.xrr, self.Spot.xlr, self.Spot.xlf], [self.Spot.ylf, self.Spot.yrf, self.Spot.yrr, self.Spot.ylr, self.Spot.ylf], [self.Spot.zlf, self.Spot.zrf, self.Spot.zrr, self.Spot.zlr, self.Spot.zlf])

        """Sustentation area lines"""
        # stance = False when leg is lifted from the floor

        linesus = []
        if (stance[0] == True):
            linesus.append(linelf[4])
        if (stance[1] == True):
            linesus.append(linerf[4])
        if (stance[2] == True):
            linesus.append(linerr[4])
        if (stance[3] == True):
            linesus.append(linelr[4])

        """ Center of gravity into sustentation area """
        linedCG = SpotAnim.display_rotate(self, -x_spot[0], -y_spot[0], -z_spot[0], [0, 0, 0, 0, 0, 0], thetax, thetaz, [CGabs[0], dCG[0]], [CGabs[1], dCG[1]], [0, 0])
        lineZMP = SpotAnim.display_rotate(self, -x_spot[0], -y_spot[0], -z_spot[0], [0, 0, 0, 0, 0, 0], thetax, thetaz, [ZMPabs[0], CGabs[0]], [ZMPabs[1], CGabs[1]], [0, 0])
        if sum(stance) > 2:
            pygame.draw.polygon(self.screen, SpotAnim.VIOLET, linesus, 0)
            pygame.draw.lines(self.screen, SpotAnim.BLACK, True, linesus, 1)

        pygame.draw.lines(self.screen, SpotAnim.DARKVIOLET, False, linesus, 4)

        pygame.draw.lines(self.screen, SpotAnim.CYAN, False, lineR, 2)
        pygame.draw.lines(self.screen, SpotAnim.GREEN, False, lineD, 2)

        if (center_display == True):
            pygame.draw.circle(self.screen, SpotAnim.BLACK, lineR[0], 5)
        pygame.draw.lines(self.screen, SpotAnim.BLACK, False, linedCG, 1)
        pygame.draw.circle(self.screen, SpotAnim.DARK_CYAN, lineR[1], 5)

        pygame.draw.lines(self.screen, SpotAnim.BLACK, False, lineCG, 1)
        if (dCG[2] == True):
            pygame.draw.circle(self.screen, SpotAnim.GREEN, lineCG[0], 3)
        else:
            pygame.draw.circle(self.screen, SpotAnim.RED, lineCG[0], 3)

        pygame.draw.circle(self.screen, SpotAnim.DARK_GREY, lineCG[1], 10)
        pygame.draw.circle(self.screen, SpotAnim.BLACK, lineZMP[0], 4)
        pygame.draw.lines(self.screen, SpotAnim.RED, False, linelf, 4)
        pygame.draw.lines(self.screen, SpotAnim.RED, False, linerf, 4)
        pygame.draw.lines(self.screen, SpotAnim.RED, False, linerr, 4)
        pygame.draw.lines(self.screen, SpotAnim.RED, False, linelr, 4)
        pygame.draw.lines(self.screen, SpotAnim.BLUE, False, lineb, 10)
        pygame.draw.lines(self.screen, SpotAnim.BLACK, False, [[angle[0]/pi*180/45*300+300, 0], [angle[0]/pi*180/45*300+300, 50]], 5)
        pygame.draw.lines(self.screen, SpotAnim.BLACK, False, [[0, angle[1]/pi*180/45*300+300], [50, angle[1]/pi*180/45*300+300]], 5)

        return
